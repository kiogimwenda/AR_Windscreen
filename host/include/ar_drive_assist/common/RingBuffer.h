#pragma once
// RingBuffer<T, N> — the message-bus primitive every host subsystem talks through. See
// docs/BUILD_GUIDE.md Part 5.3.
//
// ---------------------------------------------------------------------------------------------
// What this is, and why it needs no mutex
//
// A single-producer / single-consumer (SPSC) queue: exactly ONE thread ever calls push(), and
// exactly ONE (different) thread ever calls pop()/popLatest(). The host's thread layout (Part 5.1)
// is fixed and known, so every bus has a single writer and a single reader, and this restriction
// costs nothing. Use one RingBuffer per (producer, consumer) pair, never share one between three
// threads. That would be a data race, and no test is guaranteed to catch it.
//
// The restriction is what makes the queue lock-free. There are two indices:
//
//     head_  — written ONLY by the producer: the count of items ever pushed
//     tail_  — written ONLY by the consumer: the count of items ever popped
//
// Each index has exactly one writer, so no compare-and-swap is needed, only loads and stores.
// The buffer holds (head_ - tail_) items. Slot i lives at buf_[i % N]. Both indices only ever
// increase, and a size_t would take centuries to wrap at any rate this system produces. N is
// still required to be a power of two, so that i % N stays correct even across that wrap.
//
// ---------------------------------------------------------------------------------------------
// Memory ordering — the part that is easy to get subtly wrong
//
// Making head_ and tail_ std::atomic stops the indices themselves from tearing. On its own, that
// does NOT make the ITEMS in buf_ safe to read. Both the compiler and the CPU may reorder
// ordinary memory writes. Without further instructions, the producer's "write item into slot"
// could become visible to the consumer AFTER its "head_ = head_ + 1". The consumer would then see
// the new head_, read the slot, and get a half-written cv::Mat.
//
// Acquire/release pairs are the fix:
//
//   producer:  buf_[h % N] = item;             consumer:  h = head_.load(acquire);
//              head_.store(h + 1, release);               out = buf_[t % N];
//
// A release store publishes every write made before it. An acquire load that reads that stored
// value is guaranteed to also see all of those earlier writes. So if the consumer's acquire sees
// head_ == h + 1, the item in slot h is complete. The same pairing runs in the other direction,
// on tail_. The producer must not overwrite a slot until the consumer has finished moving the
// old item out of it.
//
// Each thread loads its OWN index with memory_order_relaxed, because nobody else writes it.
//
// ---------------------------------------------------------------------------------------------
// Overflow policy: why "drop oldest" is done by the CONSUMER
//
// Part 5.3 wants frameBus and lidarBus to drop the OLDEST item on overflow, because a stale
// frame is worse than a missing one. In an SPSC queue the producer cannot do that. Discarding the
// oldest item means advancing tail_, and tail_ belongs to the consumer, which may be moving that
// very item out of its slot at the same moment. Letting both threads write tail_ would bring back
// the locking (or CAS retry loop) this design exists to avoid.
//
// So the policy is split between the two ends:
//   - push() never overwrites. It returns false when full, and the producer decides what to do
//     (Part 5.3's "caller decides"). For frame/lidar buses the caller drops the new item and
//     counts it. For the ActuationRequest bus the caller logs an EventLog fault, because a
//     dropped brake request is a safety event.
//   - popLatest() lets a real-time consumer take the NEWEST item and discard everything older in
//     one step. That is where the drop-oldest behaviour actually happens.
// With popLatest() the buffer is emptied on every consumer iteration. It can only fill if the
// consumer has stalled for N producer periods, and that stall is itself a fault worth logging.
// ---------------------------------------------------------------------------------------------

#include <array>
#include <atomic>
#include <cstddef>
#include <type_traits>
#include <utility>

namespace ar_drive_assist {

template <typename T, std::size_t N>
class RingBuffer {
    static_assert(N >= 2, "RingBuffer needs at least two slots");
    static_assert((N & (N - 1)) == 0, "RingBuffer size must be a power of two (see header note)");
    static_assert(std::is_default_constructible_v<T>, "slots are default-constructed up front");

public:
    static constexpr std::size_t capacity() { return N; }

    // Producer thread only. Returns false (and leaves the buffer unchanged) when full.
    bool push(const T& item) { return emplace(item); }
    bool push(T&& item) { return emplace(std::move(item)); }

    // Consumer thread only. Returns false when empty.
    bool pop(T& out) {
        const std::size_t t = tail_.load(std::memory_order_relaxed);
        const std::size_t h = head_.load(std::memory_order_acquire);
        if (h == t) return false;
        out = std::move(buf_[t % N]);
        tail_.store(t + 1, std::memory_order_release);
        return true;
    }

    // Consumer thread only. Takes the newest item and discards everything older. Returns false when
    // empty. If `discarded` is non-null it receives the number of older items thrown away, so a
    // consumer can log that it is falling behind.
    //
    // Discarded slots are reset to T{} rather than left in place. For a type like cv::Mat, a slot
    // left holding a stale frame keeps that frame's pixel buffer allocated until the slot is
    // overwritten — up to N full frames of memory pinned for no reason.
    bool popLatest(T& out, std::size_t* discarded = nullptr) {
        const std::size_t t = tail_.load(std::memory_order_relaxed);
        const std::size_t h = head_.load(std::memory_order_acquire);
        if (h == t) {
            if (discarded) *discarded = 0;
            return false;
        }
        for (std::size_t i = t; i + 1 < h; ++i) buf_[i % N] = T{};
        out = std::move(buf_[(h - 1) % N]);
        tail_.store(h, std::memory_order_release);
        if (discarded) *discarded = h - t - 1;
        return true;
    }

    // Snapshot for diagnostics only. The other thread may change it immediately after this
    // returns, so never use it to decide whether a push() or pop() will succeed.
    std::size_t sizeApprox() const {
        return head_.load(std::memory_order_acquire) - tail_.load(std::memory_order_acquire);
    }

private:
    template <typename U>
    bool emplace(U&& item) {
        const std::size_t h = head_.load(std::memory_order_relaxed);
        const std::size_t t = tail_.load(std::memory_order_acquire);
        if (h - t == N) return false;
        buf_[h % N] = std::forward<U>(item);
        head_.store(h + 1, std::memory_order_release);
        return true;
    }

    std::array<T, N> buf_{};

    // head_ and tail_ are each written constantly by a different thread. If they shared a
    // 64-byte cache line, every write by one core would invalidate the other core's copy of that
    // line ("false sharing"), even though neither thread touches the other's variable. Aligning
    // each to its own line keeps the two threads out of each other's way.
    alignas(64) std::atomic<std::size_t> head_{0};
    alignas(64) std::atomic<std::size_t> tail_{0};
};

}  // namespace ar_drive_assist
