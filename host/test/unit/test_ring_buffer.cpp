// RingBuffer unit tests — see docs/BUILD_GUIDE.md Part 5.3.
//
// The single-threaded tests pin down the queue semantics (FIFO order, full/empty, wraparound,
// popLatest). The two-threaded test is the one that matters for a lock-free structure. It pushes
// a long numbered sequence through a small buffer and checks that the consumer sees every number
// exactly once, in order. A memory-ordering bug shows up there as a skipped, repeated or corrupted
// value. It shows up reliably, though, only under ThreadSanitizer. Build this suite with
// -DCMAKE_CXX_FLAGS=-fsanitize=thread to check for races (see the progress log for the command).

#include <gtest/gtest.h>

#include <cstdint>
#include <memory>
#include <thread>
#include <vector>

#include "ar_drive_assist/common/RingBuffer.h"

using ar_drive_assist::RingBuffer;

TEST(RingBuffer, StartsEmpty) {
    RingBuffer<int, 4> rb;
    int out = -1;
    EXPECT_FALSE(rb.pop(out));
    EXPECT_FALSE(rb.popLatest(out));
    EXPECT_EQ(out, -1);  // a failed pop must not touch `out`
    EXPECT_EQ(rb.sizeApprox(), 0u);
}

TEST(RingBuffer, PopsInFifoOrder) {
    RingBuffer<int, 4> rb;
    for (int i = 1; i <= 3; ++i) ASSERT_TRUE(rb.push(i));
    int out = 0;
    for (int i = 1; i <= 3; ++i) {
        ASSERT_TRUE(rb.pop(out));
        EXPECT_EQ(out, i);
    }
    EXPECT_FALSE(rb.pop(out));
}

// Capacity is exactly N. The monotonic-counter design does not sacrifice a slot to tell "full"
// from "empty", as the classic head==tail ring does.
TEST(RingBuffer, HoldsExactlyNThenRejects) {
    RingBuffer<int, 4> rb;
    for (int i = 0; i < 4; ++i) ASSERT_TRUE(rb.push(i));
    EXPECT_FALSE(rb.push(99));
    EXPECT_EQ(rb.sizeApprox(), 4u);

    // A rejected push must not have overwritten the oldest item.
    int out = -1;
    ASSERT_TRUE(rb.pop(out));
    EXPECT_EQ(out, 0);
}

TEST(RingBuffer, WrapsAroundManyTimes) {
    RingBuffer<int, 4> rb;
    int out = 0;
    for (int i = 0; i < 1000; ++i) {
        ASSERT_TRUE(rb.push(i));
        ASSERT_TRUE(rb.pop(out));
        ASSERT_EQ(out, i);
    }
}

TEST(RingBuffer, PopLatestTakesNewestAndReportsDiscards) {
    RingBuffer<int, 8> rb;
    for (int i = 1; i <= 5; ++i) ASSERT_TRUE(rb.push(i));
    int out = 0;
    std::size_t discarded = 0;
    ASSERT_TRUE(rb.popLatest(out, &discarded));
    EXPECT_EQ(out, 5);
    EXPECT_EQ(discarded, 4u);
    EXPECT_EQ(rb.sizeApprox(), 0u);  // everything older is gone, not just skipped once

    ASSERT_TRUE(rb.push(6));
    ASSERT_TRUE(rb.popLatest(out, &discarded));
    EXPECT_EQ(out, 6);
    EXPECT_EQ(discarded, 0u);
}

// popLatest() must release what the discarded slots owned, not just skip over them. Otherwise a
// frame bus pins up to N stale cv::Mat buffers in memory. shared_ptr's use_count makes the
// release observable.
TEST(RingBuffer, PopLatestReleasesDiscardedItems) {
    RingBuffer<std::shared_ptr<int>, 4> rb;
    auto stale = std::make_shared<int>(1);
    ASSERT_TRUE(rb.push(stale));
    ASSERT_TRUE(rb.push(std::make_shared<int>(2)));
    EXPECT_EQ(stale.use_count(), 2);  // ours + the slot's copy

    std::shared_ptr<int> out;
    ASSERT_TRUE(rb.popLatest(out));
    EXPECT_EQ(*out, 2);
    EXPECT_EQ(stale.use_count(), 1);  // the slot let go of it
}

// pop() moves out of the slot, so the slot does not keep its own reference either.
TEST(RingBuffer, PopReleasesSlot) {
    RingBuffer<std::shared_ptr<int>, 2> rb;
    auto item = std::make_shared<int>(7);
    ASSERT_TRUE(rb.push(item));
    std::shared_ptr<int> out;
    ASSERT_TRUE(rb.pop(out));
    EXPECT_EQ(item.use_count(), 2);  // ours + `out`; the slot holds nothing
}

TEST(RingBuffer, TwoThreadsSeeEverySequenceNumberInOrder) {
    // Small buffer and a long sequence, so the producer hits "full" and the consumer hits "empty"
    // constantly. That makes both threads keep reading each other's index.
    constexpr std::uint64_t kCount = 2'000'000;
    RingBuffer<std::uint64_t, 16> rb;

    std::thread producer([&] {
        for (std::uint64_t i = 0; i < kCount; ++i) {
            while (!rb.push(i)) std::this_thread::yield();
        }
    });

    std::uint64_t expected = 0;
    std::uint64_t mismatches = 0;
    std::uint64_t out = 0;
    while (expected < kCount) {
        if (rb.pop(out)) {
            if (out != expected) ++mismatches;
            ++expected;
        } else {
            std::this_thread::yield();
        }
    }
    producer.join();

    EXPECT_EQ(mismatches, 0u);
    EXPECT_FALSE(rb.pop(out));
}

// The popLatest() consumer may skip values but must never see one go BACKWARDS or appear
// twice. Going backwards is exactly what a stale read of head_ or of a slot would produce.
TEST(RingBuffer, TwoThreadsPopLatestIsStrictlyIncreasing) {
    constexpr std::uint64_t kCount = 1'000'000;
    RingBuffer<std::uint64_t, 8> rb;

    std::thread producer([&] {
        for (std::uint64_t i = 1; i <= kCount; ++i) {
            while (!rb.push(i)) std::this_thread::yield();
        }
    });

    std::uint64_t last = 0;
    std::uint64_t regressions = 0;
    std::uint64_t out = 0;
    while (last < kCount) {
        if (rb.popLatest(out)) {
            if (out <= last) ++regressions;
            last = out;
        } else {
            std::this_thread::yield();
        }
    }
    producer.join();

    EXPECT_EQ(regressions, 0u);
    EXPECT_EQ(last, kCount);  // the final value always arrives, however many were skipped
}
