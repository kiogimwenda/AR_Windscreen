#pragma once
// FrameCodec — the Part 3.1 frame, encoded and parsed. Header-only, no Arduino dependency, so it
// is unit-tested natively (`pio test -e native`).
//
//   +-----------+------+-------------+---------------------+-------------+
//   | 0xAA      | type | length (LE) | payload (length B)  | crc16 (LE)  |
//   +-----------+------+-------------+---------------------+-------------+
//   CRC-16/CCITT-FALSE over type + length + payload.
//
// The parser takes one byte at a time (from the USB-CDC stream) and reports a complete, CRC-valid
// frame, or an error. On any error it goes back to hunting for the next 0xAA, so garbage, a lost
// byte or a truncated frame costs at most that frame; it never desynchronises for good. A frame
// with a bad CRC is reported as an error and never delivered: the hub must never act on a
// corrupted actuation command (Part 3.1).

#include <cstddef>
#include <cstdint>
#include <cstring>

#include "hub/Crc16.h"
#include "hub/Protocol.h"

namespace hub {

// Writes a frame into `out` (at least MAX_FRAME_SIZE bytes). Returns its size, or 0 if the payload
// is too long.
inline std::size_t encodeFrame(hub_protocol::MessageType type, const void* payload, std::size_t len,
                               uint8_t* out) {
    using namespace hub_protocol;
    if (len > MAX_PAYLOAD) return 0;
    out[0] = START_BYTE;
    out[1] = static_cast<uint8_t>(type);
    out[2] = static_cast<uint8_t>(len & 0xFF);
    out[3] = static_cast<uint8_t>(len >> 8);
    if (len) std::memcpy(out + HEADER_SIZE, payload, len);
    const uint16_t crc =
        crc16_ccitt_false(out + CRC_COVERED_OFFSET, HEADER_SIZE - CRC_COVERED_OFFSET + len);
    out[HEADER_SIZE + len] = static_cast<uint8_t>(crc & 0xFF);
    out[HEADER_SIZE + len + 1] = static_cast<uint8_t>(crc >> 8);
    return HEADER_SIZE + len + CRC_SIZE;
}

class FrameParser {
public:
    enum class Result { NONE, FRAME, ERROR };

    // Received bytes go in with push(); next() processes them and returns each FRAME or ERROR as
    // it completes, NONE once everything queued has been consumed. Callers drain it:
    //     parser.push(byte);
    //     for (Result r; (r = parser.next()) != Result::NONE;) handle(r);
    // On FRAME, type()/payload()/length() describe that frame until the next call to next().
    //
    // Resynchronisation (both found by the corruption test):
    //   - when a frame fails (bad CRC, impossible length), the bytes it had swallowed after its
    //     start byte are re-scanned for the next 0xAA;
    //   - while a frame with a (possibly corrupted) long length is still arriving, a complete
    //     CRC-valid frame ending at the newest byte is delivered at once.
    // Without them, a corrupted LENGTH field held back up to 66 following bytes: ~300 ms of
    // heartbeats, enough to trip the 200 ms link timeout (safe, it only disarms, but spurious).
    void push(uint8_t b) {
        if (count() < kQueue)
            queue_[(head_ + count_++) % kQueue] = b;  // overflow: drop (never block)
    }
    Result next() {
        while (count_ > 0) {
            const uint8_t b = queue_[head_];
            head_ = (head_ + 1) % kQueue;
            --count_;
            const Result r = step(b);
            if (r != Result::NONE) return r;
        }
        return Result::NONE;
    }
    // Single-byte convenience for callers that never see two frames complete at once.
    Result feed(uint8_t b) {
        push(b);
        return next();
    }

    hub_protocol::MessageType type() const {
        return static_cast<hub_protocol::MessageType>(frame_[1]);
    }
    const uint8_t* payload() const { return frame_ + hub_protocol::HEADER_SIZE; }
    std::size_t length() const { return frameLen_; }

    // Copies the payload into a protocol struct if the length matches exactly.
    template <typename T>
    bool as(T& out) const {
        if (frameLen_ != sizeof(T)) return false;
        std::memcpy(&out, payload(), sizeof(T));
        return true;
    }

private:
    static constexpr std::size_t kQueue = 4 * hub_protocol::MAX_FRAME_SIZE;
    std::size_t count() const { return count_; }

    Result step(uint8_t b) {
        using namespace hub_protocol;
        if (state_ == State::START) {
            if (b != START_BYTE) return Result::NONE;  // bytes between frames are skipped silently
            got_ = 0;
        }
        buf_[got_++] = b;
        switch (state_) {
            case State::START:
                state_ = State::TYPE;
                return Result::NONE;
            case State::TYPE:
                state_ = State::LEN_LO;
                return Result::NONE;
            case State::LEN_LO:
                state_ = State::LEN_HI;
                return Result::NONE;
            case State::LEN_HI:
                len_ = static_cast<std::size_t>(buf_[2]) | (static_cast<std::size_t>(buf_[3]) << 8);
                if (len_ > MAX_PAYLOAD) return fail();
                state_ = State::BODY;
                return Result::NONE;
            case State::BODY:
                if (got_ < HEADER_SIZE + len_ + CRC_SIZE) {
                    // While waiting for a long body, a complete CRC-valid frame may already END
                    // here, starting at a later 0xAA: the real stream resuming after a corrupted
                    // length. Deliver it now rather than after the bogus frame finally fails (a
                    // false match needs a 1-in-65536 CRC coincidence).
                    return embeddedFrameEndingHere() ? Result::FRAME : Result::NONE;
                }
                {
                    const uint16_t rx = static_cast<uint16_t>(buf_[HEADER_SIZE + len_] |
                                                              (buf_[HEADER_SIZE + len_ + 1] << 8));
                    const uint16_t calc = crc16_ccitt_false(
                        buf_ + CRC_COVERED_OFFSET, HEADER_SIZE - CRC_COVERED_OFFSET + len_);
                    if (rx != calc) return fail();
                    std::memcpy(frame_, buf_, got_);
                    frameLen_ = len_;
                    state_ = State::START;
                    return Result::FRAME;
                }
        }
        return fail();
    }
    bool embeddedFrameEndingHere() {
        using namespace hub_protocol;
        for (std::size_t j = 1; j + HEADER_SIZE + CRC_SIZE <= got_; ++j) {
            if (buf_[j] != START_BYTE) continue;
            const std::size_t len = static_cast<std::size_t>(buf_[j + 2]) |
                                    (static_cast<std::size_t>(buf_[j + 3]) << 8);
            if (len > MAX_PAYLOAD || j + HEADER_SIZE + len + CRC_SIZE != got_) continue;
            const uint16_t rx = static_cast<uint16_t>(buf_[j + HEADER_SIZE + len] |
                                                      (buf_[j + HEADER_SIZE + len + 1] << 8));
            if (rx != crc16_ccitt_false(buf_ + j + CRC_COVERED_OFFSET,
                                        HEADER_SIZE - CRC_COVERED_OFFSET + len))
                continue;
            std::memcpy(frame_, buf_ + j, got_ - j);
            frameLen_ = len;
            state_ = State::START;
            got_ = 0;
            return true;
        }
        return false;
    }

    // Abandon the current frame and put everything after its start byte back at the FRONT of the
    // queue, so the scan resumes one byte after the false start.
    Result fail() {
        for (std::size_t i = got_; i > 1; --i) {
            if (count_ >= kQueue) break;
            head_ = (head_ + kQueue - 1) % kQueue;
            queue_[head_] = buf_[i - 1];
            ++count_;
        }
        state_ = State::START;
        got_ = 0;
        return Result::ERROR;
    }

    enum class State { START, TYPE, LEN_LO, LEN_HI, BODY };
    State state_ = State::START;
    uint8_t buf_[hub_protocol::MAX_FRAME_SIZE] = {};    // the frame being assembled
    uint8_t frame_[hub_protocol::MAX_FRAME_SIZE] = {};  // the last complete frame
    std::size_t got_ = 0, len_ = 0, frameLen_ = 0;
    uint8_t queue_[kQueue] = {};
    std::size_t head_ = 0, count_ = 0;
};

}  // namespace hub
