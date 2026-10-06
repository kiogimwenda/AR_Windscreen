#pragma once
// FrameDecoder — the host's side of the Part 3.1 frame: encode, and decode a byte stream.
// See docs/BUILD_GUIDE.md Part 3.1 and 11.1.
//
//   0xAA | type | length (2 bytes, LE) | payload | CRC-16/CCITT-FALSE over type+length+payload (LE)
//
// Written independently of the firmware's FrameCodec.h (which is sized for a microcontroller: a
// byte queue and fixed buffers) and tested AGAINST it: test_vehicle_interface links the firmware's
// encoder and parser and checks that each side reads the other's bytes. Two implementations
// agreeing is stronger evidence than one implementation agreeing with itself.
//
// Decoding keeps a byte buffer. A candidate frame starts at 0xAA; it is delivered only if its CRC
// matches. On a bad CRC or an impossible length, ONE byte is dropped (the false 0xAA) and the scan
// resumes, so every byte the bad candidate covered is re-scanned. While a candidate with a long
// (possibly corrupted) length is incomplete, a later complete, CRC-valid frame already in the
// buffer proves the candidate bogus: it is dropped at once rather than holding the stream up.
// A corrupted frame is therefore never delivered and costs at most itself.

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <optional>
#include <vector>

#include "ar_drive_assist/vehicle/Crc16.h"
#include "ar_drive_assist/vehicle/HubProtocol.h"

namespace ar_drive_assist {

struct HubFrame {
    hub_protocol::MessageType type{};
    std::vector<std::uint8_t> payload;

    // Copies the payload into a protocol struct if the length matches exactly (packed struct:
    // memcpy, never a pointer cast).
    template <typename T>
    bool as(T& out) const {
        if (payload.size() != sizeof(T)) return false;
        std::memcpy(&out, payload.data(), sizeof(T));
        return true;
    }
};

inline std::vector<std::uint8_t> encodeFrame(hub_protocol::MessageType type, const void* payload,
                                             std::size_t len) {
    using namespace hub_protocol;
    std::vector<std::uint8_t> out;
    if (len > MAX_PAYLOAD) return out;
    out.reserve(HEADER_SIZE + len + CRC_SIZE);
    out.push_back(START_BYTE);
    out.push_back(static_cast<std::uint8_t>(type));
    out.push_back(static_cast<std::uint8_t>(len & 0xFF));
    out.push_back(static_cast<std::uint8_t>(len >> 8));
    const auto* p = static_cast<const std::uint8_t*>(payload);
    out.insert(out.end(), p, p + len);
    const std::uint16_t crc = crc16_ccitt_false(out.data() + CRC_COVERED_OFFSET, 3 + len);
    out.push_back(static_cast<std::uint8_t>(crc & 0xFF));
    out.push_back(static_cast<std::uint8_t>(crc >> 8));
    return out;
}

class FrameDecoder {
public:
    void push(const std::uint8_t* data, std::size_t n) { buf_.insert(buf_.end(), data, data + n); }

    // The next complete, CRC-valid frame, or nullopt when more bytes are needed.
    std::optional<HubFrame> next() {
        using namespace hub_protocol;
        while (true) {
            std::size_t i = 0;
            while (i < buf_.size() && buf_[i] != START_BYTE) ++i;
            if (i) {
                buf_.erase(buf_.begin(), buf_.begin() + static_cast<std::ptrdiff_t>(i));
                skipped_ += i;
            }
            if (buf_.size() < HEADER_SIZE) return std::nullopt;
            const std::size_t len = buf_[2] | (static_cast<std::size_t>(buf_[3]) << 8);
            if (len > MAX_PAYLOAD) {
                dropOne();
                continue;
            }
            const std::size_t total = HEADER_SIZE + len + CRC_SIZE;
            if (buf_.size() < total) {
                if (laterValidFrame()) {  // this candidate is bogus: the real stream has resumed
                    dropOne();
                    continue;
                }
                return std::nullopt;
            }
            if (!crcOk(0, len)) {
                dropOne();
                continue;
            }
            HubFrame f;
            f.type = static_cast<MessageType>(buf_[1]);
            f.payload.assign(buf_.begin() + HEADER_SIZE, buf_.begin() + HEADER_SIZE + len);
            buf_.erase(buf_.begin(), buf_.begin() + static_cast<std::ptrdiff_t>(total));
            ++frames_;
            return f;
        }
    }

    std::uint64_t frames() const { return frames_; }
    std::uint64_t errors() const { return errors_; }  // false starts: bad CRC or impossible length
    std::uint64_t skippedBytes() const { return skipped_; }

private:
    bool crcOk(std::size_t at, std::size_t len) const {
        using namespace hub_protocol;
        const std::uint16_t rx = static_cast<std::uint16_t>(
            buf_[at + HEADER_SIZE + len] | (buf_[at + HEADER_SIZE + len + 1] << 8));
        return rx == crc16_ccitt_false(buf_.data() + at + CRC_COVERED_OFFSET, 3 + len);
    }
    bool laterValidFrame() const {
        using namespace hub_protocol;
        for (std::size_t j = 1; j + HEADER_SIZE + CRC_SIZE <= buf_.size(); ++j) {
            if (buf_[j] != START_BYTE) continue;
            const std::size_t len = buf_[j + 2] | (static_cast<std::size_t>(buf_[j + 3]) << 8);
            if (len > MAX_PAYLOAD || j + HEADER_SIZE + len + CRC_SIZE > buf_.size()) continue;
            if (crcOk(j, len)) return true;
        }
        return false;
    }
    void dropOne() {
        buf_.erase(buf_.begin());
        ++errors_;
    }

    std::vector<std::uint8_t> buf_;
    std::uint64_t frames_ = 0, errors_ = 0, skipped_ = 0;
};

}  // namespace ar_drive_assist
