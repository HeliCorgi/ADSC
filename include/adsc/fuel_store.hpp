#pragma once

#include <array>
#include <cstdint>

namespace adsc {

// Triple-Modular-Redundant scalar store for a single flight-critical value
// (fuel mass). Protects against Single-Event-Upset (SEU) bit flips in memory
// via three independent copies, a per-copy CRC32, majority voting and
// scrubbing.
//
// NOTE ON THREADING: this is intentionally NOT atomic. On the RAD5545 target
// there is no lock-free 128-bit CAS, so std::atomic<Cell> would silently fall
// back to a mutex — which is the wrong tool. TMR here guards against radiation
// bit flips in a single-threaded control loop, not against data races. If this
// value is ever shared across ISRs, wrap access at the caller with the
// platform's interrupt-masking primitive.
class FuelStore {
public:
    enum class Status {
        Ok,             // all copies agreed
        Recovered,      // a corrupted copy was outvoted and scrubbed
        Unrecoverable   // fewer than one valid copy — value is untrustworthy
    };

    struct Reading {
        double value;
        Status status;
    };

    explicit FuelStore(double initial);

    // Reads the value via majority vote. Non-const because a successful vote
    // scrubs (rewrites) the consensus back into all three copies.
    Reading read();

    // Writes a new value into all three copies with a fresh version + CRC.
    // Clamps negatives to zero.
    void write(double value);

    // Test / fault-injection hook: flips one bit in the raw storage of a copy
    // to emulate an SEU. Not for flight use.
    void inject_bitflip(std::size_t copy_index, std::size_t bit_index);

private:
    struct Cell {
        double   value;
        uint32_t version;
        uint32_t crc;
    };

    static uint32_t crc32(double value, uint32_t version);
    static bool     valid(const Cell& c);
    static double   median3(double a, double b, double c);

    std::array<Cell, 3> cells_;
    uint32_t            version_ = 0;
};

}  // namespace adsc
