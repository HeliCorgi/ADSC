// Minimal assertion-based tests for the TMR fuel store. No framework needed;
// exits non-zero on first failure so CTest can pick it up.
#include <cassert>
#include <cmath>
#include <cstdio>

#include "adsc/fuel_store.hpp"

using adsc::FuelStore;

static bool approx(double a, double b) { return std::abs(a - b) < 1e-12; }

int main() {
    // 1. Clean read returns the written value.
    {
        FuelStore fs(7.2);
        auto r = fs.read();
        assert(approx(r.value, 7.2));
        assert(r.status == FuelStore::Status::Ok);
    }

    // 2. Single-bit flip in one copy is outvoted and healed.
    {
        FuelStore fs(7.2);
        fs.inject_bitflip(1, 3);          // corrupt copy 1
        auto r = fs.read();
        assert(approx(r.value, 7.2));     // still recovers the true value
        assert(r.status == FuelStore::Status::Recovered);
        auto r2 = fs.read();              // after scrub, clean again
        assert(r2.status == FuelStore::Status::Ok);
    }

    // 3. Two corrupted copies: the lone survivor still carries the value.
    {
        FuelStore fs(4.5);
        fs.inject_bitflip(0, 5);
        fs.inject_bitflip(2, 11);
        auto r = fs.read();
        assert(approx(r.value, 4.5));
        assert(r.status == FuelStore::Status::Recovered);
    }

    // 4. All three corrupted: flagged unrecoverable (no silent bad value).
    {
        FuelStore fs(9.9);
        fs.inject_bitflip(0, 2);
        fs.inject_bitflip(1, 17);
        fs.inject_bitflip(2, 40);
        auto r = fs.read();
        assert(r.status == FuelStore::Status::Unrecoverable);
    }

    // 5. Negative writes clamp to zero.
    {
        FuelStore fs(1.0);
        fs.write(-3.0);
        assert(approx(fs.read().value, 0.0));
    }

    std::printf("fuel_store: all tests passed\n");
    return 0;
}
