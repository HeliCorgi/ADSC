#include "adsc/fuel_store.hpp"

#include <algorithm>
#include <cstring>

namespace adsc {

// Standard reflected CRC32 (IEEE 802.3, poly 0xEDB88320) over the raw bytes of
// {value, version}. Uses memcpy to read the 8-byte double — no strict-aliasing
// UB, and every byte of the mantissa is covered (the old code aliased a double
// through a uint32_t* and only hashed 4 of 8 bytes, so half of all single-bit
// flips went undetected).
uint32_t FuelStore::crc32(double value, uint32_t version) {
    unsigned char buf[sizeof(double) + sizeof(uint32_t)];
    std::memcpy(buf, &value, sizeof(double));
    std::memcpy(buf + sizeof(double), &version, sizeof(uint32_t));

    uint32_t crc = 0xFFFFFFFFu;
    for (unsigned char byte : buf) {
        crc ^= byte;
        for (int i = 0; i < 8; ++i) {
            uint32_t mask = -(crc & 1u);
            crc = (crc >> 1) ^ (0xEDB88320u & mask);
        }
    }
    return ~crc;
}

bool FuelStore::valid(const Cell& c) {
    return c.crc == crc32(c.value, c.version);
}

double FuelStore::median3(double a, double b, double c) {
    if (a > b) std::swap(a, b);
    if (b > c) std::swap(b, c);
    if (a > b) std::swap(a, b);
    return b;
}

FuelStore::FuelStore(double initial) {
    write(initial);
}

void FuelStore::write(double value) {
    if (value < 0.0) value = 0.0;
    ++version_;
    Cell fresh{value, version_, crc32(value, version_)};
    for (auto& cell : cells_) cell = fresh;
}

FuelStore::Reading FuelStore::read() {
    // Gather the copies that still pass their CRC.
    std::array<double, 3> good{};
    int n = 0;
    for (const auto& c : cells_) {
        if (valid(c)) good[n++] = c.value;
    }

    if (n == 0) {
        // Every copy corrupted: nothing trustworthy to return.
        return {cells_[0].value, Status::Unrecoverable};
    }

    double consensus;
    Status status;
    if (n == 3) {
        consensus = median3(good[0], good[1], good[2]);
        status    = Status::Ok;
    } else {
        // One or two copies survived. Take the lowest surviving value — the
        // conservative choice for a fuel reserve — and flag that we healed.
        consensus = *std::min_element(good.begin(), good.begin() + n);
        status    = Status::Recovered;
    }

    // Scrub: rewrite the healthy consensus into all three copies so a second
    // future flip starts from a clean slate. Keeps the same version.
    Cell healed{consensus, version_, crc32(consensus, version_)};
    for (auto& cell : cells_) cell = healed;

    return {consensus, status};
}

void FuelStore::inject_bitflip(std::size_t copy_index, std::size_t bit_index) {
    if (copy_index >= cells_.size()) return;
    auto* raw = reinterpret_cast<unsigned char*>(&cells_[copy_index]);
    const std::size_t byte = (bit_index / 8) % sizeof(Cell);
    raw[byte] ^= static_cast<unsigned char>(1u << (bit_index % 8));
}

}  // namespace adsc
