#pragma once

#include <cstdint>

namespace microlind {

class Clock {
public:
    explicit Clock(uint64_t frequency_hz) : frequency_hz_(frequency_hz) {}

    void advance_cycles(uint64_t cycles) { total_cycles_ += cycles; }
    [[nodiscard]] uint64_t total_cycles() const { return total_cycles_; }
    [[nodiscard]] uint64_t frequency_hz() const { return frequency_hz_; }

private:
    uint64_t frequency_hz_{};
    uint64_t total_cycles_{};
};

} // namespace microlind
