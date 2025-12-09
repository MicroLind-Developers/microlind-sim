#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include "microlind/bus.hpp"
#include "microlind/clock.hpp"
#include "microlind/cpu.hpp"

namespace microlind {

class Simulator {
public:
    Simulator(CpuMode mode, uint64_t clock_hz);

    std::optional<BusError> map_device(uint16_t start, uint16_t end, std::unique_ptr<BusDevice> device);

    CpuTickResult tick();
    void run_cycles(uint64_t count);

    Cpu& cpu() { return cpu_; }
    Bus& bus() { return bus_; }
    Clock& clock() { return clock_; }

    // Set PC from the reset vector at $FFFE/$FFFF.
    void reset_from_vector();

    // Advance only the clock/devices by a number of cycles (CPU not executed).
    void tick_clock(uint64_t cycles);

private:
    Cpu cpu_;
    Bus bus_;
    Clock clock_;
};

// Helper to create a RAM/ROM mapping for quick bring-up.
bool default_memory_map(
    Simulator& sim,
    std::size_t ram_size,
    uint16_t rom_start,
    const std::vector<uint8_t>* rom_image);

} // namespace microlind
