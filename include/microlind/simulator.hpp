#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include "microlind/bus.hpp"
#include "microlind/clock.hpp"
#include "microlind/cpu.hpp"

namespace microlind {

struct SimulatorMicrocycleResult {
    bool emitted{};
    bool instruction_started{};
    bool instruction_complete{};
    CpuTickResult instruction_result{};
    BusSignals signals{};
    std::size_t pending_bus_cycles{};
};

class Simulator {
public:
    Simulator(CpuMode mode, uint64_t clock_hz);

    std::optional<BusError> map_device(uint16_t start, uint16_t end, std::unique_ptr<BusDevice> device);
    std::optional<BusError> map_device(
        uint16_t start,
        uint16_t end,
        BusDeviceSelect select,
        std::unique_ptr<BusDevice> device);

    CpuTickResult tick();
    SimulatorMicrocycleResult tick_microcycle();
    [[nodiscard]] bool has_pending_microcycles() const { return cpu_.has_pending_micro_ops() || !pending_bus_cycles_.empty(); }
    void run_cycles(uint64_t count);

    Cpu& cpu() { return cpu_; }
    Bus& bus() { return bus_; }
    Clock& clock() { return clock_; }
    const Cpu& cpu() const { return cpu_; }
    const Bus& bus() const { return bus_; }
    const Clock& clock() const { return clock_; }

    // Set PC from the reset vector at $FFFE/$FFFF.
    void reset_from_vector();
    void reset_clock();

    // Advance only the clock/devices by a number of cycles (CPU not executed).
    void tick_clock(uint64_t cycles);

private:
    void queue_instruction_microcycles();
    SimulatorMicrocycleResult emit_pending_microcycle(bool instruction_started);
    SimulatorMicrocycleResult emit_cpu_microcycle();

    Cpu cpu_;
    Bus bus_;
    Clock clock_;
    std::vector<BusSignals> pending_bus_cycles_{};
    CpuTickResult pending_instruction_result_{};
};

// Helper to create a RAM/ROM mapping for quick bring-up.
bool default_memory_map(
    Simulator& sim,
    std::size_t ram_size,
    uint16_t rom_start,
    const std::vector<uint8_t>* rom_image);

} // namespace microlind
