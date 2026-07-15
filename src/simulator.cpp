#include "microlind/simulator.hpp"

#include <cstddef>

#include "microlind/devices/memory.hpp"

namespace microlind {

Simulator::Simulator(CpuMode mode, uint64_t clock_hz) : cpu_(mode), clock_(clock_hz) {}

std::optional<BusError> Simulator::map_device(uint16_t start, uint16_t end, std::unique_ptr<BusDevice> device) {
    return bus_.map_device(start, end, std::move(device));
}

std::optional<BusError> Simulator::map_device(
    uint16_t start,
    uint16_t end,
    BusDeviceSelect select,
    std::unique_ptr<BusDevice> device) {
    return bus_.map_device(start, end, select, std::move(device));
}

CpuTickResult Simulator::tick() {
    if (cpu_.has_pending_micro_ops()) {
        CpuTickResult result{};
        while (cpu_.has_pending_micro_ops()) {
            const auto micro = emit_cpu_microcycle();
            result = micro.instruction_result;
        }
        return result;
    }

    if (!pending_bus_cycles_.empty()) {
        const CpuTickResult result = pending_instruction_result_;
        while (!pending_bus_cycles_.empty()) {
            emit_pending_microcycle(false);
        }
        return result;
    }

    const uint64_t bus_cycles_before = bus_.bus_cycle_count();
    CpuTickResult result = cpu_.tick(bus_);
    const uint64_t emitted_bus_cycles = bus_.bus_cycle_count() - bus_cycles_before;
    if (emitted_bus_cycles < result.cycles) {
        bus_.tick_idle_cycles(
            static_cast<uint32_t>(result.cycles - emitted_bus_cycles),
            cpu_.regs().pc,
            BusCycleKind::Internal);
    }
    clock_.advance_cycles(result.cycles);
    bus_.tick_devices(result.cycles);
    return result;
}

void Simulator::queue_instruction_microcycles() {
    bus_.begin_deferred_bus_cycles();
    pending_instruction_result_ = cpu_.tick(bus_);
    const std::size_t emitted_bus_cycles = bus_.deferred_bus_cycle_count();
    if (emitted_bus_cycles < pending_instruction_result_.cycles) {
        bus_.tick_idle_cycles(
            static_cast<uint32_t>(pending_instruction_result_.cycles - emitted_bus_cycles),
            cpu_.regs().pc,
            BusCycleKind::Internal);
    }
    pending_bus_cycles_ = bus_.take_deferred_bus_cycles();
}

SimulatorMicrocycleResult Simulator::emit_pending_microcycle(bool instruction_started) {
    if (pending_bus_cycles_.empty()) {
        return {};
    }

    const BusSignals signals = pending_bus_cycles_.front();
    pending_bus_cycles_.erase(pending_bus_cycles_.begin());
    bus_.tick_bus_cycle(signals);
    clock_.advance_cycles(1);
    bus_.tick_devices(1);

    const bool instruction_complete = pending_bus_cycles_.empty();
    const CpuTickResult result = pending_instruction_result_;
    if (instruction_complete) {
        pending_instruction_result_ = {};
    }
    return SimulatorMicrocycleResult{
        true,
        instruction_started,
        instruction_complete,
        result,
        bus_.last_signals(),
        pending_bus_cycles_.size(),
    };
}

SimulatorMicrocycleResult Simulator::emit_cpu_microcycle() {
    BusSignals signals;
    CpuMicrocycleStatus prepared;
    if (!cpu_.prepare_microcycle(bus_, signals, prepared)) {
        return {};
    }

    bus_.tick_bus_cycle(signals);
    clock_.advance_cycles(1);
    bus_.tick_devices(1);

    const CpuMicrocycleStatus completed = cpu_.complete_microcycle(bus_.last_signals());
    return SimulatorMicrocycleResult{
        true,
        prepared.instruction_started,
        completed.instruction_complete,
        completed.instruction_result,
        bus_.last_signals(),
        completed.pending_bus_cycles,
    };
}

SimulatorMicrocycleResult Simulator::emit_cpu_wait_microcycle() {
    BusSignals signals;
    signals.address = cpu_.regs().pc;
    signals.cycle_kind = BusCycleKind::Internal;
    bus_.tick_bus_cycle(signals);
    clock_.advance_cycles(1);
    bus_.tick_devices(1);
    return SimulatorMicrocycleResult{
        true,
        false,
        false,
        CpuTickResult{1},
        bus_.last_signals(),
        0,
    };
}

SimulatorMicrocycleResult Simulator::tick_microcycle() {
    if (cpu_.has_pending_micro_ops()) {
        return emit_cpu_microcycle();
    }

    if (cpu_.waiting_for_interrupt() && !cpu_.interrupt_line_asserted()) {
        return emit_cpu_wait_microcycle();
    }

    if (pending_bus_cycles_.empty()) {
        const auto cpu_microcycle = emit_cpu_microcycle();
        if (cpu_microcycle.emitted) {
            return cpu_microcycle;
        }
    }

    bool instruction_started = false;
    if (pending_bus_cycles_.empty()) {
        queue_instruction_microcycles();
        instruction_started = true;
    }
    return emit_pending_microcycle(instruction_started);
}

void Simulator::run_cycles(uint64_t count) {
    for (uint64_t i = 0; i < count; ++i) {
        tick();
    }
}

void Simulator::tick_clock(uint64_t cycles) {
    bus_.tick_devices(static_cast<uint32_t>(cycles));
    bus_.tick_idle_cycles(static_cast<uint32_t>(cycles), cpu_.regs().pc);
    clock_.advance_cycles(cycles);
}

void Simulator::reset_from_vector() {
    pending_bus_cycles_.clear();
    pending_instruction_result_ = {};
    cpu_.discard_micro_ops();
    cpu_.reset();
    const uint8_t hi = bus_.read8(0xFFFE, BusCycleKind::VectorRead);
    const uint8_t lo = bus_.read8(0xFFFF, BusCycleKind::VectorRead);
    cpu_.set_pc(static_cast<uint16_t>((hi << 8) | lo));
}

void Simulator::reset_clock() {
    clock_.reset();
}

bool default_memory_map(
    Simulator& sim,
    std::size_t ram_size,
    uint16_t rom_start,
    const std::vector<uint8_t>* rom_image) {
    auto ram = std::make_unique<devices::Memory>(ram_size, true);
    const uint16_t ram_end = static_cast<uint16_t>((ram_size - 1) & 0xFFFF);
    if (auto err = sim.map_device(0x0000, ram_end, BusDeviceSelect::Ram, std::move(ram))) {
        return false;
    }

    if (rom_image) {
        auto rom = std::make_unique<devices::Memory>(rom_image->size(), false);
        rom->load(0, *rom_image);
        const uint16_t end = static_cast<uint16_t>(rom_start + static_cast<uint16_t>(rom_image->size() - 1));
        if (auto err = sim.map_device(rom_start, end, BusDeviceSelect::Rom, std::move(rom))) {
            (void)err;
            return false;
        }
    }

    return true;
}

} // namespace microlind
