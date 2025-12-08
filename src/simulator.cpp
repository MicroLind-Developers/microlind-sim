#include "microlind/simulator.hpp"

#include "microlind/devices/memory.hpp"

namespace microlind {

Simulator::Simulator(CpuMode mode, uint64_t clock_hz) : cpu_(mode), clock_(clock_hz) {}

std::optional<BusError> Simulator::map_device(uint16_t start, uint16_t end, std::unique_ptr<BusDevice> device) {
    return bus_.map_device(start, end, std::move(device));
}

CpuTickResult Simulator::tick() {
    CpuTickResult result = cpu_.tick(bus_);
    bus_.tick_devices(result.cycles);
    clock_.advance_cycles(result.cycles);
    return result;
}

void Simulator::run_cycles(uint64_t count) {
    for (uint64_t i = 0; i < count; ++i) {
        tick();
    }
}

void Simulator::reset_from_vector() {
    const uint8_t hi = bus_.read8(0xFFFE);
    const uint8_t lo = bus_.read8(0xFFFF);
    cpu_.set_pc(static_cast<uint16_t>((hi << 8) | lo));
}

bool default_memory_map(
    Simulator& sim,
    std::size_t ram_size,
    uint16_t rom_start,
    const std::vector<uint8_t>* rom_image) {
    auto ram = std::make_unique<devices::Memory>(ram_size, true);
    const uint16_t ram_end = static_cast<uint16_t>((ram_size - 1) & 0xFFFF);
    if (auto err = sim.map_device(0x0000, ram_end, std::move(ram))) {
        return false;
    }

    if (rom_image) {
        auto rom = std::make_unique<devices::Memory>(rom_image->size(), false);
        rom->load(0, *rom_image);
        const uint16_t end = static_cast<uint16_t>(rom_start + static_cast<uint16_t>(rom_image->size() - 1));
        if (auto err = sim.map_device(rom_start, end, std::move(rom))) {
            (void)err;
            return false;
        }
    }

    return true;
}

} // namespace microlind
