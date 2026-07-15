#pragma once

#include <cstdint>
#include <filesystem>
#include <initializer_list>
#include <memory>
#include <stdexcept>
#include <string>

#include "microlind/app/disassembler.hpp"
#include "microlind/app/sim_session.hpp"
#include "microlind/bus.hpp"
#include "microlind/devices/memory.hpp"

namespace microlind::test {

inline app::SimSession loaded_session() {
    app::SimSession session;
    if (!session.load_hardware_config("tests/data/hw_test.cfg")) {
        throw std::runtime_error("failed to load tests/data/hw_test.cfg");
    }
    if (!session.load_rom("examples/bios.ihex", cli::RomFormat::Ihex, 0x8000)) {
        throw std::runtime_error("failed to load examples/bios.ihex");
    }
    session.reset();
    return session;
}

inline cli::Disasm disassemble_bytes(app::SimSession& session,
                                     uint16_t address,
                                     std::initializer_list<uint8_t> bytes) {
    uint16_t offset = 0;
    for (uint8_t byte : bytes) {
        session.write_memory(static_cast<uint16_t>(address + offset), byte);
        ++offset;
    }

    auto& sim = session.simulator();
    return cli::disassemble(sim.bus(), sim.cpu(), address);
}

inline std::filesystem::path test_output_path(const std::string& name) {
    const auto dir = std::filesystem::temp_directory_path() / "microlind-sim-tests";
    std::filesystem::create_directories(dir);
    return dir / name;
}

inline void map_flat_ram(Bus& bus) {
    const auto error = bus.map_device(0x0000, 0xFFFF, std::make_unique<devices::Memory>(0x10000, true));
    if (error) {
        throw std::runtime_error("failed to map flat test RAM");
    }
}

} // namespace microlind::test
