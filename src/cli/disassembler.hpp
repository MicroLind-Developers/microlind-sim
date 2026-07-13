#pragma once

#include <cstdint>
#include <string>

namespace microlind {
class Bus;
class Cpu;
}

namespace microlind::cli {

struct Disasm {
    std::string text;
    uint8_t length{1};
};

Disasm disassemble(Bus& bus, Cpu& cpu, uint16_t pc);

} // namespace microlind::cli
