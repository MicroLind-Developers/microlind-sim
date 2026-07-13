#include "disassembler.hpp"

#include "util.hpp"

#include <cctype>
#include <cstddef>
#include <iomanip>
#include <sstream>

#include "microlind/bus.hpp"
#include "microlind/cpu.hpp"

namespace microlind::cli {
namespace {

enum class OperandKind { None, Immediate, Direct, Extended, Indexed };

OperandKind operand_kind(uint8_t prefix, uint8_t opcode, const std::string& name, uint8_t& size_override) {
    // Explicit overrides for instructions without addressing encoded in their name.
    const uint16_t key = static_cast<uint16_t>((prefix << 8) | opcode);
    switch (key) {
    case 0x113D: // LDMD #imm
    case 0x1031: // ADCR postbyte
    case 0x1032: // SUBR postbyte
    case 0x1033: // SBCR postbyte
    case 0x1035: // ORR postbyte
    case 0x1036: // EORR postbyte
    case 0x1037: // CMPR postbyte
    case 0x1138: // TFM r0+,r1+
    case 0x1139: // TFM r0-,r1-
    case 0x113A: // TFM r0+,r1
    case 0x113B: // TFM r0,r1+
        size_override = 1;
        return OperandKind::Immediate;
    default:
        break;
    }

    if (name.find(" imm") != std::string::npos) return OperandKind::Immediate;
    if (name.find(" dir") != std::string::npos) return OperandKind::Direct;
    if (name.find(" ext") != std::string::npos) return OperandKind::Extended;
    if (name.find(" idx") != std::string::npos) return OperandKind::Indexed;
    return OperandKind::None;
}

uint8_t operand_size_from_mnemonic(const std::string& name) {
    // Use the mnemonic to guess operand size (for immediate).
    // Default 1 byte.
    std::string token = name;
    const auto space = name.find(' ');
    if (space != std::string::npos) token = name.substr(0, space);
    for (auto& c : token) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));

    if (token.find("ldq") == 0 || token.find("stq") == 0 || token.find("divq") == 0 || token.find("addr") == 0 || token.find("subr") == 0) {
        // Default to 2 for inter-register (size determined by dest); 4 for Q immediates.
        if (token.find("ldq") == 0 || token.find("stq") == 0 || token.find("divq") == 0) return 4;
        return 2;
    }

    if (token.find("ldw") == 0 || token.find("stw") == 0 || token.find("addw") == 0 || token.find("subw") == 0 ||
        token.find("cmpw") == 0 || token.find("sexw") == 0) {
        return 2;
    }

    if (token.find("ldx") == 0 || token.find("ldy") == 0 || token.find("ldu") == 0 || token.find("lds") == 0 ||
        token.find("stx") == 0 || token.find("sty") == 0 || token.find("stu") == 0 || token.find("sts") == 0 ||
        token.find("cmpx") == 0 || token.find("cmpy") == 0 || token.find("cmpu") == 0 || token.find("cmps") == 0 ||
        token.find("leax") == 0 || token.find("leay") == 0 || token.find("leau") == 0 || token.find("leas") == 0) {
        return 2;
    }

    if (token.find("ldd") == 0 || token.find("std") == 0 || token.find("addd") == 0 || token.find("subd") == 0 ||
        token.find("cmpd") == 0 || token.find("adcd") == 0 || token.find("sbcd") == 0 || token.find("muld") == 0 || token.find("divd") == 0) {
        return 2;
    }

    if (token.find("ld") == 0 && token.size() == 3 && token[2] == 'q') return 4;
    return 1;
}

} // namespace

Disasm disassemble(Bus& bus, Cpu& cpu, uint16_t pc) {
    const uint8_t op0 = bus.read8(pc);
    auto fallback = [&](const std::string& name, uint8_t len) {
        std::ostringstream oss;
        oss << name;
        uint8_t size_override = 0;
        const OperandKind kind = operand_kind(0x00, op0, name, size_override);
        const uint8_t op_size = size_override ? size_override : operand_size_from_mnemonic(name);
        switch (kind) {
        case OperandKind::Immediate:
            if (op_size == 1) {
                uint8_t imm = bus.read8(static_cast<uint16_t>(pc + 1));
                oss << " #$" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(imm);
            } else if (op_size == 2) {
                uint16_t imm = static_cast<uint16_t>((bus.read8(static_cast<uint16_t>(pc + 1)) << 8) |
                                                     bus.read8(static_cast<uint16_t>(pc + 2)));
                oss << " #$" << hex4(imm);
            } else if (op_size == 4) {
                uint32_t imm = (static_cast<uint32_t>(bus.read8(static_cast<uint16_t>(pc + 1))) << 24) |
                               (static_cast<uint32_t>(bus.read8(static_cast<uint16_t>(pc + 2))) << 16) |
                               (static_cast<uint32_t>(bus.read8(static_cast<uint16_t>(pc + 3))) << 8) |
                               static_cast<uint32_t>(bus.read8(static_cast<uint16_t>(pc + 4)));
                std::ostringstream tmp;
                tmp << std::hex << std::setw(8) << std::setfill('0') << imm;
                oss << " #$" << tmp.str();
            }
            break;
        case OperandKind::Direct: {
            uint8_t addr = bus.read8(static_cast<uint16_t>(pc + 1));
            oss << " <$" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(addr);
            break;
        }
        case OperandKind::Extended: {
            uint16_t addr = static_cast<uint16_t>((bus.read8(static_cast<uint16_t>(pc + 1)) << 8) |
                                                  bus.read8(static_cast<uint16_t>(pc + 2)));
            oss << " $" << hex4(addr);
            break;
        }
        case OperandKind::Indexed: {
            uint8_t pb = bus.read8(static_cast<uint16_t>(pc + 1));
            oss << " [pb $" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(pb) << "]";
            break;
        }
        case OperandKind::None:
            break;
        }
        return Disasm{oss.str(), len};
    };
    if (op0 == 0x10) {
        const uint8_t op1 = bus.read8(static_cast<uint16_t>(pc + 1));
        const std::string name = cpu.opcode_name(0x10, op1);
        // Long conditional branches 0x21-0x2F with 16-bit offset.
        if (op1 >= 0x21 && op1 <= 0x2F) {
            static const char* names[] = {
                "lbrn","lbhi","lbls","lbcc","lbcs","lbne","lbeq","lbvc",
                "lbvs","lbpl","lbmi","lbge","lblt","lbgt","lble"
            };
            const size_t idx = op1 - 0x21;
            const uint16_t off = static_cast<uint16_t>((bus.read8(pc + 2) << 8) | bus.read8(pc + 3));
            const int16_t soff = static_cast<int16_t>(off);
            const uint16_t target = static_cast<uint16_t>(pc + 4 + soff);
            return {std::string(names[idx]) + " $" + hex4(target), 4};
        }
        // Fall back to raw opcode display.
        // Generic: prefix + opcode; determine length from operand kind.
        uint8_t size_override = 0;
        const OperandKind kind = operand_kind(0x10, op1, name, size_override);
        const uint8_t op_size = size_override ? size_override : operand_size_from_mnemonic(name);
        uint8_t len = static_cast<uint8_t>(2); // prefix + opcode
        if (kind == OperandKind::Immediate) len = static_cast<uint8_t>(len + op_size);
        else if (kind == OperandKind::Direct || kind == OperandKind::Indexed) len = static_cast<uint8_t>(len + 1);
        else if (kind == OperandKind::Extended) len = static_cast<uint8_t>(len + 2);
        return fallback(name.empty() ? "op10" : name, len);
    }

    if (op0 == 0x11) {
        const uint8_t op1 = bus.read8(static_cast<uint16_t>(pc + 1));
        const std::string name = cpu.opcode_name(0x11, op1);
        uint8_t size_override = 0;
        const OperandKind kind = operand_kind(0x11, op1, name, size_override);
        const uint8_t op_size = size_override ? size_override : operand_size_from_mnemonic(name);
        uint8_t len = static_cast<uint8_t>(2); // prefix + opcode
        if (kind == OperandKind::Immediate) len = static_cast<uint8_t>(len + op_size);
        else if (kind == OperandKind::Direct || kind == OperandKind::Indexed) len = static_cast<uint8_t>(len + 1);
        else if (kind == OperandKind::Extended) len = static_cast<uint8_t>(len + 2);
        return fallback(name.empty() ? "op11" : name, len);
    }

    // LBRA / LBSR (16-bit relative)
    if (op0 == 0x16 || op0 == 0x17) {
        const uint16_t off = static_cast<uint16_t>((bus.read8(pc + 1) << 8) | bus.read8(pc + 2));
        const int16_t soff = static_cast<int16_t>(off);
        const uint16_t target = static_cast<uint16_t>(pc + 3 + soff);
        const char* name = (op0 == 0x16) ? "lbra" : "lbsr";
        return {std::string(name) + " $" + hex4(target), 3};
    }

    // BSR (8-bit), short branches 0x20-0x2F.
    if (op0 == 0x8D) {
        const int8_t off = static_cast<int8_t>(bus.read8(static_cast<uint16_t>(pc + 1)));
        const uint16_t target = static_cast<uint16_t>(pc + 2 + off);
        return {"bsr $" + hex4(target), 2};
    }

    if (op0 >= 0x20 && op0 <= 0x2F) {
        static const char* names[] = {
            "bra","brn","bhi","bls","bcc","bcs","bne","beq",
            "bvc","bvs","bpl","bmi","bge","blt","bgt","ble"
        };
        const size_t idx = op0 - 0x20;
        const int8_t off = static_cast<int8_t>(bus.read8(static_cast<uint16_t>(pc + 1)));
        const uint16_t target = static_cast<uint16_t>(pc + 2 + off);
        return {std::string(names[idx]) + " $" + hex4(target), 2};
    }

    // Generic formatting using opcode name and operand kind.
    std::string name = cpu.opcode_name(0x00, op0);
    if (name.empty()) name = "op" + hex4(op0).substr(2);
    uint8_t size_override = 0;
    const OperandKind kind = operand_kind(0x00, op0, name, size_override);
    const uint8_t op_size = size_override ? size_override : operand_size_from_mnemonic(name);
    uint8_t len = 1;
    if (kind == OperandKind::Immediate) len = static_cast<uint8_t>(len + op_size);
    else if (kind == OperandKind::Direct || kind == OperandKind::Indexed) len = static_cast<uint8_t>(len + 1);
    else if (kind == OperandKind::Extended) len = static_cast<uint8_t>(len + 2);
    return fallback(name, len);
}

} // namespace microlind::cli
