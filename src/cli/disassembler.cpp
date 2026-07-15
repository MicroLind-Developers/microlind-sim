#include "microlind/app/disassembler.hpp"

#include "microlind/app/util.hpp"

#include <cctype>
#include <cstddef>
#include <iomanip>
#include <sstream>
#include <utility>

#include "microlind/bus.hpp"
#include "microlind/cpu.hpp"

namespace microlind::cli {
namespace {

enum class OperandKind { None, Immediate, Direct, Extended, Indexed };

struct IndexedOperand {
    std::string text;
    uint8_t size{1};
};

const char* index_register(uint8_t index) {
    switch (index & 0x03) {
    case 0: return "x";
    case 1: return "y";
    case 2: return "u";
    case 3: return "s";
    }
    return "x";
}

std::string display_name_for_kind(const std::string& name, OperandKind kind) {
    if (kind == OperandKind::Indexed && name.size() >= 4 && name.rfind(" idx") == name.size() - 4) {
        return name.substr(0, name.size() - 4);
    }
    return name;
}

std::string mnemonic_root(const std::string& name) {
    const auto space = name.find(' ');
    return space == std::string::npos ? name : name.substr(0, space);
}

const char* register_name(uint8_t code) {
    switch (code & 0x0F) {
    case 0x00: return "d";
    case 0x01: return "x";
    case 0x02: return "y";
    case 0x03: return "u";
    case 0x04: return "s";
    case 0x05: return "pc";
    case 0x06: return "w";
    case 0x07: return "v";
    case 0x08: return "a";
    case 0x09: return "b";
    case 0x0A: return "cc";
    case 0x0B: return "dp";
    case 0x0C:
    case 0x0D: return "0";
    case 0x0E: return "e";
    case 0x0F: return "f";
    }
    return "?";
}

std::string signed_hex8(int8_t value) {
    std::ostringstream out;
    if (value < 0) {
        out << "-$" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(static_cast<uint8_t>(-value));
    } else {
        out << "$" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(value);
    }
    return out.str();
}

IndexedOperand format_indexed_operand(Bus& bus, uint16_t operand_pc) {
    const uint8_t post = bus.peek8(operand_pc);
    switch (post) {
    case 0x8F:
        return {",w", 1};
    case 0xAF: {
        const uint16_t offset = static_cast<uint16_t>((bus.peek8(static_cast<uint16_t>(operand_pc + 1)) << 8) |
                                                      bus.peek8(static_cast<uint16_t>(operand_pc + 2)));
        return {"$" + hex4(offset) + ",w", 3};
    }
    case 0xCF:
        return {",w++", 1};
    case 0x90:
        return {"[,w]", 1};
    case 0xB0: {
        const uint16_t offset = static_cast<uint16_t>((bus.peek8(static_cast<uint16_t>(operand_pc + 1)) << 8) |
                                                      bus.peek8(static_cast<uint16_t>(operand_pc + 2)));
        return {"[$" + hex4(offset) + ",w]", 3};
    }
    case 0xD0:
        return {"[,w++]", 1};
    default:
        break;
    }

    if ((post & 0x9F) == 0x8E) {
        return {std::string("w,") + index_register(static_cast<uint8_t>((post >> 5) & 0x03)), 1};
    }
    if ((post & 0x9F) == 0x9E) {
        return {std::string("[w,") + index_register(static_cast<uint8_t>((post >> 5) & 0x03)) + "]", 1};
    }

    if ((post & 0x80) == 0) {
        const char* reg = index_register(static_cast<uint8_t>((post >> 5) & 0x03));
        const int8_t offset = static_cast<int8_t>((post << 3)) >> 3;
        if (offset == 0) return {std::string(",") + reg, 1};
        return {signed_hex8(offset) + "," + reg, 1};
    }

    const char* base = index_register(static_cast<uint8_t>((post >> 5) & 0x03));
    const bool indirect = (post & 0x10) != 0;
    std::string text;
    uint8_t size = 1;

    switch (post & 0x0F) {
    case 0x00: text = std::string(",") + base + "+"; break;
    case 0x01: text = std::string(",") + base + "++"; break;
    case 0x02: text = std::string(",-") + base; break;
    case 0x03: text = std::string(",--") + base; break;
    case 0x04: text = std::string(",") + base; break;
    case 0x05: text = std::string("b,") + base; break;
    case 0x06: text = std::string("a,") + base; break;
    case 0x08: {
        const int8_t offset = static_cast<int8_t>(bus.peek8(static_cast<uint16_t>(operand_pc + 1)));
        text = signed_hex8(offset) + "," + base;
        size = 2;
        break;
    }
    case 0x09: {
        const uint16_t offset = static_cast<uint16_t>((bus.peek8(static_cast<uint16_t>(operand_pc + 1)) << 8) |
                                                      bus.peek8(static_cast<uint16_t>(operand_pc + 2)));
        text = "$" + hex4(offset) + "," + base;
        size = 3;
        break;
    }
    case 0x0B: text = std::string("d,") + base; break;
    case 0x0C: {
        const int8_t offset = static_cast<int8_t>(bus.peek8(static_cast<uint16_t>(operand_pc + 1)));
        text = signed_hex8(offset) + ",pc";
        size = 2;
        break;
    }
    case 0x0D: {
        const uint16_t offset = static_cast<uint16_t>((bus.peek8(static_cast<uint16_t>(operand_pc + 1)) << 8) |
                                                      bus.peek8(static_cast<uint16_t>(operand_pc + 2)));
        text = "$" + hex4(offset) + ",pc";
        size = 3;
        break;
    }
    case 0x0F: {
        const uint16_t address = static_cast<uint16_t>((bus.peek8(static_cast<uint16_t>(operand_pc + 1)) << 8) |
                                                       bus.peek8(static_cast<uint16_t>(operand_pc + 2)));
        text = "$" + hex4(address);
        size = 3;
        break;
    }
    default:
        text = "[pb $" + hex4(post).substr(2) + "]";
        break;
    }

    if (indirect) text = "[" + text + "]";
    return {text, size};
}

bool indexed_has_immediate(uint8_t prefix, uint8_t opcode) {
    if (prefix != 0x00) return false;
    return opcode == 0x61 || opcode == 0x62 || opcode == 0x65 || opcode == 0x6B;
}

bool is_bit_immediate_direct(uint8_t prefix, uint8_t opcode) {
    if (prefix != 0x00) return false;
    return opcode == 0x01 || opcode == 0x02 || opcode == 0x05 || opcode == 0x0B;
}

bool is_bit_immediate_extended(uint8_t prefix, uint8_t opcode) {
    if (prefix != 0x00) return false;
    return opcode == 0x71 || opcode == 0x72 || opcode == 0x75 || opcode == 0x7B;
}

bool is_register_pair_instruction(uint16_t key) {
    switch (key) {
    case 0x001E: // EXG
    case 0x001F: // TFR
    case 0x1030: // ADDR
    case 0x1031: // ADCR
    case 0x1032: // SUBR
    case 0x1033: // SBCR
    case 0x1035: // ORR
    case 0x1036: // EORR
    case 0x1037: // CMPR
        return true;
    default:
        return false;
    }
}

bool is_tfm_instruction(uint16_t key) {
    return key >= 0x1138 && key <= 0x113B;
}

uint8_t indexed_operand_size(Bus& bus, uint8_t prefix, uint8_t opcode, uint16_t operand_pc) {
    if (!indexed_has_immediate(prefix, opcode)) return format_indexed_operand(bus, operand_pc).size;
    return static_cast<uint8_t>(1 + format_indexed_operand(bus, static_cast<uint16_t>(operand_pc + 1)).size);
}

std::string stack_mask_registers(uint8_t opcode, uint8_t mask) {
    const bool uses_u_stack = opcode == 0x36 || opcode == 0x37;
    const char* other_stack = uses_u_stack ? "s" : "u";
    const std::pair<uint8_t, const char*> regs[] = {
        {0x01, "cc"},
        {0x02, "a"},
        {0x04, "b"},
        {0x08, "dp"},
        {0x10, "x"},
        {0x20, "y"},
        {0x40, other_stack},
        {0x80, "pc"},
    };

    std::ostringstream out;
    bool first = true;
    for (const auto& [bit, name] : regs) {
        if ((mask & bit) == 0) continue;
        if (!first) out << ",";
        out << name;
        first = false;
    }
    if (first) return "-";
    return out.str();
}

std::string format_register_pair(const std::string& name, uint8_t post) {
    std::ostringstream out;
    out << mnemonic_root(name) << " " << register_name(static_cast<uint8_t>(post >> 4)) << ","
        << register_name(static_cast<uint8_t>(post & 0x0F));
    return out.str();
}

std::string format_tfm(const std::string& name, uint8_t opcode, uint8_t post) {
    const char* src_suffix = "";
    const char* dst_suffix = "";
    switch (opcode) {
    case 0x38:
        src_suffix = "+";
        dst_suffix = "+";
        break;
    case 0x39:
        src_suffix = "-";
        dst_suffix = "-";
        break;
    case 0x3A:
        src_suffix = "+";
        break;
    case 0x3B:
        dst_suffix = "+";
        break;
    default:
        break;
    }

    std::ostringstream out;
    out << mnemonic_root(name) << " " << register_name(static_cast<uint8_t>(post >> 4)) << src_suffix << ","
        << register_name(static_cast<uint8_t>(post & 0x0F)) << dst_suffix;
    return out.str();
}

std::string join_immediate_indexed(uint8_t immediate, const IndexedOperand& indexed) {
    std::ostringstream out;
    out << "#$" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(immediate);
    if (!indexed.text.empty() && indexed.text.front() == ',') {
        out << indexed.text;
    } else {
        out << "," << indexed.text;
    }
    return out.str();
}

OperandKind operand_kind(uint8_t prefix, uint8_t opcode, const std::string& name, uint8_t& size_override) {
    // Explicit overrides for instructions without addressing encoded in their name.
    const uint16_t key = static_cast<uint16_t>((prefix << 8) | opcode);
    switch (key) {
    case 0x001A: // ORCC #imm
    case 0x001C: // ANDCC #imm
    case 0x001E: // EXG postbyte
    case 0x001F: // TFR postbyte
    case 0x0034: // PSHS mask
    case 0x0035: // PULS mask
    case 0x0036: // PSHU mask
    case 0x0037: // PULU mask
    case 0x003C: // CWAI #imm
    case 0x1030: // ADDR postbyte
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
    const uint8_t op0 = bus.peek8(pc);
    auto fallback = [&](uint8_t prefix, uint8_t opcode, uint16_t operand_pc, const std::string& name, uint8_t len) {
        std::ostringstream oss;
        const uint16_t key = static_cast<uint16_t>((prefix << 8) | opcode);
        if (key == 0x0034 || key == 0x0035 || key == 0x0036 || key == 0x0037) {
            const uint8_t mask = bus.peek8(operand_pc);
            return Disasm{name + " " + stack_mask_registers(opcode, mask), len};
        }
        if (is_register_pair_instruction(key)) {
            return Disasm{format_register_pair(name, bus.peek8(operand_pc)), len};
        }
        if (is_tfm_instruction(key)) {
            return Disasm{format_tfm(name, opcode, bus.peek8(operand_pc)), len};
        }
        if (is_bit_immediate_direct(prefix, opcode)) {
            const uint8_t imm = bus.peek8(operand_pc);
            const uint8_t address = bus.peek8(static_cast<uint16_t>(operand_pc + 1));
            std::ostringstream bit;
            bit << mnemonic_root(name) << " #$" << std::hex << std::setw(2) << std::setfill('0')
                << static_cast<int>(imm) << ",<$" << std::setw(2) << static_cast<int>(address);
            return Disasm{bit.str(), len};
        }
        if (is_bit_immediate_extended(prefix, opcode)) {
            const uint8_t imm = bus.peek8(operand_pc);
            const uint16_t address = static_cast<uint16_t>((bus.peek8(static_cast<uint16_t>(operand_pc + 1)) << 8) |
                                                           bus.peek8(static_cast<uint16_t>(operand_pc + 2)));
            std::ostringstream bit;
            bit << mnemonic_root(name) << " #$" << std::hex << std::setw(2) << std::setfill('0')
                << static_cast<int>(imm) << ",$" << hex4(address);
            return Disasm{bit.str(), len};
        }

        uint8_t size_override = 0;
        const OperandKind kind = operand_kind(prefix, opcode, name, size_override);
        oss << display_name_for_kind(name, kind);
        const uint8_t op_size = size_override ? size_override : operand_size_from_mnemonic(name);
        switch (kind) {
        case OperandKind::Immediate:
            if (op_size == 1) {
                uint8_t imm = bus.peek8(operand_pc);
                oss << " #$" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(imm);
            } else if (op_size == 2) {
                uint16_t imm = static_cast<uint16_t>((bus.peek8(operand_pc) << 8) |
                                                     bus.peek8(static_cast<uint16_t>(operand_pc + 1)));
                oss << " #$" << hex4(imm);
            } else if (op_size == 4) {
                uint32_t imm = (static_cast<uint32_t>(bus.peek8(operand_pc)) << 24) |
                               (static_cast<uint32_t>(bus.peek8(static_cast<uint16_t>(operand_pc + 1))) << 16) |
                               (static_cast<uint32_t>(bus.peek8(static_cast<uint16_t>(operand_pc + 2))) << 8) |
                               static_cast<uint32_t>(bus.peek8(static_cast<uint16_t>(operand_pc + 3)));
                std::ostringstream tmp;
                tmp << std::hex << std::setw(8) << std::setfill('0') << imm;
                oss << " #$" << tmp.str();
            }
            break;
        case OperandKind::Direct: {
            uint8_t addr = bus.peek8(operand_pc);
            oss << " <$" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(addr);
            break;
        }
        case OperandKind::Extended: {
            uint16_t addr = static_cast<uint16_t>((bus.peek8(operand_pc) << 8) |
                                                  bus.peek8(static_cast<uint16_t>(operand_pc + 1)));
            oss << " $" << hex4(addr);
            break;
        }
        case OperandKind::Indexed: {
            if (indexed_has_immediate(prefix, opcode)) {
                const uint8_t imm = bus.peek8(operand_pc);
                const auto indexed = format_indexed_operand(bus, static_cast<uint16_t>(operand_pc + 1));
                oss << " " << join_immediate_indexed(imm, indexed);
            } else {
                const auto indexed = format_indexed_operand(bus, operand_pc);
                oss << " " << indexed.text;
            }
            break;
        }
        case OperandKind::None:
            break;
        }
        return Disasm{oss.str(), len};
    };
    if (op0 == 0x10) {
        const uint8_t op1 = bus.peek8(static_cast<uint16_t>(pc + 1));
        const std::string name = cpu.opcode_name(0x10, op1);
        // Long conditional branches 0x21-0x2F with 16-bit offset.
        if (op1 >= 0x21 && op1 <= 0x2F) {
            static const char* names[] = {
                "lbrn","lbhi","lbls","lbcc","lbcs","lbne","lbeq","lbvc",
                "lbvs","lbpl","lbmi","lbge","lblt","lbgt","lble"
            };
            const size_t idx = op1 - 0x21;
            const uint16_t off = static_cast<uint16_t>((bus.peek8(pc + 2) << 8) | bus.peek8(pc + 3));
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
        else if (kind == OperandKind::Direct) len = static_cast<uint8_t>(len + 1);
        else if (kind == OperandKind::Indexed) len = static_cast<uint8_t>(len + indexed_operand_size(bus, 0x10, op1, static_cast<uint16_t>(pc + 2)));
        else if (kind == OperandKind::Extended) len = static_cast<uint8_t>(len + 2);
        return fallback(0x10, op1, static_cast<uint16_t>(pc + 2), name.empty() ? "op10" : name, len);
    }

    if (op0 == 0x11) {
        const uint8_t op1 = bus.peek8(static_cast<uint16_t>(pc + 1));
        const std::string name = cpu.opcode_name(0x11, op1);
        uint8_t size_override = 0;
        const OperandKind kind = operand_kind(0x11, op1, name, size_override);
        const uint8_t op_size = size_override ? size_override : operand_size_from_mnemonic(name);
        uint8_t len = static_cast<uint8_t>(2); // prefix + opcode
        if (kind == OperandKind::Immediate) len = static_cast<uint8_t>(len + op_size);
        else if (kind == OperandKind::Direct) len = static_cast<uint8_t>(len + 1);
        else if (kind == OperandKind::Indexed) len = static_cast<uint8_t>(len + indexed_operand_size(bus, 0x11, op1, static_cast<uint16_t>(pc + 2)));
        else if (kind == OperandKind::Extended) len = static_cast<uint8_t>(len + 2);
        return fallback(0x11, op1, static_cast<uint16_t>(pc + 2), name.empty() ? "op11" : name, len);
    }

    // LBRA / LBSR (16-bit relative)
    if (op0 == 0x16 || op0 == 0x17) {
        const uint16_t off = static_cast<uint16_t>((bus.peek8(pc + 1) << 8) | bus.peek8(pc + 2));
        const int16_t soff = static_cast<int16_t>(off);
        const uint16_t target = static_cast<uint16_t>(pc + 3 + soff);
        const char* name = (op0 == 0x16) ? "lbra" : "lbsr";
        return {std::string(name) + " $" + hex4(target), 3};
    }

    // BSR (8-bit), short branches 0x20-0x2F.
    if (op0 == 0x8D) {
        const int8_t off = static_cast<int8_t>(bus.peek8(static_cast<uint16_t>(pc + 1)));
        const uint16_t target = static_cast<uint16_t>(pc + 2 + off);
        return {"bsr $" + hex4(target), 2};
    }

    if (op0 >= 0x20 && op0 <= 0x2F) {
        static const char* names[] = {
            "bra","brn","bhi","bls","bcc","bcs","bne","beq",
            "bvc","bvs","bpl","bmi","bge","blt","bgt","ble"
        };
        const size_t idx = op0 - 0x20;
        const int8_t off = static_cast<int8_t>(bus.peek8(static_cast<uint16_t>(pc + 1)));
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
    if (is_bit_immediate_direct(0x00, op0)) len = 3;
    else if (is_bit_immediate_extended(0x00, op0)) len = 4;
    else if (kind == OperandKind::Immediate) len = static_cast<uint8_t>(len + op_size);
    else if (kind == OperandKind::Direct) len = static_cast<uint8_t>(len + 1);
    else if (kind == OperandKind::Indexed) len = static_cast<uint8_t>(len + indexed_operand_size(bus, 0x00, op0, static_cast<uint16_t>(pc + 1)));
    else if (kind == OperandKind::Extended) len = static_cast<uint8_t>(len + 2);
    return fallback(0x00, op0, static_cast<uint16_t>(pc + 1), name, len);
}

} // namespace microlind::cli
