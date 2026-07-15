#include "microlind/cpu.hpp"

#include "microlind/bus.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <initializer_list>
#include <optional>
#include <string>
#include <utility>

namespace microlind {

namespace {
uint8_t hi(uint16_t value) { return static_cast<uint8_t>((value >> 8) & 0xFF); }
uint8_t lo(uint16_t value) { return static_cast<uint8_t>(value & 0xFF); }

int32_t sign_extend16(uint16_t value) {
    return (value & 0x8000) ? static_cast<int32_t>(value) - 0x10000 : static_cast<int32_t>(value);
}

bool is_native_hd6309(const Registers& regs, CpuMode mode) {
    return mode == CpuMode::HD6309 && (regs.md & 0x01) != 0;
}

void set_logic8_flags(Registers& regs, uint8_t value, bool clear_carry) {
    regs.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | (clear_carry ? CC_C : 0)));
    if (value == 0) {
        regs.cc |= CC_Z;
    }
    if (value & 0x80) {
        regs.cc |= CC_N;
    }
}

uint8_t add8(Registers& regs, uint8_t left, uint8_t right, uint8_t carry) {
    const uint16_t sum = static_cast<uint16_t>(left) + static_cast<uint16_t>(right) + carry;
    const uint8_t result = static_cast<uint8_t>(sum & 0xFF);
    regs.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | CC_C | CC_H));
    if (result & 0x80) {
        regs.cc |= CC_N;
    }
    if (result == 0) {
        regs.cc |= CC_Z;
    }
    if (((left ^ right ^ result) & 0x80) && !((left ^ right) & 0x80)) {
        regs.cc |= CC_V;
    }
    if (sum & 0x100) {
        regs.cc |= CC_C;
    }
    if ((left ^ right ^ result) & 0x10) {
        regs.cc |= CC_H;
    }
    return result;
}

uint8_t sub8(Registers& regs, uint8_t left, uint8_t right, uint8_t carry) {
    const uint16_t diff = static_cast<uint16_t>(left) - static_cast<uint16_t>(right) - carry;
    const uint8_t result = static_cast<uint8_t>(diff & 0xFF);
    regs.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | CC_C));
    if (result & 0x80) {
        regs.cc |= CC_N;
    }
    if (result == 0) {
        regs.cc |= CC_Z;
    }
    if (((left ^ right) & (left ^ result) & 0x80) != 0) {
        regs.cc |= CC_V;
    }
    if (diff & 0x100) {
        regs.cc |= CC_C;
    }
    return result;
}

uint16_t add16(Registers& regs, uint16_t left, uint16_t right) {
    const uint32_t sum = static_cast<uint32_t>(left) + static_cast<uint32_t>(right);
    const uint16_t result = static_cast<uint16_t>(sum & 0xFFFF);
    regs.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | CC_C));
    if (result & 0x8000) {
        regs.cc |= CC_N;
    }
    if (result == 0) {
        regs.cc |= CC_Z;
    }
    if (((left ^ right ^ result) & 0x8000) && !((left ^ right) & 0x8000)) {
        regs.cc |= CC_V;
    }
    if (sum & 0x10000) {
        regs.cc |= CC_C;
    }
    return result;
}

uint16_t sub16(Registers& regs, uint16_t left, uint16_t right) {
    const uint32_t diff = static_cast<uint32_t>(left) - static_cast<uint32_t>(right);
    const uint16_t result = static_cast<uint16_t>(diff & 0xFFFF);
    regs.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | CC_C));
    if (result & 0x8000) {
        regs.cc |= CC_N;
    }
    if (result == 0) {
        regs.cc |= CC_Z;
    }
    if (((left ^ right) & (left ^ result) & 0x8000) != 0) {
        regs.cc |= CC_V;
    }
    if (diff & 0x10000) {
        regs.cc |= CC_C;
    }
    return result;
}

bool is_alu8_immediate_opcode(uint8_t opcode) {
    switch (opcode) {
    case 0x80:
    case 0x81:
    case 0x82:
    case 0x84:
    case 0x85:
    case 0x88:
    case 0x89:
    case 0x8A:
    case 0x8B:
    case 0xC0:
    case 0xC1:
    case 0xC2:
    case 0xC4:
    case 0xC5:
    case 0xC8:
    case 0xC9:
    case 0xCA:
    case 0xCB:
        return true;
    default:
        return false;
    }
}

bool is_alu8_direct_opcode(uint8_t opcode) {
    switch (opcode) {
    case 0x90:
    case 0x91:
    case 0x92:
    case 0x94:
    case 0x95:
    case 0x98:
    case 0x99:
    case 0x9A:
    case 0x9B:
    case 0xD0:
    case 0xD1:
    case 0xD2:
    case 0xD4:
    case 0xD5:
    case 0xD8:
    case 0xD9:
    case 0xDA:
    case 0xDB:
        return true;
    default:
        return false;
    }
}

bool is_alu8_extended_opcode(uint8_t opcode) {
    switch (opcode) {
    case 0xB0:
    case 0xB1:
    case 0xB2:
    case 0xB4:
    case 0xB5:
    case 0xB8:
    case 0xB9:
    case 0xBA:
    case 0xBB:
    case 0xF0:
    case 0xF1:
    case 0xF2:
    case 0xF4:
    case 0xF5:
    case 0xF8:
    case 0xF9:
    case 0xFA:
    case 0xFB:
        return true;
    default:
        return false;
    }
}

bool is_alu16_immediate_opcode(uint8_t opcode) {
    return opcode == 0x83 || opcode == 0xC3;
}

bool is_alu16_direct_opcode(uint8_t opcode) {
    return opcode == 0x93 || opcode == 0xD3;
}

bool is_alu16_extended_opcode(uint8_t opcode) {
    return opcode == 0xB3 || opcode == 0xF3;
}

bool register_code_is_16bit(uint8_t code) {
    return (code & 0x0F) <= 0x07;
}

bool is_memory_unary_direct_opcode(uint8_t opcode) {
    switch (opcode) {
    case 0x00:
    case 0x03:
    case 0x04:
    case 0x06:
    case 0x07:
    case 0x08:
    case 0x09:
    case 0x0A:
    case 0x0C:
    case 0x0D:
    case 0x0F:
        return true;
    default:
        return false;
    }
}

bool is_memory_unary_extended_opcode(uint8_t opcode) {
    switch (opcode) {
    case 0x70:
    case 0x73:
    case 0x74:
    case 0x76:
    case 0x77:
    case 0x78:
    case 0x79:
    case 0x7A:
    case 0x7C:
    case 0x7D:
    case 0x7F:
        return true;
    default:
        return false;
    }
}

bool is_cmp16_immediate_opcode(uint8_t prefix, uint8_t opcode) {
    return (prefix == 0x00 && opcode == 0x8C) ||
        (prefix == 0x10 && opcode == 0x8C) ||
        (prefix == 0x11 && (opcode == 0x83 || opcode == 0x8C));
}

bool is_cmp16_direct_opcode(uint8_t prefix, uint8_t opcode) {
    return (prefix == 0x00 && opcode == 0x9C) ||
        (prefix == 0x10 && opcode == 0x9C) ||
        (prefix == 0x11 && (opcode == 0x93 || opcode == 0x9C));
}

bool is_cmp16_extended_opcode(uint8_t prefix, uint8_t opcode) {
    return (prefix == 0x00 && opcode == 0xBC) ||
        (prefix == 0x10 && opcode == 0xBC) ||
        (prefix == 0x11 && (opcode == 0xB3 || opcode == 0xBC));
}

bool is_word_load_immediate_opcode(uint8_t prefix, uint8_t opcode) {
    return (prefix == 0x00 && (opcode == 0x8E || opcode == 0xCE)) ||
        (prefix == 0x10 && (opcode == 0x8E || opcode == 0xCE));
}

bool is_word_load_direct_opcode(uint8_t prefix, uint8_t opcode) {
    return (prefix == 0x00 && (opcode == 0x9E || opcode == 0xDE)) ||
        (prefix == 0x10 && (opcode == 0x9E || opcode == 0xDE));
}

bool is_word_load_extended_opcode(uint8_t prefix, uint8_t opcode) {
    return (prefix == 0x00 && (opcode == 0xBE || opcode == 0xFE)) ||
        (prefix == 0x10 && (opcode == 0xBE || opcode == 0xFE));
}

bool is_word_store_direct_opcode(uint8_t prefix, uint8_t opcode) {
    return (prefix == 0x00 && (opcode == 0x9F || opcode == 0xDF)) ||
        (prefix == 0x10 && (opcode == 0x9F || opcode == 0xDF));
}

bool is_word_store_extended_opcode(uint8_t prefix, uint8_t opcode) {
    return (prefix == 0x00 && (opcode == 0xBF || opcode == 0xFF)) ||
        (prefix == 0x10 && (opcode == 0xBF || opcode == 0xFF));
}

uint8_t cmp16_immediate_cycles(uint8_t prefix, const Registers& regs, CpuMode mode) {
    if (!is_native_hd6309(regs, mode)) {
        return prefix == 0x00 ? 4 : 5;
    }
    return prefix == 0x00 ? 3 : 4;
}

uint8_t cmp16_direct_cycles(uint8_t prefix, const Registers& regs, CpuMode mode) {
    if (!is_native_hd6309(regs, mode)) {
        return prefix == 0x00 ? 6 : 7;
    }
    return prefix == 0x00 ? 4 : 5;
}

uint8_t cmp16_extended_cycles(uint8_t prefix, const Registers& regs, CpuMode mode) {
    if (!is_native_hd6309(regs, mode)) {
        return prefix == 0x00 ? 7 : 8;
    }
    return prefix == 0x00 ? 5 : 6;
}

uint8_t word_load_immediate_cycles(uint8_t prefix, uint8_t opcode, const Registers& regs, CpuMode mode) {
    if (prefix == 0x00) {
        return 3;
    }
    if (opcode == 0xCE) {
        return 4;
    }
    return is_native_hd6309(regs, mode) ? 4 : 5;
}

uint8_t word_direct_cycles(uint8_t prefix, const Registers& regs, CpuMode mode) {
    if (!is_native_hd6309(regs, mode)) {
        return prefix == 0x00 ? 5 : 6;
    }
    return prefix == 0x00 ? 4 : 5;
}

uint8_t word_extended_cycles(uint8_t prefix, const Registers& regs, CpuMode mode) {
    if (!is_native_hd6309(regs, mode)) {
        return prefix == 0x00 ? 6 : 7;
    }
    return prefix == 0x00 ? 5 : 6;
}

bool branch_condition(uint8_t opcode, const Registers& regs) {
    switch (opcode) {
    case 0x20: return true; // BRA
    case 0x21: return false; // BRN
    case 0x22: return (regs.cc & (CC_Z | CC_C)) == 0; // BHI
    case 0x23: return (regs.cc & (CC_Z | CC_C)) != 0; // BLS
    case 0x24: return (regs.cc & CC_C) == 0; // BCC/BHS
    case 0x25: return (regs.cc & CC_C) != 0; // BCS/BLO
    case 0x26: return (regs.cc & CC_Z) == 0; // BNE
    case 0x27: return (regs.cc & CC_Z) != 0; // BEQ
    case 0x28: return (regs.cc & CC_V) == 0; // BVC
    case 0x29: return (regs.cc & CC_V) != 0; // BVS
    case 0x2A: return (regs.cc & CC_N) == 0; // BPL
    case 0x2B: return (regs.cc & CC_N) != 0; // BMI
    case 0x2C: return ((regs.cc & CC_N) >> 3) == ((regs.cc & CC_V) >> 1); // BGE
    case 0x2D: return ((regs.cc & CC_N) >> 3) != ((regs.cc & CC_V) >> 1); // BLT
    case 0x2E:
        return (regs.cc & CC_Z) == 0 && ((regs.cc & CC_N) >> 3) == ((regs.cc & CC_V) >> 1); // BGT
    case 0x2F:
        return (regs.cc & CC_Z) != 0 || ((regs.cc & CC_N) >> 3) != ((regs.cc & CC_V) >> 1); // BLE
    default:
        return false;
    }
}

void apply_alu8_immediate(Registers& regs, uint8_t opcode, uint8_t operand) {
    const bool target_b = (opcode & 0x40) != 0;
    uint8_t& target = target_b ? regs.b : regs.a;
    switch (opcode & 0x8F) {
    case 0x80:
        target = sub8(regs, target, operand, 0);
        break;
    case 0x81:
        sub8(regs, target, operand, 0);
        break;
    case 0x82:
        target = sub8(regs, target, operand, (regs.cc & CC_C) ? 1 : 0);
        break;
    case 0x84:
        target = static_cast<uint8_t>(target & operand);
        set_logic8_flags(regs, target, true);
        break;
    case 0x85:
        set_logic8_flags(regs, static_cast<uint8_t>(target & operand), false);
        break;
    case 0x88:
        target = static_cast<uint8_t>(target ^ operand);
        set_logic8_flags(regs, target, true);
        break;
    case 0x89:
        target = add8(regs, target, operand, (regs.cc & CC_C) ? 1 : 0);
        break;
    case 0x8A:
        target = static_cast<uint8_t>(target | operand);
        set_logic8_flags(regs, target, true);
        break;
    case 0x8B:
        target = add8(regs, target, operand, 0);
        break;
    default:
        break;
    }
}

void apply_alu16(Registers& regs, uint8_t opcode, uint16_t operand) {
    const uint16_t d = static_cast<uint16_t>((static_cast<uint16_t>(regs.a) << 8) | regs.b);
    const bool add = opcode == 0xC3 || opcode == 0xD3 || opcode == 0xF3;
    const uint16_t result = add ? add16(regs, d, operand) : sub16(regs, d, operand);
    regs.a = hi(result);
    regs.b = lo(result);
}

void apply_cmpd(Registers& regs, uint16_t operand) {
    const uint16_t d = static_cast<uint16_t>((static_cast<uint16_t>(regs.a) << 8) | regs.b);
    sub16(regs, d, operand);
}

void apply_cmp16(Registers& regs, uint8_t prefix, uint8_t opcode, uint16_t operand) {
    uint16_t value = 0;
    if (prefix == 0x00) {
        value = regs.x;
    } else if (prefix == 0x10) {
        value = regs.y;
    } else if (prefix == 0x11 && (opcode == 0x83 || opcode == 0x93 || opcode == 0xB3)) {
        value = regs.u;
    } else if (prefix == 0x11) {
        value = regs.s;
    }
    sub16(regs, value, operand);
}

void apply_tst8(Registers& regs, uint8_t value) {
    regs.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | CC_C));
    if (value == 0) {
        regs.cc |= CC_Z;
    }
    if (value & 0x80) {
        regs.cc |= CC_N;
    }
}

uint8_t apply_memory_unary(Registers& regs, uint8_t opcode, uint8_t value) {
    switch (opcode & 0x0F) {
    case 0x00: {
        const uint8_t result = static_cast<uint8_t>(0u - value);
        regs.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | CC_C));
        if (result & 0x80) regs.cc |= CC_N;
        if (result == 0) regs.cc |= CC_Z;
        if (value == 0x80) regs.cc |= CC_V;
        if (value != 0) regs.cc |= CC_C;
        return result;
    }
    case 0x03: {
        const uint8_t result = static_cast<uint8_t>(~value);
        regs.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V));
        regs.cc |= CC_C;
        if (result & 0x80) regs.cc |= CC_N;
        if (result == 0) regs.cc |= CC_Z;
        return result;
    }
    case 0x04: {
        const uint8_t result = static_cast<uint8_t>(value >> 1);
        regs.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | CC_C));
        regs.cc |= static_cast<uint8_t>(value & 0x01);
        if (result == 0) regs.cc |= CC_Z;
        return result;
    }
    case 0x06: {
        const uint8_t carry_in = (regs.cc & CC_C) ? 0x80 : 0x00;
        const uint8_t carry_out = static_cast<uint8_t>(value & 0x01);
        const uint8_t result = static_cast<uint8_t>((value >> 1) | carry_in);
        regs.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | CC_C));
        if (result & 0x80) regs.cc |= CC_N;
        if (result == 0) regs.cc |= CC_Z;
        if (carry_out != 0) regs.cc |= CC_C;
        if (((result ^ (carry_out ? 0x80 : 0x00)) & 0x80) != 0) regs.cc |= CC_V;
        return result;
    }
    case 0x07: {
        const uint8_t carry_out = static_cast<uint8_t>(value & 0x01);
        const uint8_t result = static_cast<uint8_t>((value >> 1) | (value & 0x80));
        regs.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | CC_C));
        if (result & 0x80) regs.cc |= CC_N;
        if (result == 0) regs.cc |= CC_Z;
        if (carry_out != 0) regs.cc |= CC_C;
        return result;
    }
    case 0x08: {
        const uint8_t carry_out = static_cast<uint8_t>((value >> 7) & 0x01);
        const uint8_t result = static_cast<uint8_t>(value << 1);
        regs.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | CC_C));
        if (result & 0x80) regs.cc |= CC_N;
        if (result == 0) regs.cc |= CC_Z;
        if (carry_out != 0) regs.cc |= CC_C;
        if (((result ^ (carry_out ? 0x80 : 0x00)) & 0x80) != 0) regs.cc |= CC_V;
        return result;
    }
    case 0x09: {
        const uint8_t carry_in = (regs.cc & CC_C) ? 1 : 0;
        const uint8_t carry_out = static_cast<uint8_t>((value >> 7) & 0x01);
        const uint8_t result = static_cast<uint8_t>((value << 1) | carry_in);
        regs.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | CC_C));
        if (result & 0x80) regs.cc |= CC_N;
        if (result == 0) regs.cc |= CC_Z;
        if (carry_out != 0) regs.cc |= CC_C;
        if (((result ^ (carry_out ? 0x80 : 0x00)) & 0x80) != 0) regs.cc |= CC_V;
        return result;
    }
    case 0x0A: {
        const uint8_t result = static_cast<uint8_t>(value - 1);
        regs.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V));
        if (result == 0) regs.cc |= CC_Z;
        if (result & 0x80) regs.cc |= CC_N;
        if (result == 0x7F) regs.cc |= CC_V;
        return result;
    }
    case 0x0C: {
        const uint8_t result = static_cast<uint8_t>(value + 1);
        regs.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V));
        if (result == 0) regs.cc |= CC_Z;
        if (result & 0x80) regs.cc |= CC_N;
        if (result == 0x80) regs.cc |= CC_V;
        return result;
    }
    default:
        return value;
    }
}

uint16_t word_register_value(const Registers& regs, uint8_t prefix, uint8_t opcode) {
    if (prefix == 0x00 && opcode == 0x8E) return regs.x;
    if (prefix == 0x00 && opcode == 0x9E) return regs.x;
    if (prefix == 0x00 && opcode == 0xBE) return regs.x;
    if (prefix == 0x00 && opcode == 0x9F) return regs.x;
    if (prefix == 0x00 && opcode == 0xBF) return regs.x;
    if (prefix == 0x00) return regs.u;
    if (opcode == 0x8E || opcode == 0x9E || opcode == 0xBE || opcode == 0x9F || opcode == 0xBF) return regs.y;
    return regs.s;
}

uint8_t stack_mask_byte_count(uint8_t mask) {
    uint8_t count = 0;
    if (mask & 0x01) ++count;
    if (mask & 0x02) ++count;
    if (mask & 0x04) ++count;
    if (mask & 0x08) ++count;
    if (mask & 0x10) count = static_cast<uint8_t>(count + 2);
    if (mask & 0x20) count = static_cast<uint8_t>(count + 2);
    if (mask & 0x40) count = static_cast<uint8_t>(count + 2);
    if (mask & 0x80) count = static_cast<uint8_t>(count + 2);
    return count;
}

uint16_t stack_partner_register_value(const Registers& regs, bool use_u_stack) {
    return use_u_stack ? regs.s : regs.u;
}

uint8_t stack_push_byte(const Registers& regs, uint8_t mask, uint8_t index, bool use_u_stack) {
    uint8_t current = 0;
    const auto emit_byte = [&](uint8_t value, uint8_t& out) {
        if (current == index) {
            out = value;
            return true;
        }
        ++current;
        return false;
    };
    const auto emit_word = [&](uint16_t value, uint8_t& out) {
        if (emit_byte(lo(value), out)) return true;
        return emit_byte(hi(value), out);
    };

    uint8_t result = 0x00;
    if ((mask & 0x80) && emit_word(regs.pc, result)) return result;
    if ((mask & 0x40) && emit_word(stack_partner_register_value(regs, use_u_stack), result)) return result;
    if ((mask & 0x20) && emit_word(regs.y, result)) return result;
    if ((mask & 0x10) && emit_word(regs.x, result)) return result;
    if ((mask & 0x08) && emit_byte(regs.dp, result)) return result;
    if ((mask & 0x04) && emit_byte(regs.b, result)) return result;
    if ((mask & 0x02) && emit_byte(regs.a, result)) return result;
    if ((mask & 0x01) && emit_byte(regs.cc, result)) return result;
    return result;
}

void set_stack_partner_register(Registers& regs, bool use_u_stack, uint16_t value) {
    if (use_u_stack) {
        regs.s = value;
    } else {
        regs.u = value;
    }
}

void apply_stack_pull_byte(Registers& regs, uint8_t mask, uint8_t index, bool use_u_stack, uint8_t value, uint16_t& word_data) {
    uint8_t current = 0;
    const auto consume_byte = [&](auto&& setter) {
        if (current == index) {
            setter(value);
            return true;
        }
        ++current;
        return false;
    };
    const auto consume_word = [&](auto&& setter) {
        if (current == index) {
            word_data = static_cast<uint16_t>(static_cast<uint16_t>(value) << 8);
            return true;
        }
        ++current;
        if (current == index) {
            word_data = static_cast<uint16_t>(word_data | value);
            setter(word_data);
            return true;
        }
        ++current;
        return false;
    };

    if ((mask & 0x01) && consume_byte([&](uint8_t v) { regs.cc = v; })) return;
    if ((mask & 0x02) && consume_byte([&](uint8_t v) { regs.a = v; })) return;
    if ((mask & 0x04) && consume_byte([&](uint8_t v) { regs.b = v; })) return;
    if ((mask & 0x08) && consume_byte([&](uint8_t v) { regs.dp = v; })) return;
    if ((mask & 0x10) && consume_word([&](uint16_t v) { regs.x = v; })) return;
    if ((mask & 0x20) && consume_word([&](uint16_t v) { regs.y = v; })) return;
    if ((mask & 0x40) && consume_word([&](uint16_t v) { set_stack_partner_register(regs, use_u_stack, v); })) return;
    if ((mask & 0x80) && consume_word([&](uint16_t v) { regs.pc = v; })) return;
}

void set_word_register(Registers& regs, uint8_t prefix, uint8_t opcode, uint16_t value) {
    if (prefix == 0x00 && (opcode == 0x8E || opcode == 0x9E || opcode == 0xBE)) {
        regs.x = value;
    } else if (prefix == 0x00) {
        regs.u = value;
    } else if (opcode == 0x8E || opcode == 0x9E || opcode == 0xBE) {
        regs.y = value;
    } else {
        regs.s = value;
    }
}

BusSignals read_cycle(uint16_t address, BusCycleKind kind) {
    BusSignals signals;
    signals.rw = true;
    signals.memory_enable = true;
    signals.address = address;
    signals.data = 0xFF;
    signals.cycle_kind = kind;
    signals.apply_read = true;
    signals.log_access = true;
    return signals;
}

BusSignals write_cycle(uint16_t address, uint8_t value, BusCycleKind kind) {
    BusSignals signals;
    signals.rw = false;
    signals.memory_enable = true;
    signals.address = address;
    signals.data = value;
    signals.cycle_kind = kind;
    signals.apply_write = true;
    signals.log_access = true;
    return signals;
}

BusSignals internal_cycle(uint16_t address) {
    BusSignals signals;
    signals.rw = true;
    signals.memory_enable = false;
    signals.address = address;
    signals.data = 0xFF;
    signals.cycle_kind = BusCycleKind::Internal;
    return signals;
}
} // namespace

Cpu::Cpu(CpuMode mode) : mode_(mode) {
    reset();

    auto make_name = [](const char* fn) {
        std::string s(fn);
        if (s.rfind("op_", 0) == 0) s = s.substr(3);
        std::replace(s.begin(), s.end(), '_', ' ');
        return s;
    };

    auto deduce_address_mode = [](const std::string& name) {
        if (name.size() >= 4 && name.rfind(" imm") == name.size() - 4) return AddressMode::IMMEDIATE;
        if (name.size() >= 4 && name.rfind(" dir") == name.size() - 4) return AddressMode::DIRECT;
        if (name.size() >= 4 && name.rfind(" idx") == name.size() - 4) return AddressMode::INDEXED;
        if (name.size() >= 4 && name.rfind(" ext") == name.size() - 4) return AddressMode::EXTENDED;
        return AddressMode::IMMEDIATE;
    };

    auto make_instruction = [&](uint8_t op, Handler h, std::string nm, std::optional<AddressMode> mode = std::nullopt,
                                uint8_t bytes = 0, uint8_t cycles = 0) {
        Instruction inst{};
        inst.op = op;
        inst.name = std::move(nm);
        inst.bytes = bytes;
        inst.cycles = cycles;
        inst.address_mode = mode.value_or(deduce_address_mode(inst.name));
        inst.handler = h;
        return inst;
    };

    const Instruction invalid = make_instruction(0, &Cpu::op_invalid, "invalid");
    std::fill(std::begin(instructions0_), std::end(instructions0_), invalid);
    std::fill(std::begin(instructions10_), std::end(instructions10_), invalid);
    std::fill(std::begin(instructions11_), std::end(instructions11_), invalid);

    auto set0 = [&](uint8_t op, Handler h, std::string nm, std::optional<AddressMode> mode = std::nullopt,
                    uint8_t bytes = 0, uint8_t cycles = 0) { instructions0_[op] = make_instruction(op, h, std::move(nm), mode, bytes, cycles); };
    auto set10 = [&](uint8_t op, Handler h, std::string nm, std::optional<AddressMode> mode = std::nullopt,
                     uint8_t bytes = 0, uint8_t cycles = 0) { instructions10_[op] = make_instruction(op, h, std::move(nm), mode, bytes, cycles); };
    auto set11 = [&](uint8_t op, Handler h, std::string nm, std::optional<AddressMode> mode = std::nullopt,
                     uint8_t bytes = 0, uint8_t cycles = 0) { instructions11_[op] = make_instruction(op, h, std::move(nm), mode, bytes, cycles); };
    auto mark_hd6309_only = [](Instruction (&table)[256], std::initializer_list<uint8_t> opcodes) {
        for (uint8_t op : opcodes) {
            table[op].hd6309_only = true;
        }
    };
#define SET0(op, fn, ...) set0(op, &Cpu::fn, make_name(#fn), ##__VA_ARGS__)
#define SET10(op, fn, ...) set10(op, &Cpu::fn, make_name(#fn), ##__VA_ARGS__)
#define SET11(op, fn, ...) set11(op, &Cpu::fn, make_name(#fn), ##__VA_ARGS__)

    // Inherent
    SET0(0x12, op_nop);
    SET0(0x4F, op_clra);
    SET0(0x5F, op_clrb);
    SET0(0x39, op_rts);

    // Branches
    SET0(0x16, op_lbra);
    SET0(0x17, op_lbsr);
    SET0(0x20, op_bra);
    SET0(0x21, op_brn);
    SET0(0x8D, op_bsr);
    SET0(0x26, op_bne);
    SET0(0x27, op_beq);
    SET0(0x22, op_bhi);
    SET0(0x23, op_bls);
    SET0(0x24, op_bcc);
    SET0(0x25, op_bcs);
    SET0(0x2A, op_bpl);
    SET0(0x2B, op_bmi);
    SET0(0x28, op_bvc);
    SET0(0x29, op_bvs);
    SET0(0x2C, op_bge);
    SET0(0x2D, op_blt);
    SET0(0x2E, op_bgt);
    SET0(0x2F, op_ble);
    SET10(0x21, op_lbrn);
    SET10(0x22, op_lbhi);
    SET10(0x23, op_lbls);
    SET10(0x24, op_lbcc);
    SET10(0x25, op_lbcs);
    SET10(0x26, op_lbne);
    SET10(0x27, op_lbeq);
    SET10(0x28, op_lbvc);
    SET10(0x29, op_lbvs);
    SET10(0x2A, op_lbpl);
    SET10(0x2B, op_lbmi);
    SET10(0x2C, op_lbge);
    SET10(0x2D, op_lblt);
    SET10(0x2E, op_lbgt);
    SET10(0x2F, op_lble);

    // Jumps
    SET0(0x0E, op_jmp_dir);
    SET0(0x7E, op_jmp_ext);
    SET0(0x6E, op_jmp_idx);
    SET0(0x9D, op_jsr_dir);
    SET0(0xBD, op_jsr_ext);
    SET0(0xAD, op_jsr_idx);
    SET0(0x39, op_rts);
    SET0(0x3B, op_rti);
    SET0(0x3F, op_swi);
    SET10(0x3F, op_swi2);
    SET11(0x3F, op_swi3);
    SET0(0x3C, op_cwai);
    SET0(0x13, op_sync);
    SET0(0x3D, op_mul);

    // Loads / Stores
    SET0(0x86, op_lda_imm);
    SET0(0x96, op_lda_dir);
    SET0(0xB6, op_lda_ext);
    SET0(0xA6, op_lda_idx);
    SET0(0xC6, op_ldb_imm);
    SET0(0xD6, op_ldb_dir);
    SET0(0xF6, op_ldb_ext);
    SET0(0xE6, op_ldb_idx);
    SET0(0xCC, op_ldd_imm);
    SET0(0xDC, op_ldd_dir);
    SET0(0xFC, op_ldd_ext);
    SET0(0xEC, op_ldd_idx);

    SET0(0x97, op_sta_dir);
    SET0(0xB7, op_sta_ext);
    SET0(0xD7, op_stb_dir);
    SET0(0xF7, op_stb_ext);
    SET0(0xA7, op_sta_idx);
    SET0(0xE7, op_stb_idx);
    SET0(0xDD, op_std_dir);
    SET0(0xFD, op_std_ext);
    SET0(0xED, op_std_idx);

    SET0(0x1E, op_exg);
    SET0(0x1F, op_tfr);

    // Logical
    SET0(0x84, op_anda_imm);
    SET0(0x94, op_anda_dir);
    SET0(0xB4, op_anda_ext);
    SET0(0xA4, op_anda_idx);
    SET0(0xC4, op_andb_imm);
    SET0(0xD4, op_andb_dir);
    SET0(0xF4, op_andb_ext);
    SET0(0xE4, op_andb_idx);

    SET0(0x8A, op_ora_imm);
    SET0(0x9A, op_ora_dir);
    SET0(0xBA, op_ora_ext);
    SET0(0xAA, op_ora_idx);
    SET0(0xCA, op_orb_imm);
    SET0(0xDA, op_orb_dir);
    SET0(0xFA, op_orb_ext);
    SET0(0xEA, op_orb_idx);

    SET0(0x88, op_eora_imm);
    SET0(0x98, op_eora_dir);
    SET0(0xB8, op_eora_ext);
    SET0(0xA8, op_eora_idx);
    SET0(0xC8, op_eorb_imm);
    SET0(0xD8, op_eorb_dir);
    SET0(0xF8, op_eorb_ext);
    SET0(0xE8, op_eorb_idx);

    // Arithmetic
    SET0(0x8B, op_adda_imm);
    SET0(0x9B, op_adda_dir);
    SET0(0xBB, op_adda_ext);
    SET0(0xAB, op_adda_idx);
    SET0(0xCB, op_addb_imm);
    SET0(0xDB, op_addb_dir);
    SET0(0xFB, op_addb_ext);
    SET0(0xEB, op_addb_idx);
    SET0(0xC3, op_addd_imm);
    SET0(0xD3, op_addd_dir);
    SET0(0xF3, op_addd_ext);
    SET0(0xE3, op_addd_idx);

    SET0(0x80, op_suba_imm);
    SET0(0x90, op_suba_dir);
    SET0(0xB0, op_suba_ext);
    SET0(0xA0, op_suba_idx);
    SET0(0xC0, op_subb_imm);
    SET0(0xD0, op_subb_dir);
    SET0(0xF0, op_subb_ext);
    SET0(0xE0, op_subb_idx);
    SET0(0x83, op_subd_imm);
    SET0(0x93, op_subd_dir);
    SET0(0xB3, op_subd_ext);
    SET0(0xA3, op_subd_idx);

    SET0(0x81, op_cmpa_imm);
    SET0(0x91, op_cmpa_dir);
    SET0(0xB1, op_cmpa_ext);
    SET0(0xA1, op_cmpa_idx);
    SET0(0xC1, op_cmpb_imm);
    SET0(0xD1, op_cmpb_dir);
    SET0(0xF1, op_cmpb_ext);
    SET0(0xE1, op_cmpb_idx);
    SET10(0x83, op_cmpd_imm);
    SET10(0x93, op_cmpd_dir);
    SET10(0xB3, op_cmpd_ext);
    SET10(0xA3, op_cmpd_idx);

    // 6309 arithmetic / logic extensions
    SET0(0xCD, op_ldq_imm);
    SET10(0xDC, op_ldq_dir);
    SET10(0xEC, op_ldq_idx);
    SET10(0xFC, op_ldq_ext);
    SET10(0xDD, op_stq_dir);
    SET10(0xED, op_stq_idx);
    SET10(0xFD, op_stq_ext);

    SET10(0x86, op_ldw_imm);
    SET10(0x96, op_ldw_dir);
    SET10(0xA6, op_ldw_idx);
    SET10(0xB6, op_ldw_ext);
    SET10(0x97, op_stw_dir);
    SET10(0xA7, op_stw_idx);
    SET10(0xB7, op_stw_ext);

    SET10(0x8B, op_addw_imm);
    SET10(0x9B, op_addw_dir);
    SET10(0xAB, op_addw_idx);
    SET10(0xBB, op_addw_ext);
    SET10(0x80, op_subw_imm);
    SET10(0x90, op_subw_dir);
    SET10(0xA0, op_subw_idx);
    SET10(0xB0, op_subw_ext);
    SET10(0x81, op_cmpw_imm);
    SET10(0x91, op_cmpw_dir);
    SET10(0xA1, op_cmpw_idx);
    SET10(0xB1, op_cmpw_ext);

    SET11(0x8B, op_adde_imm);
    SET11(0x9B, op_adde_dir);
    SET11(0xAB, op_adde_idx);
    SET11(0xBB, op_adde_ext);
    SET11(0xCB, op_addf_imm);
    SET11(0xDB, op_addf_dir);
    SET11(0xEB, op_addf_idx);
    SET11(0xFB, op_addf_ext);

    SET11(0x80, op_sube_imm);
    SET11(0x90, op_sube_dir);
    SET11(0xA0, op_sube_idx);
    SET11(0xB0, op_sube_ext);
    SET11(0xC0, op_subf_imm);
    SET11(0xD0, op_subf_dir);
    SET11(0xE0, op_subf_idx);
    SET11(0xF0, op_subf_ext);

    SET11(0x81, op_cmpe_imm);
    SET11(0x91, op_cmpe_dir);
    SET11(0xA1, op_cmpe_idx);
    SET11(0xB1, op_cmpe_ext);
    SET11(0xC1, op_cmpf_imm);
    SET11(0xD1, op_cmpf_dir);
    SET11(0xE1, op_cmpf_idx);
    SET11(0xF1, op_cmpf_ext);

    SET10(0x89, op_adcd_imm);
    SET10(0x99, op_adcd_dir);
    SET10(0xA9, op_adcd_idx);
    SET10(0xB9, op_adcd_ext);
    SET10(0x82, op_sbcd_imm);
    SET10(0x92, op_sbcd_dir);
    SET10(0xA2, op_sbcd_idx);
    SET10(0xB2, op_sbcd_ext);
    SET10(0x84, op_andd_imm);
    SET10(0x94, op_andd_dir);
    SET10(0xA4, op_andd_idx);
    SET10(0xB4, op_andd_ext);
    SET10(0x85, op_bitd_imm);
    SET10(0x95, op_bitd_dir);
    SET10(0xA5, op_bitd_idx);
    SET10(0xB5, op_bitd_ext);
    SET10(0x88, op_eord_imm);
    SET10(0x98, op_eord_dir);
    SET10(0xA8, op_eord_idx);
    SET10(0xB8, op_eord_ext);
    SET10(0x8A, op_ord_imm);
    SET10(0x9A, op_ord_dir);
    SET10(0xAA, op_ord_idx);
    SET10(0xBA, op_ord_ext);

    SET11(0x8D, op_divd_imm);
    SET11(0x9D, op_divd_dir);
    SET11(0xAD, op_divd_idx);
    SET11(0xBD, op_divd_ext);
    SET11(0x8E, op_divq_imm);
    SET11(0x9E, op_divq_dir);
    SET11(0xAE, op_divq_idx);
    SET11(0xBE, op_divq_ext);

    SET11(0x8F, op_muld_imm);
    SET11(0x9F, op_muld_dir);
    SET11(0xAF, op_muld_idx);
    SET11(0xBF, op_muld_ext);

    SET10(0x30, op_addr);
    SET10(0x32, op_subr);
    SET10(0x37, op_cmpr);
    SET10(0x33, op_sbcr);
    SET10(0x31, op_adcr);
    SET10(0x34, op_andr);
    SET10(0x35, op_orr);
    SET10(0x36, op_eorr);

    // Shift/rotate 6309
    SET10(0x40, op_negd);
    SET10(0x43, op_comd);
    SET10(0x44, op_lsrd);
    SET10(0x47, op_asrd);
    SET10(0x48, op_lsl_d);
    SET10(0x49, op_rold);
    SET10(0x4A, op_decd);
    SET10(0x4C, op_incd);
    SET10(0x4D, op_tstd);
    SET10(0x4F, op_clrd);
    SET10(0x53, op_comw_inh);
    SET10(0x54, op_lsrw_inh);
    SET10(0x46, op_rord);
    SET10(0x58, op_lslw_inh);
    SET10(0x59, op_rolw);
    SET10(0x5A, op_decw_inh);
    SET10(0x56, op_rorw);
    SET10(0x5C, op_incw_inh);
    SET10(0x5D, op_tstw_inh);
    SET10(0x5F, op_clrw_inh);

    // LDMD / SEXW
    SET11(0x3C, op_bitmd);
    SET11(0x3D, op_ldmd);
    SET0(0x14, op_sexw);

    // W stack
    SET10(0x38, op_pshsw);
    SET10(0x39, op_pulsw);
    SET10(0x3A, op_pshuw);
    SET10(0x3B, op_puluw);

    // E/F inherent unary
    SET11(0x43, op_come);
    SET11(0x4A, op_dece);
    SET11(0x4C, op_ince);
    SET11(0x4D, op_tste);
    SET11(0x4F, op_clre);
    SET11(0x53, op_comf);
    SET11(0x5A, op_decf);
    SET11(0x5C, op_incf);
    SET11(0x5D, op_tstf);
    SET11(0x5F, op_clrf);

    // TFM
    SET11(0x38, op_tfm_pp);
    SET11(0x39, op_tfm_mm);
    SET11(0x3A, op_tfm_pn);
    SET11(0x3B, op_tfm_np);

    // Bit ops
    SET0(0x01, op_oim_dir);
    SET0(0x61, op_oim_idx);
    SET0(0x71, op_oim_ext);
    SET0(0x02, op_aim_dir);
    SET0(0x62, op_aim_idx);
    SET0(0x72, op_aim_ext);
    SET0(0x05, op_eim_dir);
    SET0(0x65, op_eim_idx);
    SET0(0x75, op_eim_ext);
    SET0(0x0B, op_tim_dir);
    SET0(0x6B, op_tim_idx);
    SET0(0x7B, op_tim_ext);

    // Bit transfer/logic (direct only) 0x11 prefix
    SET11(0x30, op_band);
    SET11(0x31, op_biand);
    SET11(0x32, op_bor);
    SET11(0x33, op_bior);
    SET11(0x34, op_beor);
    SET11(0x35, op_bieor);
    SET11(0x36, op_ldbt);
    SET11(0x37, op_stbt);

    SET0(0x89, op_adca_imm);
    SET0(0x99, op_adca_dir);
    SET0(0xB9, op_adca_ext);
    SET0(0xA9, op_adca_idx);
    SET0(0xC9, op_adcb_imm);
    SET0(0xD9, op_adcb_dir);
    SET0(0xF9, op_adcb_ext);
    SET0(0xE9, op_adcb_idx);

    SET0(0x82, op_sbca_imm);
    SET0(0x92, op_sbca_dir);
    SET0(0xB2, op_sbca_ext);
    SET0(0xA2, op_sbca_idx);
    SET0(0xC2, op_sbcb_imm);
    SET0(0xD2, op_sbcb_dir);
    SET0(0xF2, op_sbcb_ext);
    SET0(0xE2, op_sbcb_idx);

    SET0(0x85, op_bita_imm);
    SET0(0x95, op_bita_dir);
    SET0(0xB5, op_bita_ext);
    SET0(0xA5, op_bita_idx);
    SET0(0xC5, op_bitb_imm);
    SET0(0xD5, op_bitb_dir);
    SET0(0xF5, op_bitb_ext);
    SET0(0xE5, op_bitb_idx);

    // LEA
    SET0(0x30, op_leax);
    SET0(0x31, op_leay);
    SET0(0x32, op_leas);
    SET0(0x33, op_leau);

    // 16-bit loads/stores
    SET0(0x8E, op_ldx_imm);
    SET0(0x9E, op_ldx_dir);
    SET0(0xBE, op_ldx_ext);
    SET0(0xAE, op_ldx_idx);
    SET0(0xCE, op_ldu_imm);
    SET0(0xDE, op_ldu_dir);
    SET0(0xFE, op_ldu_ext);
    SET0(0xEE, op_ldu_idx);
    SET10(0x8E, op_ldy_imm);
    SET10(0x9E, op_ldy_dir);
    SET10(0xBE, op_ldy_ext);
    SET10(0xAE, op_ldy_idx);
    SET10(0xCE, op_lds_imm);
    SET10(0xDE, op_lds_dir);
    SET10(0xFE, op_lds_ext);
    SET10(0xEE, op_lds_idx);

    SET0(0x9F, op_stx_dir);
    SET0(0xBF, op_stx_ext);
    SET0(0xAF, op_stx_idx);
    SET0(0xDF, op_stu_dir);
    SET0(0xFF, op_stu_ext);
    SET0(0xEF, op_stu_idx);
    SET10(0x9F, op_sty_dir);
    SET10(0xBF, op_sty_ext);
    SET10(0xAF, op_sty_idx);
    SET10(0xDF, op_sts_dir);
    SET10(0xFF, op_sts_ext);
    SET10(0xEF, op_sts_idx);

    // Compare 16-bit
    SET0(0x8C, op_cmpx_imm);
    SET0(0x9C, op_cmpx_dir);
    SET0(0xBC, op_cmpx_ext);
    SET0(0xAC, op_cmpx_idx);
    SET10(0x8C, op_cmpy_imm);
    SET10(0x9C, op_cmpy_dir);
    SET10(0xBC, op_cmpy_ext);
    SET10(0xAC, op_cmpy_idx);
    SET11(0x83, op_cmpu_imm);
    SET11(0x93, op_cmpu_dir);
    SET11(0xB3, op_cmpu_ext);
    SET11(0xA3, op_cmpu_idx);
    SET11(0x8C, op_cmps_imm);
    SET11(0x9C, op_cmps_dir);
    SET11(0xBC, op_cmps_ext);
    SET11(0xAC, op_cmps_idx);

    // Misc
    SET0(0x3A, op_abx);
    SET0(0x1D, op_sex);
    SET0(0x1C, op_andcc);
    SET0(0x1A, op_orcc);
    SET0(0x19, op_daa);

    // Stack
    SET0(0x34, op_pshs);
    SET0(0x35, op_puls);
    SET0(0x36, op_pshu);
    SET0(0x37, op_pulu);

    // Accumulator unary/shift
    SET0(0x40, op_nega);
    SET0(0x50, op_negb);
    SET0(0x43, op_coma);
    SET0(0x53, op_comb);
    SET0(0x44, op_lsra);
    SET0(0x54, op_lsrb);
    SET0(0x46, op_rora);
    SET0(0x56, op_rorb);
    SET0(0x47, op_asra);
    SET0(0x57, op_asrb);
    SET0(0x48, op_asla);
    SET0(0x58, op_aslb);
    SET0(0x49, op_rola);
    SET0(0x59, op_rolb);
    SET0(0x4A, op_deca);
    SET0(0x5A, op_decb);
    SET0(0x4C, op_inca);
    SET0(0x5C, op_incb);
    SET0(0x4D, op_tsta);
    SET0(0x5D, op_tstb);

    // Memory unary/shift
    SET0(0x00, op_neg_dir);
    SET0(0x60, op_neg_idx);
    SET0(0x70, op_neg_ext);
    SET0(0x03, op_com_dir);
    SET0(0x63, op_com_idx);
    SET0(0x73, op_com_ext);
    SET0(0x04, op_lsr_dir);
    SET0(0x64, op_lsr_idx);
    SET0(0x74, op_lsr_ext);
    SET0(0x06, op_ror_dir);
    SET0(0x66, op_ror_idx);
    SET0(0x76, op_ror_ext);
    SET0(0x07, op_asr_dir);
    SET0(0x67, op_asr_idx);
    SET0(0x77, op_asr_ext);
    SET0(0x08, op_asl_dir);
    SET0(0x68, op_asl_idx);
    SET0(0x78, op_asl_ext);
    SET0(0x09, op_rol_dir);
    SET0(0x69, op_rol_idx);
    SET0(0x79, op_rol_ext);
    SET0(0x0A, op_dec_dir);
    SET0(0x6A, op_dec_idx);
    SET0(0x7A, op_dec_ext);
    SET0(0x0C, op_inc_dir);
    SET0(0x6C, op_inc_idx);
    SET0(0x7C, op_inc_ext);
    SET0(0x0D, op_tst_dir);
    SET0(0x6D, op_tst_idx);
    SET0(0x7D, op_tst_ext);
    SET0(0x0F, op_clr_dir);
    SET0(0x6F, op_clr_idx);
    SET0(0x7F, op_clr_ext);

    mark_hd6309_only(instructions0_, {
        0x01, 0x02, 0x05, 0x0B, 0x14, 0x61, 0x62, 0x65, 0x6B,
        0x71, 0x72, 0x75, 0x7B, 0xCD,
    });
    mark_hd6309_only(instructions10_, {
        0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37,
        0x38, 0x39, 0x3A, 0x3B,
        0x40, 0x43, 0x44, 0x46, 0x47, 0x48, 0x49, 0x4A, 0x4C,
        0x4D, 0x4F, 0x53, 0x54, 0x56, 0x58, 0x59, 0x5A, 0x5C,
        0x5D, 0x5F,
        0x80, 0x81, 0x82, 0x84, 0x85, 0x86, 0x88, 0x89, 0x8A, 0x8B,
        0x90, 0x91, 0x92, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9A, 0x9B,
        0xA0, 0xA1, 0xA2, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8, 0xA9, 0xAA, 0xAB,
        0xB0, 0xB1, 0xB2, 0xB4, 0xB5, 0xB6, 0xB7, 0xB8, 0xB9, 0xBA, 0xBB,
        0xDC, 0xDD, 0xEC, 0xED, 0xFC, 0xFD,
    });
    mark_hd6309_only(instructions11_, {
        0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37,
        0x38, 0x39, 0x3A, 0x3B, 0x3C, 0x3D,
        0x43, 0x4A, 0x4C, 0x4D, 0x4F, 0x53, 0x5A, 0x5C, 0x5D, 0x5F,
        0x80, 0x81, 0x86, 0x8B, 0x8D, 0x8E, 0x8F,
        0x90, 0x91, 0x96, 0x97, 0x9B, 0x9D, 0x9E, 0x9F,
        0xA0, 0xA1, 0xA6, 0xA7, 0xAB, 0xAD, 0xAE, 0xAF,
        0xB0, 0xB1, 0xB6, 0xB7, 0xBB, 0xBD, 0xBE, 0xBF,
        0xC0, 0xC1, 0xCB, 0xD0, 0xD1, 0xDB, 0xE0, 0xE1, 0xEB,
        0xF0, 0xF1, 0xFB,
    });
#undef SET0
#undef SET10
#undef SET11
}

void Cpu::reset() {
    const uint16_t preserved_v = regs_.v;
    regs_ = Registers{};
    regs_.v = preserved_v;
    // Default stack high for quick use; user can override by writing regs().s.
    regs_.s = 0xFFFF;
    regs_.cc = CC_I; // IRQ masked on reset by default.
    cycles_executed_ = 0;
    last_pc_ = 0;
    last_opcode_ = 0;
    last_prefix_ = 0;
    micro_op_ = {};
    sync_wait_ = false;
}

CpuTickResult Cpu::tick(Bus& bus) {
    last_pc_ = regs_.pc;
    const uint8_t opcode = fetch_opcode_byte(bus);
    last_prefix_ = 0x00;
    last_opcode_ = opcode;
    const Instruction* inst = &instructions0_[opcode];
    if (opcode == 0x10) {
        const uint8_t next = fetch_opcode_byte(bus);
        inst = &instructions10_[next];
        last_prefix_ = 0x10;
        last_opcode_ = next;
    } else if (opcode == 0x11) {
        const uint8_t next = fetch_opcode_byte(bus);
        inst = &instructions11_[next];
        last_prefix_ = 0x11;
        last_opcode_ = next;
    }

    const Handler handler = (inst && inst->handler && !(mode_ == CpuMode::MC6809 && inst->hd6309_only))
        ? inst->handler
        : &Cpu::op_invalid;
    const uint8_t cycles = (this->*handler)(bus);
    cycles_executed_ += cycles;
    return CpuTickResult{cycles};
}

const std::string& Cpu::opcode_name(uint8_t prefix, uint8_t opcode) const {
    if (prefix == 0x10) return instructions10_[opcode].name;
    if (prefix == 0x11) return instructions11_[opcode].name;
    return instructions0_[opcode].name;
}

uint8_t Cpu::opcode_length(Bus& bus, uint16_t pc) const {
    uint8_t op0 = bus.peek8(pc);
    if (op0 == 0x10 || op0 == 0x11) {
        const uint8_t op1 = bus.peek8(static_cast<uint16_t>(pc + 1));
        const Instruction& inst = (op0 == 0x10) ? instructions10_[op1] : instructions11_[op1];
        if (inst.bytes != 0) return inst.bytes;
        if (op0 == 0x11 && op1 == 0x3D) return 3; // LDMD immediate
        return static_cast<uint8_t>(2); // prefixes; we don't encode full length table yet
    }
    const Instruction& inst = instructions0_[op0];
    if (inst.bytes != 0) return inst.bytes;
    // Quick length hints for some instructions; default 1.
    switch (op0) {
    case 0x16: // LBRA
    case 0x17: // LBSR
        return 3;
    case 0x8D: // BSR
        return 2;
    default:
        break;
    }
    return 1;
}

bool Cpu::has_pending_micro_ops() const {
    return micro_op_.kind != MicroOpKind::None;
}

void Cpu::discard_micro_ops() {
    micro_op_ = {};
}

bool Cpu::start_micro_op(Bus& bus, uint8_t opcode) {
    MicroOpKind kind = MicroOpKind::None;
    uint8_t total_cycles = 0;
    uint8_t prefix = 0x00;
    uint8_t direct_offset = 0x00;
    bool branch_taken = false;
    switch (opcode) {
    case 0x12:
        kind = MicroOpKind::Nop;
        total_cycles = 2;
        break;
    case 0x19:
    case 0x1D:
    case 0x3A:
    case 0x3D:
        kind = MicroOpKind::MiscInherent;
        switch (opcode) {
        case 0x3D:
            total_cycles = 11;
            break;
        case 0x3A:
            total_cycles = 3;
            break;
        default:
            total_cycles = 2;
            break;
        }
        break;
    case 0x1A:
    case 0x1C:
        kind = MicroOpKind::CcImmediate;
        total_cycles = 3;
        break;
    case 0x1E:
    case 0x1F:
        kind = MicroOpKind::RegisterTransfer;
        total_cycles = opcode == 0x1E ? 8 : 6;
        break;
    case 0x86:
        kind = MicroOpKind::LdaImmediate;
        total_cycles = 2;
        break;
    case 0x96:
        kind = MicroOpKind::LdaDirect;
        total_cycles = 4;
        break;
    case 0x97:
        kind = MicroOpKind::StaDirect;
        total_cycles = 4;
        break;
    case 0xB6:
        kind = MicroOpKind::LdaExtended;
        total_cycles = 5;
        break;
    case 0xB7:
        kind = MicroOpKind::StaExtended;
        total_cycles = 5;
        break;
    case 0xC6:
        kind = MicroOpKind::LdbImmediate;
        total_cycles = 2;
        break;
    case 0xD6:
        kind = MicroOpKind::LdbDirect;
        total_cycles = 4;
        break;
    case 0xD7:
        kind = MicroOpKind::StbDirect;
        total_cycles = 4;
        break;
    case 0xF6:
        kind = MicroOpKind::LdbExtended;
        total_cycles = 5;
        break;
    case 0xF7:
        kind = MicroOpKind::StbExtended;
        total_cycles = 5;
        break;
    case 0xCC:
        kind = MicroOpKind::LddImmediate;
        total_cycles = 3;
        break;
    case 0xDC:
        kind = MicroOpKind::LddDirect;
        total_cycles = 5;
        break;
    case 0xFC:
        kind = MicroOpKind::LddExtended;
        total_cycles = 6;
        break;
    case 0xDD:
        kind = MicroOpKind::StdDirect;
        total_cycles = 5;
        break;
    case 0xFD:
        kind = MicroOpKind::StdExtended;
        total_cycles = 6;
        break;
    case 0x16:
        kind = MicroOpKind::Lbra;
        total_cycles = is_native_hd6309(regs_, mode_) ? 4 : 5;
        branch_taken = true;
        break;
    case 0x20:
    case 0x21:
    case 0x22:
    case 0x23:
    case 0x24:
    case 0x25:
    case 0x26:
    case 0x27:
    case 0x28:
    case 0x29:
    case 0x2A:
    case 0x2B:
    case 0x2C:
    case 0x2D:
    case 0x2E:
    case 0x2F:
        kind = MicroOpKind::Branch;
        branch_taken = branch_condition(opcode, regs_);
        total_cycles = branch_taken ? 3 : 2;
        break;
    case 0x8D:
        kind = MicroOpKind::Bsr;
        total_cycles = is_native_hd6309(regs_, mode_) ? 6 : 7;
        break;
    case 0x17:
        kind = MicroOpKind::Lbsr;
        total_cycles = is_native_hd6309(regs_, mode_) ? 7 : 9;
        break;
    case 0x39:
        kind = MicroOpKind::Rts;
        total_cycles = 5;
        break;
    case 0x34:
    case 0x36:
        kind = MicroOpKind::StackPush;
        direct_offset = bus.peek8(static_cast<uint16_t>(regs_.pc + 1));
        total_cycles = static_cast<uint8_t>((is_native_hd6309(regs_, mode_) ? 4 : 5) + stack_mask_byte_count(direct_offset));
        break;
    case 0x35:
    case 0x37:
        kind = MicroOpKind::StackPull;
        direct_offset = bus.peek8(static_cast<uint16_t>(regs_.pc + 1));
        total_cycles = static_cast<uint8_t>((is_native_hd6309(regs_, mode_) ? 4 : 5) + stack_mask_byte_count(direct_offset));
        break;
    case 0x9D:
        kind = MicroOpKind::JsrDirect;
        total_cycles = 5;
        break;
    case 0xBD:
        kind = MicroOpKind::JsrExtended;
        total_cycles = 7;
        break;
    case 0x0E:
        kind = MicroOpKind::JmpDirect;
        total_cycles = is_native_hd6309(regs_, mode_) ? 2 : 3;
        break;
    case 0x7E:
        kind = MicroOpKind::JmpExtended;
        total_cycles = is_native_hd6309(regs_, mode_) ? 3 : 4;
        break;
    case 0x40:
        kind = MicroOpKind::Nega;
        total_cycles = is_native_hd6309(regs_, mode_) ? 1 : 2;
        break;
    case 0x50:
        kind = MicroOpKind::Negb;
        total_cycles = is_native_hd6309(regs_, mode_) ? 1 : 2;
        break;
    case 0x43:
        kind = MicroOpKind::Coma;
        total_cycles = is_native_hd6309(regs_, mode_) ? 1 : 2;
        break;
    case 0x53:
        kind = MicroOpKind::Comb;
        total_cycles = is_native_hd6309(regs_, mode_) ? 1 : 2;
        break;
    case 0x44:
        kind = MicroOpKind::Lsra;
        total_cycles = is_native_hd6309(regs_, mode_) ? 1 : 2;
        break;
    case 0x54:
        kind = MicroOpKind::Lsrb;
        total_cycles = is_native_hd6309(regs_, mode_) ? 1 : 2;
        break;
    case 0x46:
        kind = MicroOpKind::Rora;
        total_cycles = is_native_hd6309(regs_, mode_) ? 1 : 2;
        break;
    case 0x56:
        kind = MicroOpKind::Rorb;
        total_cycles = is_native_hd6309(regs_, mode_) ? 1 : 2;
        break;
    case 0x47:
        kind = MicroOpKind::Asra;
        total_cycles = is_native_hd6309(regs_, mode_) ? 1 : 2;
        break;
    case 0x57:
        kind = MicroOpKind::Asrb;
        total_cycles = is_native_hd6309(regs_, mode_) ? 1 : 2;
        break;
    case 0x48:
        kind = MicroOpKind::Asla;
        total_cycles = is_native_hd6309(regs_, mode_) ? 1 : 2;
        break;
    case 0x58:
        kind = MicroOpKind::Aslb;
        total_cycles = is_native_hd6309(regs_, mode_) ? 1 : 2;
        break;
    case 0x49:
        kind = MicroOpKind::Rola;
        total_cycles = is_native_hd6309(regs_, mode_) ? 1 : 2;
        break;
    case 0x59:
        kind = MicroOpKind::Rolb;
        total_cycles = is_native_hd6309(regs_, mode_) ? 1 : 2;
        break;
    case 0x4F:
        kind = MicroOpKind::Clra;
        total_cycles = is_native_hd6309(regs_, mode_) ? 1 : 2;
        break;
    case 0x5F:
        kind = MicroOpKind::Clrb;
        total_cycles = is_native_hd6309(regs_, mode_) ? 1 : 2;
        break;
    case 0x4D:
        kind = MicroOpKind::Tsta;
        total_cycles = is_native_hd6309(regs_, mode_) ? 1 : 2;
        break;
    case 0x5D:
        kind = MicroOpKind::Tstb;
        total_cycles = is_native_hd6309(regs_, mode_) ? 1 : 2;
        break;
    case 0x4A:
        kind = MicroOpKind::Deca;
        total_cycles = is_native_hd6309(regs_, mode_) ? 1 : 2;
        break;
    case 0x5A:
        kind = MicroOpKind::Decb;
        total_cycles = is_native_hd6309(regs_, mode_) ? 1 : 2;
        break;
    case 0x4C:
        kind = MicroOpKind::Inca;
        total_cycles = is_native_hd6309(regs_, mode_) ? 1 : 2;
        break;
    case 0x5C:
        kind = MicroOpKind::Incb;
        total_cycles = is_native_hd6309(regs_, mode_) ? 1 : 2;
        break;
    case 0x8C:
        kind = MicroOpKind::Cmp16Immediate;
        total_cycles = cmp16_immediate_cycles(prefix, regs_, mode_);
        break;
    case 0x9C:
        kind = MicroOpKind::Cmp16Direct;
        total_cycles = cmp16_direct_cycles(prefix, regs_, mode_);
        break;
    case 0xBC:
        kind = MicroOpKind::Cmp16Extended;
        total_cycles = cmp16_extended_cycles(prefix, regs_, mode_);
        break;
    case 0x8E:
    case 0xCE:
        kind = MicroOpKind::WordLoadImmediate;
        total_cycles = word_load_immediate_cycles(prefix, opcode, regs_, mode_);
        break;
    case 0x9E:
    case 0xDE:
        kind = MicroOpKind::WordLoadDirect;
        total_cycles = word_direct_cycles(prefix, regs_, mode_);
        break;
    case 0xBE:
    case 0xFE:
        kind = MicroOpKind::WordLoadExtended;
        total_cycles = word_extended_cycles(prefix, regs_, mode_);
        break;
    case 0x9F:
    case 0xDF:
        kind = MicroOpKind::WordStoreDirect;
        total_cycles = word_direct_cycles(prefix, regs_, mode_);
        break;
    case 0xBF:
    case 0xFF:
        kind = MicroOpKind::WordStoreExtended;
        total_cycles = word_extended_cycles(prefix, regs_, mode_);
        break;
    default:
        if (opcode == 0x10) {
            const uint8_t next = bus.peek8(static_cast<uint16_t>(regs_.pc + 1));
            if (next >= 0x21 && next <= 0x2F) {
                kind = MicroOpKind::LongBranch;
                branch_taken = branch_condition(next, regs_);
                total_cycles = branch_taken ? 6 : 5;
                prefix = opcode;
                opcode = next;
                break;
            }
            if (next == 0x83) {
                kind = MicroOpKind::CmpdImmediate;
                total_cycles = is_native_hd6309(regs_, mode_) ? 4 : 5;
                prefix = opcode;
                opcode = next;
                break;
            }
            if (next == 0x93) {
                kind = MicroOpKind::CmpdDirect;
                total_cycles = is_native_hd6309(regs_, mode_) ? 5 : 7;
                prefix = opcode;
                opcode = next;
                break;
            }
            if (next == 0xB3) {
                kind = MicroOpKind::CmpdExtended;
                total_cycles = is_native_hd6309(regs_, mode_) ? 6 : 8;
                prefix = opcode;
                opcode = next;
                break;
            }
            if (is_cmp16_immediate_opcode(opcode, next)) {
                kind = MicroOpKind::Cmp16Immediate;
                total_cycles = cmp16_immediate_cycles(opcode, regs_, mode_);
                prefix = opcode;
                opcode = next;
                break;
            }
            if (is_cmp16_direct_opcode(opcode, next)) {
                kind = MicroOpKind::Cmp16Direct;
                total_cycles = cmp16_direct_cycles(opcode, regs_, mode_);
                prefix = opcode;
                opcode = next;
                break;
            }
            if (is_cmp16_extended_opcode(opcode, next)) {
                kind = MicroOpKind::Cmp16Extended;
                total_cycles = cmp16_extended_cycles(opcode, regs_, mode_);
                prefix = opcode;
                opcode = next;
                break;
            }
            if (is_word_load_immediate_opcode(opcode, next)) {
                kind = MicroOpKind::WordLoadImmediate;
                total_cycles = word_load_immediate_cycles(opcode, next, regs_, mode_);
                prefix = opcode;
                opcode = next;
                break;
            }
            if (is_word_load_direct_opcode(opcode, next)) {
                kind = MicroOpKind::WordLoadDirect;
                total_cycles = word_direct_cycles(opcode, regs_, mode_);
                prefix = opcode;
                opcode = next;
                break;
            }
            if (is_word_load_extended_opcode(opcode, next)) {
                kind = MicroOpKind::WordLoadExtended;
                total_cycles = word_extended_cycles(opcode, regs_, mode_);
                prefix = opcode;
                opcode = next;
                break;
            }
            if (is_word_store_direct_opcode(opcode, next)) {
                kind = MicroOpKind::WordStoreDirect;
                total_cycles = word_direct_cycles(opcode, regs_, mode_);
                prefix = opcode;
                opcode = next;
                break;
            }
            if (is_word_store_extended_opcode(opcode, next)) {
                kind = MicroOpKind::WordStoreExtended;
                total_cycles = word_extended_cycles(opcode, regs_, mode_);
                prefix = opcode;
                opcode = next;
                break;
            }
        }
        if (opcode == 0x11) {
            const uint8_t next = bus.peek8(static_cast<uint16_t>(regs_.pc + 1));
            if (is_cmp16_immediate_opcode(opcode, next)) {
                kind = MicroOpKind::Cmp16Immediate;
                total_cycles = cmp16_immediate_cycles(opcode, regs_, mode_);
                prefix = opcode;
                opcode = next;
                break;
            }
            if (is_cmp16_direct_opcode(opcode, next)) {
                kind = MicroOpKind::Cmp16Direct;
                total_cycles = cmp16_direct_cycles(opcode, regs_, mode_);
                prefix = opcode;
                opcode = next;
                break;
            }
            if (is_cmp16_extended_opcode(opcode, next)) {
                kind = MicroOpKind::Cmp16Extended;
                total_cycles = cmp16_extended_cycles(opcode, regs_, mode_);
                prefix = opcode;
                opcode = next;
                break;
            }
        }
        if (is_alu8_immediate_opcode(opcode)) {
            kind = MicroOpKind::Alu8Immediate;
            total_cycles = 2;
            break;
        }
        if (is_alu8_direct_opcode(opcode)) {
            kind = MicroOpKind::Alu8Direct;
            total_cycles = is_native_hd6309(regs_, mode_) ? 3 : 4;
            break;
        }
        if (is_alu8_extended_opcode(opcode)) {
            kind = MicroOpKind::Alu8Extended;
            total_cycles = is_native_hd6309(regs_, mode_) ? 4 : 5;
            break;
        }
        if (is_alu16_immediate_opcode(opcode)) {
            kind = MicroOpKind::Alu16Immediate;
            total_cycles = is_native_hd6309(regs_, mode_) ? 3 : 4;
            break;
        }
        if (is_alu16_direct_opcode(opcode)) {
            kind = MicroOpKind::Alu16Direct;
            total_cycles = is_native_hd6309(regs_, mode_) ? 4 : 6;
            break;
        }
        if (is_alu16_extended_opcode(opcode)) {
            kind = MicroOpKind::Alu16Extended;
            total_cycles = is_native_hd6309(regs_, mode_) ? 5 : 7;
            break;
        }
        if (is_memory_unary_direct_opcode(opcode)) {
            kind = MicroOpKind::MemoryUnaryDirect;
            total_cycles = 6;
            break;
        }
        if (is_memory_unary_extended_opcode(opcode)) {
            kind = MicroOpKind::MemoryUnaryExtended;
            total_cycles = 7;
            break;
        }
        return false;
    }

    last_pc_ = regs_.pc;
    last_prefix_ = prefix;
    last_opcode_ = opcode;
    micro_op_ = MicroOpState{
        kind,
        regs_.pc,
        opcode,
        prefix,
        0,
        total_cycles,
        direct_offset,
        0,
        0,
        branch_taken,
    };
    return true;
}

CpuMicrocycleStatus Cpu::micro_op_status(bool instruction_started, bool instruction_complete) const {
    const std::size_t completed_cycles = instruction_complete ? micro_op_.total_cycles : micro_op_.step;
    const std::size_t pending = micro_op_.total_cycles > completed_cycles
        ? static_cast<std::size_t>(micro_op_.total_cycles - completed_cycles)
        : 0u;
    return CpuMicrocycleStatus{
        instruction_started,
        instruction_complete,
        CpuTickResult{micro_op_.total_cycles},
        pending,
    };
}

BusSignals Cpu::micro_op_signals() const {
    switch (micro_op_.kind) {
    case MicroOpKind::Nop:
        return micro_op_.step == 0
            ? read_cycle(regs_.pc, BusCycleKind::OpcodeFetch)
            : internal_cycle(regs_.pc);
    case MicroOpKind::MiscInherent:
        return micro_op_.step == 0
            ? read_cycle(regs_.pc, BusCycleKind::OpcodeFetch)
            : internal_cycle(regs_.pc);
    case MicroOpKind::CcImmediate:
    case MicroOpKind::RegisterTransfer:
        if (micro_op_.step == 0) return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        if (micro_op_.step == 1) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        return internal_cycle(regs_.pc);
    case MicroOpKind::Alu16Immediate:
        if (micro_op_.step == 0) return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        if (micro_op_.step == 1 || micro_op_.step == 2) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        return internal_cycle(regs_.pc);
    case MicroOpKind::CmpdImmediate:
        if (micro_op_.step == 0) return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        if (micro_op_.step == 1) return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        if (micro_op_.step == 2 || micro_op_.step == 3) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        return internal_cycle(regs_.pc);
    case MicroOpKind::Cmp16Immediate: {
        const uint8_t operand_high_step = micro_op_.prefix == 0x00 ? 1 : 2;
        const uint8_t operand_low_step = micro_op_.prefix == 0x00 ? 2 : 3;
        if (micro_op_.step == 0) return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        if (micro_op_.prefix != 0x00 && micro_op_.step == 1) {
            return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        }
        if (micro_op_.step == operand_high_step || micro_op_.step == operand_low_step) {
            return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        }
        return internal_cycle(regs_.pc);
    }
    case MicroOpKind::WordLoadImmediate: {
        const uint8_t operand_high_step = micro_op_.prefix == 0x00 ? 1 : 2;
        const uint8_t operand_low_step = micro_op_.prefix == 0x00 ? 2 : 3;
        if (micro_op_.step == 0) return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        if (micro_op_.prefix != 0x00 && micro_op_.step == 1) {
            return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        }
        if (micro_op_.step == operand_high_step || micro_op_.step == operand_low_step) {
            return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        }
        return internal_cycle(regs_.pc);
    }
    case MicroOpKind::LdaImmediate:
    case MicroOpKind::LdbImmediate:
    case MicroOpKind::LddImmediate:
    case MicroOpKind::Alu8Immediate:
        return micro_op_.step == 0
            ? read_cycle(regs_.pc, BusCycleKind::OpcodeFetch)
            : read_cycle(regs_.pc, BusCycleKind::OperandRead);
    case MicroOpKind::LdaDirect:
    case MicroOpKind::LdbDirect:
    case MicroOpKind::LddDirect:
    case MicroOpKind::Alu8Direct:
    case MicroOpKind::Alu16Direct:
    case MicroOpKind::CmpdDirect: {
        if (micro_op_.step == 0) return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        if (micro_op_.kind == MicroOpKind::CmpdDirect && micro_op_.step == 1) {
            return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        }
        const uint8_t operand_step = micro_op_.kind == MicroOpKind::CmpdDirect ? 2 : 1;
        const uint8_t high_step = micro_op_.kind == MicroOpKind::CmpdDirect ? 3 : 2;
        const uint8_t low_step = micro_op_.kind == MicroOpKind::CmpdDirect ? 4 : 3;
        if (micro_op_.step == operand_step) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        if (micro_op_.step == high_step) return read_cycle(micro_op_.effective_address, BusCycleKind::OperandRead);
        if ((micro_op_.kind == MicroOpKind::LddDirect ||
             micro_op_.kind == MicroOpKind::Alu16Direct ||
             micro_op_.kind == MicroOpKind::CmpdDirect) &&
            micro_op_.step == low_step) {
            return read_cycle(static_cast<uint16_t>(micro_op_.effective_address + 1), BusCycleKind::OperandRead);
        }
        return internal_cycle(regs_.pc);
    }
    case MicroOpKind::Cmp16Direct: {
        const uint8_t operand_step = micro_op_.prefix == 0x00 ? 1 : 2;
        const uint8_t high_step = micro_op_.prefix == 0x00 ? 2 : 3;
        const uint8_t low_step = micro_op_.prefix == 0x00 ? 3 : 4;
        if (micro_op_.step == 0) return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        if (micro_op_.prefix != 0x00 && micro_op_.step == 1) {
            return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        }
        if (micro_op_.step == operand_step) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        if (micro_op_.step == high_step) return read_cycle(micro_op_.effective_address, BusCycleKind::OperandRead);
        if (micro_op_.step == low_step) {
            return read_cycle(static_cast<uint16_t>(micro_op_.effective_address + 1), BusCycleKind::OperandRead);
        }
        return internal_cycle(regs_.pc);
    }
    case MicroOpKind::WordLoadDirect: {
        const uint8_t operand_step = micro_op_.prefix == 0x00 ? 1 : 2;
        const uint8_t high_step = micro_op_.prefix == 0x00 ? 2 : 3;
        const uint8_t low_step = micro_op_.prefix == 0x00 ? 3 : 4;
        if (micro_op_.step == 0) return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        if (micro_op_.prefix != 0x00 && micro_op_.step == 1) {
            return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        }
        if (micro_op_.step == operand_step) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        if (micro_op_.step == high_step) return read_cycle(micro_op_.effective_address, BusCycleKind::OperandRead);
        if (micro_op_.step == low_step) {
            return read_cycle(static_cast<uint16_t>(micro_op_.effective_address + 1), BusCycleKind::OperandRead);
        }
        return internal_cycle(regs_.pc);
    }
    case MicroOpKind::WordStoreDirect: {
        const uint8_t operand_step = micro_op_.prefix == 0x00 ? 1 : 2;
        const uint8_t high_step = micro_op_.prefix == 0x00 ? 2 : 3;
        const uint8_t low_step = micro_op_.prefix == 0x00 ? 3 : 4;
        const uint16_t value = word_register_value(regs_, micro_op_.prefix, micro_op_.opcode);
        if (micro_op_.step == 0) return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        if (micro_op_.prefix != 0x00 && micro_op_.step == 1) {
            return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        }
        if (micro_op_.step == operand_step) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        if (micro_op_.step == high_step) {
            return write_cycle(micro_op_.effective_address, hi(value), BusCycleKind::OperandWrite);
        }
        if (micro_op_.step == low_step) {
            return write_cycle(static_cast<uint16_t>(micro_op_.effective_address + 1), lo(value), BusCycleKind::OperandWrite);
        }
        return internal_cycle(regs_.pc);
    }
    case MicroOpKind::LdaExtended:
    case MicroOpKind::LdbExtended:
    case MicroOpKind::LddExtended:
    case MicroOpKind::Alu8Extended:
    case MicroOpKind::Alu16Extended:
    case MicroOpKind::CmpdExtended: {
        if (micro_op_.step == 0) return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        if (micro_op_.kind == MicroOpKind::CmpdExtended && micro_op_.step == 1) {
            return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        }
        const uint8_t high_address_step = micro_op_.kind == MicroOpKind::CmpdExtended ? 2 : 1;
        const uint8_t low_address_step = micro_op_.kind == MicroOpKind::CmpdExtended ? 3 : 2;
        const uint8_t high_data_step = micro_op_.kind == MicroOpKind::CmpdExtended ? 4 : 3;
        const uint8_t low_data_step = micro_op_.kind == MicroOpKind::CmpdExtended ? 5 : 4;
        if (micro_op_.step == high_address_step) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        if (micro_op_.step == low_address_step) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        if (micro_op_.step == high_data_step) return read_cycle(micro_op_.effective_address, BusCycleKind::OperandRead);
        if ((micro_op_.kind == MicroOpKind::LddExtended ||
             micro_op_.kind == MicroOpKind::Alu16Extended ||
             micro_op_.kind == MicroOpKind::CmpdExtended) &&
            micro_op_.step == low_data_step) {
            return read_cycle(static_cast<uint16_t>(micro_op_.effective_address + 1), BusCycleKind::OperandRead);
        }
        return internal_cycle(regs_.pc);
    }
    case MicroOpKind::Cmp16Extended: {
        const uint8_t high_address_step = micro_op_.prefix == 0x00 ? 1 : 2;
        const uint8_t low_address_step = micro_op_.prefix == 0x00 ? 2 : 3;
        const uint8_t high_data_step = micro_op_.prefix == 0x00 ? 3 : 4;
        const uint8_t low_data_step = micro_op_.prefix == 0x00 ? 4 : 5;
        if (micro_op_.step == 0) return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        if (micro_op_.prefix != 0x00 && micro_op_.step == 1) {
            return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        }
        if (micro_op_.step == high_address_step) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        if (micro_op_.step == low_address_step) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        if (micro_op_.step == high_data_step) return read_cycle(micro_op_.effective_address, BusCycleKind::OperandRead);
        if (micro_op_.step == low_data_step) {
            return read_cycle(static_cast<uint16_t>(micro_op_.effective_address + 1), BusCycleKind::OperandRead);
        }
        return internal_cycle(regs_.pc);
    }
    case MicroOpKind::WordLoadExtended: {
        const uint8_t high_address_step = micro_op_.prefix == 0x00 ? 1 : 2;
        const uint8_t low_address_step = micro_op_.prefix == 0x00 ? 2 : 3;
        const uint8_t high_data_step = micro_op_.prefix == 0x00 ? 3 : 4;
        const uint8_t low_data_step = micro_op_.prefix == 0x00 ? 4 : 5;
        if (micro_op_.step == 0) return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        if (micro_op_.prefix != 0x00 && micro_op_.step == 1) {
            return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        }
        if (micro_op_.step == high_address_step) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        if (micro_op_.step == low_address_step) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        if (micro_op_.step == high_data_step) return read_cycle(micro_op_.effective_address, BusCycleKind::OperandRead);
        if (micro_op_.step == low_data_step) {
            return read_cycle(static_cast<uint16_t>(micro_op_.effective_address + 1), BusCycleKind::OperandRead);
        }
        return internal_cycle(regs_.pc);
    }
    case MicroOpKind::WordStoreExtended: {
        const uint8_t high_address_step = micro_op_.prefix == 0x00 ? 1 : 2;
        const uint8_t low_address_step = micro_op_.prefix == 0x00 ? 2 : 3;
        const uint8_t high_data_step = micro_op_.prefix == 0x00 ? 3 : 4;
        const uint8_t low_data_step = micro_op_.prefix == 0x00 ? 4 : 5;
        const uint16_t value = word_register_value(regs_, micro_op_.prefix, micro_op_.opcode);
        if (micro_op_.step == 0) return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        if (micro_op_.prefix != 0x00 && micro_op_.step == 1) {
            return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        }
        if (micro_op_.step == high_address_step) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        if (micro_op_.step == low_address_step) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        if (micro_op_.step == high_data_step) {
            return write_cycle(micro_op_.effective_address, hi(value), BusCycleKind::OperandWrite);
        }
        if (micro_op_.step == low_data_step) {
            return write_cycle(static_cast<uint16_t>(micro_op_.effective_address + 1), lo(value), BusCycleKind::OperandWrite);
        }
        return internal_cycle(regs_.pc);
    }
    case MicroOpKind::MemoryUnaryDirect:
        if (micro_op_.step == 0) return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        if (micro_op_.step == 1) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        if ((micro_op_.opcode & 0x0F) == 0x0F && micro_op_.step == 2) {
            return write_cycle(micro_op_.effective_address, 0x00, BusCycleKind::OperandWrite);
        }
        if (micro_op_.step == 2) return read_cycle(micro_op_.effective_address, BusCycleKind::OperandRead);
        if ((micro_op_.opcode & 0x0F) != 0x0D && (micro_op_.opcode & 0x0F) != 0x0F && micro_op_.step == 3) {
            return write_cycle(micro_op_.effective_address, static_cast<uint8_t>(micro_op_.data & 0xFF), BusCycleKind::OperandWrite);
        }
        return internal_cycle(regs_.pc);
    case MicroOpKind::MemoryUnaryExtended:
        if (micro_op_.step == 0) return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        if (micro_op_.step == 1) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        if (micro_op_.step == 2) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        if ((micro_op_.opcode & 0x0F) == 0x0F && micro_op_.step == 3) {
            return write_cycle(micro_op_.effective_address, 0x00, BusCycleKind::OperandWrite);
        }
        if (micro_op_.step == 3) return read_cycle(micro_op_.effective_address, BusCycleKind::OperandRead);
        if ((micro_op_.opcode & 0x0F) != 0x0D && (micro_op_.opcode & 0x0F) != 0x0F && micro_op_.step == 4) {
            return write_cycle(micro_op_.effective_address, static_cast<uint8_t>(micro_op_.data & 0xFF), BusCycleKind::OperandWrite);
        }
        return internal_cycle(regs_.pc);
    case MicroOpKind::StaDirect:
    case MicroOpKind::StbDirect:
    case MicroOpKind::StdDirect:
        if (micro_op_.step == 0) return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        if (micro_op_.step == 1) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        if (micro_op_.step == 2) {
            const uint8_t value = micro_op_.kind == MicroOpKind::StbDirect ? regs_.b : regs_.a;
            return write_cycle(micro_op_.effective_address, value, BusCycleKind::OperandWrite);
        }
        if (micro_op_.kind == MicroOpKind::StdDirect && micro_op_.step == 3) {
            return write_cycle(static_cast<uint16_t>(micro_op_.effective_address + 1), regs_.b, BusCycleKind::OperandWrite);
        }
        return internal_cycle(regs_.pc);
    case MicroOpKind::StaExtended:
    case MicroOpKind::StbExtended:
    case MicroOpKind::StdExtended:
        if (micro_op_.step == 0) return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        if (micro_op_.step == 1) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        if (micro_op_.step == 2) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        if (micro_op_.step == 3) {
            const uint8_t value = micro_op_.kind == MicroOpKind::StbExtended ? regs_.b : regs_.a;
            return write_cycle(micro_op_.effective_address, value, BusCycleKind::OperandWrite);
        }
        if (micro_op_.kind == MicroOpKind::StdExtended && micro_op_.step == 4) {
            return write_cycle(static_cast<uint16_t>(micro_op_.effective_address + 1), regs_.b, BusCycleKind::OperandWrite);
        }
        return internal_cycle(regs_.pc);
    case MicroOpKind::Branch:
        if (micro_op_.step == 0) return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        if (micro_op_.step == 1) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        return internal_cycle(regs_.pc);
    case MicroOpKind::Lbra:
        if (micro_op_.step == 0) return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        if (micro_op_.step == 1) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        if (micro_op_.step == 2) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        return internal_cycle(regs_.pc);
    case MicroOpKind::LongBranch:
        if (micro_op_.step == 0) return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        if (micro_op_.step == 1) return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        if (micro_op_.step == 2) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        if (micro_op_.step == 3) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        return internal_cycle(regs_.pc);
    case MicroOpKind::Bsr:
        if (micro_op_.step == 0) return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        if (micro_op_.step == 1) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        if (micro_op_.step == 2) {
            return write_cycle(static_cast<uint16_t>(regs_.s - 1), lo(micro_op_.data), BusCycleKind::StackWrite);
        }
        if (micro_op_.step == 3) {
            return write_cycle(static_cast<uint16_t>(regs_.s - 1), hi(micro_op_.data), BusCycleKind::StackWrite);
        }
        return internal_cycle(regs_.pc);
    case MicroOpKind::Lbsr:
        if (micro_op_.step == 0) return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        if (micro_op_.step == 1) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        if (micro_op_.step == 2) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        if (micro_op_.step == 3) {
            return write_cycle(static_cast<uint16_t>(regs_.s - 1), lo(micro_op_.data), BusCycleKind::StackWrite);
        }
        if (micro_op_.step == 4) {
            return write_cycle(static_cast<uint16_t>(regs_.s - 1), hi(micro_op_.data), BusCycleKind::StackWrite);
        }
        return internal_cycle(regs_.pc);
    case MicroOpKind::Rts:
        if (micro_op_.step == 0) return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        if (micro_op_.step == 1) return read_cycle(regs_.s, BusCycleKind::StackRead);
        if (micro_op_.step == 2) return read_cycle(regs_.s, BusCycleKind::StackRead);
        return internal_cycle(regs_.pc);
    case MicroOpKind::StackPush: {
        if (micro_op_.step == 0) return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        if (micro_op_.step == 1) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        const uint8_t byte_index = static_cast<uint8_t>(micro_op_.step - 2);
        if (byte_index < stack_mask_byte_count(micro_op_.direct_offset)) {
            const bool use_u_stack = micro_op_.opcode == 0x36;
            const uint16_t pointer = use_u_stack ? regs_.u : regs_.s;
            return write_cycle(
                static_cast<uint16_t>(pointer - 1),
                stack_push_byte(regs_, micro_op_.direct_offset, byte_index, use_u_stack),
                BusCycleKind::StackWrite);
        }
        return internal_cycle(regs_.pc);
    }
    case MicroOpKind::StackPull: {
        if (micro_op_.step == 0) return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        if (micro_op_.step == 1) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        const uint8_t byte_index = static_cast<uint8_t>(micro_op_.step - 2);
        if (byte_index < stack_mask_byte_count(micro_op_.direct_offset)) {
            const bool use_u_stack = micro_op_.opcode == 0x37;
            const uint16_t pointer = use_u_stack ? regs_.u : regs_.s;
            return read_cycle(pointer, BusCycleKind::StackRead);
        }
        return internal_cycle(regs_.pc);
    }
    case MicroOpKind::JsrDirect:
        if (micro_op_.step == 0) return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        if (micro_op_.step == 1) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        if (micro_op_.step == 2) {
            return write_cycle(static_cast<uint16_t>(regs_.s - 1), lo(micro_op_.data), BusCycleKind::StackWrite);
        }
        if (micro_op_.step == 3) {
            return write_cycle(static_cast<uint16_t>(regs_.s - 1), hi(micro_op_.data), BusCycleKind::StackWrite);
        }
        return internal_cycle(regs_.pc);
    case MicroOpKind::JsrExtended:
        if (micro_op_.step == 0) return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        if (micro_op_.step == 1) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        if (micro_op_.step == 2) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        if (micro_op_.step == 3) {
            return write_cycle(static_cast<uint16_t>(regs_.s - 1), lo(micro_op_.data), BusCycleKind::StackWrite);
        }
        if (micro_op_.step == 4) {
            return write_cycle(static_cast<uint16_t>(regs_.s - 1), hi(micro_op_.data), BusCycleKind::StackWrite);
        }
        return internal_cycle(regs_.pc);
    case MicroOpKind::JmpDirect:
        if (micro_op_.step == 0) return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        if (micro_op_.step == 1) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        return internal_cycle(regs_.pc);
    case MicroOpKind::JmpExtended:
        if (micro_op_.step == 0) return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        if (micro_op_.step == 1) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        if (micro_op_.step == 2) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        return internal_cycle(regs_.pc);
    case MicroOpKind::Clra:
    case MicroOpKind::Clrb:
    case MicroOpKind::Tsta:
    case MicroOpKind::Tstb:
    case MicroOpKind::Deca:
    case MicroOpKind::Decb:
    case MicroOpKind::Inca:
    case MicroOpKind::Incb:
    case MicroOpKind::Coma:
    case MicroOpKind::Comb:
    case MicroOpKind::Nega:
    case MicroOpKind::Negb:
    case MicroOpKind::Lsra:
    case MicroOpKind::Lsrb:
    case MicroOpKind::Rora:
    case MicroOpKind::Rorb:
    case MicroOpKind::Asra:
    case MicroOpKind::Asrb:
    case MicroOpKind::Asla:
    case MicroOpKind::Aslb:
    case MicroOpKind::Rola:
    case MicroOpKind::Rolb:
        return micro_op_.step == 0
            ? read_cycle(regs_.pc, BusCycleKind::OpcodeFetch)
            : internal_cycle(regs_.pc);
    case MicroOpKind::None:
        break;
    }
    return {};
}

bool Cpu::prepare_microcycle(Bus& bus, BusSignals& signals, CpuMicrocycleStatus& status) {
    bool instruction_started = false;
    if (!has_pending_micro_ops()) {
        const uint8_t opcode = bus.peek8(regs_.pc);
        if (!start_micro_op(bus, opcode)) {
            return false;
        }
        instruction_started = true;
    }

    signals = micro_op_signals();
    status = micro_op_status(instruction_started, false);
    return true;
}

CpuMicrocycleStatus Cpu::complete_microcycle(const BusSignals& signals) {
    if (!has_pending_micro_ops()) {
        return {};
    }

    const uint8_t completed_step = micro_op_.step;
    switch (micro_op_.kind) {
    case MicroOpKind::Nop:
        if (completed_step == 0) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        }
        break;
    case MicroOpKind::MiscInherent:
        if (completed_step == 0) {
            switch (micro_op_.opcode) {
            case 0x19: {
                uint8_t adjust = 0;
                bool carry = (regs_.cc & CC_C) != 0;
                if ((regs_.a & 0x0F) > 9 || (regs_.cc & CC_H)) {
                    adjust |= 0x06;
                }
                if ((regs_.a > 0x99) || carry) {
                    adjust |= 0x60;
                    carry = true;
                }
                regs_.a = static_cast<uint8_t>(regs_.a + adjust);
                regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | CC_C));
                if (regs_.a & 0x80) regs_.cc |= CC_N;
                if (regs_.a == 0) regs_.cc |= CC_Z;
                if (carry) regs_.cc |= CC_C;
                break;
            }
            case 0x1D:
                regs_.b = (regs_.a & 0x80) ? 0xFF : 0x00;
                set_flags_nz16(static_cast<uint16_t>((static_cast<uint16_t>(regs_.a) << 8) | regs_.b));
                break;
            case 0x3A:
                regs_.x = static_cast<uint16_t>(regs_.x + regs_.b);
                break;
            case 0x3D: {
                const uint16_t result = static_cast<uint16_t>(regs_.a) * static_cast<uint16_t>(regs_.b);
                regs_.a = hi(result);
                regs_.b = lo(result);
                regs_.cc &= static_cast<uint8_t>(~(CC_Z | CC_C | CC_V | CC_N));
                if (result == 0) regs_.cc |= CC_Z;
                if (result & 0x8000) regs_.cc |= CC_N;
                if (result & 0x80) regs_.cc |= CC_C;
                break;
            }
            default:
                break;
            }
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        }
        break;
    case MicroOpKind::CcImmediate:
        if (completed_step == 0) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 1) {
            if (micro_op_.opcode == 0x1A) {
                regs_.cc |= signals.data;
            } else {
                regs_.cc &= signals.data;
            }
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        }
        break;
    case MicroOpKind::RegisterTransfer:
        if (completed_step == 0) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 1) {
            const uint8_t src = static_cast<uint8_t>(signals.data >> 4);
            const uint8_t dst = static_cast<uint8_t>(signals.data & 0x0F);
            if (micro_op_.opcode == 0x1F) {
                const bool dest16 = register_code_is_16bit(dst);
                const uint16_t value = read_reg_for_dest(regs_, src, dest16);
                write_reg_sized(*this, dst, value, dest16);
            } else {
                const bool wide = register_code_is_16bit(src) || register_code_is_16bit(dst);
                const uint16_t left = read_reg_for_dest(regs_, src, wide);
                const uint16_t right = read_reg_for_dest(regs_, dst, wide);
                write_reg_sized(*this, src, right, wide);
                write_reg_sized(*this, dst, left, wide);
            }
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        }
        break;
    case MicroOpKind::LdaImmediate:
    case MicroOpKind::LdbImmediate:
    case MicroOpKind::LddImmediate:
        if (completed_step == 0) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 1) {
            if (micro_op_.kind == MicroOpKind::LddImmediate) {
                micro_op_.data = static_cast<uint16_t>(static_cast<uint16_t>(signals.data) << 8);
            } else if (micro_op_.kind == MicroOpKind::LdaImmediate) {
                regs_.a = signals.data;
                set_flags_nz8(regs_.a);
            } else {
                regs_.b = signals.data;
                set_flags_nz8(regs_.b);
            }
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 2) {
            micro_op_.data = static_cast<uint16_t>(micro_op_.data | signals.data);
            regs_.a = static_cast<uint8_t>((micro_op_.data >> 8) & 0xFF);
            regs_.b = static_cast<uint8_t>(micro_op_.data & 0xFF);
            set_flags_nz16(micro_op_.data);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        }
        break;
    case MicroOpKind::Alu8Immediate:
        if (completed_step == 0) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 1) {
            apply_alu8_immediate(regs_, micro_op_.opcode, signals.data);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        }
        break;
    case MicroOpKind::Alu16Immediate:
        if (completed_step == 0) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 1) {
            micro_op_.data = static_cast<uint16_t>(static_cast<uint16_t>(signals.data) << 8);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 2) {
            micro_op_.data = static_cast<uint16_t>(micro_op_.data | signals.data);
            apply_alu16(regs_, micro_op_.opcode, micro_op_.data);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        }
        break;
    case MicroOpKind::CmpdImmediate:
        if (completed_step == 0) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 1) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 2) {
            micro_op_.data = static_cast<uint16_t>(static_cast<uint16_t>(signals.data) << 8);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 3) {
            micro_op_.data = static_cast<uint16_t>(micro_op_.data | signals.data);
            apply_cmpd(regs_, micro_op_.data);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        }
        break;
    case MicroOpKind::Cmp16Immediate: {
        const uint8_t prefix_step = micro_op_.prefix == 0x00 ? 0xFF : 1;
        const uint8_t high_step = micro_op_.prefix == 0x00 ? 1 : 2;
        const uint8_t low_step = micro_op_.prefix == 0x00 ? 2 : 3;
        if (completed_step == 0 || completed_step == prefix_step) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == high_step) {
            micro_op_.data = static_cast<uint16_t>(static_cast<uint16_t>(signals.data) << 8);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == low_step) {
            micro_op_.data = static_cast<uint16_t>(micro_op_.data | signals.data);
            apply_cmp16(regs_, micro_op_.prefix, micro_op_.opcode, micro_op_.data);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        }
        break;
    }
    case MicroOpKind::WordLoadImmediate: {
        const uint8_t prefix_step = micro_op_.prefix == 0x00 ? 0xFF : 1;
        const uint8_t high_step = micro_op_.prefix == 0x00 ? 1 : 2;
        const uint8_t low_step = micro_op_.prefix == 0x00 ? 2 : 3;
        if (completed_step == 0 || completed_step == prefix_step) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == high_step) {
            micro_op_.data = static_cast<uint16_t>(static_cast<uint16_t>(signals.data) << 8);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == low_step) {
            micro_op_.data = static_cast<uint16_t>(micro_op_.data | signals.data);
            set_word_register(regs_, micro_op_.prefix, micro_op_.opcode, micro_op_.data);
            set_flags_nz16(micro_op_.data);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        }
        break;
    }
    case MicroOpKind::Alu8Direct:
        if (completed_step == 0) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 1) {
            micro_op_.direct_offset = signals.data;
            micro_op_.effective_address = static_cast<uint16_t>((static_cast<uint16_t>(regs_.dp) << 8) | signals.data);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 2) {
            apply_alu8_immediate(regs_, micro_op_.opcode, signals.data);
        }
        break;
    case MicroOpKind::Alu16Direct:
        if (completed_step == 0) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 1) {
            micro_op_.direct_offset = signals.data;
            micro_op_.effective_address = static_cast<uint16_t>((static_cast<uint16_t>(regs_.dp) << 8) | signals.data);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 2) {
            micro_op_.data = static_cast<uint16_t>(static_cast<uint16_t>(signals.data) << 8);
        } else if (completed_step == 3) {
            micro_op_.data = static_cast<uint16_t>(micro_op_.data | signals.data);
            apply_alu16(regs_, micro_op_.opcode, micro_op_.data);
        }
        break;
    case MicroOpKind::CmpdDirect:
        if (completed_step == 0) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 1) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 2) {
            micro_op_.direct_offset = signals.data;
            micro_op_.effective_address = static_cast<uint16_t>((static_cast<uint16_t>(regs_.dp) << 8) | signals.data);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 3) {
            micro_op_.data = static_cast<uint16_t>(static_cast<uint16_t>(signals.data) << 8);
        } else if (completed_step == 4) {
            micro_op_.data = static_cast<uint16_t>(micro_op_.data | signals.data);
            apply_cmpd(regs_, micro_op_.data);
        }
        break;
    case MicroOpKind::Cmp16Direct: {
        const uint8_t prefix_step = micro_op_.prefix == 0x00 ? 0xFF : 1;
        const uint8_t operand_step = micro_op_.prefix == 0x00 ? 1 : 2;
        const uint8_t high_step = micro_op_.prefix == 0x00 ? 2 : 3;
        const uint8_t low_step = micro_op_.prefix == 0x00 ? 3 : 4;
        if (completed_step == 0 || completed_step == prefix_step) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == operand_step) {
            micro_op_.direct_offset = signals.data;
            micro_op_.effective_address = static_cast<uint16_t>((static_cast<uint16_t>(regs_.dp) << 8) | signals.data);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == high_step) {
            micro_op_.data = static_cast<uint16_t>(static_cast<uint16_t>(signals.data) << 8);
        } else if (completed_step == low_step) {
            micro_op_.data = static_cast<uint16_t>(micro_op_.data | signals.data);
            apply_cmp16(regs_, micro_op_.prefix, micro_op_.opcode, micro_op_.data);
        }
        break;
    }
    case MicroOpKind::WordLoadDirect:
    case MicroOpKind::WordStoreDirect: {
        const uint8_t prefix_step = micro_op_.prefix == 0x00 ? 0xFF : 1;
        const uint8_t operand_step = micro_op_.prefix == 0x00 ? 1 : 2;
        const uint8_t high_step = micro_op_.prefix == 0x00 ? 2 : 3;
        const uint8_t low_step = micro_op_.prefix == 0x00 ? 3 : 4;
        if (completed_step == 0 || completed_step == prefix_step) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == operand_step) {
            micro_op_.direct_offset = signals.data;
            micro_op_.effective_address = static_cast<uint16_t>((static_cast<uint16_t>(regs_.dp) << 8) | signals.data);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == high_step && micro_op_.kind == MicroOpKind::WordLoadDirect) {
            micro_op_.data = static_cast<uint16_t>(static_cast<uint16_t>(signals.data) << 8);
        } else if (completed_step == low_step) {
            if (micro_op_.kind == MicroOpKind::WordLoadDirect) {
                micro_op_.data = static_cast<uint16_t>(micro_op_.data | signals.data);
                set_word_register(regs_, micro_op_.prefix, micro_op_.opcode, micro_op_.data);
                set_flags_nz16(micro_op_.data);
            } else {
                set_flags_nz16(word_register_value(regs_, micro_op_.prefix, micro_op_.opcode));
            }
        }
        break;
    }
    case MicroOpKind::LdaDirect:
    case MicroOpKind::LdbDirect:
    case MicroOpKind::LddDirect:
        if (completed_step == 0) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 1) {
            micro_op_.direct_offset = signals.data;
            micro_op_.effective_address = static_cast<uint16_t>((static_cast<uint16_t>(regs_.dp) << 8) | signals.data);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 2) {
            if (micro_op_.kind == MicroOpKind::LddDirect) {
                micro_op_.data = static_cast<uint16_t>(static_cast<uint16_t>(signals.data) << 8);
            } else if (micro_op_.kind == MicroOpKind::LdaDirect) {
                regs_.a = signals.data;
                set_flags_nz8(regs_.a);
            } else {
                regs_.b = signals.data;
                set_flags_nz8(regs_.b);
            }
        } else if (completed_step == 3 && micro_op_.kind == MicroOpKind::LddDirect) {
            micro_op_.data = static_cast<uint16_t>(micro_op_.data | signals.data);
            regs_.a = static_cast<uint8_t>((micro_op_.data >> 8) & 0xFF);
            regs_.b = static_cast<uint8_t>(micro_op_.data & 0xFF);
            set_flags_nz16(micro_op_.data);
        }
        break;
    case MicroOpKind::LdaExtended:
    case MicroOpKind::LdbExtended:
    case MicroOpKind::LddExtended:
    case MicroOpKind::Alu8Extended:
    case MicroOpKind::Alu16Extended:
        if (completed_step == 0) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 1) {
            micro_op_.effective_address = static_cast<uint16_t>(static_cast<uint16_t>(signals.data) << 8);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 2) {
            micro_op_.effective_address = static_cast<uint16_t>(micro_op_.effective_address | signals.data);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 3) {
            if (micro_op_.kind == MicroOpKind::Alu8Extended) {
                apply_alu8_immediate(regs_, micro_op_.opcode, signals.data);
            } else if (micro_op_.kind == MicroOpKind::Alu16Extended) {
                micro_op_.data = static_cast<uint16_t>(static_cast<uint16_t>(signals.data) << 8);
            } else if (micro_op_.kind == MicroOpKind::LddExtended) {
                micro_op_.data = static_cast<uint16_t>(static_cast<uint16_t>(signals.data) << 8);
            } else if (micro_op_.kind == MicroOpKind::LdaExtended) {
                regs_.a = signals.data;
                set_flags_nz8(regs_.a);
            } else {
                regs_.b = signals.data;
                set_flags_nz8(regs_.b);
            }
        } else if (completed_step == 4) {
            if (micro_op_.kind == MicroOpKind::Alu16Extended) {
                micro_op_.data = static_cast<uint16_t>(micro_op_.data | signals.data);
                apply_alu16(regs_, micro_op_.opcode, micro_op_.data);
            } else if (micro_op_.kind == MicroOpKind::LddExtended) {
                micro_op_.data = static_cast<uint16_t>(micro_op_.data | signals.data);
                regs_.a = static_cast<uint8_t>((micro_op_.data >> 8) & 0xFF);
                regs_.b = static_cast<uint8_t>(micro_op_.data & 0xFF);
                set_flags_nz16(micro_op_.data);
            }
        }
        break;
    case MicroOpKind::CmpdExtended:
        if (completed_step == 0) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 1) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 2) {
            micro_op_.effective_address = static_cast<uint16_t>(static_cast<uint16_t>(signals.data) << 8);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 3) {
            micro_op_.effective_address = static_cast<uint16_t>(micro_op_.effective_address | signals.data);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 4) {
            micro_op_.data = static_cast<uint16_t>(static_cast<uint16_t>(signals.data) << 8);
        } else if (completed_step == 5) {
            micro_op_.data = static_cast<uint16_t>(micro_op_.data | signals.data);
            apply_cmpd(regs_, micro_op_.data);
        }
        break;
    case MicroOpKind::Cmp16Extended: {
        const uint8_t prefix_step = micro_op_.prefix == 0x00 ? 0xFF : 1;
        const uint8_t high_address_step = micro_op_.prefix == 0x00 ? 1 : 2;
        const uint8_t low_address_step = micro_op_.prefix == 0x00 ? 2 : 3;
        const uint8_t high_data_step = micro_op_.prefix == 0x00 ? 3 : 4;
        const uint8_t low_data_step = micro_op_.prefix == 0x00 ? 4 : 5;
        if (completed_step == 0 || completed_step == prefix_step) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == high_address_step) {
            micro_op_.effective_address = static_cast<uint16_t>(static_cast<uint16_t>(signals.data) << 8);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == low_address_step) {
            micro_op_.effective_address = static_cast<uint16_t>(micro_op_.effective_address | signals.data);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == high_data_step) {
            micro_op_.data = static_cast<uint16_t>(static_cast<uint16_t>(signals.data) << 8);
        } else if (completed_step == low_data_step) {
            micro_op_.data = static_cast<uint16_t>(micro_op_.data | signals.data);
            apply_cmp16(regs_, micro_op_.prefix, micro_op_.opcode, micro_op_.data);
        }
        break;
    }
    case MicroOpKind::WordLoadExtended:
    case MicroOpKind::WordStoreExtended: {
        const uint8_t prefix_step = micro_op_.prefix == 0x00 ? 0xFF : 1;
        const uint8_t high_address_step = micro_op_.prefix == 0x00 ? 1 : 2;
        const uint8_t low_address_step = micro_op_.prefix == 0x00 ? 2 : 3;
        const uint8_t high_data_step = micro_op_.prefix == 0x00 ? 3 : 4;
        const uint8_t low_data_step = micro_op_.prefix == 0x00 ? 4 : 5;
        if (completed_step == 0 || completed_step == prefix_step) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == high_address_step) {
            micro_op_.effective_address = static_cast<uint16_t>(static_cast<uint16_t>(signals.data) << 8);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == low_address_step) {
            micro_op_.effective_address = static_cast<uint16_t>(micro_op_.effective_address | signals.data);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == high_data_step && micro_op_.kind == MicroOpKind::WordLoadExtended) {
            micro_op_.data = static_cast<uint16_t>(static_cast<uint16_t>(signals.data) << 8);
        } else if (completed_step == low_data_step) {
            if (micro_op_.kind == MicroOpKind::WordLoadExtended) {
                micro_op_.data = static_cast<uint16_t>(micro_op_.data | signals.data);
                set_word_register(regs_, micro_op_.prefix, micro_op_.opcode, micro_op_.data);
                set_flags_nz16(micro_op_.data);
            } else {
                set_flags_nz16(word_register_value(regs_, micro_op_.prefix, micro_op_.opcode));
            }
        }
        break;
    }
    case MicroOpKind::MemoryUnaryDirect:
        if (completed_step == 0) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 1) {
            micro_op_.direct_offset = signals.data;
            micro_op_.effective_address = static_cast<uint16_t>((static_cast<uint16_t>(regs_.dp) << 8) | signals.data);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 2) {
            if ((micro_op_.opcode & 0x0F) == 0x0F) {
                regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_V | CC_C));
                regs_.cc |= CC_Z;
            } else if ((micro_op_.opcode & 0x0F) == 0x0D) {
                apply_tst8(regs_, signals.data);
            } else {
                micro_op_.data = apply_memory_unary(regs_, micro_op_.opcode, signals.data);
            }
        }
        break;
    case MicroOpKind::MemoryUnaryExtended:
        if (completed_step == 0) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 1) {
            micro_op_.effective_address = static_cast<uint16_t>(static_cast<uint16_t>(signals.data) << 8);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 2) {
            micro_op_.effective_address = static_cast<uint16_t>(micro_op_.effective_address | signals.data);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 3) {
            if ((micro_op_.opcode & 0x0F) == 0x0F) {
                regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_V | CC_C));
                regs_.cc |= CC_Z;
            } else if ((micro_op_.opcode & 0x0F) == 0x0D) {
                apply_tst8(regs_, signals.data);
            } else {
                micro_op_.data = apply_memory_unary(regs_, micro_op_.opcode, signals.data);
            }
        }
        break;
    case MicroOpKind::StaDirect:
    case MicroOpKind::StbDirect:
    case MicroOpKind::StdDirect:
        if (completed_step == 0) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 1) {
            micro_op_.direct_offset = signals.data;
            micro_op_.effective_address = static_cast<uint16_t>((static_cast<uint16_t>(regs_.dp) << 8) | signals.data);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 2 && micro_op_.kind != MicroOpKind::StdDirect) {
            const uint8_t value = micro_op_.kind == MicroOpKind::StaDirect ? regs_.a : regs_.b;
            set_flags_nz8(value);
        } else if (completed_step == 3 && micro_op_.kind == MicroOpKind::StdDirect) {
            set_flags_nz16(static_cast<uint16_t>((static_cast<uint16_t>(regs_.a) << 8) | regs_.b));
        }
        break;
    case MicroOpKind::StaExtended:
    case MicroOpKind::StbExtended:
    case MicroOpKind::StdExtended:
        if (completed_step == 0) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 1) {
            micro_op_.effective_address = static_cast<uint16_t>(static_cast<uint16_t>(signals.data) << 8);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 2) {
            micro_op_.effective_address = static_cast<uint16_t>(micro_op_.effective_address | signals.data);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 3 && micro_op_.kind != MicroOpKind::StdExtended) {
            const uint8_t value = micro_op_.kind == MicroOpKind::StaExtended ? regs_.a : regs_.b;
            set_flags_nz8(value);
        } else if (completed_step == 4 && micro_op_.kind == MicroOpKind::StdExtended) {
            set_flags_nz16(static_cast<uint16_t>((static_cast<uint16_t>(regs_.a) << 8) | regs_.b));
        }
        break;
    case MicroOpKind::Branch:
        if (completed_step == 0) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 1) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
            if (micro_op_.branch_taken) {
                regs_.pc = static_cast<uint16_t>(regs_.pc + static_cast<int8_t>(signals.data));
            }
        }
        break;
    case MicroOpKind::Lbra:
        if (completed_step == 0) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 1) {
            micro_op_.data = static_cast<uint16_t>(static_cast<uint16_t>(signals.data) << 8);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 2) {
            const uint16_t offset = static_cast<uint16_t>(micro_op_.data | signals.data);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
            regs_.pc = static_cast<uint16_t>(regs_.pc + sign_extend16(offset));
        }
        break;
    case MicroOpKind::LongBranch:
        if (completed_step == 0 || completed_step == 1) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 2) {
            micro_op_.data = static_cast<uint16_t>(static_cast<uint16_t>(signals.data) << 8);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 3) {
            const uint16_t offset = static_cast<uint16_t>(micro_op_.data | signals.data);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
            if (micro_op_.branch_taken) {
                regs_.pc = static_cast<uint16_t>(regs_.pc + sign_extend16(offset));
            }
        }
        break;
    case MicroOpKind::Bsr:
        if (completed_step == 0) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 1) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
            micro_op_.data = regs_.pc;
            micro_op_.effective_address = static_cast<uint16_t>(regs_.pc + static_cast<int8_t>(signals.data));
        } else if (completed_step == 2) {
            regs_.s = static_cast<uint16_t>(regs_.s - 1);
        } else if (completed_step == 3) {
            regs_.s = static_cast<uint16_t>(regs_.s - 1);
            regs_.pc = micro_op_.effective_address;
        }
        break;
    case MicroOpKind::Lbsr:
        if (completed_step == 0) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 1) {
            micro_op_.data = static_cast<uint16_t>(static_cast<uint16_t>(signals.data) << 8);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 2) {
            const uint16_t offset = static_cast<uint16_t>(micro_op_.data | signals.data);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
            micro_op_.data = regs_.pc;
            micro_op_.effective_address = static_cast<uint16_t>(regs_.pc + sign_extend16(offset));
        } else if (completed_step == 3) {
            regs_.s = static_cast<uint16_t>(regs_.s - 1);
        } else if (completed_step == 4) {
            regs_.s = static_cast<uint16_t>(regs_.s - 1);
            regs_.pc = micro_op_.effective_address;
        }
        break;
    case MicroOpKind::Rts:
        if (completed_step == 0) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 1) {
            micro_op_.data = static_cast<uint16_t>(static_cast<uint16_t>(signals.data) << 8);
            regs_.s = static_cast<uint16_t>(regs_.s + 1);
        } else if (completed_step == 2) {
            micro_op_.data = static_cast<uint16_t>(micro_op_.data | signals.data);
            regs_.s = static_cast<uint16_t>(regs_.s + 1);
            regs_.pc = micro_op_.data;
        }
        break;
    case MicroOpKind::StackPush:
        if (completed_step == 0) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 1) {
            micro_op_.direct_offset = signals.data;
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else {
            const uint8_t byte_index = static_cast<uint8_t>(completed_step - 2);
            if (byte_index < stack_mask_byte_count(micro_op_.direct_offset)) {
                if (micro_op_.opcode == 0x36) {
                    regs_.u = static_cast<uint16_t>(regs_.u - 1);
                } else {
                    regs_.s = static_cast<uint16_t>(regs_.s - 1);
                }
            }
        }
        break;
    case MicroOpKind::StackPull:
        if (completed_step == 0) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 1) {
            micro_op_.direct_offset = signals.data;
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else {
            const uint8_t byte_index = static_cast<uint8_t>(completed_step - 2);
            if (byte_index < stack_mask_byte_count(micro_op_.direct_offset)) {
                const bool use_u_stack = micro_op_.opcode == 0x37;
                apply_stack_pull_byte(regs_, micro_op_.direct_offset, byte_index, use_u_stack, signals.data, micro_op_.data);
                if (use_u_stack) {
                    regs_.u = static_cast<uint16_t>(regs_.u + 1);
                } else {
                    regs_.s = static_cast<uint16_t>(regs_.s + 1);
                }
            }
        }
        break;
    case MicroOpKind::JsrDirect:
        if (completed_step == 0) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 1) {
            micro_op_.direct_offset = signals.data;
            micro_op_.effective_address = static_cast<uint16_t>((static_cast<uint16_t>(regs_.dp) << 8) | signals.data);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
            micro_op_.data = regs_.pc;
        } else if (completed_step == 2) {
            regs_.s = static_cast<uint16_t>(regs_.s - 1);
        } else if (completed_step == 3) {
            regs_.s = static_cast<uint16_t>(regs_.s - 1);
            regs_.pc = micro_op_.effective_address;
        }
        break;
    case MicroOpKind::JsrExtended:
        if (completed_step == 0) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 1) {
            micro_op_.effective_address = static_cast<uint16_t>(static_cast<uint16_t>(signals.data) << 8);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 2) {
            micro_op_.effective_address = static_cast<uint16_t>(micro_op_.effective_address | signals.data);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
            micro_op_.data = regs_.pc;
        } else if (completed_step == 3) {
            regs_.s = static_cast<uint16_t>(regs_.s - 1);
        } else if (completed_step == 4) {
            regs_.s = static_cast<uint16_t>(regs_.s - 1);
            regs_.pc = micro_op_.effective_address;
        }
        break;
    case MicroOpKind::JmpDirect:
        if (completed_step == 0) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 1) {
            micro_op_.effective_address = static_cast<uint16_t>((static_cast<uint16_t>(regs_.dp) << 8) | signals.data);
            regs_.pc = micro_op_.effective_address;
        }
        break;
    case MicroOpKind::JmpExtended:
        if (completed_step == 0) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 1) {
            micro_op_.effective_address = static_cast<uint16_t>(static_cast<uint16_t>(signals.data) << 8);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 2) {
            micro_op_.effective_address = static_cast<uint16_t>(micro_op_.effective_address | signals.data);
            regs_.pc = micro_op_.effective_address;
        }
        break;
    case MicroOpKind::Clra:
    case MicroOpKind::Clrb:
        if (completed_step == 0) {
            if (micro_op_.kind == MicroOpKind::Clra) {
                regs_.a = 0;
            } else {
                regs_.b = 0;
            }
            regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_V | CC_Z));
            regs_.cc |= CC_Z;
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        }
        break;
    case MicroOpKind::Tsta:
    case MicroOpKind::Tstb:
        if (completed_step == 0) {
            const uint8_t value = micro_op_.kind == MicroOpKind::Tsta ? regs_.a : regs_.b;
            regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | CC_C));
            if (value == 0) {
                regs_.cc |= CC_Z;
            }
            if (value & 0x80) {
                regs_.cc |= CC_N;
            }
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        }
        break;
    case MicroOpKind::Deca:
    case MicroOpKind::Decb:
    case MicroOpKind::Inca:
    case MicroOpKind::Incb:
        if (completed_step == 0) {
            uint8_t& target =
                (micro_op_.kind == MicroOpKind::Deca || micro_op_.kind == MicroOpKind::Inca) ? regs_.a : regs_.b;
            const bool increment = micro_op_.kind == MicroOpKind::Inca || micro_op_.kind == MicroOpKind::Incb;
            const uint8_t result = static_cast<uint8_t>(target + (increment ? 1 : -1));
            target = result;
            regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V));
            if (result == 0) {
                regs_.cc |= CC_Z;
            }
            if (result & 0x80) {
                regs_.cc |= CC_N;
            }
            if ((increment && result == 0x80) || (!increment && result == 0x7F)) {
                regs_.cc |= CC_V;
            }
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        }
        break;
    case MicroOpKind::Coma:
    case MicroOpKind::Comb:
        if (completed_step == 0) {
            uint8_t& target = micro_op_.kind == MicroOpKind::Coma ? regs_.a : regs_.b;
            target = static_cast<uint8_t>(~target);
            regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V));
            regs_.cc |= CC_C;
            if (target == 0) {
                regs_.cc |= CC_Z;
            }
            if (target & 0x80) {
                regs_.cc |= CC_N;
            }
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        }
        break;
    case MicroOpKind::Nega:
    case MicroOpKind::Negb:
        if (completed_step == 0) {
            uint8_t& target = micro_op_.kind == MicroOpKind::Nega ? regs_.a : regs_.b;
            const uint8_t operand = target;
            target = static_cast<uint8_t>(0u - operand);
            regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | CC_C));
            if (target == 0) {
                regs_.cc |= CC_Z;
            }
            if (target & 0x80) {
                regs_.cc |= CC_N;
            }
            if (operand == 0x80) {
                regs_.cc |= CC_V;
            }
            if (operand != 0) {
                regs_.cc |= CC_C;
            }
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        }
        break;
    case MicroOpKind::Lsra:
    case MicroOpKind::Lsrb:
        if (completed_step == 0) {
            uint8_t& target = micro_op_.kind == MicroOpKind::Lsra ? regs_.a : regs_.b;
            const uint8_t carry = static_cast<uint8_t>(target & 0x01);
            target = static_cast<uint8_t>(target >> 1);
            regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | CC_C));
            if (carry != 0) {
                regs_.cc |= CC_C;
            }
            if (target == 0) {
                regs_.cc |= CC_Z;
            }
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        }
        break;
    case MicroOpKind::Rora:
    case MicroOpKind::Rorb:
        if (completed_step == 0) {
            uint8_t& target = micro_op_.kind == MicroOpKind::Rora ? regs_.a : regs_.b;
            const uint8_t carry_in = (regs_.cc & CC_C) ? 0x80 : 0x00;
            const uint8_t carry_out = static_cast<uint8_t>(target & 0x01);
            target = static_cast<uint8_t>((target >> 1) | carry_in);
            regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | CC_C));
            if (target & 0x80) {
                regs_.cc |= CC_N;
            }
            if (target == 0) {
                regs_.cc |= CC_Z;
            }
            if (carry_out != 0) {
                regs_.cc |= CC_C;
            }
            if (((target ^ (carry_out ? 0x80 : 0x00)) & 0x80) != 0) {
                regs_.cc |= CC_V;
            }
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        }
        break;
    case MicroOpKind::Asra:
    case MicroOpKind::Asrb:
        if (completed_step == 0) {
            uint8_t& target = micro_op_.kind == MicroOpKind::Asra ? regs_.a : regs_.b;
            const uint8_t carry_out = static_cast<uint8_t>(target & 0x01);
            target = static_cast<uint8_t>((target >> 1) | (target & 0x80));
            regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | CC_C));
            if (target & 0x80) {
                regs_.cc |= CC_N;
            }
            if (target == 0) {
                regs_.cc |= CC_Z;
            }
            if (carry_out != 0) {
                regs_.cc |= CC_C;
            }
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        }
        break;
    case MicroOpKind::Asla:
    case MicroOpKind::Aslb:
        if (completed_step == 0) {
            uint8_t& target = micro_op_.kind == MicroOpKind::Asla ? regs_.a : regs_.b;
            const uint8_t carry_out = static_cast<uint8_t>((target >> 7) & 0x01);
            target = static_cast<uint8_t>(target << 1);
            regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | CC_C));
            if (target & 0x80) {
                regs_.cc |= CC_N;
            }
            if (target == 0) {
                regs_.cc |= CC_Z;
            }
            if (carry_out != 0) {
                regs_.cc |= CC_C;
            }
            if (((target ^ (carry_out ? 0x80 : 0x00)) & 0x80) != 0) {
                regs_.cc |= CC_V;
            }
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        }
        break;
    case MicroOpKind::Rola:
    case MicroOpKind::Rolb:
        if (completed_step == 0) {
            uint8_t& target = micro_op_.kind == MicroOpKind::Rola ? regs_.a : regs_.b;
            const uint8_t carry_in = (regs_.cc & CC_C) ? 1 : 0;
            const uint8_t carry_out = static_cast<uint8_t>((target >> 7) & 0x01);
            target = static_cast<uint8_t>((target << 1) | carry_in);
            regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | CC_C));
            if (target & 0x80) {
                regs_.cc |= CC_N;
            }
            if (target == 0) {
                regs_.cc |= CC_Z;
            }
            if (carry_out != 0) {
                regs_.cc |= CC_C;
            }
            if (((target ^ (carry_out ? 0x80 : 0x00)) & 0x80) != 0) {
                regs_.cc |= CC_V;
            }
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        }
        break;
    case MicroOpKind::None:
        break;
    }

    micro_op_.step = static_cast<uint8_t>(micro_op_.step + 1);
    const bool complete = micro_op_.step >= micro_op_.total_cycles;
    const CpuMicrocycleStatus status = micro_op_status(false, complete);
    if (complete) {
        cycles_executed_ += micro_op_.total_cycles;
        micro_op_ = {};
    }
    return status;
}

uint8_t Cpu::fetch_opcode_byte(Bus& bus) {
    return fetch_byte(bus, BusCycleKind::OpcodeFetch);
}

uint8_t Cpu::fetch_byte(Bus& bus) {
    return fetch_byte(bus, BusCycleKind::OperandRead);
}

uint8_t Cpu::fetch_byte(Bus& bus, BusCycleKind cycle_kind) {
    const uint8_t value = bus.read8(regs_.pc, cycle_kind);
    regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
    return value;
}

uint16_t Cpu::fetch_word(Bus& bus) {
    return fetch_word(bus, BusCycleKind::OperandRead);
}

uint16_t Cpu::fetch_word(Bus& bus, BusCycleKind cycle_kind) {
    const uint16_t high = fetch_byte(bus, cycle_kind);
    const uint16_t low = fetch_byte(bus, cycle_kind);
    return static_cast<uint16_t>((high << 8) | low);
}

uint8_t Cpu::read_byte(Bus& bus, uint16_t address) { return bus.read8(address, BusCycleKind::OperandRead); }

void Cpu::write_byte(Bus& bus, uint16_t address, uint8_t value) {
    bus.write8(address, value, BusCycleKind::OperandWrite);
}

uint8_t Cpu::read_stack_byte(Bus& bus, uint16_t address) {
    return bus.read8(address, BusCycleKind::StackRead);
}

void Cpu::write_stack_byte(Bus& bus, uint16_t address, uint8_t value) {
    bus.write8(address, value, BusCycleKind::StackWrite);
}

static inline uint16_t read_word(
    Bus& bus,
    uint16_t address,
    BusCycleKind cycle_kind = BusCycleKind::OperandRead) {
    return static_cast<uint16_t>(
        (bus.read8(address, cycle_kind) << 8) |
        bus.read8(static_cast<uint16_t>(address + 1), cycle_kind));
}

static inline void write_word(
    Bus& bus,
    uint16_t address,
    uint16_t value,
    BusCycleKind cycle_kind = BusCycleKind::OperandWrite) {
    bus.write8(address, static_cast<uint8_t>((value >> 8) & 0xFF), cycle_kind);
    bus.write8(static_cast<uint16_t>(address + 1), static_cast<uint8_t>(value & 0xFF), cycle_kind);
}

static constexpr uint16_t VECTOR_SWI = 0xFFFA;
static constexpr uint16_t VECTOR_SWI2 = 0xFFF4;
static constexpr uint16_t VECTOR_SWI3 = 0xFFF2;

uint16_t Cpu::direct_address(Bus& bus) {
    const uint8_t offset = fetch_byte(bus);
    return static_cast<uint16_t>((static_cast<uint16_t>(regs_.dp) << 8) | offset);
}

uint16_t Cpu::extended_address(Bus& bus) { return fetch_word(bus); }

Cpu::PostbyteResult Cpu::unsupported_indexed_address(Bus& bus) {
    if (mode_ == CpuMode::HD6309) {
        regs_.md |= 0x40;
        regs_.pc = read_word(bus, 0xFFF0, BusCycleKind::VectorRead);
    }
    return {0, 1, false};
}

Cpu::PostbyteResult Cpu::indexed_address(Bus& bus) {
    const uint8_t post = fetch_byte(bus);
    const bool native_mode = mode_ == CpuMode::HD6309 && (regs_.md & 0x01) != 0;

    if (mode_ == CpuMode::HD6309) {
        switch (post) {
        case 0x8F: // ,W
            return {reg_w(), 0};
        case 0xAF: { // n,W
            const uint16_t offset = fetch_word(bus);
            return {static_cast<uint16_t>(reg_w() + offset), static_cast<uint8_t>(native_mode ? 2 : 5)};
        }
        case 0xCF: { // ,W++
            const uint16_t addr = reg_w();
            set_reg_w(static_cast<uint16_t>(reg_w() + 2));
            return {addr, static_cast<uint8_t>(native_mode ? 1 : 3)};
        }
        case 0xEF: // ,--W
            set_reg_w(static_cast<uint16_t>(reg_w() - 2));
            return {reg_w(), static_cast<uint8_t>(native_mode ? 1 : 3)};
        case 0x90: // [,W]
            return {read_word(bus, reg_w()), 0};
        case 0xB0: { // [n,W]
            const uint16_t offset = fetch_word(bus);
            return {read_word(bus, static_cast<uint16_t>(reg_w() + offset)), 5};
        }
        case 0xD0: { // [,W++]
            const uint16_t addr = read_word(bus, reg_w());
            set_reg_w(static_cast<uint16_t>(reg_w() + 2));
            return {addr, 3};
        }
        case 0xF0: // [,--W]
            set_reg_w(static_cast<uint16_t>(reg_w() - 2));
            return {read_word(bus, reg_w()), 3};
        default:
            break;
        }
    }

    // 5-bit offset mode when bit7 == 0.
    if ((post & 0x80) == 0) {
        const IndexReg base = static_cast<IndexReg>((post >> 5) & 0x03);
        const int8_t offset = static_cast<int8_t>((post << 3)) >> 3; // sign-extend 5 bits
        const uint16_t addr = static_cast<uint16_t>(index_value(base) + offset);
        return {addr, 1};
    }

    IndexReg base = IndexReg::X;
    switch ((post >> 5) & 0x03) {
    case 0: base = IndexReg::X; break;
    case 1: base = IndexReg::Y; break;
    case 2: base = IndexReg::U; break;
    case 3: base = IndexReg::S; break;
    }

    const bool indirect = (post & 0x10) != 0;
    uint16_t addr = 0;
    uint8_t cycles = 0;

    if (indirect && ((post & 0x0F) == 0x00 || (post & 0x0F) == 0x02)) {
        return unsupported_indexed_address(bus);
    }

    switch (post & 0x0F) {
    case 0x00: // ,R+
        addr = index_value(base);
        index_ref(base) = static_cast<uint16_t>(index_ref(base) + 1);
        cycles = native_mode ? 1 : 2;
        break;
    case 0x01: // ,R++
        addr = index_value(base);
        index_ref(base) = static_cast<uint16_t>(index_ref(base) + 2);
        cycles = native_mode ? 2 : 3;
        break;
    case 0x02: // ,-R
        index_ref(base) = static_cast<uint16_t>(index_ref(base) - 1);
        addr = index_value(base);
        cycles = native_mode ? 1 : 2;
        break;
    case 0x03: // ,--R
        index_ref(base) = static_cast<uint16_t>(index_ref(base) - 2);
        addr = index_value(base);
        cycles = native_mode ? 2 : 3;
        break;
    case 0x04: // ,R
        addr = index_value(base);
        cycles = 0;
        break;
    case 0x05: // B,R
        addr = static_cast<uint16_t>(index_value(base) + static_cast<int8_t>(regs_.b));
        cycles = 1;
        break;
    case 0x06: // A,R
        addr = static_cast<uint16_t>(index_value(base) + static_cast<int8_t>(regs_.a));
        cycles = 1;
        break;
    case 0x07: // E,R
        if (mode_ != CpuMode::HD6309) return unsupported_indexed_address(bus);
        addr = static_cast<uint16_t>(index_value(base) + static_cast<int8_t>(regs_.e));
        cycles = 1;
        break;
    case 0x08: // n,R 8-bit offset
        addr = static_cast<uint16_t>(index_value(base) + static_cast<int8_t>(fetch_byte(bus)));
        cycles = 1;
        break;
    case 0x09: // n,R 16-bit offset
        addr = static_cast<uint16_t>(index_value(base) + fetch_word(bus));
        cycles = native_mode ? 3 : 4;
        break;
    case 0x0A: // F,R
        if (mode_ != CpuMode::HD6309) return unsupported_indexed_address(bus);
        addr = static_cast<uint16_t>(index_value(base) + static_cast<int8_t>(regs_.f));
        cycles = 1;
        break;
    case 0x0B: // D,R
        addr = static_cast<uint16_t>(index_value(base) + static_cast<uint16_t>((regs_.a << 8) | regs_.b));
        cycles = native_mode ? 2 : 4;
        break;
    case 0x0C: // ,PC 8-bit
        {
            const int8_t offset = static_cast<int8_t>(fetch_byte(bus));
            addr = static_cast<uint16_t>(regs_.pc + offset);
        }
        cycles = 1;
        base = IndexReg::PC;
        break;
    case 0x0D: // ,PC 16-bit
        {
            const uint16_t offset = fetch_word(bus);
            addr = static_cast<uint16_t>(regs_.pc + offset);
        }
        cycles = native_mode ? 3 : 5;
        base = IndexReg::PC;
        break;
    case 0x0E: // W,R
        if (mode_ != CpuMode::HD6309) return unsupported_indexed_address(bus);
        addr = static_cast<uint16_t>(index_value(base) + reg_w());
        cycles = native_mode ? 1 : 4;
        break;
    case 0x0F: // Extended
        addr = fetch_word(bus);
        cycles = 5;
        base = IndexReg::PC;
        break;
    default:
        return unsupported_indexed_address(bus);
    }

    if (indirect) {
        const uint16_t indirect_addr = addr;
        addr = static_cast<uint16_t>((read_byte(bus, indirect_addr) << 8) | read_byte(bus, static_cast<uint16_t>(indirect_addr + 1)));
        switch (post & 0x0F) {
        case 0x01: // [,R++]
        case 0x03: // [,--R]
            cycles = 6;
            break;
        case 0x04: // [,R]
            cycles = 3;
            break;
        case 0x05: // [B,R]
        case 0x06: // [A,R]
        case 0x0E: // [W,R]
        case 0x0B: // [D,R]
            cycles = 4;
            break;
        case 0x07: // [E,R]
        case 0x0A: // [F,R]
            cycles = 1;
            break;
        case 0x08: // [n,R] 8-bit
        case 0x0C: // [n,PC] 8-bit
            cycles = 4;
            break;
        case 0x09: // [n,R] 16-bit
            cycles = 7;
            break;
        case 0x0D: // [n,PC] 16-bit
            cycles = 8;
            break;
        case 0x0F: // [n]
            cycles = 5;
            break;
        default:
            cycles = static_cast<uint8_t>(cycles + 3);
            break;
        }
    }

    return {addr, cycles};
}

void Cpu::push_byte(Bus& bus, uint8_t value) {
    regs_.s = static_cast<uint16_t>(regs_.s - 1);
    write_stack_byte(bus, regs_.s, value);
}

void Cpu::push_word(Bus& bus, uint16_t value) {
    push_byte(bus, lo(value));
    push_byte(bus, hi(value));
}

uint8_t Cpu::pull_byte(Bus& bus) {
    const uint8_t value = read_stack_byte(bus, regs_.s);
    regs_.s = static_cast<uint16_t>(regs_.s + 1);
    return value;
}

uint16_t Cpu::pull_word(Bus& bus) {
    const uint8_t high = pull_byte(bus);
    const uint8_t low = pull_byte(bus);
    return static_cast<uint16_t>((static_cast<uint16_t>(high) << 8) | low);
}

void Cpu::set_flags_nz8(uint8_t value) {
    regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z));
    if (value == 0) {
        regs_.cc |= CC_Z;
    }
    if (value & 0x80) {
        regs_.cc |= CC_N;
    }
}

void Cpu::set_flags_nz16(uint16_t value) {
    regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z));
    if (value == 0) {
        regs_.cc |= CC_Z;
    }
    if (value & 0x8000) {
        regs_.cc |= CC_N;
    }
}

uint8_t Cpu::branch_if(Bus& bus, bool take) {
    const int8_t offset = static_cast<int8_t>(fetch_byte(bus));
    if (take) {
        regs_.pc = static_cast<uint16_t>(regs_.pc + offset);
        return 3;
    }
    return 2;
}

uint16_t& Cpu::index_ref(IndexReg reg) {
    switch (reg) {
    case IndexReg::X: return regs_.x;
    case IndexReg::Y: return regs_.y;
    case IndexReg::U: return regs_.u;
    case IndexReg::S: return regs_.s;
    case IndexReg::PC: return regs_.pc;
    }
    return regs_.x;
}

uint16_t Cpu::index_value(IndexReg reg) const {
    switch (reg) {
    case IndexReg::X: return regs_.x;
    case IndexReg::Y: return regs_.y;
    case IndexReg::U: return regs_.u;
    case IndexReg::S: return regs_.s;
    case IndexReg::PC: return regs_.pc;
    }
    return regs_.x;
}

uint8_t* Cpu::reg8_by_code(uint8_t code) {
    // 0:D,1:X,2:Y,3:U,4:S,5:PC,6:W,7:V,8:A,9:B,A:CC,B:DP,C/D:0,E:E,F:F.
    switch (code & 0x0F) {
    case 0x08: return &regs_.a;
    case 0x09: return &regs_.b;
    case 0x0A: return &regs_.cc;
    case 0x0B: return &regs_.dp;
    case 0x0E: return &regs_.e; // 6309
    case 0x0F: return &regs_.f; // 6309
    default: return nullptr;
    }
}

bool Cpu::get_reg16_by_code(uint8_t code, uint16_t& out) const {
    switch (code & 0x0F) {
    case 0x00: out = static_cast<uint16_t>((regs_.a << 8) | regs_.b); return true; // D
    case 0x01: out = regs_.x; return true;
    case 0x02: out = regs_.y; return true;
    case 0x03: out = regs_.u; return true;
    case 0x04: out = regs_.s; return true;
    case 0x05: out = regs_.pc; return true;
    case 0x06: out = reg_w(); return true; // W (E:F)
    case 0x07: out = regs_.v; return true; // V
    case 0x0C:
    case 0x0D: out = 0; return true;       // zero registers
    default: return false;
    }
}

bool Cpu::set_reg16_by_code(uint8_t code, uint16_t value) {
    switch (code & 0x0F) {
    case 0x00: regs_.a = hi(value); regs_.b = lo(value); return true;
    case 0x01: regs_.x = value; return true;
    case 0x02: regs_.y = value; return true;
    case 0x03: regs_.u = value; return true;
    case 0x04: regs_.s = value; return true;
    case 0x05: regs_.pc = value; return true;
    case 0x06: set_reg_w(value); return true;
    case 0x07: regs_.v = value; return true;
    case 0x0C:
    case 0x0D: return true; // zero registers ignore writes
    default: return false;
    }
}

bool Cpu::set_reg32_by_code(uint8_t code, uint32_t value) {
    if ((code & 0x0F) == 0x06) { // Q treated via W code
        set_reg_q(value);
        return true;
    }
    return false;
}

uint16_t Cpu::reg_w() const {
    return static_cast<uint16_t>((static_cast<uint16_t>(regs_.e) << 8) | regs_.f);
}

void Cpu::set_reg_w(uint16_t value) {
    regs_.e = hi(value);
    regs_.f = lo(value);
}

uint32_t Cpu::reg_q() const {
    const uint32_t high = static_cast<uint32_t>((regs_.a << 8) | regs_.b);
    const uint32_t low = static_cast<uint32_t>((regs_.e << 8) | regs_.f);
    return (high << 16) | low;
}

void Cpu::set_reg_q(uint32_t value) {
    regs_.a = static_cast<uint8_t>((value >> 24) & 0xFF);
    regs_.b = static_cast<uint8_t>((value >> 16) & 0xFF);
    regs_.e = static_cast<uint8_t>((value >> 8) & 0xFF);
    regs_.f = static_cast<uint8_t>(value & 0xFF);
}

uint16_t read_reg_for_dest(const Registers& regs, uint8_t src_code, bool dest_is_16) {
    const uint8_t c = src_code & 0x0F;
    if (dest_is_16) {
        switch (c) {
        case 0x00: return static_cast<uint16_t>((regs.a << 8) | regs.b); // D
        case 0x01: return regs.x;
        case 0x02: return regs.y;
        case 0x03: return regs.u;
        case 0x04: return regs.s;
        case 0x05: return regs.pc;
        case 0x06: return static_cast<uint16_t>((regs.e << 8) | regs.f); // W
        case 0x07: return regs.v;
        case 0x08: return static_cast<uint16_t>((regs.a << 8) | regs.b); // A -> D
        case 0x09: return static_cast<uint16_t>((regs.a << 8) | regs.b); // B -> D
        case 0x0A: return regs.cc;
        case 0x0B: return static_cast<uint16_t>(regs.dp) << 8;
        case 0x0C:
        case 0x0D: return 0;                                             // zero registers
        case 0x0E: return static_cast<uint16_t>((regs.e << 8) | regs.f); // E -> W
        case 0x0F: return static_cast<uint16_t>((regs.e << 8) | regs.f); // F -> W
        default: return 0;
        }
    }
    // dest is 8-bit; return lower byte of source
    switch (c) {
    case 0x00: return regs.b; // D low
    case 0x01: return regs.x & 0xFF;
    case 0x02: return regs.y & 0xFF;
    case 0x03: return regs.u & 0xFF;
    case 0x04: return regs.s & 0xFF;
    case 0x05: return regs.pc & 0xFF;
    case 0x06: return regs.f; // W low
    case 0x07: return regs.v & 0xFF;
    case 0x08: return regs.a;
    case 0x09: return regs.b;
    case 0x0A: return regs.cc;
    case 0x0B: return regs.dp;
    case 0x0C:
    case 0x0D: return 0;
    case 0x0E: return regs.e;
    case 0x0F: return regs.f;
    default: return 0;
    }
}

void write_reg_sized(Cpu& cpu, uint8_t dest_code, uint16_t value, bool dest_is_16) {
    const uint8_t c = dest_code & 0x0F;
    if (dest_is_16) {
        cpu.set_reg16_by_code(c, value);
    } else {
        if (auto* r = cpu.reg8_by_code(c)) {
            *r = static_cast<uint8_t>(value & 0xFF);
        }
    }
}

} // namespace microlind
