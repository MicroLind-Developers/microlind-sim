#pragma once

#include "microlind/cpu.hpp"

#include "microlind/bus.hpp"

#include <array>
#include <cstdint>
#include <optional>

namespace microlind {
namespace {

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"
#endif

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

uint16_t add16_carry(Registers& regs, uint16_t left, uint16_t right, bool carry_in) {
    const uint32_t sum = static_cast<uint32_t>(left) + static_cast<uint32_t>(right) + (carry_in ? 1u : 0u);
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

uint16_t sub16_carry(Registers& regs, uint16_t left, uint16_t right, bool carry_in) {
    const uint32_t diff = static_cast<uint32_t>(left) - static_cast<uint32_t>(right) - (carry_in ? 1u : 0u);
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

void set_reg_d_value(Registers& regs, uint16_t value) {
    regs.a = hi(value);
    regs.b = lo(value);
}

void set_logic16_flags(Registers& regs, uint16_t value, bool clear_carry) {
    regs.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | (clear_carry ? CC_C : 0)));
    if (value == 0) {
        regs.cc |= CC_Z;
    }
    if (value & 0x8000) {
        regs.cc |= CC_N;
    }
}

void set_tst16_flags(Registers& regs, uint16_t value) {
    regs.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | CC_C));
    if (value == 0) regs.cc |= CC_Z;
    if (value & 0x8000) regs.cc |= CC_N;
}

uint16_t neg16_unary(Registers& regs, uint16_t value) {
    const uint16_t result = static_cast<uint16_t>(0u - value);
    regs.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | CC_C));
    if (result & 0x8000) regs.cc |= CC_N;
    if (result == 0) regs.cc |= CC_Z;
    if (value == 0x8000) regs.cc |= CC_V;
    if (value != 0) regs.cc |= CC_C;
    return result;
}

uint16_t com16_unary(Registers& regs, uint16_t value) {
    const uint16_t result = static_cast<uint16_t>(~value);
    regs.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V));
    regs.cc |= CC_C;
    if (result & 0x8000) regs.cc |= CC_N;
    if (result == 0) regs.cc |= CC_Z;
    return result;
}

uint16_t lsr16_unary(Registers& regs, uint16_t value) {
    const uint16_t result = static_cast<uint16_t>(value >> 1);
    regs.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | CC_C));
    if (value & 0x0001) regs.cc |= CC_C;
    if (result == 0) regs.cc |= CC_Z;
    return result;
}

uint16_t asr16_unary(Registers& regs, uint16_t value) {
    const uint16_t result = static_cast<uint16_t>((value >> 1) | (value & 0x8000));
    regs.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | CC_C));
    if (result & 0x8000) regs.cc |= CC_N;
    if (result == 0) regs.cc |= CC_Z;
    if (value & 0x0001) regs.cc |= CC_C;
    return result;
}

uint16_t dec16_unary(Registers& regs, uint16_t value) {
    const uint16_t result = static_cast<uint16_t>(value - 1);
    regs.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V));
    if (result == 0) regs.cc |= CC_Z;
    if (result & 0x8000) regs.cc |= CC_N;
    if (result == 0x7FFF) regs.cc |= CC_V;
    return result;
}

uint16_t inc16_unary(Registers& regs, uint16_t value) {
    const uint16_t result = static_cast<uint16_t>(value + 1);
    regs.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V));
    if (result == 0) regs.cc |= CC_Z;
    if (result & 0x8000) regs.cc |= CC_N;
    if (result == 0x8000) regs.cc |= CC_V;
    return result;
}

uint16_t lsl16_unary(Registers& regs, uint16_t value) {
    const uint16_t result = static_cast<uint16_t>(value << 1);
    regs.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | CC_C));
    if (result & 0x8000) regs.cc |= CC_N;
    if (result == 0) regs.cc |= CC_Z;
    if (value & 0x8000) regs.cc |= CC_C;
    if (((result ^ (regs.cc & CC_C ? 0x8000 : 0)) & 0x8000) != 0) regs.cc |= CC_V;
    return result;
}

uint16_t rol16_unary(Registers& regs, uint16_t value) {
    const uint16_t result = static_cast<uint16_t>((value << 1) | ((regs.cc & CC_C) ? 1 : 0));
    regs.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | CC_C));
    if (result & 0x8000) regs.cc |= CC_N;
    if (result == 0) regs.cc |= CC_Z;
    if (value & 0x8000) regs.cc |= CC_C;
    if (((result ^ (regs.cc & CC_C ? 0x8000 : 0)) & 0x8000) != 0) regs.cc |= CC_V;
    return result;
}

uint16_t ror16_unary(Registers& regs, uint16_t value) {
    const uint16_t result = static_cast<uint16_t>((value >> 1) | ((regs.cc & CC_C) ? 0x8000 : 0));
    regs.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | CC_C));
    if (result & 0x8000) regs.cc |= CC_N;
    if (result == 0) regs.cc |= CC_Z;
    if (value & 0x0001) regs.cc |= CC_C;
    if (((result ^ (regs.cc & CC_C ? 0x8000 : 0)) & 0x8000) != 0) regs.cc |= CC_V;
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
    switch (opcode) {
    case 0x83:
    case 0xC3:
        return true;
    default:
        return false;
    }
}

bool is_alu16_direct_opcode(uint8_t opcode) {
    switch (opcode) {
    case 0x93:
    case 0xD3:
        return true;
    default:
        return false;
    }
}

bool is_alu16_extended_opcode(uint8_t opcode) {
    switch (opcode) {
    case 0xB3:
    case 0xF3:
        return true;
    default:
        return false;
    }
}

bool is_alu16_indexed_opcode(uint8_t opcode) {
    switch (opcode) {
    case 0xA3:
    case 0xE3:
        return true;
    default:
        return false;
    }
}

bool is_hd6309_d_alu_opcode(uint8_t opcode) {
    switch (opcode & 0x8F) {
    case 0x82:
    case 0x84:
    case 0x85:
    case 0x88:
    case 0x89:
    case 0x8A:
        return true;
    default:
        return false;
    }
}

bool is_hd6309_d_alu_immediate_opcode(uint8_t opcode) {
    return (opcode & 0xF0) == 0x80 && is_hd6309_d_alu_opcode(opcode);
}

bool is_hd6309_d_alu_direct_opcode(uint8_t opcode) {
    return (opcode & 0xF0) == 0x90 && is_hd6309_d_alu_opcode(opcode);
}

bool is_hd6309_d_alu_indexed_opcode(uint8_t opcode) {
    return (opcode & 0xF0) == 0xA0 && is_hd6309_d_alu_opcode(opcode);
}

bool is_hd6309_d_alu_extended_opcode(uint8_t opcode) {
    return (opcode & 0xF0) == 0xB0 && is_hd6309_d_alu_opcode(opcode);
}

bool is_cmp16_indexed_opcode(uint8_t prefix, uint8_t opcode) {
    switch (prefix) {
    case 0x00:
    case 0x10:
        return opcode == 0xAC;
    case 0x11:
        switch (opcode) {
        case 0xA3:
        case 0xAC:
            return true;
        default:
            return false;
        }
    default:
        return false;
    }
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

bool is_memory_unary_indexed_opcode(uint8_t opcode) {
    switch (opcode) {
    case 0x60:
    case 0x63:
    case 0x64:
    case 0x66:
    case 0x67:
    case 0x68:
    case 0x69:
    case 0x6A:
    case 0x6C:
    case 0x6D:
    case 0x6F:
        return true;
    default:
        return false;
    }
}

bool is_immediate_memory_direct_opcode(uint8_t opcode) {
    switch (opcode) {
    case 0x01:
    case 0x02:
    case 0x05:
    case 0x0B:
        return true;
    default:
        return false;
    }
}

bool is_immediate_memory_indexed_opcode(uint8_t opcode) {
    switch (opcode) {
    case 0x61:
    case 0x62:
    case 0x65:
    case 0x6B:
        return true;
    default:
        return false;
    }
}

bool is_immediate_memory_extended_opcode(uint8_t opcode) {
    switch (opcode) {
    case 0x71:
    case 0x72:
    case 0x75:
    case 0x7B:
        return true;
    default:
        return false;
    }
}

bool immediate_memory_writes(uint8_t opcode) {
    return (opcode & 0x0F) != 0x0B;
}

uint8_t apply_immediate_memory_value(uint8_t opcode, uint8_t value, uint8_t mask) {
    switch (opcode & 0x0F) {
    case 0x01:
        return static_cast<uint8_t>(value | mask);
    case 0x02:
    case 0x0B:
        return static_cast<uint8_t>(value & mask);
    case 0x05:
        return static_cast<uint8_t>(value ^ mask);
    default:
        return value;
    }
}

bool is_bit_transfer_opcode(uint8_t opcode) {
    return opcode >= 0x30 && opcode <= 0x37;
}

bool bit_transfer_postbyte_is_valid(uint8_t post) {
    return ((post >> 6) & 0x03) != 0x03;
}

bool bit_transfer_writes_memory(uint8_t opcode) {
    return opcode != 0x36;
}

uint8_t* bit_transfer_register(Registers& regs, uint8_t post) {
    switch ((post >> 6) & 0x03) {
    case 0: return &regs.cc;
    case 1: return &regs.a;
    case 2: return &regs.b;
    default: return nullptr;
    }
}

uint8_t apply_bit_transfer_memory_result(uint8_t opcode, uint8_t reg_value, uint8_t post, uint8_t memory) {
    const uint8_t src_bit = static_cast<uint8_t>((post >> 3) & 0x07);
    const uint8_t dst_bit = static_cast<uint8_t>(post & 0x07);
    const uint8_t reg_bit = static_cast<uint8_t>((reg_value >> src_bit) & 0x01);
    const uint8_t mem_bit = static_cast<uint8_t>((memory >> dst_bit) & 0x01);
    uint8_t new_bit = 0;
    switch (opcode) {
    case 0x30:
        new_bit = static_cast<uint8_t>(reg_bit & mem_bit);
        break;
    case 0x31:
        new_bit = static_cast<uint8_t>(reg_bit & (mem_bit ^ 0x01));
        break;
    case 0x32:
        new_bit = static_cast<uint8_t>(reg_bit | mem_bit);
        break;
    case 0x33:
        new_bit = static_cast<uint8_t>(reg_bit | (mem_bit ^ 0x01));
        break;
    case 0x34:
        new_bit = static_cast<uint8_t>(reg_bit ^ mem_bit);
        break;
    case 0x35:
        new_bit = static_cast<uint8_t>(reg_bit ^ (mem_bit ^ 0x01));
        break;
    case 0x37:
        new_bit = reg_bit;
        break;
    default:
        new_bit = mem_bit;
        break;
    }
    return static_cast<uint8_t>((memory & static_cast<uint8_t>(~(1u << dst_bit))) | (new_bit << dst_bit));
}

bool is_divd_opcode(uint8_t opcode) {
    switch (opcode) {
    case 0x8D:
    case 0x9D:
    case 0xAD:
    case 0xBD:
        return true;
    default:
        return false;
    }
}

bool is_divq_opcode(uint8_t opcode) {
    switch (opcode) {
    case 0x8E:
    case 0x9E:
    case 0xAE:
    case 0xBE:
        return true;
    default:
        return false;
    }
}

void apply_divd_result(Registers& regs, uint8_t divisor) {
    const int16_t dividend = static_cast<int16_t>(static_cast<uint16_t>((static_cast<uint16_t>(regs.a) << 8) | regs.b));
    const int8_t signed_divisor = static_cast<int8_t>(divisor);
    const int quotient_wide = dividend / signed_divisor;
    const int remainder_wide = dividend % signed_divisor;
    if (quotient_wide < -32768 || quotient_wide > 32767) {
        regs.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_C));
        regs.cc |= CC_V;
        return;
    }

    const int16_t quotient = static_cast<int16_t>(quotient_wide);
    const int16_t remainder = static_cast<int16_t>(remainder_wide);
    regs.e = hi(static_cast<uint16_t>(quotient));
    regs.f = lo(static_cast<uint16_t>(quotient));
    set_reg_d_value(regs, static_cast<uint16_t>(remainder));
    regs.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | CC_C));
    if (quotient == 0) regs.cc |= CC_Z;
    if (quotient < 0) regs.cc |= CC_N;
    if ((quotient & 0x01) != 0) regs.cc |= CC_C;
}

void apply_divq_result(Registers& regs, uint16_t divisor) {
    const uint32_t dividend =
        (static_cast<uint32_t>((static_cast<uint16_t>(regs.a) << 8) | regs.b) << 16) |
        static_cast<uint32_t>((static_cast<uint16_t>(regs.e) << 8) | regs.f);
    const int32_t signed_dividend = static_cast<int32_t>(dividend);
    const int32_t signed_divisor = static_cast<int16_t>(divisor);
    const int32_t quotient = signed_dividend / signed_divisor;
    const int32_t remainder = signed_dividend % signed_divisor;
    if (quotient < -32768 || quotient > 32767) {
        regs.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_C));
        regs.cc |= CC_V;
        return;
    }

    const uint16_t quotient_word = static_cast<uint16_t>(quotient & 0xFFFF);
    regs.e = hi(quotient_word);
    regs.f = lo(quotient_word);
    const uint16_t rem = static_cast<uint16_t>(remainder & 0xFFFF);
    regs.a = hi(rem);
    regs.b = lo(rem);
    regs.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | CC_C));
    if (quotient == 0) regs.cc |= CC_Z;
    if (quotient & 0x8000) regs.cc |= CC_N;
    if (quotient & 0x01) regs.cc |= CC_C;
}

bool is_tfm_opcode(uint8_t opcode) {
    return opcode >= 0x38 && opcode <= 0x3B;
}

bool tfm_register_code_is_valid(uint8_t code) {
    return code >= 0x01 && code <= 0x04;
}

bool tfm_postbyte_is_valid(uint8_t post) {
    return tfm_register_code_is_valid(static_cast<uint8_t>(post >> 4)) &&
        tfm_register_code_is_valid(static_cast<uint8_t>(post & 0x0F));
}

uint16_t* tfm_register_pointer(Registers& regs, uint8_t code) {
    switch (code & 0x0F) {
    case 0x01: return &regs.x;
    case 0x02: return &regs.y;
    case 0x03: return &regs.u;
    case 0x04: return &regs.s;
    default: return nullptr;
    }
}

const uint16_t* tfm_register_pointer(const Registers& regs, uint8_t code) {
    switch (code & 0x0F) {
    case 0x01: return &regs.x;
    case 0x02: return &regs.y;
    case 0x03: return &regs.u;
    case 0x04: return &regs.s;
    default: return nullptr;
    }
}

int8_t tfm_source_delta(uint8_t opcode) {
    switch (opcode) {
    case 0x38:
    case 0x3A:
        return 1;
    case 0x39:
        return -1;
    default:
        return 0;
    }
}

int8_t tfm_destination_delta(uint8_t opcode) {
    switch (opcode) {
    case 0x38:
    case 0x3B:
        return 1;
    case 0x39:
        return -1;
    default:
        return 0;
    }
}

uint32_t tfm_transfer_count(const Registers& regs) {
    const uint16_t w = static_cast<uint16_t>((static_cast<uint16_t>(regs.e) << 8) | regs.f);
    return w == 0 ? 0x10000u : w;
}

bool is_hd6309_d_or_w_unary_opcode(uint8_t opcode) {
    switch (opcode) {
    case 0x40:
    case 0x43:
    case 0x44:
    case 0x46:
    case 0x47:
    case 0x48:
    case 0x49:
    case 0x4A:
    case 0x4C:
    case 0x4D:
    case 0x4F:
    case 0x53:
    case 0x54:
    case 0x56:
    case 0x58:
    case 0x59:
    case 0x5A:
    case 0x5C:
    case 0x5D:
    case 0x5F:
        return true;
    default:
        return false;
    }
}

bool is_hd6309_e_or_f_unary_opcode(uint8_t opcode) {
    switch (opcode) {
    case 0x43:
    case 0x4A:
    case 0x4C:
    case 0x4D:
    case 0x4F:
    case 0x53:
    case 0x5A:
    case 0x5C:
    case 0x5D:
    case 0x5F:
        return true;
    default:
        return false;
    }
}

bool is_ef_alu_immediate_opcode(uint8_t opcode) {
    switch (opcode) {
    case 0x80:
    case 0x81:
    case 0x8B:
    case 0xC0:
    case 0xC1:
    case 0xCB:
        return true;
    default:
        return false;
    }
}

bool is_ef_alu_direct_opcode(uint8_t opcode) {
    switch (opcode) {
    case 0x90:
    case 0x91:
    case 0x9B:
    case 0xD0:
    case 0xD1:
    case 0xDB:
        return true;
    default:
        return false;
    }
}

bool is_ef_alu_indexed_opcode(uint8_t opcode) {
    switch (opcode) {
    case 0xA0:
    case 0xA1:
    case 0xAB:
    case 0xE0:
    case 0xE1:
    case 0xEB:
        return true;
    default:
        return false;
    }
}

bool is_ef_alu_extended_opcode(uint8_t opcode) {
    switch (opcode) {
    case 0xB0:
    case 0xB1:
    case 0xBB:
    case 0xF0:
    case 0xF1:
    case 0xFB:
        return true;
    default:
        return false;
    }
}

bool is_indexed_data_opcode(uint8_t opcode) {
    switch (opcode) {
    case 0xA6:
    case 0xE6:
    case 0xEC:
    case 0xA7:
    case 0xE7:
    case 0xED:
        return true;
    default:
        return false;
    }
}

bool is_alu8_indexed_opcode(uint8_t opcode) {
    switch (opcode) {
    case 0xA0:
    case 0xA1:
    case 0xA2:
    case 0xA4:
    case 0xA5:
    case 0xA8:
    case 0xA9:
    case 0xAA:
    case 0xAB:
    case 0xE0:
    case 0xE1:
    case 0xE2:
    case 0xE4:
    case 0xE5:
    case 0xE8:
    case 0xE9:
    case 0xEA:
    case 0xEB:
        return true;
    default:
        return false;
    }
}

uint8_t indexed_extension_byte_count(uint8_t post) {
    if ((post & 0x80) == 0) {
        return 0;
    }
    switch (post & 0x0F) {
    case 0x08:
    case 0x0C:
        return 1;
    case 0x09:
    case 0x0D:
    case 0x0F:
        return 2;
    default:
        return 0;
    }
}

bool is_hd6309_w_special_postbyte(uint8_t post) {
    switch (post) {
    case 0x8F:
    case 0xAF:
    case 0xCF:
    case 0xEF:
    case 0x90:
    case 0xB0:
    case 0xD0:
    case 0xF0:
        return true;
    default:
        return false;
    }
}

std::optional<uint8_t> indexed_cycle_add_for_micro_op(uint8_t post, const Registers& regs, CpuMode mode) {
    const bool native_mode = is_native_hd6309(regs, mode);
    if (mode == CpuMode::HD6309 && is_hd6309_w_special_postbyte(post)) {
        return std::nullopt;
    }
    const bool indirect = (post & 0x80) != 0 && (post & 0x10) != 0;
    if ((post & 0x80) == 0) {
        return 1;
    }

    switch (post & 0x0F) {
    case 0x00:
        return indirect ? std::nullopt : std::optional<uint8_t>{static_cast<uint8_t>(native_mode ? 1 : 2)};
    case 0x01:
        return indirect ? std::optional<uint8_t>{6} : std::optional<uint8_t>{static_cast<uint8_t>(native_mode ? 2 : 3)};
    case 0x02:
        return indirect ? std::nullopt : std::optional<uint8_t>{static_cast<uint8_t>(native_mode ? 1 : 2)};
    case 0x03:
        return indirect ? std::optional<uint8_t>{6} : std::optional<uint8_t>{static_cast<uint8_t>(native_mode ? 2 : 3)};
    case 0x04:
        return indirect ? std::optional<uint8_t>{3} : std::optional<uint8_t>{0};
    case 0x05:
    case 0x06:
        return indirect ? std::optional<uint8_t>{4} : std::optional<uint8_t>{1};
    case 0x07:
        if (mode != CpuMode::HD6309) return std::nullopt;
        return indirect ? std::optional<uint8_t>{1} : std::optional<uint8_t>{1};
    case 0x08:
        return indirect ? std::optional<uint8_t>{4} : std::optional<uint8_t>{1};
    case 0x09:
        return indirect ? std::optional<uint8_t>{7} : std::optional<uint8_t>{static_cast<uint8_t>(native_mode ? 3 : 4)};
    case 0x0A:
        if (mode != CpuMode::HD6309) return std::nullopt;
        return indirect ? std::optional<uint8_t>{1} : std::optional<uint8_t>{1};
    case 0x0B:
        return indirect ? std::optional<uint8_t>{4} : std::optional<uint8_t>{static_cast<uint8_t>(native_mode ? 2 : 4)};
    case 0x0C:
        return indirect ? std::optional<uint8_t>{4} : std::optional<uint8_t>{1};
    case 0x0D:
        return indirect ? std::optional<uint8_t>{8} : std::optional<uint8_t>{static_cast<uint8_t>(native_mode ? 3 : 5)};
    case 0x0E:
        if (mode != CpuMode::HD6309) return std::nullopt;
        return indirect ? std::optional<uint8_t>{4} : std::optional<uint8_t>{static_cast<uint8_t>(native_mode ? 1 : 4)};
    case 0x0F:
        return indirect ? std::optional<uint8_t>{5} : std::optional<uint8_t>{5};
    default:
        return std::nullopt;
    }
}

uint16_t indexed_base_value(const Registers& regs, uint8_t base) {
    switch (base & 0x03) {
    case 0: return regs.x;
    case 1: return regs.y;
    case 2: return regs.u;
    default: return regs.s;
    }
}

void set_indexed_base_value(Registers& regs, uint8_t base, uint16_t value) {
    switch (base & 0x03) {
    case 0: regs.x = value; break;
    case 1: regs.y = value; break;
    case 2: regs.u = value; break;
    default: regs.s = value; break;
    }
}

uint16_t reg_d_value(const Registers& regs) {
    return static_cast<uint16_t>((static_cast<uint16_t>(regs.a) << 8) | regs.b);
}

uint16_t indexed_effective_address_and_update(Registers& regs, uint8_t post, uint16_t extension) {
    if ((post & 0x80) == 0) {
        const uint8_t base = static_cast<uint8_t>((post >> 5) & 0x03);
        const int8_t offset = static_cast<int8_t>(static_cast<int8_t>(post << 3) >> 3);
        return static_cast<uint16_t>(indexed_base_value(regs, base) + offset);
    }

    const uint8_t base = static_cast<uint8_t>((post >> 5) & 0x03);
    switch (post & 0x0F) {
    case 0x00: {
        const uint16_t address = indexed_base_value(regs, base);
        set_indexed_base_value(regs, base, static_cast<uint16_t>(address + 1));
        return address;
    }
    case 0x01: {
        const uint16_t address = indexed_base_value(regs, base);
        set_indexed_base_value(regs, base, static_cast<uint16_t>(address + 2));
        return address;
    }
    case 0x02: {
        const uint16_t address = static_cast<uint16_t>(indexed_base_value(regs, base) - 1);
        set_indexed_base_value(regs, base, address);
        return address;
    }
    case 0x03: {
        const uint16_t address = static_cast<uint16_t>(indexed_base_value(regs, base) - 2);
        set_indexed_base_value(regs, base, address);
        return address;
    }
    case 0x04:
        return indexed_base_value(regs, base);
    case 0x05:
        return static_cast<uint16_t>(indexed_base_value(regs, base) + static_cast<int8_t>(regs.b));
    case 0x06:
        return static_cast<uint16_t>(indexed_base_value(regs, base) + static_cast<int8_t>(regs.a));
    case 0x07:
        return static_cast<uint16_t>(indexed_base_value(regs, base) + static_cast<int8_t>(regs.e));
    case 0x08:
        return static_cast<uint16_t>(indexed_base_value(regs, base) + static_cast<int8_t>(extension & 0xFF));
    case 0x09:
        return static_cast<uint16_t>(indexed_base_value(regs, base) + extension);
    case 0x0A:
        return static_cast<uint16_t>(indexed_base_value(regs, base) + static_cast<int8_t>(regs.f));
    case 0x0B:
        return static_cast<uint16_t>(indexed_base_value(regs, base) + reg_d_value(regs));
    case 0x0C:
        return static_cast<uint16_t>(regs.pc + static_cast<int8_t>(extension & 0xFF));
    case 0x0D:
        return static_cast<uint16_t>(regs.pc + extension);
    case 0x0E:
        return static_cast<uint16_t>(indexed_base_value(regs, base) + static_cast<uint16_t>((static_cast<uint16_t>(regs.e) << 8) | regs.f));
    case 0x0F:
        return extension;
    default:
        return 0;
    }
}

bool is_cmp16_immediate_opcode(uint8_t prefix, uint8_t opcode) {
    switch (prefix) {
    case 0x00:
    case 0x10:
        return opcode == 0x8C;
    case 0x11:
        switch (opcode) {
        case 0x83:
        case 0x8C:
            return true;
        default:
            return false;
        }
    default:
        return false;
    }
}

bool is_cmp16_direct_opcode(uint8_t prefix, uint8_t opcode) {
    switch (prefix) {
    case 0x00:
    case 0x10:
        return opcode == 0x9C;
    case 0x11:
        switch (opcode) {
        case 0x93:
        case 0x9C:
            return true;
        default:
            return false;
        }
    default:
        return false;
    }
}

bool is_cmp16_extended_opcode(uint8_t prefix, uint8_t opcode) {
    switch (prefix) {
    case 0x00:
    case 0x10:
        return opcode == 0xBC;
    case 0x11:
        switch (opcode) {
        case 0xB3:
        case 0xBC:
            return true;
        default:
            return false;
        }
    default:
        return false;
    }
}

bool is_word_load_immediate_opcode(uint8_t prefix, uint8_t opcode) {
    switch (prefix) {
    case 0x00:
    case 0x10:
        switch (opcode) {
        case 0x8E:
        case 0xCE:
            return true;
        default:
            return false;
        }
    default:
        return false;
    }
}

bool is_word_load_direct_opcode(uint8_t prefix, uint8_t opcode) {
    switch (prefix) {
    case 0x00:
    case 0x10:
        switch (opcode) {
        case 0x9E:
        case 0xDE:
            return true;
        default:
            return false;
        }
    default:
        return false;
    }
}

bool is_word_load_extended_opcode(uint8_t prefix, uint8_t opcode) {
    switch (prefix) {
    case 0x00:
    case 0x10:
        switch (opcode) {
        case 0xBE:
        case 0xFE:
            return true;
        default:
            return false;
        }
    default:
        return false;
    }
}

bool is_word_load_indexed_opcode(uint8_t prefix, uint8_t opcode) {
    switch (prefix) {
    case 0x00:
    case 0x10:
        switch (opcode) {
        case 0xAE:
        case 0xEE:
            return true;
        default:
            return false;
        }
    default:
        return false;
    }
}

bool is_word_store_direct_opcode(uint8_t prefix, uint8_t opcode) {
    switch (prefix) {
    case 0x00:
    case 0x10:
        switch (opcode) {
        case 0x9F:
        case 0xDF:
            return true;
        default:
            return false;
        }
    default:
        return false;
    }
}

bool is_word_store_extended_opcode(uint8_t prefix, uint8_t opcode) {
    switch (prefix) {
    case 0x00:
    case 0x10:
        switch (opcode) {
        case 0xBF:
        case 0xFF:
            return true;
        default:
            return false;
        }
    default:
        return false;
    }
}

bool is_word_store_indexed_opcode(uint8_t prefix, uint8_t opcode) {
    switch (prefix) {
    case 0x00:
    case 0x10:
        switch (opcode) {
        case 0xAF:
        case 0xEF:
            return true;
        default:
            return false;
        }
    default:
        return false;
    }
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

uint8_t word_indexed_base_cycles(uint8_t prefix) {
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

void apply_ef_alu8(Registers& regs, uint8_t opcode, uint8_t operand) {
    uint8_t& target = (opcode & 0x40) != 0 ? regs.f : regs.e;
    switch (opcode & 0x8F) {
    case 0x80:
        target = sub8(regs, target, operand, 0);
        break;
    case 0x81:
        sub8(regs, target, operand, 0);
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
    const bool add = opcode == 0xC3 || opcode == 0xD3 || opcode == 0xE3 || opcode == 0xF3;
    const uint16_t result = add ? add16(regs, d, operand) : sub16(regs, d, operand);
    regs.a = hi(result);
    regs.b = lo(result);
}

void apply_cmpd(Registers& regs, uint16_t operand) {
    const uint16_t d = static_cast<uint16_t>((static_cast<uint16_t>(regs.a) << 8) | regs.b);
    sub16(regs, d, operand);
}

void apply_hd6309_d_alu(Registers& regs, uint8_t opcode, uint16_t operand) {
    const uint16_t d = reg_d_value(regs);
    switch (opcode & 0x8F) {
    case 0x82:
        set_reg_d_value(regs, sub16_carry(regs, d, operand, (regs.cc & CC_C) != 0));
        break;
    case 0x84: {
        const uint16_t result = static_cast<uint16_t>(d & operand);
        set_reg_d_value(regs, result);
        set_logic16_flags(regs, result, true);
        break;
    }
    case 0x85:
        set_logic16_flags(regs, static_cast<uint16_t>(d & operand), false);
        break;
    case 0x88: {
        const uint16_t result = static_cast<uint16_t>(d ^ operand);
        set_reg_d_value(regs, result);
        set_logic16_flags(regs, result, true);
        break;
    }
    case 0x89:
        set_reg_d_value(regs, add16_carry(regs, d, operand, (regs.cc & CC_C) != 0));
        break;
    case 0x8A: {
        const uint16_t result = static_cast<uint16_t>(d | operand);
        set_reg_d_value(regs, result);
        set_logic16_flags(regs, result, true);
        break;
    }
    default:
        break;
    }
}

void apply_register_alu(Cpu& cpu, Registers& regs, uint8_t opcode, uint8_t post) {
    const uint8_t dst = static_cast<uint8_t>(post & 0x0F);
    const uint8_t src = static_cast<uint8_t>(post >> 4);
    const bool dest16 = register_code_is_16bit(dst);
    const uint16_t source = read_reg_for_dest(regs, src, dest16);
    const uint16_t dest = read_reg_for_dest(regs, dst, dest16);
    uint16_t result = dest;
    bool write_result = true;

    switch (opcode) {
    case 0x30:
        result = dest16
            ? add16(regs, dest, source)
            : add8(regs, static_cast<uint8_t>(dest & 0xFF), static_cast<uint8_t>(source & 0xFF), 0);
        break;
    case 0x31:
        result = dest16
            ? add16_carry(regs, dest, source, (regs.cc & CC_C) != 0)
            : add8(regs, static_cast<uint8_t>(dest & 0xFF), static_cast<uint8_t>(source & 0xFF), (regs.cc & CC_C) ? 1 : 0);
        break;
    case 0x32:
        result = dest16
            ? sub16(regs, dest, source)
            : sub8(regs, static_cast<uint8_t>(dest & 0xFF), static_cast<uint8_t>(source & 0xFF), 0);
        break;
    case 0x33:
        result = dest16
            ? sub16_carry(regs, dest, source, (regs.cc & CC_C) != 0)
            : sub8(regs, static_cast<uint8_t>(dest & 0xFF), static_cast<uint8_t>(source & 0xFF), (regs.cc & CC_C) ? 1 : 0);
        break;
    case 0x34:
        result = static_cast<uint16_t>(dest & source);
        set_logic16_flags(regs, result, false);
        if (!dest16) set_logic8_flags(regs, static_cast<uint8_t>(result & 0xFF), false);
        break;
    case 0x35:
        result = static_cast<uint16_t>(dest | source);
        set_logic16_flags(regs, result, false);
        if (!dest16) set_logic8_flags(regs, static_cast<uint8_t>(result & 0xFF), false);
        break;
    case 0x36:
        result = static_cast<uint16_t>(dest ^ source);
        set_logic16_flags(regs, result, false);
        if (!dest16) set_logic8_flags(regs, static_cast<uint8_t>(result & 0xFF), false);
        break;
    case 0x37:
        if (dest16) {
            sub16(regs, dest, source);
        } else {
            sub8(regs, static_cast<uint8_t>(dest & 0xFF), static_cast<uint8_t>(source & 0xFF), 0);
        }
        write_result = false;
        break;
    default:
        write_result = false;
        break;
    }

    if (write_result) {
        write_reg_sized(cpu, dst, result, dest16);
    }
}

void apply_cmp16(Registers& regs, uint8_t prefix, uint8_t opcode, uint16_t operand) {
    uint16_t value = 0;
    if (prefix == 0x00) {
        value = regs.x;
    } else if (prefix == 0x10) {
        value = regs.y;
    } else if (prefix == 0x11 && (opcode == 0x83 || opcode == 0x93 || opcode == 0xA3 || opcode == 0xB3)) {
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
    const bool x_or_y_opcode = opcode == 0x8E || opcode == 0x9E || opcode == 0xAE ||
        opcode == 0xBE || opcode == 0x9F || opcode == 0xAF || opcode == 0xBF;
    if (prefix == 0x00 && x_or_y_opcode) return regs.x;
    if (prefix == 0x00) return regs.u;
    if (x_or_y_opcode) return regs.y;
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

uint8_t full_interrupt_stack_push_byte(const Registers& regs, uint8_t index) {
    Registers stacked = regs;
    stacked.cc = static_cast<uint8_t>(stacked.cc | CC_E);
    return stack_push_byte(stacked, 0xFF, index, false);
}

enum class InterruptSource : uint8_t {
    Irq = 0,
    Firq = 1,
    Nmi = 2,
};

bool native_hd6309_frame(const Registers& regs, CpuMode mode) {
    return mode == CpuMode::HD6309 && (regs.md & 0x01) != 0;
}

bool interrupt_uses_full_frame(const Registers& regs, InterruptSource source) {
    return source != InterruptSource::Firq || (regs.md & 0x02) != 0;
}

uint16_t interrupt_vector(InterruptSource source) {
    switch (source) {
    case InterruptSource::Firq: return 0xFFF6;
    case InterruptSource::Nmi: return 0xFFFC;
    case InterruptSource::Irq:
    default:
        return 0xFFF8;
    }
}

uint8_t interrupt_stack_byte_count(const Registers& regs, CpuMode mode, InterruptSource source) {
    if (!interrupt_uses_full_frame(regs, source)) {
        return 3;
    }
    return native_hd6309_frame(regs, mode) ? 14 : 12;
}

uint8_t interrupt_total_cycles(const Registers& regs, CpuMode mode, InterruptSource source) {
    if (!interrupt_uses_full_frame(regs, source)) {
        return 10;
    }
    return native_hd6309_frame(regs, mode) ? 21 : 19;
}

uint8_t interrupt_stack_push_byte(const Registers& regs, CpuMode mode, InterruptSource source, uint8_t index) {
    if (!interrupt_uses_full_frame(regs, source)) {
        Registers stacked = regs;
        stacked.cc = static_cast<uint8_t>(stacked.cc & ~CC_E);
        return stack_push_byte(stacked, 0x81, index, false);
    }

    if (!native_hd6309_frame(regs, mode)) {
        return full_interrupt_stack_push_byte(regs, index);
    }

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
    if (emit_word(regs.pc, result)) return result;
    if (emit_word(regs.u, result)) return result;
    if (emit_word(regs.y, result)) return result;
    if (emit_word(regs.x, result)) return result;
    if (emit_byte(regs.dp, result)) return result;
    if (emit_byte(regs.f, result)) return result;
    if (emit_byte(regs.e, result)) return result;
    if (emit_byte(regs.b, result)) return result;
    if (emit_byte(regs.a, result)) return result;
    if (emit_byte(static_cast<uint8_t>(regs.cc | CC_E), result)) return result;
    return result;
}

void apply_native_interrupt_pull_byte(Registers& regs, uint8_t index, uint8_t value, uint16_t& word_data) {
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

    if (consume_byte([&](uint8_t v) { regs.cc = v; })) return;
    if (consume_byte([&](uint8_t v) { regs.a = v; })) return;
    if (consume_byte([&](uint8_t v) { regs.b = v; })) return;
    if (consume_byte([&](uint8_t v) { regs.e = v; })) return;
    if (consume_byte([&](uint8_t v) { regs.f = v; })) return;
    if (consume_byte([&](uint8_t v) { regs.dp = v; })) return;
    if (consume_word([&](uint16_t v) { regs.x = v; })) return;
    if (consume_word([&](uint16_t v) { regs.y = v; })) return;
    if (consume_word([&](uint16_t v) { regs.u = v; })) return;
    consume_word([&](uint16_t v) { regs.pc = v; });
}

void apply_interrupt_masks(Registers& regs, InterruptSource source) {
    switch (source) {
    case InterruptSource::Irq:
        regs.cc = static_cast<uint8_t>(regs.cc | CC_I);
        break;
    case InterruptSource::Firq:
    case InterruptSource::Nmi:
        regs.cc = static_cast<uint8_t>(regs.cc | CC_I | CC_F);
        break;
    }
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
    const bool x_or_y_opcode = opcode == 0x8E || opcode == 0x9E || opcode == 0xAE || opcode == 0xBE;
    if (prefix == 0x00 && x_or_y_opcode) {
        regs.x = value;
    } else if (prefix == 0x00) {
        regs.u = value;
    } else if (x_or_y_opcode) {
        regs.y = value;
    } else {
        regs.s = value;
    }
}

void set_lea_register_value(Registers& regs, uint8_t opcode, uint16_t value) {
    switch (opcode) {
    case 0x30:
        regs.x = value;
        break;
    case 0x31:
        regs.y = value;
        break;
    case 0x32:
        regs.s = value;
        break;
    case 0x33:
        regs.u = value;
        break;
    default:
        break;
    }
}

bool lea_sets_flags(uint8_t opcode) {
    return opcode != 0x32;
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

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic pop
#endif

} // namespace
} // namespace microlind
