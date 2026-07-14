#include "microlind/cpu.hpp"

#include "microlind/bus.hpp"

#include <cstdint>

namespace microlind {

namespace {
uint8_t hi(uint16_t value) { return static_cast<uint8_t>((value >> 8) & 0xFF); }
uint8_t lo(uint16_t value) { return static_cast<uint8_t>(value & 0xFF); }

uint16_t read_word(Bus& bus, uint16_t address) {
    return static_cast<uint16_t>((bus.read8(address) << 8) | bus.read8(static_cast<uint16_t>(address + 1)));
}

void write_word(Bus& bus, uint16_t address, uint16_t value) {
    bus.write8(address, static_cast<uint8_t>((value >> 8) & 0xFF));
    bus.write8(static_cast<uint16_t>(address + 1), static_cast<uint8_t>(value & 0xFF));
}

constexpr uint16_t VECTOR_SWI = 0xFFFA;
constexpr uint16_t VECTOR_SWI2 = 0xFFF4;
constexpr uint16_t VECTOR_SWI3 = 0xFFF2;
} // namespace

static inline void flags_logic8(Registers& r, uint8_t value) {
    r.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | CC_C));
    if (value == 0) r.cc |= CC_Z;
    if (value & 0x80) r.cc |= CC_N;
}

static inline uint16_t reg_d(const Registers& r) {
    return static_cast<uint16_t>((static_cast<uint16_t>(r.a) << 8) | r.b);
}

static inline void set_reg_d(Registers& r, uint16_t value) {
    r.a = hi(value);
    r.b = lo(value);
}

static inline void flags_logic16(Registers& r, uint16_t value) {
    r.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | CC_C));
    if (value == 0) r.cc |= CC_Z;
    if (value & 0x8000) r.cc |= CC_N;
}

static inline void flags_bit16(Registers& r, uint16_t value) {
    r.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V));
    if (value == 0) r.cc |= CC_Z;
    if (value & 0x8000) r.cc |= CC_N;
}

static inline uint8_t add8(Registers& r, uint8_t a, uint8_t b) {
    const uint16_t sum = static_cast<uint16_t>(a) + static_cast<uint16_t>(b);
    const uint8_t res = static_cast<uint8_t>(sum & 0xFF);
    r.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | CC_C | CC_H));
    if (res & 0x80) r.cc |= CC_N;
    if (res == 0) r.cc |= CC_Z;
    if (((a ^ b ^ res) & 0x80) && !((a ^ b) & 0x80)) r.cc |= CC_V;
    if (sum & 0x100) r.cc |= CC_C;
    if (((a & b) | (b & ~res) | (a & ~res)) & 0x10) r.cc |= CC_H;
    return res;
}

static inline uint8_t sub8(Registers& r, uint8_t a, uint8_t b) {
    const uint16_t diff = static_cast<uint16_t>(a) - static_cast<uint16_t>(b);
    const uint8_t res = static_cast<uint8_t>(diff & 0xFF);
    r.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | CC_C));
    if (res & 0x80) r.cc |= CC_N;
    if (res == 0) r.cc |= CC_Z;
    if (((a ^ b) & (a ^ res) & 0x80) != 0) r.cc |= CC_V;
    if (diff & 0x100) r.cc |= CC_C;
    return res;
}

static inline uint16_t add16(Registers& r, uint16_t a, uint16_t b) {
    const uint32_t sum = static_cast<uint32_t>(a) + static_cast<uint32_t>(b);
    const uint16_t res = static_cast<uint16_t>(sum & 0xFFFF);
    r.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | CC_C));
    if (res & 0x8000) r.cc |= CC_N;
    if (res == 0) r.cc |= CC_Z;
    if (((a ^ b ^ res) & 0x8000) && !((a ^ b) & 0x8000)) r.cc |= CC_V;
    if (sum & 0x10000) r.cc |= CC_C;
    return res;
}

static inline uint16_t sub16(Registers& r, uint16_t a, uint16_t b) {
    const uint32_t diff = static_cast<uint32_t>(a) - static_cast<uint32_t>(b);
    const uint16_t res = static_cast<uint16_t>(diff & 0xFFFF);
    r.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | CC_C));
    if (res & 0x8000) r.cc |= CC_N;
    if (res == 0) r.cc |= CC_Z;
    if (((a ^ b) & (a ^ res) & 0x8000) != 0) r.cc |= CC_V;
    if (diff & 0x10000) r.cc |= CC_C;
    return res;
}

static inline uint16_t add16_carry(Registers& r, uint16_t a, uint16_t b, bool carry_in) {
    const uint32_t sum = static_cast<uint32_t>(a) + static_cast<uint32_t>(b) + (carry_in ? 1u : 0u);
    const uint16_t res = static_cast<uint16_t>(sum & 0xFFFF);
    r.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | CC_C));
    if (res & 0x8000) r.cc |= CC_N;
    if (res == 0) r.cc |= CC_Z;
    if (((a ^ b ^ res) & 0x8000) && !((a ^ b) & 0x8000)) r.cc |= CC_V;
    if (sum & 0x10000) r.cc |= CC_C;
    return res;
}

static inline uint16_t sub16_carry(Registers& r, uint16_t a, uint16_t b, bool carry_in) {
    const uint32_t diff = static_cast<uint32_t>(a) - static_cast<uint32_t>(b) - (carry_in ? 1u : 0u);
    const uint16_t res = static_cast<uint16_t>(diff & 0xFFFF);
    r.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | CC_C));
    if (res & 0x8000) r.cc |= CC_N;
    if (res == 0) r.cc |= CC_Z;
    if (((a ^ b) & (a ^ res) & 0x8000) != 0) r.cc |= CC_V;
    if (diff & 0x10000) r.cc |= CC_C;
    return res;
}

static inline uint8_t adc8(Registers& r, uint8_t a, uint8_t b) {
    const uint8_t carry = (r.cc & CC_C) ? 1 : 0;
    const uint16_t sum = static_cast<uint16_t>(a) + static_cast<uint16_t>(b) + carry;
    const uint8_t res = static_cast<uint8_t>(sum & 0xFF);
    r.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | CC_C | CC_H));
    if (res & 0x80) r.cc |= CC_N;
    if (res == 0) r.cc |= CC_Z;
    if (((a ^ b ^ res) & 0x80) && !((a ^ b) & 0x80)) r.cc |= CC_V;
    if (sum & 0x100) r.cc |= CC_C;
    if (((a & b) | (b & ~res) | (a & ~res)) & 0x10) r.cc |= CC_H;
    return res;
}

static inline uint8_t sbc8(Registers& r, uint8_t a, uint8_t b) {
    const uint8_t carry = (r.cc & CC_C) ? 1 : 0;
    const uint16_t diff = static_cast<uint16_t>(a) - static_cast<uint16_t>(b) - carry;
    const uint8_t res = static_cast<uint8_t>(diff & 0xFF);
    r.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | CC_C));
    if (res & 0x80) r.cc |= CC_N;
    if (res == 0) r.cc |= CC_Z;
    if (((a ^ b) & (a ^ res) & 0x80) != 0) r.cc |= CC_V;
    if (diff & 0x100) r.cc |= CC_C;
    return res;
}

static inline uint8_t dec8(Registers& r, uint8_t v) {
    const uint8_t res = static_cast<uint8_t>(v - 1);
    r.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V));
    if (res == 0) r.cc |= CC_Z;
    if (res & 0x80) r.cc |= CC_N;
    if (res == 0x7F) r.cc |= CC_V;
    return res;
}

static inline uint8_t inc8(Registers& r, uint8_t v) {
    const uint8_t res = static_cast<uint8_t>(v + 1);
    r.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V));
    if (res == 0) r.cc |= CC_Z;
    if (res & 0x80) r.cc |= CC_N;
    if (res == 0x80) r.cc |= CC_V;
    return res;
}

static inline uint16_t dec16(Registers& r, uint16_t v) {
    const uint16_t res = static_cast<uint16_t>(v - 1);
    r.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V));
    if (res == 0) r.cc |= CC_Z;
    if (res & 0x8000) r.cc |= CC_N;
    if (res == 0x7FFF) r.cc |= CC_V;
    return res;
}

static inline uint16_t inc16(Registers& r, uint16_t v) {
    const uint16_t res = static_cast<uint16_t>(v + 1);
    r.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V));
    if (res == 0) r.cc |= CC_Z;
    if (res & 0x8000) r.cc |= CC_N;
    if (res == 0x8000) r.cc |= CC_V;
    return res;
}

static inline void set_flags_tst(Registers& r, uint8_t v) {
    r.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | CC_C));
    if (v == 0) r.cc |= CC_Z;
    if (v & 0x80) r.cc |= CC_N;
}

static inline void set_flags_tst16(Registers& r, uint16_t v) {
    r.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | CC_C));
    if (v == 0) r.cc |= CC_Z;
    if (v & 0x8000) r.cc |= CC_N;
}

static inline uint8_t neg8_op(Registers& r, uint8_t v) {
    const uint8_t res = static_cast<uint8_t>(0u - v);
    r.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | CC_C));
    if (res & 0x80) r.cc |= CC_N;
    if (res == 0) r.cc |= CC_Z;
    if (v == 0x80) r.cc |= CC_V;
    if (v != 0) r.cc |= CC_C;
    return res;
}

static inline uint16_t neg16_op(Registers& r, uint16_t v) {
    const uint16_t res = static_cast<uint16_t>(0u - v);
    r.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | CC_C));
    if (res & 0x8000) r.cc |= CC_N;
    if (res == 0) r.cc |= CC_Z;
    if (v == 0x8000) r.cc |= CC_V;
    if (v != 0) r.cc |= CC_C;
    return res;
}

static inline uint8_t com8_op(Registers& r, uint8_t v) {
    const uint8_t res = static_cast<uint8_t>(~v);
    r.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V));
    r.cc |= CC_C;
    if (res & 0x80) r.cc |= CC_N;
    if (res == 0) r.cc |= CC_Z;
    return res;
}

static inline uint16_t com16_op(Registers& r, uint16_t v) {
    const uint16_t res = static_cast<uint16_t>(~v);
    r.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V));
    r.cc |= CC_C;
    if (res & 0x8000) r.cc |= CC_N;
    if (res == 0) r.cc |= CC_Z;
    return res;
}

static inline uint8_t lsr8_op(Registers& r, uint8_t v) {
    const uint8_t c = v & 0x01;
    const uint8_t res = static_cast<uint8_t>(v >> 1);
    r.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | CC_C));
    r.cc |= c;
    if (res == 0) r.cc |= CC_Z;
    return res;
}

static inline uint16_t lsr16_op(Registers& r, uint16_t v) {
    const uint16_t c = v & 0x01;
    const uint16_t res = static_cast<uint16_t>(v >> 1);
    r.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | CC_C));
    if (c != 0) r.cc |= CC_C;
    if (res == 0) r.cc |= CC_Z;
    return res;
}

static inline uint8_t ror8_op(Registers& r, uint8_t v) {
    const uint8_t c_in = (r.cc & CC_C) ? 0x80 : 0x00;
    const uint8_t c_out = v & 0x01;
    const uint8_t res = static_cast<uint8_t>((v >> 1) | c_in);
    r.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | CC_C));
    if (res & 0x80) r.cc |= CC_N;
    if (res == 0) r.cc |= CC_Z;
    if (c_out) r.cc |= CC_C;
    if (((res ^ (c_out ? 0x80 : 0x00)) & 0x80) != 0) r.cc |= CC_V; // V = N xor C
    return res;
}

static inline uint8_t asr8_op(Registers& r, uint8_t v) {
    const uint8_t c_out = v & 0x01;
    const uint8_t res = static_cast<uint8_t>((v >> 1) | (v & 0x80));
    r.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | CC_C));
    if (res & 0x80) r.cc |= CC_N;
    if (res == 0) r.cc |= CC_Z;
    if (c_out) r.cc |= CC_C;
    return res;
}

static inline uint16_t asr16_op(Registers& r, uint16_t v) {
    const uint16_t c_out = v & 0x01;
    const uint16_t res = static_cast<uint16_t>((v >> 1) | (v & 0x8000));
    r.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | CC_C));
    if (res & 0x8000) r.cc |= CC_N;
    if (res == 0) r.cc |= CC_Z;
    if (c_out != 0) r.cc |= CC_C;
    return res;
}

static inline uint8_t asl8_op(Registers& r, uint8_t v) {
    const uint8_t c_out = static_cast<uint8_t>((v >> 7) & 0x01);
    const uint8_t res = static_cast<uint8_t>(v << 1);
    r.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | CC_C));
    if (res & 0x80) r.cc |= CC_N;
    if (res == 0) r.cc |= CC_Z;
    if (c_out) r.cc |= CC_C;
    if (((res ^ (c_out ? 0x80 : 0x00)) & 0x80) != 0) r.cc |= CC_V; // V = N xor C
    return res;
}

static inline uint8_t rol8_op(Registers& r, uint8_t v) {
    const uint8_t c_in = (r.cc & CC_C) ? 1 : 0;
    const uint8_t c_out = static_cast<uint8_t>((v >> 7) & 0x01);
    const uint8_t res = static_cast<uint8_t>((v << 1) | c_in);
    r.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | CC_C));
    if (res & 0x80) r.cc |= CC_N;
    if (res == 0) r.cc |= CC_Z;
    if (c_out) r.cc |= CC_C;
    if (((res ^ (c_out ? 0x80 : 0x00)) & 0x80) != 0) r.cc |= CC_V; // V = N xor C
    return res;
}


bool reg_is_16bit(uint8_t code) {
    const uint8_t c = code & 0x0F;
    return c <= 0x07; // D,X,Y,U,S,PC,W,V
}


// ---------- Instruction implementations ----------

uint8_t Cpu::op_invalid(Bus& bus) {
    if (mode_ == CpuMode::HD6309) {
        regs_.md |= 0x40;
        regs_.pc = read_word(bus, 0xFFF0);
    }
    return 1;
}

uint8_t Cpu::op_nop(Bus&) { return 2; }

uint8_t Cpu::op_clra(Bus&) {
    regs_.a = 0;
    regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_V | CC_Z));
    regs_.cc |= CC_Z;
    return 2;
}

uint8_t Cpu::op_clrb(Bus&) {
    regs_.b = 0;
    regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_V | CC_Z));
    regs_.cc |= CC_Z;
    return 2;
}

uint8_t Cpu::op_lda_imm(Bus& bus) {
    regs_.a = fetch_byte(bus);
    set_flags_nz8(regs_.a);
    return 2;
}

uint8_t Cpu::op_lda_dir(Bus& bus) {
    const uint16_t addr = direct_address(bus);
    regs_.a = read_byte(bus, addr);
    set_flags_nz8(regs_.a);
    return 4;
}

uint8_t Cpu::op_lda_ext(Bus& bus) {
    const uint16_t addr = extended_address(bus);
    regs_.a = read_byte(bus, addr);
    set_flags_nz8(regs_.a);
    return 5;
}

uint8_t Cpu::op_lda_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    if (!pb.valid) return pb.cycles;
    regs_.a = read_byte(bus, pb.address);
    set_flags_nz8(regs_.a);
    return static_cast<uint8_t>(4 + pb.cycles);
}

uint8_t Cpu::op_ldb_imm(Bus& bus) {
    regs_.b = fetch_byte(bus);
    set_flags_nz8(regs_.b);
    return 2;
}

uint8_t Cpu::op_ldb_dir(Bus& bus) {
    const uint16_t addr = direct_address(bus);
    regs_.b = read_byte(bus, addr);
    set_flags_nz8(regs_.b);
    return 4;
}

uint8_t Cpu::op_ldb_ext(Bus& bus) {
    const uint16_t addr = extended_address(bus);
    regs_.b = read_byte(bus, addr);
    set_flags_nz8(regs_.b);
    return 5;
}

uint8_t Cpu::op_ldb_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    if (!pb.valid) return pb.cycles;
    regs_.b = read_byte(bus, pb.address);
    set_flags_nz8(regs_.b);
    return static_cast<uint8_t>(4 + pb.cycles);
}

uint8_t Cpu::op_ldd_imm(Bus& bus) {
    const uint16_t value = fetch_word(bus);
    regs_.a = hi(value);
    regs_.b = lo(value);
    set_flags_nz16(value);
    return 3;
}

uint8_t Cpu::op_ldd_dir(Bus& bus) {
    const uint16_t addr = direct_address(bus);
    const uint16_t value = static_cast<uint16_t>((read_byte(bus, addr) << 8) | read_byte(bus, static_cast<uint16_t>(addr + 1)));
    regs_.a = hi(value);
    regs_.b = lo(value);
    set_flags_nz16(value);
    return 5;
}

uint8_t Cpu::op_ldd_ext(Bus& bus) {
    const uint16_t addr = extended_address(bus);
    const uint16_t value = static_cast<uint16_t>((read_byte(bus, addr) << 8) | read_byte(bus, static_cast<uint16_t>(addr + 1)));
    regs_.a = hi(value);
    regs_.b = lo(value);
    set_flags_nz16(value);
    return 6;
}

uint8_t Cpu::op_ldd_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    if (!pb.valid) return pb.cycles;
    const uint16_t value =
        static_cast<uint16_t>((read_byte(bus, pb.address) << 8) | read_byte(bus, static_cast<uint16_t>(pb.address + 1)));
    regs_.a = hi(value);
    regs_.b = lo(value);
    set_flags_nz16(value);
    return static_cast<uint8_t>(5 + pb.cycles);
}

uint8_t Cpu::op_sta_dir(Bus& bus) {
    const uint16_t addr = direct_address(bus);
    write_byte(bus, addr, regs_.a);
    set_flags_nz8(regs_.a);
    return 4;
}

uint8_t Cpu::op_sta_ext(Bus& bus) {
    const uint16_t addr = extended_address(bus);
    write_byte(bus, addr, regs_.a);
    set_flags_nz8(regs_.a);
    return 5;
}

uint8_t Cpu::op_sta_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    if (!pb.valid) return pb.cycles;
    write_byte(bus, pb.address, regs_.a);
    set_flags_nz8(regs_.a);
    return static_cast<uint8_t>(4 + pb.cycles);
}

uint8_t Cpu::op_stb_dir(Bus& bus) {
    const uint16_t addr = direct_address(bus);
    write_byte(bus, addr, regs_.b);
    set_flags_nz8(regs_.b);
    return 4;
}

uint8_t Cpu::op_stb_ext(Bus& bus) {
    const uint16_t addr = extended_address(bus);
    write_byte(bus, addr, regs_.b);
    set_flags_nz8(regs_.b);
    return 5;
}

uint8_t Cpu::op_stb_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    if (!pb.valid) return pb.cycles;
    write_byte(bus, pb.address, regs_.b);
    set_flags_nz8(regs_.b);
    return static_cast<uint8_t>(4 + pb.cycles);
}

uint8_t Cpu::op_std_dir(Bus& bus) {
    const uint16_t addr = direct_address(bus);
    write_byte(bus, addr, regs_.a);
    write_byte(bus, static_cast<uint16_t>(addr + 1), regs_.b);
    set_flags_nz16(static_cast<uint16_t>((regs_.a << 8) | regs_.b));
    return 5;
}

uint8_t Cpu::op_std_ext(Bus& bus) {
    const uint16_t addr = extended_address(bus);
    write_byte(bus, addr, regs_.a);
    write_byte(bus, static_cast<uint16_t>(addr + 1), regs_.b);
    set_flags_nz16(static_cast<uint16_t>((regs_.a << 8) | regs_.b));
    return 6;
}

uint8_t Cpu::op_std_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    if (!pb.valid) return pb.cycles;
    write_byte(bus, pb.address, regs_.a);
    write_byte(bus, static_cast<uint16_t>(pb.address + 1), regs_.b);
    set_flags_nz16(static_cast<uint16_t>((regs_.a << 8) | regs_.b));
    return static_cast<uint8_t>(5 + pb.cycles);
}

uint8_t Cpu::op_jmp_dir(Bus& bus) {
    regs_.pc = direct_address(bus);
    return 3;
}

uint8_t Cpu::op_jmp_ext(Bus& bus) {
    regs_.pc = extended_address(bus);
    return 4;
}

uint8_t Cpu::op_jmp_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    if (!pb.valid) return pb.cycles;
    regs_.pc = pb.address;
    return static_cast<uint8_t>(3 + pb.cycles);
}

uint8_t Cpu::op_jsr_dir(Bus& bus) {
    const uint16_t target = direct_address(bus);
    push_word(bus, regs_.pc);
    regs_.pc = target;
    return 5;
}

uint8_t Cpu::op_jsr_ext(Bus& bus) {
    const uint16_t target = extended_address(bus);
    push_word(bus, regs_.pc);
    regs_.pc = target;
    return 7;
}

uint8_t Cpu::op_jsr_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    if (!pb.valid) return pb.cycles;
    push_word(bus, regs_.pc);
    regs_.pc = pb.address;
    return static_cast<uint8_t>(7 + pb.cycles);
}

uint8_t Cpu::op_rts(Bus& bus) {
    regs_.pc = pull_word(bus);
    return 5;
}

uint8_t Cpu::op_rti(Bus& bus) {
    regs_.cc = pull_byte(bus);
    uint8_t cycles = 6;
    if (regs_.cc & CC_E) {
        regs_.a = pull_byte(bus);
        regs_.b = pull_byte(bus);
        regs_.dp = pull_byte(bus);
        regs_.x = pull_word(bus);
        regs_.y = pull_word(bus);
        regs_.u = pull_word(bus);
        cycles = 15;
    }
    regs_.pc = pull_word(bus);
    return cycles;
}

uint8_t Cpu::op_swi(Bus& bus) {
    regs_.cc |= CC_E;
    push_word(bus, regs_.pc);
    push_word(bus, regs_.u);
    push_word(bus, regs_.y);
    push_word(bus, regs_.x);
    push_byte(bus, regs_.dp);
    push_byte(bus, regs_.b);
    push_byte(bus, regs_.a);
    push_byte(bus, regs_.cc);
    regs_.cc |= static_cast<uint8_t>(CC_I | CC_F);
    regs_.pc = read_word(bus, VECTOR_SWI);
    return 19;
}

uint8_t Cpu::op_swi2(Bus& bus) {
    regs_.cc |= CC_E;
    push_word(bus, regs_.pc);
    push_word(bus, regs_.u);
    push_word(bus, regs_.y);
    push_word(bus, regs_.x);
    push_byte(bus, regs_.dp);
    push_byte(bus, regs_.b);
    push_byte(bus, regs_.a);
    push_byte(bus, regs_.cc);
    regs_.cc |= CC_I;
    regs_.pc = read_word(bus, VECTOR_SWI2);
    return 20;
}

uint8_t Cpu::op_swi3(Bus& bus) {
    regs_.cc |= CC_E;
    push_word(bus, regs_.pc);
    push_word(bus, regs_.u);
    push_word(bus, regs_.y);
    push_word(bus, regs_.x);
    push_byte(bus, regs_.dp);
    push_byte(bus, regs_.b);
    push_byte(bus, regs_.a);
    push_byte(bus, regs_.cc);
    regs_.cc |= CC_I;
    regs_.pc = read_word(bus, VECTOR_SWI3);
    return 20;
}

uint8_t Cpu::op_cwai(Bus& bus) {
    const uint8_t mask = fetch_byte(bus);
    regs_.cc &= mask;
    regs_.cc |= CC_E;
    push_word(bus, regs_.pc);
    push_word(bus, regs_.u);
    push_word(bus, regs_.y);
    push_word(bus, regs_.x);
    push_byte(bus, regs_.dp);
    push_byte(bus, regs_.b);
    push_byte(bus, regs_.a);
    push_byte(bus, regs_.cc);
    regs_.cc |= CC_I;
    // No waiting state modeled yet.
    return 19;
}

uint8_t Cpu::op_sync(Bus&) {
    sync_wait_ = true; // not modeled yet
    return 2;
}

uint8_t Cpu::op_mul(Bus&) {
    const uint16_t res = static_cast<uint16_t>(regs_.a) * static_cast<uint16_t>(regs_.b);
    regs_.a = hi(res);
    regs_.b = lo(res);
    regs_.cc &= static_cast<uint8_t>(~(CC_Z | CC_C | CC_V | CC_N));
    if (res == 0) regs_.cc |= CC_Z;
    if (res & 0x8000) regs_.cc |= CC_N;
    if (res & 0x80) regs_.cc |= CC_C;
    return 11;
}

uint8_t Cpu::op_bra(Bus& bus) {
    const int8_t offset = static_cast<int8_t>(fetch_byte(bus));
    regs_.pc = static_cast<uint16_t>(regs_.pc + offset);
    return 3; // unconditional relative
}

uint8_t Cpu::op_bsr(Bus& bus) {
    const int8_t offset = static_cast<int8_t>(fetch_byte(bus));
    push_word(bus, regs_.pc);
    regs_.pc = static_cast<uint16_t>(regs_.pc + offset);
    return 7;
}

uint8_t Cpu::op_bne(Bus& bus) {
    return branch_if(bus, (regs_.cc & CC_Z) == 0);
}

uint8_t Cpu::op_beq(Bus& bus) { return branch_if(bus, (regs_.cc & CC_Z) != 0); }
uint8_t Cpu::op_bhi(Bus& bus) { return branch_if(bus, (regs_.cc & (CC_Z | CC_C)) == 0); }
uint8_t Cpu::op_bls(Bus& bus) { return branch_if(bus, (regs_.cc & (CC_Z | CC_C)) != 0); }
uint8_t Cpu::op_bcc(Bus& bus) { return branch_if(bus, (regs_.cc & CC_C) == 0); }
uint8_t Cpu::op_bcs(Bus& bus) { return branch_if(bus, (regs_.cc & CC_C) != 0); }
uint8_t Cpu::op_bpl(Bus& bus) { return branch_if(bus, (regs_.cc & CC_N) == 0); }
uint8_t Cpu::op_bmi(Bus& bus) { return branch_if(bus, (regs_.cc & CC_N) != 0); }
uint8_t Cpu::op_bvc(Bus& bus) { return branch_if(bus, (regs_.cc & CC_V) == 0); }
uint8_t Cpu::op_bvs(Bus& bus) { return branch_if(bus, (regs_.cc & CC_V) != 0); }
uint8_t Cpu::op_bge(Bus& bus) { return branch_if(bus, ((regs_.cc & CC_N) >> 3) == ((regs_.cc & CC_V) >> 1)); }
uint8_t Cpu::op_blt(Bus& bus) { return branch_if(bus, ((regs_.cc & CC_N) >> 3) != ((regs_.cc & CC_V) >> 1)); }
uint8_t Cpu::op_bgt(Bus& bus) { return branch_if(bus, (regs_.cc & CC_Z) == 0 &&
                                                  (((regs_.cc & CC_N) >> 3) == ((regs_.cc & CC_V) >> 1))); }
uint8_t Cpu::op_ble(Bus& bus) { return branch_if(bus, (regs_.cc & CC_Z) != 0 ||
                                                  (((regs_.cc & CC_N) >> 3) != ((regs_.cc & CC_V) >> 1))); }

// ----- Logical -----

uint8_t Cpu::op_anda_imm(Bus& bus) {
    regs_.a &= fetch_byte(bus);
    flags_logic8(regs_, regs_.a);
    return 2;
}
uint8_t Cpu::op_anda_dir(Bus& bus) {
    const uint16_t addr = direct_address(bus);
    regs_.a &= read_byte(bus, addr);
    flags_logic8(regs_, regs_.a);
    return 4;
}
uint8_t Cpu::op_anda_ext(Bus& bus) {
    const uint16_t addr = extended_address(bus);
    regs_.a &= read_byte(bus, addr);
    flags_logic8(regs_, regs_.a);
    return 5;
}
uint8_t Cpu::op_anda_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    if (!pb.valid) return pb.cycles;
    regs_.a &= read_byte(bus, pb.address);
    flags_logic8(regs_, regs_.a);
    return static_cast<uint8_t>(4 + pb.cycles);
}

uint8_t Cpu::op_bita_imm(Bus& bus) {
    const uint8_t res = static_cast<uint8_t>(regs_.a & fetch_byte(bus));
    regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V));
    if (res == 0) regs_.cc |= CC_Z;
    if (res & 0x80) regs_.cc |= CC_N;
    return 2;
}
uint8_t Cpu::op_bita_dir(Bus& bus) {
    const uint16_t addr = direct_address(bus);
    const uint8_t res = static_cast<uint8_t>(regs_.a & read_byte(bus, addr));
    regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V));
    if (res == 0) regs_.cc |= CC_Z;
    if (res & 0x80) regs_.cc |= CC_N;
    return 4;
}
uint8_t Cpu::op_bita_ext(Bus& bus) {
    const uint16_t addr = extended_address(bus);
    const uint8_t res = static_cast<uint8_t>(regs_.a & read_byte(bus, addr));
    regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V));
    if (res == 0) regs_.cc |= CC_Z;
    if (res & 0x80) regs_.cc |= CC_N;
    return 5;
}
uint8_t Cpu::op_bita_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    if (!pb.valid) return pb.cycles;
    const uint8_t res = static_cast<uint8_t>(regs_.a & read_byte(bus, pb.address));
    regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V));
    if (res == 0) regs_.cc |= CC_Z;
    if (res & 0x80) regs_.cc |= CC_N;
    return static_cast<uint8_t>(4 + pb.cycles);
}

uint8_t Cpu::op_andb_imm(Bus& bus) {
    regs_.b &= fetch_byte(bus);
    flags_logic8(regs_, regs_.b);
    return 2;
}
uint8_t Cpu::op_andb_dir(Bus& bus) {
    const uint16_t addr = direct_address(bus);
    regs_.b &= read_byte(bus, addr);
    flags_logic8(regs_, regs_.b);
    return 4;
}
uint8_t Cpu::op_andb_ext(Bus& bus) {
    const uint16_t addr = extended_address(bus);
    regs_.b &= read_byte(bus, addr);
    flags_logic8(regs_, regs_.b);
    return 5;
}
uint8_t Cpu::op_andb_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    if (!pb.valid) return pb.cycles;
    regs_.b &= read_byte(bus, pb.address);
    flags_logic8(regs_, regs_.b);
    return static_cast<uint8_t>(4 + pb.cycles);
}

uint8_t Cpu::op_bitb_imm(Bus& bus) {
    const uint8_t res = static_cast<uint8_t>(regs_.b & fetch_byte(bus));
    regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V));
    if (res == 0) regs_.cc |= CC_Z;
    if (res & 0x80) regs_.cc |= CC_N;
    return 2;
}
uint8_t Cpu::op_bitb_dir(Bus& bus) {
    const uint16_t addr = direct_address(bus);
    const uint8_t res = static_cast<uint8_t>(regs_.b & read_byte(bus, addr));
    regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V));
    if (res == 0) regs_.cc |= CC_Z;
    if (res & 0x80) regs_.cc |= CC_N;
    return 4;
}
uint8_t Cpu::op_bitb_ext(Bus& bus) {
    const uint16_t addr = extended_address(bus);
    const uint8_t res = static_cast<uint8_t>(regs_.b & read_byte(bus, addr));
    regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V));
    if (res == 0) regs_.cc |= CC_Z;
    if (res & 0x80) regs_.cc |= CC_N;
    return 5;
}
uint8_t Cpu::op_bitb_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    if (!pb.valid) return pb.cycles;
    const uint8_t res = static_cast<uint8_t>(regs_.b & read_byte(bus, pb.address));
    regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V));
    if (res == 0) regs_.cc |= CC_Z;
    if (res & 0x80) regs_.cc |= CC_N;
    return static_cast<uint8_t>(4 + pb.cycles);
}

uint8_t Cpu::op_ora_imm(Bus& bus) {
    regs_.a |= fetch_byte(bus);
    flags_logic8(regs_, regs_.a);
    return 2;
}
uint8_t Cpu::op_ora_dir(Bus& bus) {
    const uint16_t addr = direct_address(bus);
    regs_.a |= read_byte(bus, addr);
    flags_logic8(regs_, regs_.a);
    return 4;
}
uint8_t Cpu::op_ora_ext(Bus& bus) {
    const uint16_t addr = extended_address(bus);
    regs_.a |= read_byte(bus, addr);
    flags_logic8(regs_, regs_.a);
    return 5;
}
uint8_t Cpu::op_ora_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    if (!pb.valid) return pb.cycles;
    regs_.a |= read_byte(bus, pb.address);
    flags_logic8(regs_, regs_.a);
    return static_cast<uint8_t>(4 + pb.cycles);
}

uint8_t Cpu::op_orb_imm(Bus& bus) {
    regs_.b |= fetch_byte(bus);
    flags_logic8(regs_, regs_.b);
    return 2;
}
uint8_t Cpu::op_orb_dir(Bus& bus) {
    const uint16_t addr = direct_address(bus);
    regs_.b |= read_byte(bus, addr);
    flags_logic8(regs_, regs_.b);
    return 4;
}
uint8_t Cpu::op_orb_ext(Bus& bus) {
    const uint16_t addr = extended_address(bus);
    regs_.b |= read_byte(bus, addr);
    flags_logic8(regs_, regs_.b);
    return 5;
}
uint8_t Cpu::op_orb_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    if (!pb.valid) return pb.cycles;
    regs_.b |= read_byte(bus, pb.address);
    flags_logic8(regs_, regs_.b);
    return static_cast<uint8_t>(4 + pb.cycles);
}

uint8_t Cpu::op_eora_imm(Bus& bus) {
    regs_.a ^= fetch_byte(bus);
    flags_logic8(regs_, regs_.a);
    return 2;
}
uint8_t Cpu::op_eora_dir(Bus& bus) {
    const uint16_t addr = direct_address(bus);
    regs_.a ^= read_byte(bus, addr);
    flags_logic8(regs_, regs_.a);
    return 4;
}
uint8_t Cpu::op_eora_ext(Bus& bus) {
    const uint16_t addr = extended_address(bus);
    regs_.a ^= read_byte(bus, addr);
    flags_logic8(regs_, regs_.a);
    return 5;
}
uint8_t Cpu::op_eora_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    if (!pb.valid) return pb.cycles;
    regs_.a ^= read_byte(bus, pb.address);
    flags_logic8(regs_, regs_.a);
    return static_cast<uint8_t>(4 + pb.cycles);
}

uint8_t Cpu::op_eorb_imm(Bus& bus) {
    regs_.b ^= fetch_byte(bus);
    flags_logic8(regs_, regs_.b);
    return 2;
}
uint8_t Cpu::op_eorb_dir(Bus& bus) {
    const uint16_t addr = direct_address(bus);
    regs_.b ^= read_byte(bus, addr);
    flags_logic8(regs_, regs_.b);
    return 4;
}
uint8_t Cpu::op_eorb_ext(Bus& bus) {
    const uint16_t addr = extended_address(bus);
    regs_.b ^= read_byte(bus, addr);
    flags_logic8(regs_, regs_.b);
    return 5;
}
uint8_t Cpu::op_eorb_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    if (!pb.valid) return pb.cycles;
    regs_.b ^= read_byte(bus, pb.address);
    flags_logic8(regs_, regs_.b);
    return static_cast<uint8_t>(4 + pb.cycles);
}

// ----- Arithmetic -----

uint8_t Cpu::op_adca_imm(Bus& bus) {
    regs_.a = adc8(regs_, regs_.a, fetch_byte(bus));
    return 2;
}
uint8_t Cpu::op_adca_dir(Bus& bus) {
    const uint16_t addr = direct_address(bus);
    regs_.a = adc8(regs_, regs_.a, read_byte(bus, addr));
    return 4;
}
uint8_t Cpu::op_adca_ext(Bus& bus) {
    const uint16_t addr = extended_address(bus);
    regs_.a = adc8(regs_, regs_.a, read_byte(bus, addr));
    return 5;
}
uint8_t Cpu::op_adca_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    if (!pb.valid) return pb.cycles;
    regs_.a = adc8(regs_, regs_.a, read_byte(bus, pb.address));
    return static_cast<uint8_t>(4 + pb.cycles);
}

uint8_t Cpu::op_adcb_imm(Bus& bus) {
    regs_.b = adc8(regs_, regs_.b, fetch_byte(bus));
    return 2;
}
uint8_t Cpu::op_adcb_dir(Bus& bus) {
    const uint16_t addr = direct_address(bus);
    regs_.b = adc8(regs_, regs_.b, read_byte(bus, addr));
    return 4;
}
uint8_t Cpu::op_adcb_ext(Bus& bus) {
    const uint16_t addr = extended_address(bus);
    regs_.b = adc8(regs_, regs_.b, read_byte(bus, addr));
    return 5;
}
uint8_t Cpu::op_adcb_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    if (!pb.valid) return pb.cycles;
    regs_.b = adc8(regs_, regs_.b, read_byte(bus, pb.address));
    return static_cast<uint8_t>(4 + pb.cycles);
}

uint8_t Cpu::op_sbca_imm(Bus& bus) {
    regs_.a = sbc8(regs_, regs_.a, fetch_byte(bus));
    return 2;
}
uint8_t Cpu::op_sbca_dir(Bus& bus) {
    const uint16_t addr = direct_address(bus);
    regs_.a = sbc8(regs_, regs_.a, read_byte(bus, addr));
    return 4;
}
uint8_t Cpu::op_sbca_ext(Bus& bus) {
    const uint16_t addr = extended_address(bus);
    regs_.a = sbc8(regs_, regs_.a, read_byte(bus, addr));
    return 5;
}
uint8_t Cpu::op_sbca_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    if (!pb.valid) return pb.cycles;
    regs_.a = sbc8(regs_, regs_.a, read_byte(bus, pb.address));
    return static_cast<uint8_t>(4 + pb.cycles);
}

uint8_t Cpu::op_sbcb_imm(Bus& bus) {
    regs_.b = sbc8(regs_, regs_.b, fetch_byte(bus));
    return 2;
}
uint8_t Cpu::op_sbcb_dir(Bus& bus) {
    const uint16_t addr = direct_address(bus);
    regs_.b = sbc8(regs_, regs_.b, read_byte(bus, addr));
    return 4;
}
uint8_t Cpu::op_sbcb_ext(Bus& bus) {
    const uint16_t addr = extended_address(bus);
    regs_.b = sbc8(regs_, regs_.b, read_byte(bus, addr));
    return 5;
}
uint8_t Cpu::op_sbcb_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    if (!pb.valid) return pb.cycles;
    regs_.b = sbc8(regs_, regs_.b, read_byte(bus, pb.address));
    return static_cast<uint8_t>(4 + pb.cycles);
}

uint8_t Cpu::op_adda_imm(Bus& bus) {
    regs_.a = add8(regs_, regs_.a, fetch_byte(bus));
    return 2;
}
uint8_t Cpu::op_adda_dir(Bus& bus) {
    const uint16_t addr = direct_address(bus);
    regs_.a = add8(regs_, regs_.a, read_byte(bus, addr));
    return 4;
}
uint8_t Cpu::op_adda_ext(Bus& bus) {
    const uint16_t addr = extended_address(bus);
    regs_.a = add8(regs_, regs_.a, read_byte(bus, addr));
    return 5;
}
uint8_t Cpu::op_adda_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    if (!pb.valid) return pb.cycles;
    regs_.a = add8(regs_, regs_.a, read_byte(bus, pb.address));
    return static_cast<uint8_t>(4 + pb.cycles);
}

uint8_t Cpu::op_addb_imm(Bus& bus) {
    regs_.b = add8(regs_, regs_.b, fetch_byte(bus));
    return 2;
}
uint8_t Cpu::op_addb_dir(Bus& bus) {
    const uint16_t addr = direct_address(bus);
    regs_.b = add8(regs_, regs_.b, read_byte(bus, addr));
    return 4;
}
uint8_t Cpu::op_addb_ext(Bus& bus) {
    const uint16_t addr = extended_address(bus);
    regs_.b = add8(regs_, regs_.b, read_byte(bus, addr));
    return 5;
}
uint8_t Cpu::op_addb_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    if (!pb.valid) return pb.cycles;
    regs_.b = add8(regs_, regs_.b, read_byte(bus, pb.address));
    return static_cast<uint8_t>(4 + pb.cycles);
}

uint8_t Cpu::op_suba_imm(Bus& bus) {
    regs_.a = sub8(regs_, regs_.a, fetch_byte(bus));
    return 2;
}
uint8_t Cpu::op_suba_dir(Bus& bus) {
    const uint16_t addr = direct_address(bus);
    regs_.a = sub8(regs_, regs_.a, read_byte(bus, addr));
    return 4;
}
uint8_t Cpu::op_suba_ext(Bus& bus) {
    const uint16_t addr = extended_address(bus);
    regs_.a = sub8(regs_, regs_.a, read_byte(bus, addr));
    return 5;
}
uint8_t Cpu::op_suba_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    if (!pb.valid) return pb.cycles;
    regs_.a = sub8(regs_, regs_.a, read_byte(bus, pb.address));
    return static_cast<uint8_t>(4 + pb.cycles);
}

uint8_t Cpu::op_subb_imm(Bus& bus) {
    regs_.b = sub8(regs_, regs_.b, fetch_byte(bus));
    return 2;
}
uint8_t Cpu::op_subb_dir(Bus& bus) {
    const uint16_t addr = direct_address(bus);
    regs_.b = sub8(regs_, regs_.b, read_byte(bus, addr));
    return 4;
}
uint8_t Cpu::op_subb_ext(Bus& bus) {
    const uint16_t addr = extended_address(bus);
    regs_.b = sub8(regs_, regs_.b, read_byte(bus, addr));
    return 5;
}
uint8_t Cpu::op_subb_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    if (!pb.valid) return pb.cycles;
    regs_.b = sub8(regs_, regs_.b, read_byte(bus, pb.address));
    return static_cast<uint8_t>(4 + pb.cycles);
}

uint8_t Cpu::op_cmpa_imm(Bus& bus) {
    sub8(regs_, regs_.a, fetch_byte(bus));
    return 2;
}
uint8_t Cpu::op_cmpa_dir(Bus& bus) {
    const uint16_t addr = direct_address(bus);
    sub8(regs_, regs_.a, read_byte(bus, addr));
    return 4;
}
uint8_t Cpu::op_cmpa_ext(Bus& bus) {
    const uint16_t addr = extended_address(bus);
    sub8(regs_, regs_.a, read_byte(bus, addr));
    return 5;
}
uint8_t Cpu::op_cmpa_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    if (!pb.valid) return pb.cycles;
    sub8(regs_, regs_.a, read_byte(bus, pb.address));
    return static_cast<uint8_t>(4 + pb.cycles);
}

uint8_t Cpu::op_cmpb_imm(Bus& bus) {
    sub8(regs_, regs_.b, fetch_byte(bus));
    return 2;
}
uint8_t Cpu::op_cmpb_dir(Bus& bus) {
    const uint16_t addr = direct_address(bus);
    sub8(regs_, regs_.b, read_byte(bus, addr));
    return 4;
}
uint8_t Cpu::op_cmpb_ext(Bus& bus) {
    const uint16_t addr = extended_address(bus);
    sub8(regs_, regs_.b, read_byte(bus, addr));
    return 5;
}
uint8_t Cpu::op_cmpb_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    if (!pb.valid) return pb.cycles;
    sub8(regs_, regs_.b, read_byte(bus, pb.address));
    return static_cast<uint8_t>(4 + pb.cycles);
}

uint8_t Cpu::op_addd_imm(Bus& bus) {
    const uint16_t value = fetch_word(bus);
    const uint16_t d = static_cast<uint16_t>((regs_.a << 8) | regs_.b);
    const uint16_t res = add16(regs_, d, value);
    regs_.a = hi(res);
    regs_.b = lo(res);
    return 4;
}

uint8_t Cpu::op_addd_dir(Bus& bus) {
    const uint16_t addr = direct_address(bus);
    const uint16_t value = static_cast<uint16_t>((read_byte(bus, addr) << 8) | read_byte(bus, static_cast<uint16_t>(addr + 1)));
    const uint16_t d = static_cast<uint16_t>((regs_.a << 8) | regs_.b);
    const uint16_t res = add16(regs_, d, value);
    regs_.a = hi(res);
    regs_.b = lo(res);
    return 6;
}

uint8_t Cpu::op_addd_ext(Bus& bus) {
    const uint16_t addr = extended_address(bus);
    const uint16_t value = static_cast<uint16_t>((read_byte(bus, addr) << 8) | read_byte(bus, static_cast<uint16_t>(addr + 1)));
    const uint16_t d = static_cast<uint16_t>((regs_.a << 8) | regs_.b);
    const uint16_t res = add16(regs_, d, value);
    regs_.a = hi(res);
    regs_.b = lo(res);
    return 7;
}

uint8_t Cpu::op_addd_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    if (!pb.valid) return pb.cycles;
    const uint16_t value =
        static_cast<uint16_t>((read_byte(bus, pb.address) << 8) | read_byte(bus, static_cast<uint16_t>(pb.address + 1)));
    const uint16_t d = static_cast<uint16_t>((regs_.a << 8) | regs_.b);
    const uint16_t res = add16(regs_, d, value);
    regs_.a = hi(res);
    regs_.b = lo(res);
    return static_cast<uint8_t>(6 + pb.cycles);
}

uint8_t Cpu::op_subd_imm(Bus& bus) {
    const uint16_t value = fetch_word(bus);
    const uint16_t d = static_cast<uint16_t>((regs_.a << 8) | regs_.b);
    const uint16_t res = sub16(regs_, d, value);
    regs_.a = hi(res);
    regs_.b = lo(res);
    return 4;
}

uint8_t Cpu::op_subd_dir(Bus& bus) {
    const uint16_t addr = direct_address(bus);
    const uint16_t value = static_cast<uint16_t>((read_byte(bus, addr) << 8) | read_byte(bus, static_cast<uint16_t>(addr + 1)));
    const uint16_t d = static_cast<uint16_t>((regs_.a << 8) | regs_.b);
    const uint16_t res = sub16(regs_, d, value);
    regs_.a = hi(res);
    regs_.b = lo(res);
    return 6;
}

uint8_t Cpu::op_subd_ext(Bus& bus) {
    const uint16_t addr = extended_address(bus);
    const uint16_t value = static_cast<uint16_t>((read_byte(bus, addr) << 8) | read_byte(bus, static_cast<uint16_t>(addr + 1)));
    const uint16_t d = static_cast<uint16_t>((regs_.a << 8) | regs_.b);
    const uint16_t res = sub16(regs_, d, value);
    regs_.a = hi(res);
    regs_.b = lo(res);
    return 7;
}

uint8_t Cpu::op_subd_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    if (!pb.valid) return pb.cycles;
    const uint16_t value =
        static_cast<uint16_t>((read_byte(bus, pb.address) << 8) | read_byte(bus, static_cast<uint16_t>(pb.address + 1)));
    const uint16_t d = static_cast<uint16_t>((regs_.a << 8) | regs_.b);
    const uint16_t res = sub16(regs_, d, value);
    regs_.a = hi(res);
    regs_.b = lo(res);
    return static_cast<uint8_t>(6 + pb.cycles);
}

uint8_t Cpu::op_cmpd_imm(Bus& bus) {
    const uint16_t value = fetch_word(bus);
    const uint16_t d = static_cast<uint16_t>((regs_.a << 8) | regs_.b);
    sub16(regs_, d, value);
    return 5; // prefix opcode adds one cycle
}

uint8_t Cpu::op_cmpd_dir(Bus& bus) {
    const uint16_t addr = direct_address(bus);
    const uint16_t value = static_cast<uint16_t>((read_byte(bus, addr) << 8) | read_byte(bus, static_cast<uint16_t>(addr + 1)));
    const uint16_t d = static_cast<uint16_t>((regs_.a << 8) | regs_.b);
    sub16(regs_, d, value);
    return 7;
}

uint8_t Cpu::op_cmpd_ext(Bus& bus) {
    const uint16_t addr = extended_address(bus);
    const uint16_t value = static_cast<uint16_t>((read_byte(bus, addr) << 8) | read_byte(bus, static_cast<uint16_t>(addr + 1)));
    const uint16_t d = static_cast<uint16_t>((regs_.a << 8) | regs_.b);
    sub16(regs_, d, value);
    return 8;
}

uint8_t Cpu::op_cmpd_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    if (!pb.valid) return pb.cycles;
    const uint16_t value =
        static_cast<uint16_t>((read_byte(bus, pb.address) << 8) | read_byte(bus, static_cast<uint16_t>(pb.address + 1)));
    const uint16_t d = static_cast<uint16_t>((regs_.a << 8) | regs_.b);
    sub16(regs_, d, value);
    return static_cast<uint8_t>(7 + pb.cycles);
}

uint8_t Cpu::op_cmpx_imm(Bus& bus) {
    const uint16_t value = fetch_word(bus);
    sub16(regs_, regs_.x, value);
    return 4;
}
uint8_t Cpu::op_cmpx_dir(Bus& bus) {
    const uint16_t addr = direct_address(bus);
    sub16(regs_, regs_.x, read_word(bus, addr));
    return 6;
}
uint8_t Cpu::op_cmpx_ext(Bus& bus) {
    const uint16_t addr = extended_address(bus);
    sub16(regs_, regs_.x, read_word(bus, addr));
    return 7;
}
uint8_t Cpu::op_cmpx_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    if (!pb.valid) return pb.cycles;
    sub16(regs_, regs_.x, read_word(bus, pb.address));
    return static_cast<uint8_t>(6 + pb.cycles);
}

uint8_t Cpu::op_cmpy_imm(Bus& bus) {
    const uint16_t value = fetch_word(bus);
    sub16(regs_, regs_.y, value);
    return 5;
}
uint8_t Cpu::op_cmpy_dir(Bus& bus) {
    const uint16_t addr = direct_address(bus);
    sub16(regs_, regs_.y, read_word(bus, addr));
    return 7;
}
uint8_t Cpu::op_cmpy_ext(Bus& bus) {
    const uint16_t addr = extended_address(bus);
    sub16(regs_, regs_.y, read_word(bus, addr));
    return 8;
}
uint8_t Cpu::op_cmpy_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    if (!pb.valid) return pb.cycles;
    sub16(regs_, regs_.y, read_word(bus, pb.address));
    return static_cast<uint8_t>(7 + pb.cycles);
}

uint8_t Cpu::op_cmpu_imm(Bus& bus) {
    const uint16_t value = fetch_word(bus);
    sub16(regs_, regs_.u, value);
    return 5;
}
uint8_t Cpu::op_cmpu_dir(Bus& bus) {
    const uint16_t addr = direct_address(bus);
    sub16(regs_, regs_.u, read_word(bus, addr));
    return 7;
}
uint8_t Cpu::op_cmpu_ext(Bus& bus) {
    const uint16_t addr = extended_address(bus);
    sub16(regs_, regs_.u, read_word(bus, addr));
    return 8;
}
uint8_t Cpu::op_cmpu_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    if (!pb.valid) return pb.cycles;
    sub16(regs_, regs_.u, read_word(bus, pb.address));
    return static_cast<uint8_t>(7 + pb.cycles);
}

uint8_t Cpu::op_cmps_imm(Bus& bus) {
    const uint16_t value = fetch_word(bus);
    sub16(regs_, regs_.s, value);
    return 5;
}
uint8_t Cpu::op_cmps_dir(Bus& bus) {
    const uint16_t addr = direct_address(bus);
    sub16(regs_, regs_.s, read_word(bus, addr));
    return 7;
}
uint8_t Cpu::op_cmps_ext(Bus& bus) {
    const uint16_t addr = extended_address(bus);
    sub16(regs_, regs_.s, read_word(bus, addr));
    return 8;
}
uint8_t Cpu::op_cmps_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    if (!pb.valid) return pb.cycles;
    sub16(regs_, regs_.s, read_word(bus, pb.address));
    return static_cast<uint8_t>(7 + pb.cycles);
}

// ----- 16-bit loads/stores -----

uint8_t Cpu::op_ldx_imm(Bus& bus) {
    regs_.x = fetch_word(bus);
    set_flags_nz16(regs_.x);
    return 3;
}
uint8_t Cpu::op_ldx_dir(Bus& bus) {
    const uint16_t addr = direct_address(bus);
    regs_.x = read_word(bus, addr);
    set_flags_nz16(regs_.x);
    return 5;
}
uint8_t Cpu::op_ldx_ext(Bus& bus) {
    const uint16_t addr = extended_address(bus);
    regs_.x = read_word(bus, addr);
    set_flags_nz16(regs_.x);
    return 6;
}
uint8_t Cpu::op_ldx_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    if (!pb.valid) return pb.cycles;
    regs_.x = read_word(bus, pb.address);
    set_flags_nz16(regs_.x);
    return static_cast<uint8_t>(5 + pb.cycles);
}

uint8_t Cpu::op_ldy_imm(Bus& bus) {
    regs_.y = fetch_word(bus);
    set_flags_nz16(regs_.y);
    return 4;
}
uint8_t Cpu::op_ldy_dir(Bus& bus) {
    const uint16_t addr = direct_address(bus);
    regs_.y = read_word(bus, addr);
    set_flags_nz16(regs_.y);
    return 6;
}
uint8_t Cpu::op_ldy_ext(Bus& bus) {
    const uint16_t addr = extended_address(bus);
    regs_.y = read_word(bus, addr);
    set_flags_nz16(regs_.y);
    return 7;
}
uint8_t Cpu::op_ldy_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    if (!pb.valid) return pb.cycles;
    regs_.y = read_word(bus, pb.address);
    set_flags_nz16(regs_.y);
    return static_cast<uint8_t>(6 + pb.cycles);
}

uint8_t Cpu::op_ldu_imm(Bus& bus) {
    regs_.u = fetch_word(bus);
    set_flags_nz16(regs_.u);
    return 3;
}
uint8_t Cpu::op_ldu_dir(Bus& bus) {
    const uint16_t addr = direct_address(bus);
    regs_.u = read_word(bus, addr);
    set_flags_nz16(regs_.u);
    return 5;
}
uint8_t Cpu::op_ldu_ext(Bus& bus) {
    const uint16_t addr = extended_address(bus);
    regs_.u = read_word(bus, addr);
    set_flags_nz16(regs_.u);
    return 6;
}
uint8_t Cpu::op_ldu_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    if (!pb.valid) return pb.cycles;
    regs_.u = read_word(bus, pb.address);
    set_flags_nz16(regs_.u);
    return static_cast<uint8_t>(5 + pb.cycles);
}

uint8_t Cpu::op_lds_imm(Bus& bus) {
    regs_.s = fetch_word(bus);
    set_flags_nz16(regs_.s);
    return 4;
}
uint8_t Cpu::op_lds_dir(Bus& bus) {
    const uint16_t addr = direct_address(bus);
    regs_.s = read_word(bus, addr);
    set_flags_nz16(regs_.s);
    return 6;
}
uint8_t Cpu::op_lds_ext(Bus& bus) {
    const uint16_t addr = extended_address(bus);
    regs_.s = read_word(bus, addr);
    set_flags_nz16(regs_.s);
    return 7;
}
uint8_t Cpu::op_lds_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    if (!pb.valid) return pb.cycles;
    regs_.s = read_word(bus, pb.address);
    set_flags_nz16(regs_.s);
    return static_cast<uint8_t>(6 + pb.cycles);
}

uint8_t Cpu::op_stx_dir(Bus& bus) {
    const uint16_t addr = direct_address(bus);
    write_word(bus, addr, regs_.x);
    set_flags_nz16(regs_.x);
    return 5;
}
uint8_t Cpu::op_stx_ext(Bus& bus) {
    const uint16_t addr = extended_address(bus);
    write_word(bus, addr, regs_.x);
    set_flags_nz16(regs_.x);
    return 6;
}
uint8_t Cpu::op_stx_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    if (!pb.valid) return pb.cycles;
    write_word(bus, pb.address, regs_.x);
    set_flags_nz16(regs_.x);
    return static_cast<uint8_t>(5 + pb.cycles);
}

uint8_t Cpu::op_sty_dir(Bus& bus) {
    const uint16_t addr = direct_address(bus);
    write_word(bus, addr, regs_.y);
    set_flags_nz16(regs_.y);
    return 6;
}
uint8_t Cpu::op_sty_ext(Bus& bus) {
    const uint16_t addr = extended_address(bus);
    write_word(bus, addr, regs_.y);
    set_flags_nz16(regs_.y);
    return 7;
}
uint8_t Cpu::op_sty_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    if (!pb.valid) return pb.cycles;
    write_word(bus, pb.address, regs_.y);
    set_flags_nz16(regs_.y);
    return static_cast<uint8_t>(6 + pb.cycles);
}

uint8_t Cpu::op_stu_dir(Bus& bus) {
    const uint16_t addr = direct_address(bus);
    write_word(bus, addr, regs_.u);
    set_flags_nz16(regs_.u);
    return 5;
}
uint8_t Cpu::op_stu_ext(Bus& bus) {
    const uint16_t addr = extended_address(bus);
    write_word(bus, addr, regs_.u);
    set_flags_nz16(regs_.u);
    return 6;
}
uint8_t Cpu::op_stu_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    if (!pb.valid) return pb.cycles;
    write_word(bus, pb.address, regs_.u);
    set_flags_nz16(regs_.u);
    return static_cast<uint8_t>(5 + pb.cycles);
}

uint8_t Cpu::op_sts_dir(Bus& bus) {
    const uint16_t addr = direct_address(bus);
    write_word(bus, addr, regs_.s);
    set_flags_nz16(regs_.s);
    return 6;
}
uint8_t Cpu::op_sts_ext(Bus& bus) {
    const uint16_t addr = extended_address(bus);
    write_word(bus, addr, regs_.s);
    set_flags_nz16(regs_.s);
    return 7;
}
uint8_t Cpu::op_sts_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    if (!pb.valid) return pb.cycles;
    write_word(bus, pb.address, regs_.s);
    set_flags_nz16(regs_.s);
    return static_cast<uint8_t>(6 + pb.cycles);
}

// ----- LEA -----

uint8_t Cpu::op_leax(Bus& bus) {
    const auto pb = indexed_address(bus);
    if (!pb.valid) return pb.cycles;
    regs_.x = pb.address;
    set_flags_nz16(regs_.x);
    return static_cast<uint8_t>(4 + pb.cycles);
}
uint8_t Cpu::op_leay(Bus& bus) {
    const auto pb = indexed_address(bus);
    if (!pb.valid) return pb.cycles;
    regs_.y = pb.address;
    set_flags_nz16(regs_.y);
    return static_cast<uint8_t>(4 + pb.cycles);
}
uint8_t Cpu::op_leas(Bus& bus) {
    const auto pb = indexed_address(bus);
    if (!pb.valid) return pb.cycles;
    regs_.s = pb.address;
    return static_cast<uint8_t>(4 + pb.cycles);
}
uint8_t Cpu::op_leau(Bus& bus) {
    const auto pb = indexed_address(bus);
    if (!pb.valid) return pb.cycles;
    regs_.u = pb.address;
    set_flags_nz16(regs_.u);
    return static_cast<uint8_t>(4 + pb.cycles);
}

// ----- Stack -----

uint8_t Cpu::op_pshs(Bus& bus) {
    const uint8_t mask = fetch_byte(bus);
    uint8_t count = 0;
    if (mask & 0x80) { push_byte(bus, regs_.cc); ++count; }
    if (mask & 0x40) { push_byte(bus, regs_.a); ++count; }
    if (mask & 0x20) { push_byte(bus, regs_.b); ++count; }
    if (mask & 0x10) { push_byte(bus, regs_.dp); ++count; }
    if (mask & 0x08) { push_word(bus, regs_.x); count += 2; }
    if (mask & 0x04) { push_word(bus, regs_.y); count += 2; }
    if (mask & 0x02) { push_word(bus, regs_.u); count += 2; }
    if (mask & 0x01) { push_word(bus, regs_.pc); count += 2; }
    return static_cast<uint8_t>(5 + count);
}

uint8_t Cpu::op_puls(Bus& bus) {
    const uint8_t mask = fetch_byte(bus);
    uint8_t count = 0;
    if (mask & 0x01) { regs_.pc = pull_word(bus); count += 2; }
    if (mask & 0x02) { regs_.u = pull_word(bus); count += 2; }
    if (mask & 0x04) { regs_.y = pull_word(bus); count += 2; }
    if (mask & 0x08) { regs_.x = pull_word(bus); count += 2; }
    if (mask & 0x10) { regs_.dp = pull_byte(bus); ++count; }
    if (mask & 0x20) { regs_.b = pull_byte(bus); ++count; }
    if (mask & 0x40) { regs_.a = pull_byte(bus); ++count; }
    if (mask & 0x80) { regs_.cc = pull_byte(bus); ++count; }
    return static_cast<uint8_t>(5 + count);
}

uint8_t Cpu::op_pshu(Bus& bus) {
    const uint8_t mask = fetch_byte(bus);
    uint8_t count = 0;
    if (mask & 0x80) { regs_.u = static_cast<uint16_t>(regs_.u - 1); write_byte(bus, regs_.u, regs_.cc); ++count; }
    if (mask & 0x40) { regs_.u = static_cast<uint16_t>(regs_.u - 1); write_byte(bus, regs_.u, regs_.a); ++count; }
    if (mask & 0x20) { regs_.u = static_cast<uint16_t>(regs_.u - 1); write_byte(bus, regs_.u, regs_.b); ++count; }
    if (mask & 0x10) { regs_.u = static_cast<uint16_t>(regs_.u - 1); write_byte(bus, regs_.u, regs_.dp); ++count; }
    if (mask & 0x08) { regs_.u = static_cast<uint16_t>(regs_.u - 2); write_byte(bus, static_cast<uint16_t>(regs_.u + 1), hi(regs_.x)); write_byte(bus, regs_.u, lo(regs_.x)); count += 2; }
    if (mask & 0x04) { regs_.u = static_cast<uint16_t>(regs_.u - 2); write_byte(bus, static_cast<uint16_t>(regs_.u + 1), hi(regs_.y)); write_byte(bus, regs_.u, lo(regs_.y)); count += 2; }
    if (mask & 0x02) { regs_.u = static_cast<uint16_t>(regs_.u - 2); write_byte(bus, static_cast<uint16_t>(regs_.u + 1), hi(regs_.s)); write_byte(bus, regs_.u, lo(regs_.s)); count += 2; }
    if (mask & 0x01) { regs_.u = static_cast<uint16_t>(regs_.u - 2); write_byte(bus, static_cast<uint16_t>(regs_.u + 1), hi(regs_.pc)); write_byte(bus, regs_.u, lo(regs_.pc)); count += 2; }
    return static_cast<uint8_t>(5 + count);
}

uint8_t Cpu::op_pulu(Bus& bus) {
    const uint8_t mask = fetch_byte(bus);
    uint8_t count = 0;
    if (mask & 0x01) { regs_.pc = static_cast<uint16_t>((read_byte(bus, regs_.u + 1) << 8) | read_byte(bus, regs_.u)); regs_.u = static_cast<uint16_t>(regs_.u + 2); count += 2; }
    if (mask & 0x02) { regs_.s = static_cast<uint16_t>((read_byte(bus, regs_.u + 1) << 8) | read_byte(bus, regs_.u)); regs_.u = static_cast<uint16_t>(regs_.u + 2); count += 2; }
    if (mask & 0x04) { regs_.y = static_cast<uint16_t>((read_byte(bus, regs_.u + 1) << 8) | read_byte(bus, regs_.u)); regs_.u = static_cast<uint16_t>(regs_.u + 2); count += 2; }
    if (mask & 0x08) { regs_.x = static_cast<uint16_t>((read_byte(bus, regs_.u + 1) << 8) | read_byte(bus, regs_.u)); regs_.u = static_cast<uint16_t>(regs_.u + 2); count += 2; }
    if (mask & 0x10) { regs_.dp = read_byte(bus, regs_.u); regs_.u = static_cast<uint16_t>(regs_.u + 1); ++count; }
    if (mask & 0x20) { regs_.b = read_byte(bus, regs_.u); regs_.u = static_cast<uint16_t>(regs_.u + 1); ++count; }
    if (mask & 0x40) { regs_.a = read_byte(bus, regs_.u); regs_.u = static_cast<uint16_t>(regs_.u + 1); ++count; }
    if (mask & 0x80) { regs_.cc = read_byte(bus, regs_.u); regs_.u = static_cast<uint16_t>(regs_.u + 1); ++count; }
    return static_cast<uint8_t>(5 + count);
}

// ----- Accumulator unary/shift -----

uint8_t Cpu::op_nega(Bus&) {
    regs_.a = neg8_op(regs_, regs_.a);
    return 2;
}
uint8_t Cpu::op_negb(Bus&) {
    regs_.b = neg8_op(regs_, regs_.b);
    return 2;
}
uint8_t Cpu::op_coma(Bus&) {
    regs_.a = com8_op(regs_, regs_.a);
    return 2;
}
uint8_t Cpu::op_comb(Bus&) {
    regs_.b = com8_op(regs_, regs_.b);
    return 2;
}
uint8_t Cpu::op_lsra(Bus&) {
    regs_.a = lsr8_op(regs_, regs_.a);
    return 2;
}
uint8_t Cpu::op_lsrb(Bus&) {
    regs_.b = lsr8_op(regs_, regs_.b);
    return 2;
}
uint8_t Cpu::op_rora(Bus&) {
    regs_.a = ror8_op(regs_, regs_.a);
    return 2;
}
uint8_t Cpu::op_rorb(Bus&) {
    regs_.b = ror8_op(regs_, regs_.b);
    return 2;
}
uint8_t Cpu::op_asra(Bus&) {
    regs_.a = asr8_op(regs_, regs_.a);
    return 2;
}
uint8_t Cpu::op_asrb(Bus&) {
    regs_.b = asr8_op(regs_, regs_.b);
    return 2;
}
uint8_t Cpu::op_asla(Bus&) {
    regs_.a = asl8_op(regs_, regs_.a);
    return 2;
}
uint8_t Cpu::op_aslb(Bus&) {
    regs_.b = asl8_op(regs_, regs_.b);
    return 2;
}
uint8_t Cpu::op_rola(Bus&) {
    regs_.a = rol8_op(regs_, regs_.a);
    return 2;
}
uint8_t Cpu::op_rolb(Bus&) {
    regs_.b = rol8_op(regs_, regs_.b);
    return 2;
}
uint8_t Cpu::op_deca(Bus&) {
    regs_.a = dec8(regs_, regs_.a);
    return 2;
}
uint8_t Cpu::op_decb(Bus&) {
    regs_.b = dec8(regs_, regs_.b);
    return 2;
}
uint8_t Cpu::op_inca(Bus&) {
    regs_.a = inc8(regs_, regs_.a);
    return 2;
}
uint8_t Cpu::op_incb(Bus&) {
    regs_.b = inc8(regs_, regs_.b);
    return 2;
}
uint8_t Cpu::op_tsta(Bus&) {
    set_flags_tst(regs_, regs_.a);
    return 2;
}
uint8_t Cpu::op_tstb(Bus&) {
    set_flags_tst(regs_, regs_.b);
    return 2;
}

// ----- Memory unary/shift -----

uint8_t Cpu::op_neg_dir(Bus& bus) {
    const uint16_t addr = direct_address(bus);
    const uint8_t res = neg8_op(regs_, read_byte(bus, addr));
    write_byte(bus, addr, res);
    return 6;
}
uint8_t Cpu::op_neg_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    if (!pb.valid) return pb.cycles;
    const uint8_t res = neg8_op(regs_, read_byte(bus, pb.address));
    write_byte(bus, pb.address, res);
    return static_cast<uint8_t>(6 + pb.cycles);
}
uint8_t Cpu::op_neg_ext(Bus& bus) {
    const uint16_t addr = extended_address(bus);
    const uint8_t res = neg8_op(regs_, read_byte(bus, addr));
    write_byte(bus, addr, res);
    return 7;
}

uint8_t Cpu::op_com_dir(Bus& bus) {
    const uint16_t addr = direct_address(bus);
    const uint8_t res = com8_op(regs_, read_byte(bus, addr));
    write_byte(bus, addr, res);
    return 6;
}
uint8_t Cpu::op_com_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    if (!pb.valid) return pb.cycles;
    const uint8_t res = com8_op(regs_, read_byte(bus, pb.address));
    write_byte(bus, pb.address, res);
    return static_cast<uint8_t>(6 + pb.cycles);
}
uint8_t Cpu::op_com_ext(Bus& bus) {
    const uint16_t addr = extended_address(bus);
    const uint8_t res = com8_op(regs_, read_byte(bus, addr));
    write_byte(bus, addr, res);
    return 7;
}

uint8_t Cpu::op_lsr_dir(Bus& bus) {
    const uint16_t addr = direct_address(bus);
    const uint8_t res = lsr8_op(regs_, read_byte(bus, addr));
    write_byte(bus, addr, res);
    return 6;
}
uint8_t Cpu::op_lsr_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    if (!pb.valid) return pb.cycles;
    const uint8_t res = lsr8_op(regs_, read_byte(bus, pb.address));
    write_byte(bus, pb.address, res);
    return static_cast<uint8_t>(6 + pb.cycles);
}
uint8_t Cpu::op_lsr_ext(Bus& bus) {
    const uint16_t addr = extended_address(bus);
    const uint8_t res = lsr8_op(regs_, read_byte(bus, addr));
    write_byte(bus, addr, res);
    return 7;
}

uint8_t Cpu::op_ror_dir(Bus& bus) {
    const uint16_t addr = direct_address(bus);
    const uint8_t res = ror8_op(regs_, read_byte(bus, addr));
    write_byte(bus, addr, res);
    return 6;
}
uint8_t Cpu::op_ror_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    if (!pb.valid) return pb.cycles;
    const uint8_t res = ror8_op(regs_, read_byte(bus, pb.address));
    write_byte(bus, pb.address, res);
    return static_cast<uint8_t>(6 + pb.cycles);
}
uint8_t Cpu::op_ror_ext(Bus& bus) {
    const uint16_t addr = extended_address(bus);
    const uint8_t res = ror8_op(regs_, read_byte(bus, addr));
    write_byte(bus, addr, res);
    return 7;
}

uint8_t Cpu::op_asr_dir(Bus& bus) {
    const uint16_t addr = direct_address(bus);
    const uint8_t res = asr8_op(regs_, read_byte(bus, addr));
    write_byte(bus, addr, res);
    return 6;
}
uint8_t Cpu::op_asr_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    if (!pb.valid) return pb.cycles;
    const uint8_t res = asr8_op(regs_, read_byte(bus, pb.address));
    write_byte(bus, pb.address, res);
    return static_cast<uint8_t>(6 + pb.cycles);
}
uint8_t Cpu::op_asr_ext(Bus& bus) {
    const uint16_t addr = extended_address(bus);
    const uint8_t res = asr8_op(regs_, read_byte(bus, addr));
    write_byte(bus, addr, res);
    return 7;
}

uint8_t Cpu::op_asl_dir(Bus& bus) {
    const uint16_t addr = direct_address(bus);
    const uint8_t res = asl8_op(regs_, read_byte(bus, addr));
    write_byte(bus, addr, res);
    return 6;
}
uint8_t Cpu::op_asl_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    if (!pb.valid) return pb.cycles;
    const uint8_t res = asl8_op(regs_, read_byte(bus, pb.address));
    write_byte(bus, pb.address, res);
    return static_cast<uint8_t>(6 + pb.cycles);
}
uint8_t Cpu::op_asl_ext(Bus& bus) {
    const uint16_t addr = extended_address(bus);
    const uint8_t res = asl8_op(regs_, read_byte(bus, addr));
    write_byte(bus, addr, res);
    return 7;
}

uint8_t Cpu::op_rol_dir(Bus& bus) {
    const uint16_t addr = direct_address(bus);
    const uint8_t res = rol8_op(regs_, read_byte(bus, addr));
    write_byte(bus, addr, res);
    return 6;
}
uint8_t Cpu::op_rol_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    if (!pb.valid) return pb.cycles;
    const uint8_t res = rol8_op(regs_, read_byte(bus, pb.address));
    write_byte(bus, pb.address, res);
    return static_cast<uint8_t>(6 + pb.cycles);
}
uint8_t Cpu::op_rol_ext(Bus& bus) {
    const uint16_t addr = extended_address(bus);
    const uint8_t res = rol8_op(regs_, read_byte(bus, addr));
    write_byte(bus, addr, res);
    return 7;
}

uint8_t Cpu::op_dec_dir(Bus& bus) {
    const uint16_t addr = direct_address(bus);
    const uint8_t res = dec8(regs_, read_byte(bus, addr));
    write_byte(bus, addr, res);
    return 6;
}
uint8_t Cpu::op_dec_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    if (!pb.valid) return pb.cycles;
    const uint8_t res = dec8(regs_, read_byte(bus, pb.address));
    write_byte(bus, pb.address, res);
    return static_cast<uint8_t>(6 + pb.cycles);
}
uint8_t Cpu::op_dec_ext(Bus& bus) {
    const uint16_t addr = extended_address(bus);
    const uint8_t res = dec8(regs_, read_byte(bus, addr));
    write_byte(bus, addr, res);
    return 7;
}

uint8_t Cpu::op_inc_dir(Bus& bus) {
    const uint16_t addr = direct_address(bus);
    const uint8_t res = inc8(regs_, read_byte(bus, addr));
    write_byte(bus, addr, res);
    return 6;
}
uint8_t Cpu::op_inc_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    if (!pb.valid) return pb.cycles;
    const uint8_t res = inc8(regs_, read_byte(bus, pb.address));
    write_byte(bus, pb.address, res);
    return static_cast<uint8_t>(6 + pb.cycles);
}
uint8_t Cpu::op_inc_ext(Bus& bus) {
    const uint16_t addr = extended_address(bus);
    const uint8_t res = inc8(regs_, read_byte(bus, addr));
    write_byte(bus, addr, res);
    return 7;
}

uint8_t Cpu::op_tst_dir(Bus& bus) {
    const uint16_t addr = direct_address(bus);
    const uint8_t val = read_byte(bus, addr);
    set_flags_tst(regs_, val);
    return 6;
}
uint8_t Cpu::op_tst_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    if (!pb.valid) return pb.cycles;
    const uint8_t val = read_byte(bus, pb.address);
    set_flags_tst(regs_, val);
    return static_cast<uint8_t>(6 + pb.cycles);
}
uint8_t Cpu::op_tst_ext(Bus& bus) {
    const uint16_t addr = extended_address(bus);
    const uint8_t val = read_byte(bus, addr);
    set_flags_tst(regs_, val);
    return 7;
}

uint8_t Cpu::op_clr_dir(Bus& bus) {
    const uint16_t addr = direct_address(bus);
    write_byte(bus, addr, 0);
    regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_V | CC_C));
    regs_.cc |= CC_Z;
    return 6;
}
uint8_t Cpu::op_clr_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    if (!pb.valid) return pb.cycles;
    write_byte(bus, pb.address, 0);
    regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_V | CC_C));
    regs_.cc |= CC_Z;
    return static_cast<uint8_t>(6 + pb.cycles);
}
uint8_t Cpu::op_clr_ext(Bus& bus) {
    const uint16_t addr = extended_address(bus);
    write_byte(bus, addr, 0);
    regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_V | CC_C));
    regs_.cc |= CC_Z;
    return 7;
}

// ----- Misc -----

uint8_t Cpu::op_abx(Bus&) {
    regs_.x = static_cast<uint16_t>(regs_.x + regs_.b);
    return 3;
}

uint8_t Cpu::op_sex(Bus&) {
    regs_.b = (regs_.a & 0x80) ? 0xFF : 0x00;
    set_flags_nz16(static_cast<uint16_t>((regs_.a << 8) | regs_.b));
    return 2;
}

uint8_t Cpu::op_andcc(Bus& bus) {
    const uint8_t mask = fetch_byte(bus);
    regs_.cc &= mask;
    return 3;
}

uint8_t Cpu::op_orcc(Bus& bus) {
    const uint8_t mask = fetch_byte(bus);
    regs_.cc |= mask;
    return 3;
}

uint8_t Cpu::op_daa(Bus&) {
    uint8_t adjust = 0;
    bool carry = (regs_.cc & CC_C) != 0;
    if ((regs_.a & 0x0F) > 9 || (regs_.cc & CC_H)) {
        adjust |= 0x06;
    }
    if ((regs_.a > 0x99) || carry) {
        adjust |= 0x60;
        carry = true;
    }
    const uint8_t result = static_cast<uint8_t>(regs_.a + adjust);
    regs_.a = result;
    regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | CC_C));
    if (result & 0x80) regs_.cc |= CC_N;
    if (result == 0) regs_.cc |= CC_Z;
    if (carry) regs_.cc |= CC_C;
    return 2;
}
uint8_t Cpu::op_tfr(Bus& bus) {
    (void)bus;
    const uint8_t post = fetch_byte(bus);
    const uint8_t src = post >> 4;
    const uint8_t dst = post & 0x0F;

    const bool dest16 = reg_is_16bit(dst);
    const uint16_t value = read_reg_for_dest(regs_, src, dest16);
    write_reg_sized(*this, dst, value, dest16);
    return 6;
}

uint8_t Cpu::op_exg(Bus& bus) {
    (void)bus;
    const uint8_t post = fetch_byte(bus);
    const uint8_t r1c = post >> 4;
    const uint8_t r2c = post & 0x0F;

    const bool wide = reg_is_16bit(r1c) || reg_is_16bit(r2c);
    const uint16_t v1 = read_reg_for_dest(regs_, r1c, wide);
    const uint16_t v2 = read_reg_for_dest(regs_, r2c, wide);
    write_reg_sized(*this, r1c, v2, wide);
    write_reg_sized(*this, r2c, v1, wide);
    return 8;
}

// ---- 6309 extensions ----

uint8_t Cpu::op_ldq_imm(Bus& bus) {
    const uint32_t val = (static_cast<uint32_t>(fetch_word(bus)) << 16) | fetch_word(bus);
    set_reg_q(val);
    regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z));
    if (val == 0) regs_.cc |= CC_Z;
    if (val & 0x80000000u) regs_.cc |= CC_N;
    return 5;
}
uint8_t Cpu::op_ldq_dir(Bus& bus) {
    const uint16_t addr = direct_address(bus);
    const uint32_t val = (static_cast<uint32_t>(read_word(bus, addr)) << 16) |
                         read_word(bus, static_cast<uint16_t>(addr + 2));
    set_reg_q(val);
    regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z));
    if (val == 0) regs_.cc |= CC_Z;
    if (val & 0x80000000u) regs_.cc |= CC_N;
    return 8;
}
uint8_t Cpu::op_ldq_ext(Bus& bus) {
    const uint16_t addr = extended_address(bus);
    const uint32_t val = (static_cast<uint32_t>(read_word(bus, addr)) << 16) |
                         read_word(bus, static_cast<uint16_t>(addr + 2));
    set_reg_q(val);
    regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z));
    if (val == 0) regs_.cc |= CC_Z;
    if (val & 0x80000000u) regs_.cc |= CC_N;
    return 9;
}
uint8_t Cpu::op_ldq_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    if (!pb.valid) return pb.cycles;
    const uint32_t val = (static_cast<uint32_t>(read_word(bus, pb.address)) << 16) |
                         read_word(bus, static_cast<uint16_t>(pb.address + 2));
    set_reg_q(val);
    regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z));
    if (val == 0) regs_.cc |= CC_Z;
    if (val & 0x80000000u) regs_.cc |= CC_N;
    return static_cast<uint8_t>(8 + pb.cycles);
}
uint8_t Cpu::op_stq_dir(Bus& bus) {
    const uint16_t addr = direct_address(bus);
    write_word(bus, addr, static_cast<uint16_t>((reg_q() >> 16) & 0xFFFF));
    write_word(bus, static_cast<uint16_t>(addr + 2), static_cast<uint16_t>(reg_q() & 0xFFFF));
    regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z));
    if (reg_q() == 0) regs_.cc |= CC_Z;
    if (reg_q() & 0x80000000u) regs_.cc |= CC_N;
    return 8;
}
uint8_t Cpu::op_stq_ext(Bus& bus) {
    const uint16_t addr = extended_address(bus);
    write_word(bus, addr, static_cast<uint16_t>((reg_q() >> 16) & 0xFFFF));
    write_word(bus, static_cast<uint16_t>(addr + 2), static_cast<uint16_t>(reg_q() & 0xFFFF));
    regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z));
    if (reg_q() == 0) regs_.cc |= CC_Z;
    if (reg_q() & 0x80000000u) regs_.cc |= CC_N;
    return 9;
}
uint8_t Cpu::op_stq_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    if (!pb.valid) return pb.cycles;
    write_word(bus, pb.address, static_cast<uint16_t>((reg_q() >> 16) & 0xFFFF));
    write_word(bus, static_cast<uint16_t>(pb.address + 2), static_cast<uint16_t>(reg_q() & 0xFFFF));
    regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z));
    if (reg_q() == 0) regs_.cc |= CC_Z;
    if (reg_q() & 0x80000000u) regs_.cc |= CC_N;
    return static_cast<uint8_t>(8 + pb.cycles);
}

uint8_t Cpu::op_ldw_imm(Bus& bus) {
    set_reg_w(fetch_word(bus));
    set_flags_nz16(reg_w());
    return 4;
}
uint8_t Cpu::op_ldw_dir(Bus& bus) {
    set_reg_w(read_word(bus, direct_address(bus)));
    set_flags_nz16(reg_w());
    return 6;
}
uint8_t Cpu::op_ldw_ext(Bus& bus) {
    set_reg_w(read_word(bus, extended_address(bus)));
    set_flags_nz16(reg_w());
    return 7;
}
uint8_t Cpu::op_ldw_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    if (!pb.valid) return pb.cycles;
    set_reg_w(read_word(bus, pb.address));
    set_flags_nz16(reg_w());
    return static_cast<uint8_t>(6 + pb.cycles);
}
uint8_t Cpu::op_stw_dir(Bus& bus) {
    write_word(bus, direct_address(bus), reg_w());
    set_flags_nz16(reg_w());
    return 6;
}
uint8_t Cpu::op_stw_ext(Bus& bus) {
    write_word(bus, extended_address(bus), reg_w());
    set_flags_nz16(reg_w());
    return 7;
}
uint8_t Cpu::op_stw_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    if (!pb.valid) return pb.cycles;
    write_word(bus, pb.address, reg_w());
    set_flags_nz16(reg_w());
    return static_cast<uint8_t>(6 + pb.cycles);
}

uint8_t Cpu::op_addw_imm(Bus& bus) {
    set_reg_w(add16(regs_, reg_w(), fetch_word(bus)));
    return 5;
}
uint8_t Cpu::op_addw_dir(Bus& bus) {
    set_reg_w(add16(regs_, reg_w(), read_word(bus, direct_address(bus))));
    return 7;
}
uint8_t Cpu::op_addw_ext(Bus& bus) {
    set_reg_w(add16(regs_, reg_w(), read_word(bus, extended_address(bus))));
    return 8;
}
uint8_t Cpu::op_addw_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    if (!pb.valid) return pb.cycles;
    set_reg_w(add16(regs_, reg_w(), read_word(bus, pb.address)));
    return static_cast<uint8_t>(7 + pb.cycles);
}
uint8_t Cpu::op_subw_imm(Bus& bus) {
    set_reg_w(sub16(regs_, reg_w(), fetch_word(bus)));
    return 5;
}
uint8_t Cpu::op_subw_dir(Bus& bus) {
    set_reg_w(sub16(regs_, reg_w(), read_word(bus, direct_address(bus))));
    return 7;
}
uint8_t Cpu::op_subw_ext(Bus& bus) {
    set_reg_w(sub16(regs_, reg_w(), read_word(bus, extended_address(bus))));
    return 8;
}
uint8_t Cpu::op_subw_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    if (!pb.valid) return pb.cycles;
    set_reg_w(sub16(regs_, reg_w(), read_word(bus, pb.address)));
    return static_cast<uint8_t>(7 + pb.cycles);
}
uint8_t Cpu::op_cmpw_imm(Bus& bus) {
    sub16(regs_, reg_w(), fetch_word(bus));
    return 5;
}
uint8_t Cpu::op_cmpw_dir(Bus& bus) {
    sub16(regs_, reg_w(), read_word(bus, direct_address(bus)));
    return 7;
}
uint8_t Cpu::op_cmpw_ext(Bus& bus) {
    sub16(regs_, reg_w(), read_word(bus, extended_address(bus)));
    return 8;
}
uint8_t Cpu::op_cmpw_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    if (!pb.valid) return pb.cycles;
    sub16(regs_, reg_w(), read_word(bus, pb.address));
    return static_cast<uint8_t>(7 + pb.cycles);
}

uint8_t Cpu::op_incw_inh(Bus&) {
    set_reg_w(inc16(regs_, reg_w()));
    return 3;
}
uint8_t Cpu::op_decw_inh(Bus&) {
    set_reg_w(dec16(regs_, reg_w()));
    return 3;
}
uint8_t Cpu::op_tstw_inh(Bus&) {
    set_flags_tst16(regs_, reg_w());
    return 3;
}
uint8_t Cpu::op_clrw_inh(Bus&) {
    set_reg_w(0);
    regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_V | CC_C));
    regs_.cc |= CC_Z;
    return 3;
}

uint8_t Cpu::op_come(Bus&) {
    regs_.e = com8_op(regs_, regs_.e);
    return 3;
}
uint8_t Cpu::op_comf(Bus&) {
    regs_.f = com8_op(regs_, regs_.f);
    return 3;
}
uint8_t Cpu::op_dece(Bus&) {
    regs_.e = dec8(regs_, regs_.e);
    return 3;
}
uint8_t Cpu::op_decf(Bus&) {
    regs_.f = dec8(regs_, regs_.f);
    return 3;
}
uint8_t Cpu::op_ince(Bus&) {
    regs_.e = inc8(regs_, regs_.e);
    return 3;
}
uint8_t Cpu::op_incf(Bus&) {
    regs_.f = inc8(regs_, regs_.f);
    return 3;
}
uint8_t Cpu::op_tste(Bus&) {
    set_flags_tst(regs_, regs_.e);
    return 3;
}
uint8_t Cpu::op_tstf(Bus&) {
    set_flags_tst(regs_, regs_.f);
    return 3;
}
uint8_t Cpu::op_clre(Bus&) {
    regs_.e = 0;
    regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_V | CC_C));
    regs_.cc |= CC_Z;
    return 3;
}
uint8_t Cpu::op_clrf(Bus&) {
    regs_.f = 0;
    regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_V | CC_C));
    regs_.cc |= CC_Z;
    return 3;
}

uint8_t Cpu::op_adde_imm(Bus& bus) { regs_.e = add8(regs_, regs_.e, fetch_byte(bus)); return 3; }
uint8_t Cpu::op_adde_dir(Bus& bus) { regs_.e = add8(regs_, regs_.e, read_byte(bus, direct_address(bus))); return 5; }
uint8_t Cpu::op_adde_ext(Bus& bus) { regs_.e = add8(regs_, regs_.e, read_byte(bus, extended_address(bus))); return 6; }
uint8_t Cpu::op_adde_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    if (!pb.valid) return pb.cycles;
    regs_.e = add8(regs_, regs_.e, read_byte(bus, pb.address));
    return static_cast<uint8_t>(5 + pb.cycles);
}
uint8_t Cpu::op_addf_imm(Bus& bus) { regs_.f = add8(regs_, regs_.f, fetch_byte(bus)); return 3; }
uint8_t Cpu::op_addf_dir(Bus& bus) { regs_.f = add8(regs_, regs_.f, read_byte(bus, direct_address(bus))); return 5; }
uint8_t Cpu::op_addf_ext(Bus& bus) { regs_.f = add8(regs_, regs_.f, read_byte(bus, extended_address(bus))); return 6; }
uint8_t Cpu::op_addf_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    if (!pb.valid) return pb.cycles;
    regs_.f = add8(regs_, regs_.f, read_byte(bus, pb.address));
    return static_cast<uint8_t>(5 + pb.cycles);
}
uint8_t Cpu::op_sube_imm(Bus& bus) { regs_.e = sub8(regs_, regs_.e, fetch_byte(bus)); return 3; }
uint8_t Cpu::op_sube_dir(Bus& bus) { regs_.e = sub8(regs_, regs_.e, read_byte(bus, direct_address(bus))); return 5; }
uint8_t Cpu::op_sube_ext(Bus& bus) { regs_.e = sub8(regs_, regs_.e, read_byte(bus, extended_address(bus))); return 6; }
uint8_t Cpu::op_sube_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    if (!pb.valid) return pb.cycles;
    regs_.e = sub8(regs_, regs_.e, read_byte(bus, pb.address));
    return static_cast<uint8_t>(5 + pb.cycles);
}
uint8_t Cpu::op_subf_imm(Bus& bus) { regs_.f = sub8(regs_, regs_.f, fetch_byte(bus)); return 3; }
uint8_t Cpu::op_subf_dir(Bus& bus) { regs_.f = sub8(regs_, regs_.f, read_byte(bus, direct_address(bus))); return 5; }
uint8_t Cpu::op_subf_ext(Bus& bus) { regs_.f = sub8(regs_, regs_.f, read_byte(bus, extended_address(bus))); return 6; }
uint8_t Cpu::op_subf_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    if (!pb.valid) return pb.cycles;
    regs_.f = sub8(regs_, regs_.f, read_byte(bus, pb.address));
    return static_cast<uint8_t>(5 + pb.cycles);
}
uint8_t Cpu::op_cmpe_imm(Bus& bus) { sub8(regs_, regs_.e, fetch_byte(bus)); return 3; }
uint8_t Cpu::op_cmpe_dir(Bus& bus) { sub8(regs_, regs_.e, read_byte(bus, direct_address(bus))); return 5; }
uint8_t Cpu::op_cmpe_ext(Bus& bus) { sub8(regs_, regs_.e, read_byte(bus, extended_address(bus))); return 6; }
uint8_t Cpu::op_cmpe_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    if (!pb.valid) return pb.cycles;
    sub8(regs_, regs_.e, read_byte(bus, pb.address));
    return static_cast<uint8_t>(5 + pb.cycles);
}
uint8_t Cpu::op_cmpf_imm(Bus& bus) { sub8(regs_, regs_.f, fetch_byte(bus)); return 3; }
uint8_t Cpu::op_cmpf_dir(Bus& bus) { sub8(regs_, regs_.f, read_byte(bus, direct_address(bus))); return 5; }
uint8_t Cpu::op_cmpf_ext(Bus& bus) { sub8(regs_, regs_.f, read_byte(bus, extended_address(bus))); return 6; }
uint8_t Cpu::op_cmpf_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    if (!pb.valid) return pb.cycles;
    sub8(regs_, regs_.f, read_byte(bus, pb.address));
    return static_cast<uint8_t>(5 + pb.cycles);
}

uint8_t Cpu::op_adcd_dir(Bus& bus) {
    const uint16_t val = read_word(bus, direct_address(bus));
    const uint16_t res = add16_carry(regs_, static_cast<uint16_t>((regs_.a << 8) | regs_.b), val, (regs_.cc & CC_C) != 0);
    regs_.a = hi(res);
    regs_.b = lo(res);
    return 7;
}
uint8_t Cpu::op_adcd_imm(Bus& bus) {
    const uint16_t val = fetch_word(bus);
    const uint16_t res = add16_carry(regs_, static_cast<uint16_t>((regs_.a << 8) | regs_.b), val, (regs_.cc & CC_C) != 0);
    regs_.a = hi(res);
    regs_.b = lo(res);
    return 5;
}
uint8_t Cpu::op_adcd_ext(Bus& bus) {
    const uint16_t val = read_word(bus, extended_address(bus));
    const uint16_t res = add16_carry(regs_, static_cast<uint16_t>((regs_.a << 8) | regs_.b), val, (regs_.cc & CC_C) != 0);
    regs_.a = hi(res);
    regs_.b = lo(res);
    return 8;
}
uint8_t Cpu::op_adcd_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    if (!pb.valid) return pb.cycles;
    const uint16_t val = read_word(bus, pb.address);
    const uint16_t res = add16_carry(regs_, static_cast<uint16_t>((regs_.a << 8) | regs_.b), val, (regs_.cc & CC_C) != 0);
    regs_.a = hi(res);
    regs_.b = lo(res);
    return static_cast<uint8_t>(7 + pb.cycles);
}
uint8_t Cpu::op_sbcd_imm(Bus& bus) {
    const uint16_t val = fetch_word(bus);
    const uint16_t res = sub16_carry(regs_, static_cast<uint16_t>((regs_.a << 8) | regs_.b), val, (regs_.cc & CC_C) != 0);
    regs_.a = hi(res);
    regs_.b = lo(res);
    return 5;
}
uint8_t Cpu::op_sbcd_dir(Bus& bus) {
    const uint16_t val = read_word(bus, direct_address(bus));
    const uint16_t res = sub16_carry(regs_, static_cast<uint16_t>((regs_.a << 8) | regs_.b), val, (regs_.cc & CC_C) != 0);
    regs_.a = hi(res);
    regs_.b = lo(res);
    return 7;
}
uint8_t Cpu::op_sbcd_ext(Bus& bus) {
    const uint16_t val = read_word(bus, extended_address(bus));
    const uint16_t res = sub16_carry(regs_, static_cast<uint16_t>((regs_.a << 8) | regs_.b), val, (regs_.cc & CC_C) != 0);
    regs_.a = hi(res);
    regs_.b = lo(res);
    return 8;
}
uint8_t Cpu::op_sbcd_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    if (!pb.valid) return pb.cycles;
    const uint16_t val = read_word(bus, pb.address);
    const uint16_t res = sub16_carry(regs_, static_cast<uint16_t>((regs_.a << 8) | regs_.b), val, (regs_.cc & CC_C) != 0);
    regs_.a = hi(res);
    regs_.b = lo(res);
    return static_cast<uint8_t>(7 + pb.cycles);
}

uint8_t Cpu::op_ord_dir(Bus& bus) {
    const uint16_t val = read_word(bus, direct_address(bus));
    const uint16_t res = static_cast<uint16_t>(((regs_.a << 8) | regs_.b) | val);
    regs_.a = hi(res);
    regs_.b = lo(res);
    regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | CC_C));
    if (res == 0) regs_.cc |= CC_Z;
    if (res & 0x8000) regs_.cc |= CC_N;
    return 7;
}
uint8_t Cpu::op_ord_imm(Bus& bus) {
    const uint16_t val = fetch_word(bus);
    const uint16_t res = static_cast<uint16_t>(((regs_.a << 8) | regs_.b) | val);
    regs_.a = hi(res);
    regs_.b = lo(res);
    regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | CC_C));
    if (res == 0) regs_.cc |= CC_Z;
    if (res & 0x8000) regs_.cc |= CC_N;
    return 5;
}
uint8_t Cpu::op_ord_ext(Bus& bus) {
    const uint16_t val = read_word(bus, extended_address(bus));
    const uint16_t res = static_cast<uint16_t>(((regs_.a << 8) | regs_.b) | val);
    regs_.a = hi(res);
    regs_.b = lo(res);
    regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | CC_C));
    if (res == 0) regs_.cc |= CC_Z;
    if (res & 0x8000) regs_.cc |= CC_N;
    return 8;
}
uint8_t Cpu::op_ord_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    if (!pb.valid) return pb.cycles;
    const uint16_t val = read_word(bus, pb.address);
    const uint16_t res = static_cast<uint16_t>(((regs_.a << 8) | regs_.b) | val);
    regs_.a = hi(res);
    regs_.b = lo(res);
    regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | CC_C));
    if (res == 0) regs_.cc |= CC_Z;
    if (res & 0x8000) regs_.cc |= CC_N;
    return static_cast<uint8_t>(7 + pb.cycles);
}

uint8_t Cpu::op_andd_imm(Bus& bus) {
    const uint16_t res = static_cast<uint16_t>(reg_d(regs_) & fetch_word(bus));
    set_reg_d(regs_, res);
    flags_logic16(regs_, res);
    return 5;
}
uint8_t Cpu::op_andd_dir(Bus& bus) {
    const uint16_t res = static_cast<uint16_t>(reg_d(regs_) & read_word(bus, direct_address(bus)));
    set_reg_d(regs_, res);
    flags_logic16(regs_, res);
    return 7;
}
uint8_t Cpu::op_andd_ext(Bus& bus) {
    const uint16_t res = static_cast<uint16_t>(reg_d(regs_) & read_word(bus, extended_address(bus)));
    set_reg_d(regs_, res);
    flags_logic16(regs_, res);
    return 8;
}
uint8_t Cpu::op_andd_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    if (!pb.valid) return pb.cycles;
    const uint16_t res = static_cast<uint16_t>(reg_d(regs_) & read_word(bus, pb.address));
    set_reg_d(regs_, res);
    flags_logic16(regs_, res);
    return static_cast<uint8_t>(7 + pb.cycles);
}

uint8_t Cpu::op_bitd_imm(Bus& bus) {
    flags_bit16(regs_, static_cast<uint16_t>(reg_d(regs_) & fetch_word(bus)));
    return 5;
}
uint8_t Cpu::op_bitd_dir(Bus& bus) {
    flags_bit16(regs_, static_cast<uint16_t>(reg_d(regs_) & read_word(bus, direct_address(bus))));
    return 7;
}
uint8_t Cpu::op_bitd_ext(Bus& bus) {
    flags_bit16(regs_, static_cast<uint16_t>(reg_d(regs_) & read_word(bus, extended_address(bus))));
    return 8;
}
uint8_t Cpu::op_bitd_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    if (!pb.valid) return pb.cycles;
    flags_bit16(regs_, static_cast<uint16_t>(reg_d(regs_) & read_word(bus, pb.address)));
    return static_cast<uint8_t>(7 + pb.cycles);
}

uint8_t Cpu::op_eord_imm(Bus& bus) {
    const uint16_t res = static_cast<uint16_t>(reg_d(regs_) ^ fetch_word(bus));
    set_reg_d(regs_, res);
    flags_logic16(regs_, res);
    return 5;
}
uint8_t Cpu::op_eord_dir(Bus& bus) {
    const uint16_t res = static_cast<uint16_t>(reg_d(regs_) ^ read_word(bus, direct_address(bus)));
    set_reg_d(regs_, res);
    flags_logic16(regs_, res);
    return 7;
}
uint8_t Cpu::op_eord_ext(Bus& bus) {
    const uint16_t res = static_cast<uint16_t>(reg_d(regs_) ^ read_word(bus, extended_address(bus)));
    set_reg_d(regs_, res);
    flags_logic16(regs_, res);
    return 8;
}
uint8_t Cpu::op_eord_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    if (!pb.valid) return pb.cycles;
    const uint16_t res = static_cast<uint16_t>(reg_d(regs_) ^ read_word(bus, pb.address));
    set_reg_d(regs_, res);
    flags_logic16(regs_, res);
    return static_cast<uint8_t>(7 + pb.cycles);
}

static uint8_t finish_divd(Registers& regs, Bus& bus, uint8_t divisor, uint8_t cycles) {
    if (divisor == 0) {
        regs.md |= 0x80;
        regs.pc = read_word(bus, 0xFFF0);
        return cycles;
    }

    const int16_t dividend = static_cast<int16_t>(reg_d(regs));
    const int8_t s_divisor = static_cast<int8_t>(divisor);
    const int quotient_wide = dividend / s_divisor;
    const int remainder_wide = dividend % s_divisor;
    if (quotient_wide < -32768 || quotient_wide > 32767) {
        regs.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_C));
        regs.cc |= CC_V;
        return cycles;
    }
    const int16_t quotient = static_cast<int16_t>(quotient_wide);
    const int16_t remainder = static_cast<int16_t>(remainder_wide);
    regs.e = hi(static_cast<uint16_t>(quotient));
    regs.f = lo(static_cast<uint16_t>(quotient));
    set_reg_d(regs, static_cast<uint16_t>(remainder));
    regs.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | CC_C));
    if (quotient == 0) regs.cc |= CC_Z;
    if (quotient < 0) regs.cc |= CC_N;
    if ((quotient & 0x01) != 0) regs.cc |= CC_C;
    return cycles;
}

uint8_t Cpu::op_divd_imm(Bus& bus) {
    return finish_divd(regs_, bus, fetch_byte(bus), 25);
}
uint8_t Cpu::op_divd_dir(Bus& bus) {
    return finish_divd(regs_, bus, read_byte(bus, direct_address(bus)), 27);
}
uint8_t Cpu::op_divd_ext(Bus& bus) {
    return finish_divd(regs_, bus, read_byte(bus, extended_address(bus)), 28);
}
uint8_t Cpu::op_divd_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    if (!pb.valid) return pb.cycles;
    return finish_divd(regs_, bus, read_byte(bus, pb.address), static_cast<uint8_t>(27 + pb.cycles));
}

uint8_t Cpu::op_divq_imm(Bus& bus) {
    const uint16_t divisor = fetch_word(bus);
    const uint32_t dividend = reg_q();
    if (divisor == 0) {
        regs_.md |= 0x80;
        regs_.pc = read_word(bus, 0xFFF0);
        return 36;
    }
    const int32_t s_dividend = static_cast<int32_t>(dividend);
    const int32_t s_divisor = static_cast<int16_t>(divisor);
    const int32_t q = s_dividend / s_divisor;
    const int32_t r = s_dividend % s_divisor;
    if (q < -32768 || q > 32767) {
        regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_C));
        regs_.cc |= CC_V;
        return 36;
    }
    set_reg_w(static_cast<uint16_t>(q & 0xFFFF));
    const uint16_t rem = static_cast<uint16_t>(r & 0xFFFF);
    regs_.a = hi(rem);
    regs_.b = lo(rem);
    regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | CC_C));
    if (q == 0) regs_.cc |= CC_Z;
    if (q & 0x8000) regs_.cc |= CC_N;
    if (q & 0x1) regs_.cc |= CC_C;
    return 36;
}
uint8_t Cpu::op_divq_dir(Bus& bus) {
    const uint16_t val = read_word(bus, direct_address(bus));
    const uint32_t dividend = reg_q();
    if (val == 0) {
        regs_.md |= 0x80;
        regs_.pc = read_word(bus, 0xFFF0);
        return 36;
    }
    const int32_t s_dividend = static_cast<int32_t>(dividend);
    const int32_t s_divisor = static_cast<int16_t>(val);
    const int32_t q = s_dividend / s_divisor;
    const int32_t r = s_dividend % s_divisor;
    if (q < -32768 || q > 32767) {
        regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_C));
        regs_.cc |= CC_V;
        return 36;
    }
    set_reg_w(static_cast<uint16_t>(q & 0xFFFF));
    const uint16_t rem = static_cast<uint16_t>(r & 0xFFFF);
    regs_.a = hi(rem);
    regs_.b = lo(rem);
    regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | CC_C));
    if (q == 0) regs_.cc |= CC_Z;
    if (q & 0x8000) regs_.cc |= CC_N;
    if (q & 0x1) regs_.cc |= CC_C;
    return 36;
}
uint8_t Cpu::op_divq_ext(Bus& bus) {
    const uint16_t val = read_word(bus, extended_address(bus));
    const uint32_t dividend = reg_q();
    if (val == 0) {
        regs_.md |= 0x80;
        regs_.pc = read_word(bus, 0xFFF0);
        return 37;
    }
    const int32_t s_dividend = static_cast<int32_t>(dividend);
    const int32_t s_divisor = static_cast<int16_t>(val);
    const int32_t q = s_dividend / s_divisor;
    const int32_t r = s_dividend % s_divisor;
    if (q < -32768 || q > 32767) {
        regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_C));
        regs_.cc |= CC_V;
        return 37;
    }
    set_reg_w(static_cast<uint16_t>(q & 0xFFFF));
    const uint16_t rem = static_cast<uint16_t>(r & 0xFFFF);
    regs_.a = hi(rem);
    regs_.b = lo(rem);
    regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | CC_C));
    if (q == 0) regs_.cc |= CC_Z;
    if (q & 0x8000) regs_.cc |= CC_N;
    if (q & 0x1) regs_.cc |= CC_C;
    return 37;
}
uint8_t Cpu::op_divq_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    if (!pb.valid) return pb.cycles;
    const uint16_t val = read_word(bus, pb.address);
    const uint32_t dividend = reg_q();
    if (val == 0) {
        regs_.md |= 0x80;
        regs_.pc = read_word(bus, 0xFFF0);
        return static_cast<uint8_t>(36 + pb.cycles);
    }
    const int32_t s_dividend = static_cast<int32_t>(dividend);
    const int32_t s_divisor = static_cast<int16_t>(val);
    const int32_t q = s_dividend / s_divisor;
    const int32_t r = s_dividend % s_divisor;
    if (q < -32768 || q > 32767) {
        regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_C));
        regs_.cc |= CC_V;
        return static_cast<uint8_t>(36 + pb.cycles);
    }
    set_reg_w(static_cast<uint16_t>(q & 0xFFFF));
    const uint16_t rem = static_cast<uint16_t>(r & 0xFFFF);
    regs_.a = hi(rem);
    regs_.b = lo(rem);
    regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | CC_C));
    if (q == 0) regs_.cc |= CC_Z;
    if (q & 0x8000) regs_.cc |= CC_N;
    if (q & 0x1) regs_.cc |= CC_C;
    return static_cast<uint8_t>(37 + pb.cycles);
}

uint8_t Cpu::op_muld_imm(Bus& bus) {
    const int16_t m = static_cast<int16_t>(fetch_word(bus));
    const int32_t prod = static_cast<int16_t>((regs_.a << 8) | regs_.b) * m;
    set_reg_q(static_cast<uint32_t>(prod));
    regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z));
    if (prod == 0) regs_.cc |= CC_Z;
    if (prod & 0x80000000) regs_.cc |= CC_N;
    return 28;
}
uint8_t Cpu::op_muld_dir(Bus& bus) {
    const int16_t m = static_cast<int16_t>(read_word(bus, direct_address(bus)));
    const int32_t prod = static_cast<int16_t>((regs_.a << 8) | regs_.b) * m;
    set_reg_q(static_cast<uint32_t>(prod));
    regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z));
    if (prod == 0) regs_.cc |= CC_Z;
    if (prod & 0x80000000) regs_.cc |= CC_N;
    return 30;
}
uint8_t Cpu::op_muld_ext(Bus& bus) {
    const int16_t m = static_cast<int16_t>(read_word(bus, extended_address(bus)));
    const int32_t prod = static_cast<int16_t>((regs_.a << 8) | regs_.b) * m;
    set_reg_q(static_cast<uint32_t>(prod));
    regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z));
    if (prod == 0) regs_.cc |= CC_Z;
    if (prod & 0x80000000) regs_.cc |= CC_N;
    return 31;
}
uint8_t Cpu::op_muld_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    if (!pb.valid) return pb.cycles;
    const int16_t m = static_cast<int16_t>(read_word(bus, pb.address));
    const int32_t prod = static_cast<int16_t>((regs_.a << 8) | regs_.b) * m;
    set_reg_q(static_cast<uint32_t>(prod));
    regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z));
    if (prod == 0) regs_.cc |= CC_Z;
    if (prod & 0x80000000) regs_.cc |= CC_N;
    return static_cast<uint8_t>(31 + pb.cycles);
}

uint8_t Cpu::op_addr(Bus& bus) {
    const uint8_t post = fetch_byte(bus);
    const uint8_t dst = post & 0x0F;
    const uint8_t src = post >> 4;
    const bool dest16 = reg_is_16bit(dst);
    const uint16_t s = read_reg_for_dest(regs_, src, dest16);
    uint16_t d = read_reg_for_dest(regs_, dst, dest16);
    const uint16_t res = dest16 ? add16(regs_, d, s) : add8(regs_, static_cast<uint8_t>(d & 0xFF), static_cast<uint8_t>(s & 0xFF));
    write_reg_sized(*this, dst, res, dest16);
    return 4;
}
uint8_t Cpu::op_subr(Bus& bus) {
    const uint8_t post = fetch_byte(bus);
    const uint8_t dst = post & 0x0F;
    const uint8_t src = post >> 4;
    const bool dest16 = reg_is_16bit(dst);
    const uint16_t s = read_reg_for_dest(regs_, src, dest16);
    uint16_t d = read_reg_for_dest(regs_, dst, dest16);
    const uint16_t res = dest16 ? sub16(regs_, d, s) : sub8(regs_, static_cast<uint8_t>(d & 0xFF), static_cast<uint8_t>(s & 0xFF));
    write_reg_sized(*this, dst, res, dest16);
    return 4;
}
uint8_t Cpu::op_cmpr(Bus& bus) {
    const uint8_t post = fetch_byte(bus);
    const uint8_t dst = post & 0x0F;
    const uint8_t src = post >> 4;
    const bool dest16 = reg_is_16bit(dst);
    const uint16_t s = read_reg_for_dest(regs_, src, dest16);
    const uint16_t d = read_reg_for_dest(regs_, dst, dest16);
    if (dest16) sub16(regs_, d, s); else sub8(regs_, static_cast<uint8_t>(d & 0xFF), static_cast<uint8_t>(s & 0xFF));
    return 4;
}
uint8_t Cpu::op_sbcr(Bus& bus) {
    const uint8_t post = fetch_byte(bus);
    const uint8_t dst = post & 0x0F;
    const uint8_t src = post >> 4;
    const bool dest16 = reg_is_16bit(dst);
    const uint16_t s = read_reg_for_dest(regs_, src, dest16);
    uint16_t d = read_reg_for_dest(regs_, dst, dest16);
    uint16_t res = dest16 ? sub16_carry(regs_, d, s, (regs_.cc & CC_C) != 0)
                          : sbc8(regs_, static_cast<uint8_t>(d & 0xFF), static_cast<uint8_t>(s & 0xFF));
    write_reg_sized(*this, dst, res, dest16);
    return 4;
}
uint8_t Cpu::op_adcr(Bus& bus) {
    const uint8_t post = fetch_byte(bus);
    const uint8_t dst = post & 0x0F;
    const uint8_t src = post >> 4;
    const bool dest16 = reg_is_16bit(dst);
    const uint16_t s = read_reg_for_dest(regs_, src, dest16);
    uint16_t d = read_reg_for_dest(regs_, dst, dest16);
    uint16_t res = dest16 ? add16_carry(regs_, d, s, (regs_.cc & CC_C) != 0)
                          : adc8(regs_, static_cast<uint8_t>(d & 0xFF), static_cast<uint8_t>(s & 0xFF));
    write_reg_sized(*this, dst, res, dest16);
    return 4;
}
uint8_t Cpu::op_andr(Bus& bus) {
    const uint8_t post = fetch_byte(bus);
    const uint8_t dst = post & 0x0F;
    const uint8_t src = post >> 4;
    const bool dest16 = reg_is_16bit(dst);
    const uint16_t s = read_reg_for_dest(regs_, src, dest16);
    const uint16_t d = read_reg_for_dest(regs_, dst, dest16);
    const uint16_t res = static_cast<uint16_t>(d & s);
    regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V));
    if (dest16) {
        if (res & 0x8000) regs_.cc |= CC_N;
        if (res == 0) regs_.cc |= CC_Z;
    } else {
        if (res & 0x80) regs_.cc |= CC_N;
        if ((res & 0xFF) == 0) regs_.cc |= CC_Z;
    }
    write_reg_sized(*this, dst, res, dest16);
    return 4;
}
uint8_t Cpu::op_orr(Bus& bus) {
    const uint8_t post = fetch_byte(bus);
    const uint8_t dst = post & 0x0F;
    const uint8_t src = post >> 4;
    const bool dest16 = reg_is_16bit(dst);
    const uint16_t s = read_reg_for_dest(regs_, src, dest16);
    const uint16_t d = read_reg_for_dest(regs_, dst, dest16);
    const uint16_t res = static_cast<uint16_t>(d | s);
    regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V));
    if (dest16) {
        if (res & 0x8000) regs_.cc |= CC_N;
        if (res == 0) regs_.cc |= CC_Z;
    } else {
        if (res & 0x80) regs_.cc |= CC_N;
        if ((res & 0xFF) == 0) regs_.cc |= CC_Z;
    }
    write_reg_sized(*this, dst, res, dest16);
    return 4;
}
uint8_t Cpu::op_eorr(Bus& bus) {
    const uint8_t post = fetch_byte(bus);
    const uint8_t dst = post & 0x0F;
    const uint8_t src = post >> 4;
    const bool dest16 = reg_is_16bit(dst);
    const uint16_t s = read_reg_for_dest(regs_, src, dest16);
    const uint16_t d = read_reg_for_dest(regs_, dst, dest16);
    const uint16_t res = static_cast<uint16_t>(d ^ s);
    regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V));
    if (dest16) {
        if (res & 0x8000) regs_.cc |= CC_N;
        if (res == 0) regs_.cc |= CC_Z;
    } else {
        if (res & 0x80) regs_.cc |= CC_N;
        if ((res & 0xFF) == 0) regs_.cc |= CC_Z;
    }
    write_reg_sized(*this, dst, res, dest16);
    return 4;
}

uint8_t Cpu::op_negd(Bus&) {
    set_reg_d(regs_, neg16_op(regs_, reg_d(regs_)));
    return 3;
}
uint8_t Cpu::op_comd(Bus&) {
    set_reg_d(regs_, com16_op(regs_, reg_d(regs_)));
    return 3;
}
uint8_t Cpu::op_lsrd(Bus&) {
    set_reg_d(regs_, lsr16_op(regs_, reg_d(regs_)));
    return 3;
}
uint8_t Cpu::op_asrd(Bus&) {
    set_reg_d(regs_, asr16_op(regs_, reg_d(regs_)));
    return 3;
}
uint8_t Cpu::op_decd(Bus&) {
    set_reg_d(regs_, dec16(regs_, reg_d(regs_)));
    return 3;
}
uint8_t Cpu::op_incd(Bus&) {
    set_reg_d(regs_, inc16(regs_, reg_d(regs_)));
    return 3;
}
uint8_t Cpu::op_tstd(Bus&) {
    set_flags_tst16(regs_, reg_d(regs_));
    return 3;
}
uint8_t Cpu::op_clrd(Bus&) {
    set_reg_d(regs_, 0);
    regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_V | CC_C));
    regs_.cc |= CC_Z;
    return 3;
}

uint8_t Cpu::op_lsl_d(Bus&) {
    const uint16_t val = static_cast<uint16_t>((regs_.a << 8) | regs_.b);
    const uint16_t res = static_cast<uint16_t>(val << 1);
    regs_.a = hi(res);
    regs_.b = lo(res);
    regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | CC_C));
    if (res & 0x8000) regs_.cc |= CC_N;
    if (res == 0) regs_.cc |= CC_Z;
    if (val & 0x8000) regs_.cc |= CC_C;
    if (((res ^ (regs_.cc & CC_C ? 0x8000 : 0)) & 0x8000) != 0) regs_.cc |= CC_V;
    return 3;
}
uint8_t Cpu::op_rold(Bus&) {
    const uint16_t val = static_cast<uint16_t>((regs_.a << 8) | regs_.b);
    const uint16_t res = static_cast<uint16_t>((val << 1) | ((regs_.cc & CC_C) ? 1 : 0));
    regs_.a = hi(res);
    regs_.b = lo(res);
    regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | CC_C));
    if (res & 0x8000) regs_.cc |= CC_N;
    if (res == 0) regs_.cc |= CC_Z;
    if (val & 0x8000) regs_.cc |= CC_C;
    if (((res ^ (regs_.cc & CC_C ? 0x8000 : 0)) & 0x8000) != 0) regs_.cc |= CC_V;
    return 3;
}
uint8_t Cpu::op_rord(Bus&) {
    const uint16_t val = static_cast<uint16_t>((regs_.a << 8) | regs_.b);
    const uint16_t res = static_cast<uint16_t>((val >> 1) | ((regs_.cc & CC_C) ? 0x8000 : 0));
    regs_.a = hi(res);
    regs_.b = lo(res);
    regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | CC_C));
    if (res & 0x8000) regs_.cc |= CC_N;
    if (res == 0) regs_.cc |= CC_Z;
    if (val & 0x1) regs_.cc |= CC_C;
    if (((res ^ (regs_.cc & CC_C ? 0x8000 : 0)) & 0x8000) != 0) regs_.cc |= CC_V;
    return 3;
}
uint8_t Cpu::op_comw_inh(Bus&) {
    set_reg_w(com16_op(regs_, reg_w()));
    return 3;
}
uint8_t Cpu::op_lsrw_inh(Bus&) {
    set_reg_w(lsr16_op(regs_, reg_w()));
    return 3;
}
uint8_t Cpu::op_lslw_inh(Bus&) {
    const uint16_t val = reg_w();
    const uint16_t res = static_cast<uint16_t>(val << 1);
    set_reg_w(res);
    regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | CC_C));
    if (res & 0x8000) regs_.cc |= CC_N;
    if (res == 0) regs_.cc |= CC_Z;
    if (val & 0x8000) regs_.cc |= CC_C;
    if (((res ^ (regs_.cc & CC_C ? 0x8000 : 0)) & 0x8000) != 0) regs_.cc |= CC_V;
    return 3;
}
uint8_t Cpu::op_rolw(Bus&) {
    const uint16_t val = reg_w();
    const uint16_t res = static_cast<uint16_t>((val << 1) | ((regs_.cc & CC_C) ? 1 : 0));
    set_reg_w(res);
    regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | CC_C));
    if (res & 0x8000) regs_.cc |= CC_N;
    if (res == 0) regs_.cc |= CC_Z;
    if (val & 0x8000) regs_.cc |= CC_C;
    if (((res ^ (regs_.cc & CC_C ? 0x8000 : 0)) & 0x8000) != 0) regs_.cc |= CC_V;
    return 3;
}
uint8_t Cpu::op_rorw(Bus&) {
    const uint16_t val = reg_w();
    const uint16_t res = static_cast<uint16_t>((val >> 1) | ((regs_.cc & CC_C) ? 0x8000 : 0));
    set_reg_w(res);
    regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | CC_C));
    if (res & 0x8000) regs_.cc |= CC_N;
    if (res == 0) regs_.cc |= CC_Z;
    if (val & 0x1) regs_.cc |= CC_C;
    if (((res ^ (regs_.cc & CC_C ? 0x8000 : 0)) & 0x8000) != 0) regs_.cc |= CC_V;
    return 3;
}

uint8_t Cpu::op_ldmd(Bus& bus) {
    regs_.md = fetch_byte(bus);
    return 5;
}

uint8_t Cpu::op_bitmd(Bus& bus) {
    const uint8_t res = static_cast<uint8_t>(regs_.md & fetch_byte(bus));
    regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V));
    if (res == 0) regs_.cc |= CC_Z;
    if (res & 0x80) regs_.cc |= CC_N;
    return 4;
}

uint8_t Cpu::op_sexw(Bus&) {
    const uint16_t w = reg_w();
    const uint32_t q = (static_cast<int32_t>(static_cast<int16_t>(w)) << 16) | static_cast<uint16_t>(w);
    set_reg_q(q);
    regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z));
    if (q == 0) regs_.cc |= CC_Z;
    if (q & 0x80000000u) regs_.cc |= CC_N;
    return 4;
}

uint8_t Cpu::op_pshsw(Bus& bus) {
    push_word(bus, reg_w());
    return 6;
}
uint8_t Cpu::op_pulsw(Bus& bus) {
    set_reg_w(pull_word(bus));
    return 6;
}
uint8_t Cpu::op_pshuw(Bus& bus) {
    regs_.u = static_cast<uint16_t>(regs_.u - 2);
    write_byte(bus, static_cast<uint16_t>(regs_.u + 1), hi(reg_w()));
    write_byte(bus, regs_.u, lo(reg_w()));
    return 6;
}
uint8_t Cpu::op_puluw(Bus& bus) {
    set_reg_w(static_cast<uint16_t>((read_byte(bus, static_cast<uint16_t>(regs_.u + 1)) << 8) | read_byte(bus, regs_.u)));
    regs_.u = static_cast<uint16_t>(regs_.u + 2);
    return 6;
}

uint8_t Cpu::op_tfm_common(Bus& bus, bool inc_src, bool inc_dst) {
    const uint8_t post = fetch_byte(bus);
    const uint8_t src_code = post >> 4;
    const uint8_t dst_code = post & 0x0F;
    // Only X,Y,U,S,D are valid
    auto reg_ptr = [&](uint8_t code) -> uint16_t* {
        switch (code & 0x0F) {
        case 0x00: return nullptr; // D not addressable as pointer here
        case 0x01: return &regs_.x;
        case 0x02: return &regs_.y;
        case 0x03: return &regs_.u;
        case 0x04: return &regs_.s;
        default: return nullptr;
        }
    };
    uint16_t* src_ptr = reg_ptr(src_code);
    uint16_t* dst_ptr = reg_ptr(dst_code);
    if (!src_ptr || !dst_ptr) {
        regs_.pc = read_word(bus, 0xFFF0);
        return 6;
    }
    uint32_t count = reg_w();
    if (count == 0) count = 0x10000;
    for (uint32_t i = 0; i < count; ++i) {
        const uint8_t byte = read_byte(bus, *src_ptr);
        if (inc_src) *src_ptr = static_cast<uint16_t>(*src_ptr + 1);
        else *src_ptr = static_cast<uint16_t>(*src_ptr - 1);
        write_byte(bus, *dst_ptr, byte);
        if (inc_dst) *dst_ptr = static_cast<uint16_t>(*dst_ptr + 1);
        else *dst_ptr = static_cast<uint16_t>(*dst_ptr - 1);
        set_reg_w(static_cast<uint16_t>(reg_w() - 1));
        if (reg_w() == 0) break;
    }
    return 6; // base cycles; ignores per-byte timing.
}

uint8_t Cpu::op_tfm_pp(Bus& bus) { return op_tfm_common(bus, true, true); }
uint8_t Cpu::op_tfm_mm(Bus& bus) { return op_tfm_common(bus, false, false); }
uint8_t Cpu::op_tfm_pn(Bus& bus) { return op_tfm_common(bus, true, false); }
uint8_t Cpu::op_tfm_np(Bus& bus) { return op_tfm_common(bus, false, true); }

uint8_t Cpu::op_aim_dir(Bus& bus) {
    const uint8_t mask = fetch_byte(bus);
    const uint16_t addr = direct_address(bus);
    const uint8_t res = read_byte(bus, addr) & mask;
    write_byte(bus, addr, res);
    set_flags_nz8(res);
    regs_.cc &= static_cast<uint8_t>(~CC_V);
    return 6;
}
uint8_t Cpu::op_aim_idx(Bus& bus) {
    const uint8_t mask = fetch_byte(bus);
    const auto pb = indexed_address(bus);
    if (!pb.valid) return pb.cycles;
    const uint8_t res = read_byte(bus, pb.address) & mask;
    write_byte(bus, pb.address, res);
    set_flags_nz8(res);
    regs_.cc &= static_cast<uint8_t>(~CC_V);
    return static_cast<uint8_t>(7 + pb.cycles);
}
uint8_t Cpu::op_aim_ext(Bus& bus) {
    const uint8_t mask = fetch_byte(bus);
    const uint16_t addr = extended_address(bus);
    const uint8_t res = read_byte(bus, addr) & mask;
    write_byte(bus, addr, res);
    set_flags_nz8(res);
    regs_.cc &= static_cast<uint8_t>(~CC_V);
    return 7;
}
uint8_t Cpu::op_eim_dir(Bus& bus) {
    const uint8_t mask = fetch_byte(bus);
    const uint16_t addr = direct_address(bus);
    const uint8_t res = read_byte(bus, addr) ^ mask;
    write_byte(bus, addr, res);
    set_flags_nz8(res);
    regs_.cc &= static_cast<uint8_t>(~CC_V);
    return 6;
}
uint8_t Cpu::op_eim_idx(Bus& bus) {
    const uint8_t mask = fetch_byte(bus);
    const auto pb = indexed_address(bus);
    if (!pb.valid) return pb.cycles;
    const uint8_t res = read_byte(bus, pb.address) ^ mask;
    write_byte(bus, pb.address, res);
    set_flags_nz8(res);
    regs_.cc &= static_cast<uint8_t>(~CC_V);
    return static_cast<uint8_t>(7 + pb.cycles);
}
uint8_t Cpu::op_eim_ext(Bus& bus) {
    const uint8_t mask = fetch_byte(bus);
    const uint16_t addr = extended_address(bus);
    const uint8_t res = read_byte(bus, addr) ^ mask;
    write_byte(bus, addr, res);
    set_flags_nz8(res);
    regs_.cc &= static_cast<uint8_t>(~CC_V);
    return 7;
}
uint8_t Cpu::op_oim_dir(Bus& bus) {
    const uint8_t mask = fetch_byte(bus);
    const uint16_t addr = direct_address(bus);
    const uint8_t res = read_byte(bus, addr) | mask;
    write_byte(bus, addr, res);
    set_flags_nz8(res);
    regs_.cc &= static_cast<uint8_t>(~CC_V);
    return 6;
}
uint8_t Cpu::op_oim_idx(Bus& bus) {
    const uint8_t mask = fetch_byte(bus);
    const auto pb = indexed_address(bus);
    if (!pb.valid) return pb.cycles;
    const uint8_t res = read_byte(bus, pb.address) | mask;
    write_byte(bus, pb.address, res);
    set_flags_nz8(res);
    regs_.cc &= static_cast<uint8_t>(~CC_V);
    return static_cast<uint8_t>(7 + pb.cycles);
}
uint8_t Cpu::op_oim_ext(Bus& bus) {
    const uint8_t mask = fetch_byte(bus);
    const uint16_t addr = extended_address(bus);
    const uint8_t res = read_byte(bus, addr) | mask;
    write_byte(bus, addr, res);
    set_flags_nz8(res);
    regs_.cc &= static_cast<uint8_t>(~CC_V);
    return 7;
}
uint8_t Cpu::op_tim_dir(Bus& bus) {
    const uint8_t mask = fetch_byte(bus);
    const uint16_t addr = direct_address(bus);
    const uint8_t res = read_byte(bus, addr) & mask;
    set_flags_nz8(res);
    regs_.cc &= static_cast<uint8_t>(~CC_V);
    return 6;
}
uint8_t Cpu::op_tim_idx(Bus& bus) {
    const uint8_t mask = fetch_byte(bus);
    const auto pb = indexed_address(bus);
    if (!pb.valid) return pb.cycles;
    const uint8_t res = read_byte(bus, pb.address) & mask;
    set_flags_nz8(res);
    regs_.cc &= static_cast<uint8_t>(~CC_V);
    return static_cast<uint8_t>(7 + pb.cycles);
}
uint8_t Cpu::op_tim_ext(Bus& bus) {
    const uint8_t mask = fetch_byte(bus);
    const uint16_t addr = extended_address(bus);
    const uint8_t res = read_byte(bus, addr) & mask;
    set_flags_nz8(res);
    regs_.cc &= static_cast<uint8_t>(~CC_V);
    return 7;
}

static inline uint8_t* bitop_reg_ptr(Registers& regs, uint8_t reg_code) {
    switch ((reg_code >> 6) & 0x03) {
    case 0: return &regs.cc;
    case 1: return &regs.a;
    case 2: return &regs.b;
    default: return nullptr;
    }
}

uint8_t Cpu::op_band(Bus& bus) {
    const uint8_t post = fetch_byte(bus);
    const uint16_t addr = direct_address(bus);
    uint8_t* regp = bitop_reg_ptr(regs_, post);
    if (!regp) { regs_.pc = read_word(bus, 0xFFF0); return 7; }
    const uint8_t src_bit = (post >> 3) & 0x07;
    const uint8_t dst_bit = post & 0x07;
    uint8_t mem = read_byte(bus, addr);
    const uint8_t reg_bit = (*regp >> src_bit) & 0x01;
    const uint8_t mem_bit = (mem >> dst_bit) & 0x01;
    const uint8_t new_bit = static_cast<uint8_t>(reg_bit & mem_bit);
    mem = static_cast<uint8_t>((mem & static_cast<uint8_t>(~(1u << dst_bit))) | (new_bit << dst_bit));
    write_byte(bus, addr, mem);
    return 7;
}
uint8_t Cpu::op_biand(Bus& bus) {
    const uint8_t post = fetch_byte(bus);
    const uint16_t addr = direct_address(bus);
    uint8_t* regp = bitop_reg_ptr(regs_, post);
    if (!regp) { regs_.pc = read_word(bus, 0xFFF0); return 7; }
    const uint8_t src_bit = (post >> 3) & 0x07;
    const uint8_t dst_bit = post & 0x07;
    uint8_t mem = read_byte(bus, addr);
    const uint8_t reg_bit = (*regp >> src_bit) & 0x01;
    const uint8_t mem_bit = (mem >> dst_bit) & 0x01;
    const uint8_t new_bit = static_cast<uint8_t>(reg_bit & static_cast<uint8_t>(mem_bit ^ 0x01));
    mem = static_cast<uint8_t>((mem & static_cast<uint8_t>(~(1u << dst_bit))) | (new_bit << dst_bit));
    write_byte(bus, addr, mem);
    return 7;
}
uint8_t Cpu::op_bor(Bus& bus) {
    const uint8_t post = fetch_byte(bus);
    const uint16_t addr = direct_address(bus);
    uint8_t* regp = bitop_reg_ptr(regs_, post);
    if (!regp) { regs_.pc = read_word(bus, 0xFFF0); return 7; }
    const uint8_t src_bit = (post >> 3) & 0x07;
    const uint8_t dst_bit = post & 0x07;
    uint8_t mem = read_byte(bus, addr);
    const uint8_t reg_bit = (*regp >> src_bit) & 0x01;
    const uint8_t mem_bit = (mem >> dst_bit) & 0x01;
    const uint8_t new_bit = static_cast<uint8_t>(reg_bit | mem_bit);
    mem = static_cast<uint8_t>((mem & static_cast<uint8_t>(~(1u << dst_bit))) | (new_bit << dst_bit));
    write_byte(bus, addr, mem);
    return 7;
}
uint8_t Cpu::op_bior(Bus& bus) {
    const uint8_t post = fetch_byte(bus);
    const uint16_t addr = direct_address(bus);
    uint8_t* regp = bitop_reg_ptr(regs_, post);
    if (!regp) { regs_.pc = read_word(bus, 0xFFF0); return 7; }
    const uint8_t src_bit = (post >> 3) & 0x07;
    const uint8_t dst_bit = post & 0x07;
    uint8_t mem = read_byte(bus, addr);
    const uint8_t reg_bit = (*regp >> src_bit) & 0x01;
    const uint8_t mem_bit = (mem >> dst_bit) & 0x01;
    const uint8_t new_bit = static_cast<uint8_t>(reg_bit | static_cast<uint8_t>(mem_bit ^ 0x01));
    mem = static_cast<uint8_t>((mem & static_cast<uint8_t>(~(1u << dst_bit))) | (new_bit << dst_bit));
    write_byte(bus, addr, mem);
    return 7;
}
uint8_t Cpu::op_beor(Bus& bus) {
    const uint8_t post = fetch_byte(bus);
    const uint16_t addr = direct_address(bus);
    uint8_t* regp = bitop_reg_ptr(regs_, post);
    if (!regp) { regs_.pc = read_word(bus, 0xFFF0); return 7; }
    const uint8_t src_bit = (post >> 3) & 0x07;
    const uint8_t dst_bit = post & 0x07;
    uint8_t mem = read_byte(bus, addr);
    const uint8_t reg_bit = (*regp >> src_bit) & 0x01;
    const uint8_t mem_bit = (mem >> dst_bit) & 0x01;
    const uint8_t new_bit = static_cast<uint8_t>(reg_bit ^ mem_bit);
    mem = static_cast<uint8_t>((mem & static_cast<uint8_t>(~(1u << dst_bit))) | (new_bit << dst_bit));
    write_byte(bus, addr, mem);
    return 7;
}
uint8_t Cpu::op_bieor(Bus& bus) {
    const uint8_t post = fetch_byte(bus);
    const uint16_t addr = direct_address(bus);
    uint8_t* regp = bitop_reg_ptr(regs_, post);
    if (!regp) { regs_.pc = read_word(bus, 0xFFF0); return 7; }
    const uint8_t src_bit = (post >> 3) & 0x07;
    const uint8_t dst_bit = post & 0x07;
    uint8_t mem = read_byte(bus, addr);
    const uint8_t reg_bit = (*regp >> src_bit) & 0x01;
    const uint8_t mem_bit = (mem >> dst_bit) & 0x01;
    const uint8_t new_bit = static_cast<uint8_t>(reg_bit ^ static_cast<uint8_t>(mem_bit ^ 0x01));
    mem = static_cast<uint8_t>((mem & static_cast<uint8_t>(~(1u << dst_bit))) | (new_bit << dst_bit));
    write_byte(bus, addr, mem);
    return 7;
}
uint8_t Cpu::op_ldbt(Bus& bus) {
    const uint8_t post = fetch_byte(bus);
    const uint16_t addr = direct_address(bus);
    uint8_t* regp = bitop_reg_ptr(regs_, post);
    if (!regp) { regs_.pc = read_word(bus, 0xFFF0); return 7; }
    const uint8_t src_bit = (post >> 3) & 0x07;
    const uint8_t dst_bit = post & 0x07;
    const uint8_t mem = read_byte(bus, addr);
    const uint8_t mem_bit = (mem >> src_bit) & 0x01;
    *regp = static_cast<uint8_t>((*regp & static_cast<uint8_t>(~(1u << dst_bit))) | (mem_bit << dst_bit));
    return 7;
}
uint8_t Cpu::op_stbt(Bus& bus) {
    const uint8_t post = fetch_byte(bus);
    const uint16_t addr = direct_address(bus);
    uint8_t* regp = bitop_reg_ptr(regs_, post);
    if (!regp) { regs_.pc = read_word(bus, 0xFFF0); return 8; }
    const uint8_t src_bit = (post >> 3) & 0x07;
    const uint8_t dst_bit = post & 0x07;
    const uint8_t reg_bit = (*regp >> src_bit) & 0x01;
    uint8_t mem = read_byte(bus, addr);
    mem = static_cast<uint8_t>((mem & static_cast<uint8_t>(~(1u << dst_bit))) | (reg_bit << dst_bit));
    write_byte(bus, addr, mem);
    return 8;
}


} // namespace microlind
