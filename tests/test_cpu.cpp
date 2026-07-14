#include <gtest/gtest.h>

#include <cstdint>
#include <initializer_list>
#include <string>
#include <vector>

#include "microlind/bus.hpp"
#include "microlind/cpu.hpp"

#include "test_harness.hpp"

namespace {

void write_bytes(microlind::Bus& bus, uint16_t address, std::initializer_list<uint8_t> bytes) {
    for (uint8_t byte : bytes) {
        bus.write8(address++, byte);
    }
}

void write_bytes(microlind::Bus& bus, uint16_t address, const std::vector<uint8_t>& bytes) {
    for (uint8_t byte : bytes) {
        bus.write8(address++, byte);
    }
}

void prime_hd6309_test_state(microlind::Bus& bus, microlind::Cpu& cpu) {
    bus.write8(0xFFF0, 0x34);
    bus.write8(0xFFF1, 0x56);
    bus.write8(0x0020, 0x12);
    bus.write8(0x0021, 0x34);
    bus.write8(0x1234, 0x56);
    bus.write8(0x1235, 0x78);
    bus.write8(0x2000, 0x12);
    bus.write8(0x2001, 0x34);
    bus.write8(0x9000, 0x12);
    bus.write8(0x9001, 0x34);
    bus.write8(0xA000, 0x78);
    bus.write8(0xA001, 0x56);

    auto& regs = cpu.regs();
    regs.a = 0x12;
    regs.b = 0x34;
    regs.e = 0x00;
    regs.f = 0x01;
    regs.dp = 0x00;
    regs.cc = 0x00;
    regs.md = 0x00;
    regs.x = 0x2000;
    regs.y = 0x2100;
    regs.u = 0xA000;
    regs.s = 0x9002;
}

TEST(CpuExecutionTest, HD6309InvalidOpcodeTrapsThroughFFF0Vector) {
    microlind::Bus bus;
    microlind::test::map_flat_ram(bus);
    bus.write8(0x0100, 0x15);
    bus.write8(0xFFF0, 0x12);
    bus.write8(0xFFF1, 0x34);

    microlind::Cpu cpu(microlind::CpuMode::HD6309);
    cpu.set_pc(0x0100);

    const auto result = cpu.tick(bus);
    EXPECT_EQ(result.cycles, 1u);
    EXPECT_EQ(cpu.regs().pc, 0x1234);
    EXPECT_NE(cpu.regs().md & 0x40, 0);
}

TEST(CpuExecutionTest, MC6809DoesNotExecuteHD6309SingleByteOpcode) {
    microlind::Bus bus;
    microlind::test::map_flat_ram(bus);
    bus.write8(0x0100, 0xCD);
    bus.write8(0x0101, 0x12);
    bus.write8(0x0102, 0x34);
    bus.write8(0x0103, 0x56);
    bus.write8(0x0104, 0x78);

    microlind::Cpu cpu(microlind::CpuMode::MC6809);
    cpu.set_pc(0x0100);

    const auto result = cpu.tick(bus);
    EXPECT_EQ(result.cycles, 1u);
    EXPECT_EQ(cpu.regs().pc, 0x0101);
    EXPECT_EQ(cpu.regs().a, 0x00);
    EXPECT_EQ(cpu.regs().b, 0x00);
    EXPECT_EQ(cpu.regs().e, 0x00);
    EXPECT_EQ(cpu.regs().f, 0x00);
    EXPECT_EQ(cpu.regs().md & 0x40, 0);
}

TEST(CpuExecutionTest, MC6809DoesNotExecuteHD6309PrefixedOpcode) {
    microlind::Bus bus;
    microlind::test::map_flat_ram(bus);
    bus.write8(0x0200, 0x10);
    bus.write8(0x0201, 0x30);
    bus.write8(0x0202, 0x01);

    microlind::Cpu cpu(microlind::CpuMode::MC6809);
    cpu.set_pc(0x0200);
    cpu.regs().a = 0x00;
    cpu.regs().b = 0x02;
    cpu.regs().x = 0x1000;

    const auto result = cpu.tick(bus);
    EXPECT_EQ(result.cycles, 1u);
    EXPECT_EQ(cpu.regs().pc, 0x0202);
    EXPECT_EQ(cpu.regs().x, 0x1000);
    EXPECT_EQ(cpu.regs().md & 0x40, 0);
}

TEST(CpuExecutionTest, MC6809DoesNotExecuteNewHD6309DRegisterOpcode) {
    microlind::Bus bus;
    microlind::test::map_flat_ram(bus);
    write_bytes(bus, 0x0200, {0x10, 0x84, 0x0F, 0x0F});

    microlind::Cpu cpu(microlind::CpuMode::MC6809);
    cpu.set_pc(0x0200);
    cpu.regs().a = 0xFF;
    cpu.regs().b = 0xFF;

    const auto result = cpu.tick(bus);
    EXPECT_EQ(result.cycles, 1u);
    EXPECT_EQ(cpu.regs().pc, 0x0202);
    EXPECT_EQ(cpu.regs().a, 0xFF);
    EXPECT_EQ(cpu.regs().b, 0xFF);
}

TEST(CpuExecutionTest, HD6309ExecutesHD6309SingleByteOpcode) {
    microlind::Bus bus;
    microlind::test::map_flat_ram(bus);
    bus.write8(0x0100, 0xCD);
    bus.write8(0x0101, 0x12);
    bus.write8(0x0102, 0x34);
    bus.write8(0x0103, 0x56);
    bus.write8(0x0104, 0x78);

    microlind::Cpu cpu(microlind::CpuMode::HD6309);
    cpu.set_pc(0x0100);

    const auto result = cpu.tick(bus);
    EXPECT_EQ(result.cycles, 5u);
    EXPECT_EQ(cpu.regs().pc, 0x0105);
    EXPECT_EQ(cpu.regs().a, 0x12);
    EXPECT_EQ(cpu.regs().b, 0x34);
    EXPECT_EQ(cpu.regs().e, 0x56);
    EXPECT_EQ(cpu.regs().f, 0x78);
}

TEST(CpuExecutionTest, LbraUsesSignedSixteenBitOffsetAndWraps) {
    microlind::Bus bus;
    microlind::test::map_flat_ram(bus);
    write_bytes(bus, 0xFF34, {0x16, 0x00, 0xC9});

    microlind::Cpu cpu(microlind::CpuMode::HD6309);
    cpu.set_pc(0xFF34);

    const auto result = cpu.tick(bus);
    EXPECT_EQ(result.cycles, 5u);
    EXPECT_EQ(cpu.regs().pc, 0x0000);
    EXPECT_EQ(cpu.regs().md & 0x40, 0);
}

TEST(CpuExecutionTest, LbraUsesNativeHD6309Timing) {
    microlind::Bus bus;
    microlind::test::map_flat_ram(bus);
    write_bytes(bus, 0x0100, {0x16, 0xFF, 0xFD});

    microlind::Cpu cpu(microlind::CpuMode::HD6309);
    cpu.set_pc(0x0100);
    cpu.regs().md = 0x01;

    const auto result = cpu.tick(bus);
    EXPECT_EQ(result.cycles, 4u);
    EXPECT_EQ(cpu.regs().pc, 0x0100);
}

TEST(CpuExecutionTest, JmpDirectAndExtendedUseNativeHD6309Timing) {
    struct JumpCase {
        std::string name;
        std::vector<uint8_t> bytes;
        uint8_t dp{};
        uint8_t md{};
        uint8_t expected_cycles{};
        uint16_t expected_pc{};
    };

    const JumpCase cases[] = {
        {"JMP direct emulation", {0x0E, 0x34}, 0x12, 0x00, 3, 0x1234},
        {"JMP direct native", {0x0E, 0x34}, 0x12, 0x01, 2, 0x1234},
        {"JMP extended emulation", {0x7E, 0x20, 0x40}, 0x00, 0x00, 4, 0x2040},
        {"JMP extended native", {0x7E, 0x20, 0x40}, 0x00, 0x01, 3, 0x2040},
    };

    for (const auto& test : cases) {
        SCOPED_TRACE(test.name);
        microlind::Bus bus;
        microlind::test::map_flat_ram(bus);
        write_bytes(bus, 0x0100, test.bytes);

        microlind::Cpu cpu(microlind::CpuMode::HD6309);
        cpu.set_pc(0x0100);
        cpu.regs().dp = test.dp;
        cpu.regs().md = test.md;

        const auto result = cpu.tick(bus);
        EXPECT_EQ(result.cycles, test.expected_cycles);
        EXPECT_EQ(cpu.regs().pc, test.expected_pc);
    }
}

TEST(CpuExecutionTest, RegisterUnaryInstructionsUseNativeHD6309TimingAndSetFlags) {
    struct ClearCase {
        std::string name;
        uint8_t opcode{};
        uint8_t md{};
        uint8_t expected_cycles{};
        bool target_a{};
        uint8_t initial_target{};
        uint8_t initial_other{};
        uint8_t expected_target{};
        bool expect_z{};
        bool expect_n{};
        bool expect_v{};
        bool expect_carry{};
    };

    const ClearCase cases[] = {
        {"CLRA emulation", 0x4F, 0x00, 2, true, 0x80, 0x40, 0x00, true, false, false, true},
        {"CLRA native", 0x4F, 0x01, 1, true, 0x80, 0x40, 0x00, true, false, false, true},
        {"CLRB emulation", 0x5F, 0x00, 2, false, 0x40, 0x80, 0x00, true, false, false, true},
        {"CLRB native", 0x5F, 0x01, 1, false, 0x40, 0x80, 0x00, true, false, false, true},
        {"TSTA emulation", 0x4D, 0x00, 2, true, 0x80, 0x40, 0x80, false, true, false, false},
        {"TSTA native", 0x4D, 0x01, 1, true, 0x80, 0x40, 0x80, false, true, false, false},
        {"TSTB emulation", 0x5D, 0x00, 2, false, 0x00, 0x80, 0x00, true, false, false, false},
        {"TSTB native", 0x5D, 0x01, 1, false, 0x00, 0x80, 0x00, true, false, false, false},
        {"DECA emulation", 0x4A, 0x00, 2, true, 0x80, 0x40, 0x7F, false, false, true, true},
        {"DECA native", 0x4A, 0x01, 1, true, 0x01, 0x40, 0x00, true, false, false, true},
        {"DECB emulation", 0x5A, 0x00, 2, false, 0x80, 0x40, 0x7F, false, false, true, true},
        {"DECB native", 0x5A, 0x01, 1, false, 0x01, 0x40, 0x00, true, false, false, true},
        {"INCA emulation", 0x4C, 0x00, 2, true, 0x7F, 0x40, 0x80, false, true, true, true},
        {"INCA native", 0x4C, 0x01, 1, true, 0xFF, 0x40, 0x00, true, false, false, true},
        {"INCB emulation", 0x5C, 0x00, 2, false, 0x7F, 0x40, 0x80, false, true, true, true},
        {"INCB native", 0x5C, 0x01, 1, false, 0xFF, 0x40, 0x00, true, false, false, true},
        {"COMA emulation", 0x43, 0x00, 2, true, 0x00, 0x40, 0xFF, false, true, false, true},
        {"COMA native", 0x43, 0x01, 1, true, 0xFF, 0x40, 0x00, true, false, false, true},
        {"COMB emulation", 0x53, 0x00, 2, false, 0x00, 0x40, 0xFF, false, true, false, true},
        {"COMB native", 0x53, 0x01, 1, false, 0xFF, 0x40, 0x00, true, false, false, true},
        {"NEGA emulation", 0x40, 0x00, 2, true, 0x80, 0x40, 0x80, false, true, true, true},
        {"NEGA native", 0x40, 0x01, 1, true, 0x00, 0x40, 0x00, true, false, false, false},
        {"NEGB emulation", 0x50, 0x00, 2, false, 0x80, 0x40, 0x80, false, true, true, true},
        {"NEGB native", 0x50, 0x01, 1, false, 0x00, 0x40, 0x00, true, false, false, false},
        {"LSRA emulation", 0x44, 0x00, 2, true, 0x01, 0x40, 0x00, true, false, false, true},
        {"LSRA native", 0x44, 0x01, 1, true, 0x80, 0x40, 0x40, false, false, false, false},
        {"LSRB emulation", 0x54, 0x00, 2, false, 0x01, 0x40, 0x00, true, false, false, true},
        {"LSRB native", 0x54, 0x01, 1, false, 0x80, 0x40, 0x40, false, false, false, false},
        {"RORA emulation", 0x46, 0x00, 2, true, 0x02, 0x40, 0x81, false, true, true, false},
        {"RORA native", 0x46, 0x01, 1, true, 0x01, 0x40, 0x80, false, true, false, true},
        {"RORB emulation", 0x56, 0x00, 2, false, 0x02, 0x40, 0x81, false, true, true, false},
        {"RORB native", 0x56, 0x01, 1, false, 0x01, 0x40, 0x80, false, true, false, true},
        {"ASRA emulation", 0x47, 0x00, 2, true, 0x81, 0x40, 0xC0, false, true, false, true},
        {"ASRA native", 0x47, 0x01, 1, true, 0x02, 0x40, 0x01, false, false, false, false},
        {"ASRB emulation", 0x57, 0x00, 2, false, 0x81, 0x40, 0xC0, false, true, false, true},
        {"ASRB native", 0x57, 0x01, 1, false, 0x02, 0x40, 0x01, false, false, false, false},
        {"ASLA emulation", 0x48, 0x00, 2, true, 0x80, 0x40, 0x00, true, false, true, true},
        {"ASLA native", 0x48, 0x01, 1, true, 0x40, 0x20, 0x80, false, true, true, false},
        {"ASLB emulation", 0x58, 0x00, 2, false, 0x80, 0x40, 0x00, true, false, true, true},
        {"ASLB native", 0x58, 0x01, 1, false, 0x40, 0x20, 0x80, false, true, true, false},
        {"ROLA emulation", 0x49, 0x00, 2, true, 0x40, 0x20, 0x81, false, true, true, false},
        {"ROLA native", 0x49, 0x01, 1, true, 0x80, 0x40, 0x01, false, false, true, true},
        {"ROLB emulation", 0x59, 0x00, 2, false, 0x40, 0x20, 0x81, false, true, true, false},
        {"ROLB native", 0x59, 0x01, 1, false, 0x80, 0x40, 0x01, false, false, true, true},
    };

    for (const auto& test : cases) {
        SCOPED_TRACE(test.name);
        microlind::Bus bus;
        microlind::test::map_flat_ram(bus);
        bus.write8(0x0100, test.opcode);

        microlind::Cpu cpu(microlind::CpuMode::HD6309);
        cpu.set_pc(0x0100);
        cpu.regs().a = test.target_a ? test.initial_target : test.initial_other;
        cpu.regs().b = test.target_a ? test.initial_other : test.initial_target;
        cpu.regs().cc = static_cast<uint8_t>(microlind::CC_N | microlind::CC_V | microlind::CC_C);
        cpu.regs().md = test.md;

        const auto result = cpu.tick(bus);
        EXPECT_EQ(result.cycles, test.expected_cycles);
        EXPECT_EQ(cpu.regs().pc, 0x0101);
        EXPECT_EQ(test.target_a ? cpu.regs().a : cpu.regs().b, test.expected_target);
        EXPECT_EQ(test.target_a ? cpu.regs().b : cpu.regs().a, test.initial_other);
        EXPECT_EQ((cpu.regs().cc & microlind::CC_Z) != 0, test.expect_z);
        EXPECT_EQ((cpu.regs().cc & microlind::CC_N) != 0, test.expect_n);
        EXPECT_EQ((cpu.regs().cc & microlind::CC_V) != 0, test.expect_v);
        EXPECT_EQ((cpu.regs().cc & microlind::CC_C) != 0, test.expect_carry);
    }
}

TEST(CpuExecutionTest, ImmediateAccumulatorAluInstructionsSetFlags) {
    struct AluCase {
        std::string name;
        uint8_t opcode{};
        bool target_a{};
        uint8_t initial_target{};
        uint8_t initial_other{};
        uint8_t operand{};
        uint8_t initial_cc{};
        uint8_t expected_target{};
        bool expect_z{};
        bool expect_n{};
        bool expect_v{};
        bool expect_carry{};
        bool expect_half{};
    };

    const AluCase cases[] = {
        {"SUBA immediate", 0x80, true, 0x10, 0x42, 0x20, 0, 0xF0, false, true, false, true, false},
        {"CMPA immediate", 0x81, true, 0x20, 0x42, 0x20, 0, 0x20, true, false, false, false, false},
        {"SBCA immediate", 0x82, true, 0x00, 0x42, 0x00, microlind::CC_C, 0xFF, false, true, false, true, false},
        {"ANDA immediate", 0x84, true, 0xF0, 0x42, 0x0F, microlind::CC_C, 0x00, true, false, false, false, false},
        {"BITA immediate", 0x85, true, 0x80, 0x42, 0x80, microlind::CC_C, 0x80, false, true, false, true, false},
        {"EORA immediate", 0x88, true, 0xFF, 0x42, 0x0F, microlind::CC_C, 0xF0, false, true, false, false, false},
        {"ADCA immediate", 0x89, true, 0x7F, 0x42, 0x00, microlind::CC_C, 0x80, false, true, true, false, true},
        {"ORA immediate", 0x8A, true, 0x10, 0x42, 0x80, microlind::CC_C, 0x90, false, true, false, false, false},
        {"ADDA immediate", 0x8B, true, 0x0F, 0x42, 0x01, 0, 0x10, false, false, false, false, true},
        {"SUBB immediate", 0xC0, false, 0x10, 0x42, 0x20, 0, 0xF0, false, true, false, true, false},
        {"CMPB immediate", 0xC1, false, 0x20, 0x42, 0x20, 0, 0x20, true, false, false, false, false},
        {"SBCB immediate", 0xC2, false, 0x00, 0x42, 0x00, microlind::CC_C, 0xFF, false, true, false, true, false},
        {"ANDB immediate", 0xC4, false, 0xF0, 0x42, 0x0F, microlind::CC_C, 0x00, true, false, false, false, false},
        {"BITB immediate", 0xC5, false, 0x80, 0x42, 0x80, microlind::CC_C, 0x80, false, true, false, true, false},
        {"EORB immediate", 0xC8, false, 0xFF, 0x42, 0x0F, microlind::CC_C, 0xF0, false, true, false, false, false},
        {"ADCB immediate", 0xC9, false, 0x7F, 0x42, 0x00, microlind::CC_C, 0x80, false, true, true, false, true},
        {"ORB immediate", 0xCA, false, 0x10, 0x42, 0x80, microlind::CC_C, 0x90, false, true, false, false, false},
        {"ADDB immediate", 0xCB, false, 0x0F, 0x42, 0x01, 0, 0x10, false, false, false, false, true},
    };

    for (const auto& test : cases) {
        SCOPED_TRACE(test.name);
        microlind::Bus bus;
        microlind::test::map_flat_ram(bus);
        write_bytes(bus, 0x0100, {test.opcode, test.operand});

        microlind::Cpu cpu(microlind::CpuMode::HD6309);
        cpu.set_pc(0x0100);
        cpu.regs().a = test.target_a ? test.initial_target : test.initial_other;
        cpu.regs().b = test.target_a ? test.initial_other : test.initial_target;
        cpu.regs().cc = test.initial_cc;

        const auto result = cpu.tick(bus);
        EXPECT_EQ(result.cycles, 2u);
        EXPECT_EQ(cpu.regs().pc, 0x0102);
        EXPECT_EQ(test.target_a ? cpu.regs().a : cpu.regs().b, test.expected_target);
        EXPECT_EQ(test.target_a ? cpu.regs().b : cpu.regs().a, test.initial_other);
        EXPECT_EQ((cpu.regs().cc & microlind::CC_Z) != 0, test.expect_z);
        EXPECT_EQ((cpu.regs().cc & microlind::CC_N) != 0, test.expect_n);
        EXPECT_EQ((cpu.regs().cc & microlind::CC_V) != 0, test.expect_v);
        EXPECT_EQ((cpu.regs().cc & microlind::CC_C) != 0, test.expect_carry);
        EXPECT_EQ((cpu.regs().cc & microlind::CC_H) != 0, test.expect_half);
    }
}

TEST(CpuExecutionTest, LbsrPushesReturnAddressAndBranches) {
    microlind::Bus bus;
    microlind::test::map_flat_ram(bus);
    write_bytes(bus, 0x0100, {0x17, 0x00, 0x03});

    microlind::Cpu cpu(microlind::CpuMode::HD6309);
    cpu.set_pc(0x0100);
    cpu.regs().s = 0x9000;

    const auto result = cpu.tick(bus);
    EXPECT_EQ(result.cycles, 9u);
    EXPECT_EQ(cpu.regs().pc, 0x0106);
    EXPECT_EQ(cpu.regs().s, 0x8FFE);
    EXPECT_EQ(bus.peek8(0x8FFE), 0x01);
    EXPECT_EQ(bus.peek8(0x8FFF), 0x03);
}

TEST(CpuExecutionTest, BsrUsesNativeHD6309Timing) {
    microlind::Bus bus;
    microlind::test::map_flat_ram(bus);
    write_bytes(bus, 0x0100, {0x8D, 0xFE});

    microlind::Cpu cpu(microlind::CpuMode::HD6309);
    cpu.set_pc(0x0100);
    cpu.regs().s = 0x9000;
    cpu.regs().md = 0x01;

    const auto result = cpu.tick(bus);
    EXPECT_EQ(result.cycles, 6u);
    EXPECT_EQ(cpu.regs().pc, 0x0100);
    EXPECT_EQ(cpu.regs().s, 0x8FFE);
    EXPECT_EQ(bus.peek8(0x8FFE), 0x01);
    EXPECT_EQ(bus.peek8(0x8FFF), 0x02);
}

TEST(CpuExecutionTest, LbsrUsesNativeHD6309Timing) {
    microlind::Bus bus;
    microlind::test::map_flat_ram(bus);
    write_bytes(bus, 0x0100, {0x17, 0xFF, 0xFD});

    microlind::Cpu cpu(microlind::CpuMode::HD6309);
    cpu.set_pc(0x0100);
    cpu.regs().s = 0x9000;
    cpu.regs().md = 0x01;

    const auto result = cpu.tick(bus);
    EXPECT_EQ(result.cycles, 7u);
    EXPECT_EQ(cpu.regs().pc, 0x0100);
}

TEST(CpuExecutionTest, UsesDocumentedIndexedCycleAdditions) {
    struct CycleCase {
        std::string name;
        std::vector<uint8_t> bytes;
        uint16_t x{};
        uint8_t a{};
        uint8_t b{};
        uint8_t md{};
        uint8_t expected_cycles{};
        uint16_t expected_pc{};
    };

    const std::vector<CycleCase> cases = {
        {"LDB ,X", {0xE6, 0x84}, 0x2000, 0x00, 0x00, 0x00, 4, 0x0102},
        {"LDB 5-bit,X", {0xE6, 0x01}, 0x2000, 0x00, 0x00, 0x00, 5, 0x0102},
        {"LDB 8-bit,X", {0xE6, 0x88, 0x01}, 0x2000, 0x00, 0x00, 0x00, 5, 0x0103},
        {"LDB 16-bit,X emulation", {0xE6, 0x89, 0x00, 0x01}, 0x2000, 0x00, 0x00, 0x00, 8, 0x0104},
        {"LDB 16-bit,X native", {0xE6, 0x89, 0x00, 0x01}, 0x2000, 0x00, 0x00, 0x01, 7, 0x0104},
        {"LDB D,X emulation", {0xE6, 0x8B}, 0x2000, 0x00, 0x01, 0x00, 8, 0x0102},
        {"LDB D,X native", {0xE6, 0x8B}, 0x2000, 0x00, 0x01, 0x01, 6, 0x0102},
        {"JMP ,X", {0x6E, 0x84}, 0x2100, 0x00, 0x00, 0x00, 3, 0x2100},
        {"JMP 16-bit,PC emulation", {0x6E, 0x8D, 0x00, 0x04}, 0x2000, 0x00, 0x00, 0x00, 8, 0x0108},
        {"JMP 16-bit,PC native", {0x6E, 0x8D, 0x00, 0x04}, 0x2000, 0x00, 0x00, 0x01, 6, 0x0108},
    };

    for (const auto& test : cases) {
        SCOPED_TRACE(test.name);
        microlind::Bus bus;
        microlind::test::map_flat_ram(bus);
        write_bytes(bus, 0x0100, test.bytes);
        bus.write8(0x2000, 0x5A);
        bus.write8(0x2001, 0xA5);
        bus.write8(0x2100, 0x12);

        microlind::Cpu cpu(microlind::CpuMode::HD6309);
        cpu.set_pc(0x0100);
        cpu.regs().x = test.x;
        cpu.regs().a = test.a;
        cpu.regs().b = test.b;
        cpu.regs().md = test.md;

        const auto result = cpu.tick(bus);
        EXPECT_EQ(result.cycles, test.expected_cycles);
        EXPECT_EQ(cpu.regs().pc, test.expected_pc);
    }
}

TEST(CpuExecutionTest, UsesDocumentedIndexedIndirectCycleExceptions) {
    struct CycleCase {
        std::string name;
        std::vector<uint8_t> bytes;
        uint16_t x{};
        uint8_t a{};
        uint8_t b{};
        uint8_t expected_cycles{};
    };

    const std::vector<CycleCase> cases = {
        {"LDB [D,X]", {0xE6, 0x9B}, 0x2000, 0x00, 0x10, 8},
        {"LDB [16-bit,PC]", {0xE6, 0x9D, 0x20, 0x00}, 0x2000, 0x00, 0x00, 12},
        {"LDB [extended]", {0xE6, 0x9F, 0x20, 0x10}, 0x2000, 0x00, 0x00, 9},
    };

    for (const auto& test : cases) {
        SCOPED_TRACE(test.name);
        microlind::Bus bus;
        microlind::test::map_flat_ram(bus);
        write_bytes(bus, 0x0100, test.bytes);
        bus.write8(0x2010, 0x21);
        bus.write8(0x2011, 0x00);
        bus.write8(0x2100, 0xA5);
        bus.write8(0x2104, 0x21);
        bus.write8(0x2105, 0x00);

        microlind::Cpu cpu(microlind::CpuMode::HD6309);
        cpu.set_pc(0x0100);
        cpu.regs().x = test.x;
        cpu.regs().a = test.a;
        cpu.regs().b = test.b;

        const auto result = cpu.tick(bus);
        EXPECT_EQ(result.cycles, test.expected_cycles);
        EXPECT_EQ(cpu.regs().b, 0xA5);
    }
}

TEST(CpuExecutionTest, UnsupportedIndexedPostbytesTrapWithoutMemorySideEffects) {
    struct TrapCase {
        std::string name;
        std::vector<uint8_t> bytes;
    };

    const std::vector<TrapCase> cases = {
        {"LDB [,-X]", {0xE6, 0x92}},
        {"STA [,-X]", {0xA7, 0x92}},
    };

    for (const auto& test : cases) {
        SCOPED_TRACE(test.name);
        microlind::Bus bus;
        microlind::test::map_flat_ram(bus);
        write_bytes(bus, 0x0100, test.bytes);
        bus.write8(0xFFF0, 0x45);
        bus.write8(0xFFF1, 0x67);
        bus.write8(0x1FFF, 0x11);
        bus.write8(0x2000, 0x22);

        microlind::Cpu cpu(microlind::CpuMode::HD6309);
        cpu.set_pc(0x0100);
        cpu.regs().a = 0x99;
        cpu.regs().b = 0x88;
        cpu.regs().x = 0x2000;

        const auto result = cpu.tick(bus);
        EXPECT_EQ(result.cycles, 1u);
        EXPECT_EQ(cpu.regs().pc, 0x4567);
        EXPECT_NE(cpu.regs().md & 0x40, 0);
        EXPECT_EQ(cpu.regs().x, 0x2000);
        EXPECT_EQ(cpu.regs().a, 0x99);
        EXPECT_EQ(cpu.regs().b, 0x88);
        EXPECT_EQ(bus.read8(0x1FFF), 0x11);
        EXPECT_EQ(bus.read8(0x2000), 0x22);
    }
}

TEST(CpuExecutionTest, MC6809RejectsHD6309OnlyIndexedPostbytes) {
    microlind::Bus bus;
    microlind::test::map_flat_ram(bus);
    write_bytes(bus, 0x0100, {0xE6, 0x87}); // LDB E,X on HD6309; unsupported on MC6809.
    bus.write8(0x2000, 0x55);

    microlind::Cpu cpu(microlind::CpuMode::MC6809);
    cpu.set_pc(0x0100);
    cpu.regs().b = 0x88;
    cpu.regs().e = 0x00;
    cpu.regs().x = 0x2000;

    const auto result = cpu.tick(bus);
    EXPECT_EQ(result.cycles, 1u);
    EXPECT_EQ(cpu.regs().pc, 0x0102);
    EXPECT_EQ(cpu.regs().b, 0x88);
    EXPECT_EQ(cpu.regs().md & 0x40, 0);
}

TEST(CpuExecutionTest, ResolvesHD6309EFWRegisterIndexedAddresses) {
    struct AddressCase {
        std::string name;
        std::vector<uint8_t> bytes;
        uint8_t e{};
        uint8_t f{};
        uint16_t w{};
        uint8_t expected_value{};
        uint8_t expected_cycles{};
    };

    const std::vector<AddressCase> cases = {
        {"LDB E,X", {0xE6, 0x87}, 0x05, 0x00, 0x0000, 0xE5, 5},
        {"LDB F,X", {0xE6, 0x8A}, 0x00, 0xFC, 0x0000, 0xF2, 5},
        {"LDB W,X", {0xE6, 0x8E}, 0x00, 0x00, 0x0010, 0xA1, 8},
        {"LDB [E,X]", {0xE6, 0x97}, 0x06, 0x00, 0x0000, 0x5E, 5},
        {"LDB [F,X]", {0xE6, 0x9A}, 0x00, 0xFD, 0x0000, 0x5F, 5},
        {"LDB [W,X]", {0xE6, 0x9E}, 0x00, 0x00, 0x0012, 0x5A, 8},
    };

    for (const auto& test : cases) {
        SCOPED_TRACE(test.name);
        microlind::Bus bus;
        microlind::test::map_flat_ram(bus);
        write_bytes(bus, 0x0100, test.bytes);
        bus.write8(0x2005, 0xE5);
        bus.write8(0x1FFC, 0xF2);
        bus.write8(0x2010, 0xA1);
        bus.write8(0x2006, 0x21);
        bus.write8(0x2007, 0x5E);
        bus.write8(0x1FFD, 0x21);
        bus.write8(0x1FFE, 0x5F);
        bus.write8(0x2012, 0x21);
        bus.write8(0x2013, 0x5A);
        bus.write8(0x215A, 0x5A);
        bus.write8(0x215E, 0x5E);
        bus.write8(0x215F, 0x5F);

        microlind::Cpu cpu(microlind::CpuMode::HD6309);
        cpu.set_pc(0x0100);
        cpu.regs().x = 0x2000;
        cpu.regs().e = test.e;
        cpu.regs().f = test.f;
        cpu.regs().md = 0x00;
        if (test.w != 0) {
            cpu.regs().e = static_cast<uint8_t>((test.w >> 8) & 0xFF);
            cpu.regs().f = static_cast<uint8_t>(test.w & 0xFF);
        }

        const auto result = cpu.tick(bus);
        EXPECT_EQ(result.cycles, test.expected_cycles);
        EXPECT_EQ(cpu.regs().b, test.expected_value);
    }
}

TEST(CpuExecutionTest, ResolvesHD6309WRelativeIndexedAddresses) {
    struct AddressCase {
        std::string name;
        std::vector<uint8_t> bytes;
        uint16_t w{};
        uint8_t md{};
        uint8_t expected_value{};
        uint8_t expected_cycles{};
        uint16_t expected_w{};
    };

    const std::vector<AddressCase> cases = {
        {"LDB ,W", {0xE6, 0x8F}, 0x2100, 0x00, 0xA0, 4, 0x2100},
        {"LDB n,W emulation", {0xE6, 0xAF, 0x00, 0x04}, 0x2100, 0x00, 0xA4, 9, 0x2100},
        {"LDB n,W native", {0xE6, 0xAF, 0x00, 0x04}, 0x2100, 0x01, 0xA4, 6, 0x2100},
        {"LDB ,W++", {0xE6, 0xCF}, 0x2100, 0x00, 0xA0, 7, 0x2102},
        {"LDB ,--W", {0xE6, 0xEF}, 0x2102, 0x00, 0xA0, 7, 0x2100},
        {"LDB [,W]", {0xE6, 0x90}, 0x2200, 0x00, 0xB0, 4, 0x2200},
        {"LDB [n,W]", {0xE6, 0xB0, 0x00, 0x04}, 0x2200, 0x00, 0xB4, 9, 0x2200},
        {"LDB [,W++]", {0xE6, 0xD0}, 0x2200, 0x00, 0xB0, 7, 0x2202},
        {"LDB [,--W]", {0xE6, 0xF0}, 0x2202, 0x00, 0xB0, 7, 0x2200},
    };

    for (const auto& test : cases) {
        SCOPED_TRACE(test.name);
        microlind::Bus bus;
        microlind::test::map_flat_ram(bus);
        write_bytes(bus, 0x0100, test.bytes);
        bus.write8(0x2100, 0xA0);
        bus.write8(0x2104, 0xA4);
        bus.write8(0x2200, 0x23);
        bus.write8(0x2201, 0x00);
        bus.write8(0x2204, 0x23);
        bus.write8(0x2205, 0x04);
        bus.write8(0x2300, 0xB0);
        bus.write8(0x2304, 0xB4);

        microlind::Cpu cpu(microlind::CpuMode::HD6309);
        cpu.set_pc(0x0100);
        cpu.regs().e = static_cast<uint8_t>((test.w >> 8) & 0xFF);
        cpu.regs().f = static_cast<uint8_t>(test.w & 0xFF);
        cpu.regs().md = test.md;

        const auto result = cpu.tick(bus);
        EXPECT_EQ(result.cycles, test.expected_cycles);
        EXPECT_EQ(cpu.regs().b, test.expected_value);
        EXPECT_EQ(static_cast<uint16_t>((cpu.regs().e << 8) | cpu.regs().f), test.expected_w);
    }
}

TEST(CpuExecutionTest, UsesDocumentedDivqImmediateCycles) {
    microlind::Bus bus;
    microlind::test::map_flat_ram(bus);
    write_bytes(bus, 0x0100, {0x11, 0x8E, 0x00, 0x02});

    microlind::Cpu cpu(microlind::CpuMode::HD6309);
    cpu.set_pc(0x0100);
    cpu.regs().e = 0x00;
    cpu.regs().f = 0x10;

    const auto result = cpu.tick(bus);
    EXPECT_EQ(result.cycles, 36u);
    EXPECT_EQ(cpu.regs().e, 0x00);
    EXPECT_EQ(cpu.regs().f, 0x08);
}

TEST(CpuExecutionTest, HD6309OnlyOpcodesAreRejectedInMC6809AndAcceptedInHD6309) {
    struct OpcodeCase {
        std::string name;
        std::vector<uint8_t> bytes;
    };

    const std::vector<OpcodeCase> opcodes = {
        {"OIM direct", {0x01, 0x01, 0x20}},
        {"AIM direct", {0x02, 0x01, 0x20}},
        {"EIM direct", {0x05, 0x01, 0x20}},
        {"TIM direct", {0x0B, 0x01, 0x20}},
        {"SEXW", {0x14}},
        {"OIM indexed", {0x61, 0x01, 0x84}},
        {"AIM indexed", {0x62, 0x01, 0x84}},
        {"EIM indexed", {0x65, 0x01, 0x84}},
        {"TIM indexed", {0x6B, 0x01, 0x84}},
        {"OIM extended", {0x71, 0x01, 0x20, 0x00}},
        {"AIM extended", {0x72, 0x01, 0x20, 0x00}},
        {"EIM extended", {0x75, 0x01, 0x20, 0x00}},
        {"TIM extended", {0x7B, 0x01, 0x20, 0x00}},
        {"LDQ immediate", {0xCD, 0x12, 0x34, 0x56, 0x78}},

        {"ADDR", {0x10, 0x30, 0x01}},
        {"ADCR", {0x10, 0x31, 0x01}},
        {"SUBR", {0x10, 0x32, 0x01}},
        {"SBCR", {0x10, 0x33, 0x01}},
        {"ANDR", {0x10, 0x34, 0x01}},
        {"ORR", {0x10, 0x35, 0x01}},
        {"EORR", {0x10, 0x36, 0x01}},
        {"CMPR", {0x10, 0x37, 0x01}},
        {"PSHSW", {0x10, 0x38}},
        {"PULSW", {0x10, 0x39}},
        {"PSHUW", {0x10, 0x3A}},
        {"PULUW", {0x10, 0x3B}},
        {"NEGD", {0x10, 0x40}},
        {"COMD", {0x10, 0x43}},
        {"LSRD", {0x10, 0x44}},
        {"RORD", {0x10, 0x46}},
        {"ASRD", {0x10, 0x47}},
        {"LSLD", {0x10, 0x48}},
        {"ROLD", {0x10, 0x49}},
        {"DECD", {0x10, 0x4A}},
        {"INCD", {0x10, 0x4C}},
        {"TSTD", {0x10, 0x4D}},
        {"CLRD", {0x10, 0x4F}},
        {"COMW", {0x10, 0x53}},
        {"LSRW", {0x10, 0x54}},
        {"RORW", {0x10, 0x56}},
        {"LSLW", {0x10, 0x58}},
        {"ROLW", {0x10, 0x59}},
        {"DECW", {0x10, 0x5A}},
        {"INCW", {0x10, 0x5C}},
        {"TSTW", {0x10, 0x5D}},
        {"CLRW", {0x10, 0x5F}},
        {"SUBW immediate", {0x10, 0x80, 0x00, 0x01}},
        {"CMPW immediate", {0x10, 0x81, 0x00, 0x01}},
        {"SBCD immediate", {0x10, 0x82, 0x00, 0x01}},
        {"ANDD immediate", {0x10, 0x84, 0xFF, 0xFF}},
        {"BITD immediate", {0x10, 0x85, 0xFF, 0xFF}},
        {"LDW immediate", {0x10, 0x86, 0x00, 0x01}},
        {"EORD immediate", {0x10, 0x88, 0xFF, 0xFF}},
        {"ADCD immediate", {0x10, 0x89, 0x00, 0x01}},
        {"ORD immediate", {0x10, 0x8A, 0x00, 0x01}},
        {"ADDW immediate", {0x10, 0x8B, 0x00, 0x01}},
        {"SUBW direct", {0x10, 0x90, 0x20}},
        {"CMPW direct", {0x10, 0x91, 0x20}},
        {"SBCD direct", {0x10, 0x92, 0x20}},
        {"ANDD direct", {0x10, 0x94, 0x20}},
        {"BITD direct", {0x10, 0x95, 0x20}},
        {"LDW direct", {0x10, 0x96, 0x20}},
        {"STW direct", {0x10, 0x97, 0x20}},
        {"EORD direct", {0x10, 0x98, 0x20}},
        {"ADCD direct", {0x10, 0x99, 0x20}},
        {"ORD direct", {0x10, 0x9A, 0x20}},
        {"ADDW direct", {0x10, 0x9B, 0x20}},
        {"SUBW indexed", {0x10, 0xA0, 0x84}},
        {"CMPW indexed", {0x10, 0xA1, 0x84}},
        {"SBCD indexed", {0x10, 0xA2, 0x84}},
        {"ANDD indexed", {0x10, 0xA4, 0x84}},
        {"BITD indexed", {0x10, 0xA5, 0x84}},
        {"LDW indexed", {0x10, 0xA6, 0x84}},
        {"STW indexed", {0x10, 0xA7, 0x84}},
        {"EORD indexed", {0x10, 0xA8, 0x84}},
        {"ADCD indexed", {0x10, 0xA9, 0x84}},
        {"ORD indexed", {0x10, 0xAA, 0x84}},
        {"ADDW indexed", {0x10, 0xAB, 0x84}},
        {"SUBW extended", {0x10, 0xB0, 0x20, 0x00}},
        {"CMPW extended", {0x10, 0xB1, 0x20, 0x00}},
        {"SBCD extended", {0x10, 0xB2, 0x20, 0x00}},
        {"ANDD extended", {0x10, 0xB4, 0x20, 0x00}},
        {"BITD extended", {0x10, 0xB5, 0x20, 0x00}},
        {"LDW extended", {0x10, 0xB6, 0x20, 0x00}},
        {"STW extended", {0x10, 0xB7, 0x20, 0x00}},
        {"EORD extended", {0x10, 0xB8, 0x20, 0x00}},
        {"ADCD extended", {0x10, 0xB9, 0x20, 0x00}},
        {"ORD extended", {0x10, 0xBA, 0x20, 0x00}},
        {"ADDW extended", {0x10, 0xBB, 0x20, 0x00}},
        {"LDQ direct", {0x10, 0xDC, 0x20}},
        {"STQ direct", {0x10, 0xDD, 0x20}},
        {"LDQ indexed", {0x10, 0xEC, 0x84}},
        {"STQ indexed", {0x10, 0xED, 0x84}},
        {"LDQ extended", {0x10, 0xFC, 0x20, 0x00}},
        {"STQ extended", {0x10, 0xFD, 0x20, 0x00}},

        {"BAND", {0x11, 0x30, 0x00, 0x20}},
        {"BIAND", {0x11, 0x31, 0x00, 0x20}},
        {"BOR", {0x11, 0x32, 0x00, 0x20}},
        {"BIOR", {0x11, 0x33, 0x00, 0x20}},
        {"BEOR", {0x11, 0x34, 0x00, 0x20}},
        {"BIEOR", {0x11, 0x35, 0x00, 0x20}},
        {"LDBT", {0x11, 0x36, 0x00, 0x20}},
        {"STBT", {0x11, 0x37, 0x00, 0x20}},
        {"TFM plus plus", {0x11, 0x38, 0x12}},
        {"TFM minus minus", {0x11, 0x39, 0x12}},
        {"TFM plus minus", {0x11, 0x3A, 0x12}},
        {"TFM minus plus", {0x11, 0x3B, 0x12}},
        {"BITMD", {0x11, 0x3C, 0x40}},
        {"LDMD", {0x11, 0x3D, 0x40}},
        {"COME", {0x11, 0x43}},
        {"DECE", {0x11, 0x4A}},
        {"INCE", {0x11, 0x4C}},
        {"TSTE", {0x11, 0x4D}},
        {"CLRE", {0x11, 0x4F}},
        {"COMF", {0x11, 0x53}},
        {"DECF", {0x11, 0x5A}},
        {"INCF", {0x11, 0x5C}},
        {"TSTF", {0x11, 0x5D}},
        {"CLRF", {0x11, 0x5F}},
        {"SUBE immediate", {0x11, 0x80, 0x01}},
        {"CMPE immediate", {0x11, 0x81, 0x01}},
        {"ADDE immediate", {0x11, 0x8B, 0x01}},
        {"DIVD immediate", {0x11, 0x8D, 0x02}},
        {"DIVQ immediate", {0x11, 0x8E, 0x00, 0x02}},
        {"MULD immediate", {0x11, 0x8F, 0x00, 0x02}},
        {"SUBE direct", {0x11, 0x90, 0x20}},
        {"CMPE direct", {0x11, 0x91, 0x20}},
        {"ADDE direct", {0x11, 0x9B, 0x20}},
        {"DIVD direct", {0x11, 0x9D, 0x20}},
        {"DIVQ direct", {0x11, 0x9E, 0x20}},
        {"MULD direct", {0x11, 0x9F, 0x20}},
        {"SUBE indexed", {0x11, 0xA0, 0x84}},
        {"CMPE indexed", {0x11, 0xA1, 0x84}},
        {"ADDE indexed", {0x11, 0xAB, 0x84}},
        {"DIVD indexed", {0x11, 0xAD, 0x84}},
        {"DIVQ indexed", {0x11, 0xAE, 0x84}},
        {"MULD indexed", {0x11, 0xAF, 0x84}},
        {"SUBE extended", {0x11, 0xB0, 0x20, 0x00}},
        {"CMPE extended", {0x11, 0xB1, 0x20, 0x00}},
        {"ADDE extended", {0x11, 0xBB, 0x20, 0x00}},
        {"DIVD extended", {0x11, 0xBD, 0x20, 0x00}},
        {"DIVQ extended", {0x11, 0xBE, 0x20, 0x00}},
        {"MULD extended", {0x11, 0xBF, 0x20, 0x00}},
        {"SUBF immediate", {0x11, 0xC0, 0x01}},
        {"CMPF immediate", {0x11, 0xC1, 0x01}},
        {"ADDF immediate", {0x11, 0xCB, 0x01}},
        {"SUBF direct", {0x11, 0xD0, 0x20}},
        {"CMPF direct", {0x11, 0xD1, 0x20}},
        {"ADDF direct", {0x11, 0xDB, 0x20}},
        {"SUBF indexed", {0x11, 0xE0, 0x84}},
        {"CMPF indexed", {0x11, 0xE1, 0x84}},
        {"ADDF indexed", {0x11, 0xEB, 0x84}},
        {"SUBF extended", {0x11, 0xF0, 0x20, 0x00}},
        {"CMPF extended", {0x11, 0xF1, 0x20, 0x00}},
        {"ADDF extended", {0x11, 0xFB, 0x20, 0x00}},
    };

    for (const auto& opcode : opcodes) {
        SCOPED_TRACE(opcode.name);

        {
            microlind::Bus bus;
            microlind::test::map_flat_ram(bus);
            microlind::Cpu cpu(microlind::CpuMode::MC6809);
            prime_hd6309_test_state(bus, cpu);
            cpu.set_pc(0x0100);
            for (std::size_t i = 0; i < opcode.bytes.size(); ++i) {
                bus.write8(static_cast<uint16_t>(0x0100 + i), opcode.bytes[i]);
            }

            const auto result = cpu.tick(bus);
            const uint16_t expected_pc = static_cast<uint16_t>(0x0100 + (opcode.bytes[0] == 0x10 || opcode.bytes[0] == 0x11 ? 2 : 1));
            EXPECT_EQ(result.cycles, 1u);
            EXPECT_EQ(cpu.regs().pc, expected_pc);
            EXPECT_EQ(cpu.regs().md & 0x40, 0);
        }

        {
            microlind::Bus bus;
            microlind::test::map_flat_ram(bus);
            microlind::Cpu cpu(microlind::CpuMode::HD6309);
            prime_hd6309_test_state(bus, cpu);
            cpu.set_pc(0x0100);
            for (std::size_t i = 0; i < opcode.bytes.size(); ++i) {
                bus.write8(static_cast<uint16_t>(0x0100 + i), opcode.bytes[i]);
            }

            const auto result = cpu.tick(bus);
            EXPECT_NE(result.cycles, 1u);
            EXPECT_NE(cpu.regs().pc, 0x3456);
            EXPECT_EQ(cpu.regs().md & 0x80, 0);
        }
    }
}

TEST(CpuExecutionTest, HD6309ExecutesDRegisterLogicInstructions) {
    microlind::Bus bus;
    microlind::test::map_flat_ram(bus);
    write_bytes(bus, 0x0100, {
        0x10, 0x84, 0x0F, 0x0F, // ANDD #$0F0F
        0x10, 0x88, 0xF0, 0x00, // EORD #$F000
        0x10, 0x85, 0xF0, 0x00, // BITD #$F000
    });

    microlind::Cpu cpu(microlind::CpuMode::HD6309);
    cpu.set_pc(0x0100);
    cpu.regs().a = 0xF0;
    cpu.regs().b = 0xF0;

    EXPECT_EQ(cpu.tick(bus).cycles, 5u);
    EXPECT_EQ(cpu.regs().a, 0x00);
    EXPECT_EQ(cpu.regs().b, 0x00);
    EXPECT_NE(cpu.regs().cc & microlind::CC_Z, 0);

    EXPECT_EQ(cpu.tick(bus).cycles, 5u);
    EXPECT_EQ(cpu.regs().a, 0xF0);
    EXPECT_EQ(cpu.regs().b, 0x00);
    EXPECT_NE(cpu.regs().cc & microlind::CC_N, 0);

    EXPECT_EQ(cpu.tick(bus).cycles, 5u);
    EXPECT_EQ(cpu.regs().a, 0xF0);
    EXPECT_EQ(cpu.regs().b, 0x00);
    EXPECT_NE(cpu.regs().cc & microlind::CC_N, 0);
    EXPECT_EQ(cpu.regs().cc & microlind::CC_Z, 0);
}

TEST(CpuExecutionTest, UpdatesArithmeticAndShiftFlags) {
    microlind::Bus bus;
    microlind::test::map_flat_ram(bus);
    write_bytes(bus, 0x0100, {
        0xC3, 0x00, 0x01,       // ADDD #$0001
        0x83, 0x00, 0x01,       // SUBD #$0001
        0x10, 0x48,             // LSLD
        0x10, 0x8B, 0x00, 0x01, // ADDW #$0001
    });

    microlind::Cpu cpu(microlind::CpuMode::HD6309);
    cpu.set_pc(0x0100);
    cpu.regs().a = 0x7F;
    cpu.regs().b = 0xFF;

    EXPECT_EQ(cpu.tick(bus).cycles, 4u);
    EXPECT_EQ(cpu.regs().a, 0x80);
    EXPECT_EQ(cpu.regs().b, 0x00);
    EXPECT_NE(cpu.regs().cc & microlind::CC_N, 0);
    EXPECT_NE(cpu.regs().cc & microlind::CC_V, 0);
    EXPECT_EQ(cpu.regs().cc & microlind::CC_C, 0);

    cpu.regs().a = 0x00;
    cpu.regs().b = 0x00;
    EXPECT_EQ(cpu.tick(bus).cycles, 4u);
    EXPECT_EQ(cpu.regs().a, 0xFF);
    EXPECT_EQ(cpu.regs().b, 0xFF);
    EXPECT_NE(cpu.regs().cc & microlind::CC_N, 0);
    EXPECT_NE(cpu.regs().cc & microlind::CC_C, 0);

    cpu.regs().a = 0x80;
    cpu.regs().b = 0x01;
    EXPECT_EQ(cpu.tick(bus).cycles, 3u);
    EXPECT_EQ(cpu.regs().a, 0x00);
    EXPECT_EQ(cpu.regs().b, 0x02);
    EXPECT_NE(cpu.regs().cc & microlind::CC_C, 0);
    EXPECT_NE(cpu.regs().cc & microlind::CC_V, 0);

    cpu.regs().e = 0xFF;
    cpu.regs().f = 0xFF;
    EXPECT_EQ(cpu.tick(bus).cycles, 5u);
    EXPECT_EQ(cpu.regs().e, 0x00);
    EXPECT_EQ(cpu.regs().f, 0x00);
    EXPECT_NE(cpu.regs().cc & microlind::CC_Z, 0);
    EXPECT_NE(cpu.regs().cc & microlind::CC_C, 0);
}

TEST(CpuExecutionTest, HD6309ExecutesDAndEFUnaryInstructions) {
    microlind::Bus bus;
    microlind::test::map_flat_ram(bus);
    write_bytes(bus, 0x0100, {
        0x10, 0x43, // COMD
        0x10, 0x4A, // DECD
        0x10, 0x4C, // INCD
        0x10, 0x4F, // CLRD
        0x11, 0x43, // COME
        0x11, 0x5A, // DECF
        0x11, 0x5C, // INCF
        0x11, 0x5D, // TSTF
        0x11, 0x4F, // CLRE
    });

    microlind::Cpu cpu(microlind::CpuMode::HD6309);
    cpu.set_pc(0x0100);
    cpu.regs().a = 0x00;
    cpu.regs().b = 0xF0;
    cpu.regs().e = 0x00;
    cpu.regs().f = 0x01;

    EXPECT_EQ(cpu.tick(bus).cycles, 3u);
    EXPECT_EQ(cpu.regs().a, 0xFF);
    EXPECT_EQ(cpu.regs().b, 0x0F);
    EXPECT_NE(cpu.regs().cc & microlind::CC_C, 0);

    EXPECT_EQ(cpu.tick(bus).cycles, 3u);
    EXPECT_EQ(cpu.regs().a, 0xFF);
    EXPECT_EQ(cpu.regs().b, 0x0E);

    EXPECT_EQ(cpu.tick(bus).cycles, 3u);
    EXPECT_EQ(cpu.regs().a, 0xFF);
    EXPECT_EQ(cpu.regs().b, 0x0F);

    EXPECT_EQ(cpu.tick(bus).cycles, 3u);
    EXPECT_EQ(cpu.regs().a, 0x00);
    EXPECT_EQ(cpu.regs().b, 0x00);
    EXPECT_NE(cpu.regs().cc & microlind::CC_Z, 0);

    EXPECT_EQ(cpu.tick(bus).cycles, 3u);
    EXPECT_EQ(cpu.regs().e, 0xFF);
    EXPECT_NE(cpu.regs().cc & microlind::CC_N, 0);

    EXPECT_EQ(cpu.tick(bus).cycles, 3u);
    EXPECT_EQ(cpu.regs().f, 0x00);
    EXPECT_NE(cpu.regs().cc & microlind::CC_Z, 0);

    EXPECT_EQ(cpu.tick(bus).cycles, 3u);
    EXPECT_EQ(cpu.regs().f, 0x01);

    EXPECT_EQ(cpu.tick(bus).cycles, 3u);
    EXPECT_EQ(cpu.regs().f, 0x01);
    EXPECT_EQ(cpu.regs().cc & microlind::CC_Z, 0);

    EXPECT_EQ(cpu.tick(bus).cycles, 3u);
    EXPECT_EQ(cpu.regs().e, 0x00);
    EXPECT_NE(cpu.regs().cc & microlind::CC_Z, 0);
}

TEST(CpuExecutionTest, HD6309ExecutesDivdAddressingModes) {
    microlind::Bus bus;
    microlind::test::map_flat_ram(bus);
    write_bytes(bus, 0x0100, {
        0x11, 0x8D, 0x05,       // DIVD #$05
        0x11, 0x9D, 0x20,       // DIVD <$20
        0x11, 0xAD, 0x84,       // DIVD ,X
        0x11, 0xBD, 0x20, 0x00, // DIVD $2000
    });
    bus.write8(0x0020, 0x05);
    bus.write8(0x1234, 0xFE);
    bus.write8(0x2000, 0x07);

    microlind::Cpu cpu(microlind::CpuMode::HD6309);
    cpu.set_pc(0x0100);

    cpu.regs().a = 0x00;
    cpu.regs().b = 0x15;
    EXPECT_EQ(cpu.tick(bus).cycles, 25u);
    EXPECT_EQ(cpu.regs().e, 0x00);
    EXPECT_EQ(cpu.regs().f, 0x04);
    EXPECT_EQ(cpu.regs().a, 0x00);
    EXPECT_EQ(cpu.regs().b, 0x01);

    cpu.regs().a = 0x00;
    cpu.regs().b = 0x16;
    EXPECT_EQ(cpu.tick(bus).cycles, 27u);
    EXPECT_EQ(cpu.regs().e, 0x00);
    EXPECT_EQ(cpu.regs().f, 0x04);
    EXPECT_EQ(cpu.regs().a, 0x00);
    EXPECT_EQ(cpu.regs().b, 0x02);

    cpu.regs().x = 0x1234;
    cpu.regs().a = 0xFF;
    cpu.regs().b = 0xF7;
    EXPECT_EQ(cpu.tick(bus).cycles, 27u);
    EXPECT_EQ(cpu.regs().e, 0x00);
    EXPECT_EQ(cpu.regs().f, 0x04);
    EXPECT_EQ(cpu.regs().a, 0xFF);
    EXPECT_EQ(cpu.regs().b, 0xFF);

    cpu.regs().a = 0x00;
    cpu.regs().b = 0x0F;
    EXPECT_EQ(cpu.tick(bus).cycles, 28u);
    EXPECT_EQ(cpu.regs().e, 0x00);
    EXPECT_EQ(cpu.regs().f, 0x02);
    EXPECT_EQ(cpu.regs().a, 0x00);
    EXPECT_EQ(cpu.regs().b, 0x01);
}

TEST(CpuExecutionTest, HD6309DivdByZeroTrapsThroughFFF0Vector) {
    microlind::Bus bus;
    microlind::test::map_flat_ram(bus);
    write_bytes(bus, 0x0100, {0x11, 0x8D, 0x00});
    bus.write8(0xFFF0, 0x45);
    bus.write8(0xFFF1, 0x67);

    microlind::Cpu cpu(microlind::CpuMode::HD6309);
    cpu.set_pc(0x0100);
    cpu.regs().a = 0x12;
    cpu.regs().b = 0x34;

    EXPECT_EQ(cpu.tick(bus).cycles, 25u);
    EXPECT_EQ(cpu.regs().pc, 0x4567);
    EXPECT_NE(cpu.regs().md & 0x80, 0);
}

TEST(CpuExecutionTest, HD6309DivisionByZeroTrapsAcrossAddressingModes) {
    struct TrapCase {
        std::string name;
        std::vector<uint8_t> bytes;
        uint8_t expected_cycles{};
    };

    const std::vector<TrapCase> cases = {
        {"DIVD immediate", {0x11, 0x8D, 0x00}, 25},
        {"DIVD direct", {0x11, 0x9D, 0x20}, 27},
        {"DIVD indexed", {0x11, 0xAD, 0x84}, 27},
        {"DIVD extended", {0x11, 0xBD, 0x20, 0x00}, 28},
        {"DIVQ immediate", {0x11, 0x8E, 0x00, 0x00}, 36},
        {"DIVQ direct", {0x11, 0x9E, 0x20}, 36},
        {"DIVQ indexed", {0x11, 0xAE, 0x84}, 36},
        {"DIVQ extended", {0x11, 0xBE, 0x20, 0x00}, 37},
    };

    for (const auto& test : cases) {
        SCOPED_TRACE(test.name);
        microlind::Bus bus;
        microlind::test::map_flat_ram(bus);
        write_bytes(bus, 0x0100, test.bytes);
        bus.write8(0xFFF0, 0x45);
        bus.write8(0xFFF1, 0x67);
        bus.write8(0x0020, 0x00);
        bus.write8(0x0021, 0x00);
        bus.write8(0x2000, 0x00);
        bus.write8(0x2001, 0x00);

        microlind::Cpu cpu(microlind::CpuMode::HD6309);
        cpu.set_pc(0x0100);
        cpu.regs().a = 0x12;
        cpu.regs().b = 0x34;
        cpu.regs().e = 0x56;
        cpu.regs().f = 0x78;
        cpu.regs().x = 0x2000;

        const auto result = cpu.tick(bus);
        EXPECT_EQ(result.cycles, test.expected_cycles);
        EXPECT_EQ(cpu.regs().pc, 0x4567);
        EXPECT_NE(cpu.regs().md & 0x80, 0);
    }
}

TEST(CpuExecutionTest, HD6309ExecutesBitmdAndAndr) {
    microlind::Bus bus;
    microlind::test::map_flat_ram(bus);
    write_bytes(bus, 0x0100, {
        0x11, 0x3C, 0x40, // BITMD #$40
        0x10, 0x34, 0x01, // ANDR D,X
    });

    microlind::Cpu cpu(microlind::CpuMode::HD6309);
    cpu.set_pc(0x0100);
    cpu.regs().md = 0x40;
    cpu.regs().a = 0x0F;
    cpu.regs().b = 0x0F;
    cpu.regs().x = 0xFF00;

    EXPECT_EQ(cpu.tick(bus).cycles, 4u);
    EXPECT_EQ(cpu.regs().cc & microlind::CC_Z, 0);

    EXPECT_EQ(cpu.tick(bus).cycles, 4u);
    EXPECT_EQ(cpu.regs().x, 0x0F00);
    EXPECT_EQ(cpu.regs().cc & microlind::CC_Z, 0);
}

TEST(CpuExecutionTest, PulsPullsBAndPcFromSystemStack) {
    microlind::Bus bus;
    microlind::test::map_flat_ram(bus);
    write_bytes(bus, 0x03C0, {0x35, 0x84}); // PULS B,PC
    bus.write8(0x9000, 0x7A);
    bus.write8(0x9001, 0x12);
    bus.write8(0x9002, 0x34);

    microlind::Cpu cpu(microlind::CpuMode::HD6309);
    cpu.set_pc(0x03C0);
    cpu.regs().s = 0x9000;

    const auto result = cpu.tick(bus);
    EXPECT_EQ(result.cycles, 8u);
    EXPECT_EQ(cpu.regs().b, 0x7A);
    EXPECT_EQ(cpu.regs().pc, 0x1234);
    EXPECT_EQ(cpu.regs().s, 0x9003);
}

TEST(CpuExecutionTest, PshsUsesDocumentedStackMaskOrder) {
    microlind::Bus bus;
    microlind::test::map_flat_ram(bus);
    write_bytes(bus, 0x03C0, {0x34, 0x84}); // PSHS B,PC

    microlind::Cpu cpu(microlind::CpuMode::HD6309);
    cpu.set_pc(0x03C0);
    cpu.regs().s = 0x9000;
    cpu.regs().b = 0x7A;

    const auto result = cpu.tick(bus);
    EXPECT_EQ(result.cycles, 8u);
    EXPECT_EQ(cpu.regs().s, 0x8FFD);
    EXPECT_EQ(bus.peek8(0x8FFD), 0x7A);
    EXPECT_EQ(bus.peek8(0x8FFE), 0x03);
    EXPECT_EQ(bus.peek8(0x8FFF), 0xC2);
}

TEST(CpuExecutionTest, PuluPullsBAndPcFromUserStack) {
    microlind::Bus bus;
    microlind::test::map_flat_ram(bus);
    write_bytes(bus, 0x0400, {0x37, 0x84}); // PULU B,PC
    bus.write8(0xA000, 0x5C);
    bus.write8(0xA001, 0x56);
    bus.write8(0xA002, 0x78);

    microlind::Cpu cpu(microlind::CpuMode::HD6309);
    cpu.set_pc(0x0400);
    cpu.regs().u = 0xA000;

    const auto result = cpu.tick(bus);
    EXPECT_EQ(result.cycles, 8u);
    EXPECT_EQ(cpu.regs().b, 0x5C);
    EXPECT_EQ(cpu.regs().pc, 0x5678);
    EXPECT_EQ(cpu.regs().u, 0xA003);
}

TEST(CpuExecutionTest, HD6309ExecutesWStackShortcuts) {
    microlind::Bus bus;
    microlind::test::map_flat_ram(bus);
    write_bytes(bus, 0x0100, {
        0x10, 0x38, // PSHSW
        0x10, 0x39, // PULSW
        0x10, 0x3A, // PSHUW
        0x10, 0x3B, // PULUW
    });

    microlind::Cpu cpu(microlind::CpuMode::HD6309);
    cpu.set_pc(0x0100);
    cpu.regs().s = 0x9000;
    cpu.regs().u = 0xA000;
    cpu.regs().e = 0xBE;
    cpu.regs().f = 0xEF;

    EXPECT_EQ(cpu.tick(bus).cycles, 6u);
    EXPECT_EQ(cpu.regs().s, 0x8FFE);
    EXPECT_EQ(bus.read8(0x8FFE), 0xBE);
    EXPECT_EQ(bus.read8(0x8FFF), 0xEF);

    cpu.regs().e = 0x00;
    cpu.regs().f = 0x00;
    EXPECT_EQ(cpu.tick(bus).cycles, 6u);
    EXPECT_EQ(cpu.regs().s, 0x9000);
    EXPECT_EQ(cpu.regs().e, 0xBE);
    EXPECT_EQ(cpu.regs().f, 0xEF);

    EXPECT_EQ(cpu.tick(bus).cycles, 6u);
    EXPECT_EQ(cpu.regs().u, 0x9FFE);
    EXPECT_EQ(bus.read8(0x9FFE), 0xBE);
    EXPECT_EQ(bus.read8(0x9FFF), 0xEF);

    cpu.regs().e = 0x00;
    cpu.regs().f = 0x00;
    EXPECT_EQ(cpu.tick(bus).cycles, 6u);
    EXPECT_EQ(cpu.regs().u, 0xA000);
    EXPECT_EQ(cpu.regs().e, 0xBE);
    EXPECT_EQ(cpu.regs().f, 0xEF);
}

} // namespace
