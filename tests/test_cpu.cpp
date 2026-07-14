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
        {"CMPS immediate", {0x11, 0x8E, 0x00, 0x01}},
        {"DIVQ immediate", {0x11, 0x8F, 0x00, 0x02}},
        {"SUBE direct", {0x11, 0x90, 0x20}},
        {"CMPE direct", {0x11, 0x91, 0x20}},
        {"ADDE direct", {0x11, 0x9B, 0x20}},
        {"DIVD direct", {0x11, 0x9D, 0x20}},
        {"CMPS direct", {0x11, 0x9E, 0x20}},
        {"MULD direct", {0x11, 0x9F, 0x20}},
        {"SUBE indexed", {0x11, 0xA0, 0x84}},
        {"CMPE indexed", {0x11, 0xA1, 0x84}},
        {"ADDE indexed", {0x11, 0xAB, 0x84}},
        {"DIVD indexed", {0x11, 0xAD, 0x84}},
        {"CMPS indexed", {0x11, 0xAE, 0x84}},
        {"MULD indexed", {0x11, 0xAF, 0x84}},
        {"SUBE extended", {0x11, 0xB0, 0x20, 0x00}},
        {"CMPE extended", {0x11, 0xB1, 0x20, 0x00}},
        {"ADDE extended", {0x11, 0xBB, 0x20, 0x00}},
        {"DIVD extended", {0x11, 0xBD, 0x20, 0x00}},
        {"CMPS extended", {0x11, 0xBE, 0x20, 0x00}},
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
    EXPECT_EQ(bus.read8(0x9FFE), 0xEF);
    EXPECT_EQ(bus.read8(0x9FFF), 0xBE);

    cpu.regs().e = 0x00;
    cpu.regs().f = 0x00;
    EXPECT_EQ(cpu.tick(bus).cycles, 6u);
    EXPECT_EQ(cpu.regs().u, 0xA000);
    EXPECT_EQ(cpu.regs().e, 0xBE);
    EXPECT_EQ(cpu.regs().f, 0xEF);
}

} // namespace
