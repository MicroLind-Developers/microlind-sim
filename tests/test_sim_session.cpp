#include <cstdint>
#include <initializer_list>
#include <stdexcept>
#include <string>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "microlind/app/disassembler.hpp"
#include "microlind/app/sim_session.hpp"

namespace {

using ::testing::SizeIs;

microlind::app::SimSession loaded_session() {
    microlind::app::SimSession session;
    if (!session.load_hardware_config("tests/data/hw_test.cfg")) {
        throw std::runtime_error("failed to load tests/data/hw_test.cfg");
    }
    if (!session.load_rom("examples/bios.ihex", microlind::cli::RomFormat::Ihex, 0x8000)) {
        throw std::runtime_error("failed to load examples/bios.ihex");
    }
    session.reset();
    return session;
}

microlind::cli::Disasm disassemble_bytes(microlind::app::SimSession& session,
                                          uint16_t address,
                                          std::initializer_list<uint8_t> bytes) {
    uint16_t offset = 0;
    for (uint8_t byte : bytes) {
        session.write_memory(static_cast<uint16_t>(address + offset), byte);
        ++offset;
    }

    auto& sim = session.simulator();
    return microlind::cli::disassemble(sim.bus(), sim.cpu(), address);
}

TEST(SimSessionTest, LoadsBiosResetsStepsAndRecordsTrace) {
    auto session = loaded_session();
    EXPECT_EQ(session.simulator().cpu().regs().pc, 0xFF00);

    const auto result = session.run_instructions(1);
    EXPECT_EQ(result.executed, 1u);
    EXPECT_EQ(session.simulator().cpu().regs().pc, 0xFF03);
    ASSERT_THAT(session.trace(), SizeIs(1));
    EXPECT_EQ(session.trace().front().instruction, "jmp ext $ff03");

    auto& sim = session.simulator();
    const auto disasm = microlind::cli::disassemble(sim.bus(), sim.cpu(), sim.cpu().regs().pc);
    EXPECT_EQ(disasm.text, "orcc #$50");
    EXPECT_EQ(disasm.length, 2);
}

TEST(SimSessionTest, ReportsExplicitMapperWindowsAndCompactFlashSnapshot) {
    auto session = loaded_session();

    const auto mapper = session.mapper_snapshot();
    ASSERT_TRUE(mapper.present);
    ASSERT_THAT(mapper.windows, SizeIs(4));
    EXPECT_EQ(mapper.windows[0].start, 0x0000);
    EXPECT_EQ(mapper.windows[0].end, 0x3FFF);
    EXPECT_EQ(mapper.windows[3].start, 0xC000);
    EXPECT_EQ(mapper.windows[3].end, 0xDFFF);

    const auto cf = session.cf_snapshot();
    ASSERT_TRUE(cf.present);
    EXPECT_EQ(cf.start, 0xF418);
    EXPECT_EQ(cf.end, 0xF41F);
    EXPECT_EQ(cf.sector_count, 512u);
    EXPECT_FALSE(cf.read_only);
    EXPECT_EQ(cf.status, 0x50);
    EXPECT_EQ(cf.transfer_mode, microlind::app::CfTransferMode::None);
}

TEST(SimSessionTest, MergesReadWriteWatchpointsAndStopsOnHit) {
    auto session = loaded_session();

    ASSERT_TRUE(session.add_watchpoint(0xFF00, microlind::app::WatchpointType::Read));
    ASSERT_TRUE(session.add_watchpoint(0xFF00, microlind::app::WatchpointType::Write));
    ASSERT_THAT(session.watchpoints(), SizeIs(1));
    EXPECT_EQ(session.watchpoints().front().type, microlind::app::WatchpointType::ReadWrite);
    EXPECT_TRUE(session.is_watchpoint(0xFF00, microlind::app::WatchpointType::Read));
    EXPECT_TRUE(session.is_watchpoint(0xFF00, microlind::app::WatchpointType::Write));

    const auto result = session.run_instructions(1);
    EXPECT_TRUE(result.hit_watchpoint);
    EXPECT_EQ(result.watchpoint_address, 0xFF00);
    EXPECT_EQ(result.watchpoint_type, microlind::BusAccessType::Read);
}

TEST(DisassemblerTest, FormatsIndexedModesAndStackMasks) {
    microlind::app::SimSession session;

    auto disasm = disassemble_bytes(session, 0x0100, {0x6E, 0x84});
    EXPECT_EQ(disasm.text, "jmp ,x");
    EXPECT_EQ(disasm.length, 2);

    disasm = disassemble_bytes(session, 0x0110, {0x6E, 0x89, 0x12, 0x34});
    EXPECT_EQ(disasm.text, "jmp $1234,x");
    EXPECT_EQ(disasm.length, 4);

    disasm = disassemble_bytes(session, 0x0120, {0x34, 0x36});
    EXPECT_EQ(disasm.text, "pshs a,b,x,y");
    EXPECT_EQ(disasm.length, 2);

    disasm = disassemble_bytes(session, 0x0130, {0x6E, 0x8F});
    EXPECT_EQ(disasm.text, "jmp ,w");
    EXPECT_EQ(disasm.length, 2);

    disasm = disassemble_bytes(session, 0x0140, {0x6E, 0xAF, 0x12, 0x34});
    EXPECT_EQ(disasm.text, "jmp $1234,w");
    EXPECT_EQ(disasm.length, 4);

    disasm = disassemble_bytes(session, 0x0150, {0x6E, 0x90});
    EXPECT_EQ(disasm.text, "jmp [,w]");
    EXPECT_EQ(disasm.length, 2);

    disasm = disassemble_bytes(session, 0x0160, {0x6E, 0x9E});
    EXPECT_EQ(disasm.text, "jmp [w,x]");
    EXPECT_EQ(disasm.length, 2);
}

TEST(DisassemblerTest, FormatsRegisterPairAndTfmInstructions) {
    microlind::app::SimSession session;

    auto disasm = disassemble_bytes(session, 0x0200, {0x1F, 0x12});
    EXPECT_EQ(disasm.text, "tfr x,y");
    EXPECT_EQ(disasm.length, 2);

    disasm = disassemble_bytes(session, 0x0210, {0x1E, 0x98});
    EXPECT_EQ(disasm.text, "exg b,a");
    EXPECT_EQ(disasm.length, 2);

    disasm = disassemble_bytes(session, 0x0220, {0x10, 0x30, 0x01});
    EXPECT_EQ(disasm.text, "addr d,x");
    EXPECT_EQ(disasm.length, 3);

    disasm = disassemble_bytes(session, 0x0230, {0x11, 0x38, 0x12});
    EXPECT_EQ(disasm.text, "tfm x+,y+");
    EXPECT_EQ(disasm.length, 3);

    disasm = disassemble_bytes(session, 0x0240, {0x11, 0x39, 0x34});
    EXPECT_EQ(disasm.text, "tfm u-,s-");
    EXPECT_EQ(disasm.length, 3);

    disasm = disassemble_bytes(session, 0x0250, {0x11, 0x3A, 0x12});
    EXPECT_EQ(disasm.text, "tfm x+,y");
    EXPECT_EQ(disasm.length, 3);

    disasm = disassemble_bytes(session, 0x0260, {0x11, 0x3B, 0x12});
    EXPECT_EQ(disasm.text, "tfm x,y+");
    EXPECT_EQ(disasm.length, 3);
}

TEST(DisassemblerTest, FormatsBitImmediateOperands) {
    microlind::app::SimSession session;

    auto disasm = disassemble_bytes(session, 0x0300, {0x01, 0x12, 0x34});
    EXPECT_EQ(disasm.text, "oim #$12,<$34");
    EXPECT_EQ(disasm.length, 3);

    disasm = disassemble_bytes(session, 0x0310, {0x61, 0x12, 0x84});
    EXPECT_EQ(disasm.text, "oim #$12,x");
    EXPECT_EQ(disasm.length, 3);

    disasm = disassemble_bytes(session, 0x0320, {0x71, 0x12, 0x34, 0x56});
    EXPECT_EQ(disasm.text, "oim #$12,$3456");
    EXPECT_EQ(disasm.length, 4);

    disasm = disassemble_bytes(session, 0x0330, {0x62, 0xF0, 0x89, 0x00, 0x10});
    EXPECT_EQ(disasm.text, "aim #$f0,$0010,x");
    EXPECT_EQ(disasm.length, 5);
}

} // namespace
