#include <filesystem>
#include <fstream>
#include <string>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "microlind/app/disassembler.hpp"
#include "microlind/app/session_file.hpp"
#include "microlind/app/sim_session.hpp"
#include "microlind/bus.hpp"
#include "microlind/cpu.hpp"

#include "test_harness.hpp"

namespace {

using ::testing::SizeIs;
using ::testing::Contains;
using microlind::test::disassemble_bytes;
using microlind::test::loaded_session;
using microlind::test::test_output_path;

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
    EXPECT_THAT(session.log(), Contains("PLD validation OK."));

    const auto mapper = session.mapper_snapshot();
    ASSERT_TRUE(mapper.present);
    ASSERT_THAT(mapper.windows, SizeIs(4));
    EXPECT_EQ(mapper.windows[0].start, 0x0000);
    EXPECT_EQ(mapper.windows[0].end, 0x3FFF);
    EXPECT_EQ(mapper.windows[3].start, 0xC000);
    EXPECT_EQ(mapper.windows[3].end, 0xDFFF);

    const auto cf = session.cf_snapshot();
    ASSERT_TRUE(cf.present);
    EXPECT_FALSE(cf.image_loaded);
    EXPECT_EQ(cf.start, 0xF418);
    EXPECT_EQ(cf.end, 0xF41F);
    EXPECT_EQ(cf.sector_count, 0u);
    EXPECT_FALSE(cf.read_only);
    EXPECT_EQ(cf.status, 0xFF);
    EXPECT_EQ(cf.transfer_mode, microlind::app::CfTransferMode::None);

    const auto parallel = session.parallel_snapshot();
    ASSERT_TRUE(parallel.present);
    EXPECT_EQ(parallel.start, 0xF420);
    EXPECT_EQ(parallel.end, 0xF42F);
    EXPECT_EQ(parallel.port_a, 0xFF);
    EXPECT_EQ(parallel.port_b, 0xFF);

    const auto vdc = session.vdc_snapshot();
    ASSERT_TRUE(vdc.present);
    EXPECT_EQ(vdc.start, 0xF440);
    EXPECT_EQ(vdc.end, 0xF441);
    EXPECT_EQ(vdc.status & 0x80, 0x80);
    EXPECT_EQ(vdc.columns, 80);
    EXPECT_EQ(vdc.rows, 25);
    EXPECT_EQ(vdc.chars[0], ' ');
}

TEST(SimSessionTest, CompactFlashReturnsFFWhenNoImageIsLoaded) {
    auto session = loaded_session();

    for (uint16_t address = 0xF418; address <= 0xF41F; ++address) {
        EXPECT_EQ(session.peek_memory(address), 0xFF) << "peek " << std::hex << address;
        EXPECT_EQ(session.read_memory(address), 0xFF) << "read " << std::hex << address;
    }

    session.write_memory(0xF41F, 0xEC);
    EXPECT_EQ(session.peek_memory(0xF41F), 0xFF);
}

TEST(SimSessionTest, CompactFlashImageCanBeAttachedAndRemoved) {
    auto session = loaded_session();
    const auto image_path = test_output_path("cf-remove.img");
    {
        std::ofstream image(image_path, std::ios::binary);
        const std::string sector(512, '\0');
        image.write(sector.data(), static_cast<std::streamsize>(sector.size()));
    }

    ASSERT_TRUE(session.attach_cf_image(image_path, 1));
    auto cf = session.cf_snapshot();
    EXPECT_TRUE(cf.image_loaded);
    EXPECT_EQ(cf.image_path, image_path);
    EXPECT_EQ(cf.sector_count, 1u);
    EXPECT_EQ(session.peek_memory(0xF41F), 0x50);

    ASSERT_TRUE(session.remove_cf_image());
    cf = session.cf_snapshot();
    EXPECT_FALSE(cf.image_loaded);
    EXPECT_TRUE(cf.image_path.empty());
    EXPECT_EQ(cf.sector_count, 0u);
    EXPECT_EQ(session.peek_memory(0xF41F), 0xFF);
}

TEST(SimSessionTest, MapperRegisterWritesSwitchVisibleRamBank) {
    auto session = loaded_session();

    auto mapper = session.mapper_snapshot();
    ASSERT_TRUE(mapper.present);
    ASSERT_EQ(mapper.bank_registers[0], 0xF400);

    session.write_memory(0xF400, 0x00);
    session.write_memory(0x0000, 0x11);
    EXPECT_EQ(session.peek_memory(0x0000), 0x11);

    session.write_memory(0xF400, 0x01);
    mapper = session.mapper_snapshot();
    EXPECT_EQ(mapper.selected_banks[0], 0x01);
    session.write_memory(0x0000, 0x22);
    EXPECT_EQ(session.peek_memory(0x0000), 0x22);

    session.write_memory(0xF400, 0x00);
    mapper = session.mapper_snapshot();
    EXPECT_EQ(mapper.selected_banks[0], 0x00);
    EXPECT_EQ(session.peek_memory(0x0000), 0x11);

    session.write_memory(0xF400, 0x01);
    EXPECT_EQ(session.peek_memory(0x0000), 0x22);
}

TEST(SimSessionTest, ReportsSerialLedSnapshotForGui) {
    auto session = loaded_session();

    auto serial = session.serial_snapshot();
    ASSERT_TRUE(serial.present);
    EXPECT_EQ(serial.output_port, 0x00);
    EXPECT_FALSE(serial.led_red);
    EXPECT_FALSE(serial.led_green);
    EXPECT_FALSE(serial.led_blue);

    session.write_memory(0xF43E, 0x50);
    serial = session.serial_snapshot();
    EXPECT_EQ(serial.output_port, 0x50);
    EXPECT_TRUE(serial.led_red);
    EXPECT_FALSE(serial.led_green);
    EXPECT_TRUE(serial.led_blue);

    session.write_memory(0xF43E, 0x20);
    session.write_memory(0xF43F, 0x10);
    serial = session.serial_snapshot();
    EXPECT_EQ(serial.output_port, 0x60);
    EXPECT_FALSE(serial.led_red);
    EXPECT_TRUE(serial.led_green);
    EXPECT_TRUE(serial.led_blue);
}

TEST(SimSessionTest, ReportsPldLogicDecodeSnapshotForGui) {
    auto session = loaded_session();

    const auto rom = session.logic_decode_snapshot(0xF000, true);
    EXPECT_TRUE(rom.configured);
    EXPECT_EQ(rom.bus_mode, microlind::BusDecodeMode::Route);
    ASSERT_TRUE(rom.available);
    ASSERT_TRUE(rom.decoded.ok());
    EXPECT_TRUE(rom.decoded.rom_en);
    EXPECT_FALSE(rom.decoded.io_en);

    const auto cf_write = session.logic_decode_snapshot(0xF418, false);
    EXPECT_TRUE(cf_write.configured);
    ASSERT_TRUE(cf_write.available);
    ASSERT_TRUE(cf_write.decoded.ok());
    EXPECT_TRUE(cf_write.decoded.io_en);
    EXPECT_TRUE(cf_write.decoded.cf_en);
    EXPECT_TRUE(cf_write.decoded.wr);
}

TEST(SimSessionTest, CanSwitchPldBusModeForGui) {
    auto session = loaded_session();
    EXPECT_EQ(session.logic_bus_mode(), microlind::BusDecodeMode::Route);

    ASSERT_TRUE(session.set_logic_bus_mode(microlind::BusDecodeMode::RangeMap));
    EXPECT_EQ(session.logic_bus_mode(), microlind::BusDecodeMode::RangeMap);
    EXPECT_EQ(session.logic_decode_snapshot(0xF000, true).bus_mode, microlind::BusDecodeMode::RangeMap);
    EXPECT_EQ(session.simulator().bus().decode_mode(), microlind::BusDecodeMode::RangeMap);

    ASSERT_TRUE(session.set_logic_bus_mode(microlind::BusDecodeMode::Validate));
    EXPECT_EQ(session.logic_bus_mode(), microlind::BusDecodeMode::Validate);
    EXPECT_EQ(session.simulator().bus().decode_mode(), microlind::BusDecodeMode::Validate);
}

TEST(SimSessionTest, StepsSingleMicrocycleForGui) {
    auto session = loaded_session();
    auto& sim = session.simulator();
    sim.bus().clear_access_log();

    const auto result = session.step_microcycle();

    EXPECT_TRUE(result.emitted);
    EXPECT_TRUE(result.instruction_started);
    EXPECT_FALSE(result.instruction_complete);
    EXPECT_TRUE(sim.has_pending_microcycles());
    ASSERT_THAT(sim.bus().access_log(), SizeIs(1));
    EXPECT_EQ(sim.bus().access_log().front().address, 0xFF00);
    EXPECT_EQ(sim.bus().access_log().front().cycle_kind, microlind::BusCycleKind::OpcodeFetch);
    EXPECT_EQ(result.signals.cycle_kind, microlind::BusCycleKind::OpcodeFetch);
}

TEST(SimSessionTest, RecordsTraceWhenMicroSteppedInstructionCompletes) {
    auto session = loaded_session();

    auto result = session.step_microcycle();
    ASSERT_TRUE(result.emitted);
    EXPECT_TRUE(result.instruction_started);
    EXPECT_FALSE(result.instruction_complete);
    EXPECT_TRUE(session.trace().empty());

    for (int i = 0; i < 16 && !result.instruction_complete; ++i) {
        result = session.step_microcycle();
    }

    ASSERT_TRUE(result.instruction_complete);
    ASSERT_THAT(session.trace(), SizeIs(1));
    EXPECT_EQ(session.trace().front().pc, 0xFF00);
    EXPECT_EQ(session.trace().front().instruction, "jmp ext $ff03");
    EXPECT_EQ(session.trace().front().cycles, result.instruction_result.cycles);
}

TEST(SimSessionTest, RealtimeRunAdvancesCyclesWithoutDebuggerTrace) {
    auto session = loaded_session();
    auto& sim = session.simulator();
    ASSERT_TRUE(session.add_breakpoint(0xFF00, "ignored by realtime run"));

    const auto result = session.run_realtime_cycles(16);

    EXPECT_GE(result.cycles, 16u);
    EXPECT_GT(result.instructions, 0u);
    EXPECT_TRUE(session.trace().empty());
    EXPECT_TRUE(sim.bus().access_log().empty());
    EXPECT_EQ(session.breakpoints().front().hits, 0u);
    EXPECT_GT(sim.clock().total_cycles(), 0u);
}

TEST(SimSessionTest, MergesReadWriteWatchpointsAndStopsOnHit) {
    auto session = loaded_session();

    ASSERT_TRUE(session.add_watchpoint(0xFF00, microlind::app::WatchpointType::Read, "reset vector jump"));
    ASSERT_TRUE(session.add_watchpoint(0xFF00, microlind::app::WatchpointType::Write));
    ASSERT_THAT(session.watchpoints(), SizeIs(1));
    EXPECT_EQ(session.watchpoints().front().type, microlind::app::WatchpointType::ReadWrite);
    EXPECT_EQ(session.watchpoints().front().label, "reset vector jump");
    EXPECT_TRUE(session.is_watchpoint(0xFF00, microlind::app::WatchpointType::Read));
    EXPECT_TRUE(session.is_watchpoint(0xFF00, microlind::app::WatchpointType::Write));

    const auto result = session.run_instructions(1);
    EXPECT_TRUE(result.hit_watchpoint);
    EXPECT_EQ(result.watchpoint_address, 0xFF00);
    EXPECT_EQ(result.watchpoint_type, microlind::BusAccessType::Read);
    EXPECT_EQ(session.watchpoints().front().hits, 1u);
}

TEST(SimSessionTest, PeekMemoryDoesNotConsumeSerialOrLogBusAccess) {
    auto session = loaded_session();
    ASSERT_TRUE(session.inject_serial_text("A"));

    auto& bus = session.simulator().bus();
    bus.clear_access_log();
    bus.clear_phase_log();

    EXPECT_EQ(session.peek_memory(0xF433), 'A');
    EXPECT_EQ(session.peek_memory(0xF433), 'A');
    EXPECT_TRUE(bus.access_log().empty());
    EXPECT_TRUE(bus.phase_log().empty());

    EXPECT_EQ(session.read_memory(0xF433), 'A');
    EXPECT_EQ(session.read_memory(0xF433), 0x00);
}

TEST(SimSessionTest, BreakpointsCanBeDisabledLabeledAndCountHits) {
    auto session = loaded_session();

    ASSERT_TRUE(session.add_breakpoint(0xFF00, "entry"));
    ASSERT_THAT(session.breakpoints(), SizeIs(1));
    EXPECT_EQ(session.breakpoints().front().label, "entry");
    ASSERT_TRUE(session.set_breakpoint_label(0xFF00, "reset entry"));
    EXPECT_EQ(session.breakpoints().front().label, "reset entry");

    ASSERT_TRUE(session.set_breakpoint_enabled(0xFF00, false));
    auto result = session.run_instructions(1);
    EXPECT_FALSE(result.hit_breakpoint);
    EXPECT_EQ(session.breakpoints().front().hits, 0u);

    session.reset();
    ASSERT_TRUE(session.set_breakpoint_enabled(0xFF00, true));
    result = session.run_instructions(1);
    EXPECT_TRUE(result.hit_breakpoint);
    EXPECT_EQ(result.breakpoint_address, 0xFF00);
    EXPECT_EQ(session.breakpoints().front().hits, 1u);
}

TEST(SimSessionTest, WatchpointsCanBeDisabledLabeledAndCountHits) {
    auto session = loaded_session();

    ASSERT_TRUE(session.add_watchpoint(0xFF00, microlind::app::WatchpointType::Read, "fetch"));
    ASSERT_TRUE(session.set_watchpoint_label(0xFF00, "read entry"));
    ASSERT_TRUE(session.set_watchpoint_enabled(0xFF00, false));
    auto result = session.run_instructions(1);
    EXPECT_FALSE(result.hit_watchpoint);
    EXPECT_EQ(session.watchpoints().front().hits, 0u);

    session.reset();
    ASSERT_TRUE(session.set_watchpoint_enabled(0xFF00, true));
    result = session.run_instructions(1);
    EXPECT_TRUE(result.hit_watchpoint);
    EXPECT_EQ(result.watchpoint_address, 0xFF00);
    EXPECT_EQ(session.watchpoints().front().label, "read entry");
    EXPECT_EQ(session.watchpoints().front().hits, 1u);
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

TEST(DisassemblerTest, DoesNotRecordBusAccessWhileFormatting) {
    auto session = loaded_session();
    auto& sim = session.simulator();

    sim.bus().clear_access_log();
    sim.bus().clear_phase_log();
    const auto disasm = microlind::cli::disassemble(sim.bus(), sim.cpu(), 0xFF00);
    EXPECT_EQ(disasm.text, "jmp ext $ff03");
    EXPECT_TRUE(sim.bus().access_log().empty());
    EXPECT_TRUE(sim.bus().phase_log().empty());
}

TEST(DisassemblerTest, FormatsAdditionalWIndexedModesAndStackMasks) {
    microlind::app::SimSession session;

    auto disasm = disassemble_bytes(session, 0x0400, {0x6E, 0xCF});
    EXPECT_EQ(disasm.text, "jmp ,w++");
    EXPECT_EQ(disasm.length, 2);

    disasm = disassemble_bytes(session, 0x0410, {0x6E, 0xB0, 0x12, 0x34});
    EXPECT_EQ(disasm.text, "jmp [$1234,w]");
    EXPECT_EQ(disasm.length, 4);

    disasm = disassemble_bytes(session, 0x0420, {0x6E, 0xD0});
    EXPECT_EQ(disasm.text, "jmp [,w++]");
    EXPECT_EQ(disasm.length, 2);

    disasm = disassemble_bytes(session, 0x0430, {0x6E, 0xAE});
    EXPECT_EQ(disasm.text, "jmp w,y");
    EXPECT_EQ(disasm.length, 2);

    disasm = disassemble_bytes(session, 0x0440, {0x6E, 0xFE});
    EXPECT_EQ(disasm.text, "jmp [w,s]");
    EXPECT_EQ(disasm.length, 2);

    disasm = disassemble_bytes(session, 0x0450, {0x36, 0xC3});
    EXPECT_EQ(disasm.text, "pshu cc,a,s,pc");
    EXPECT_EQ(disasm.length, 2);

    disasm = disassemble_bytes(session, 0x0460, {0x35, 0x00});
    EXPECT_EQ(disasm.text, "puls -");
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

TEST(SessionFileTest, LoadsPathsLayoutAndPersistedDebuggerState) {
    const auto path = test_output_path("load-session.session");
    {
        std::ofstream file(path);
        file << "[Session]\n";
        file << "CONFIG=configs/hw.cfg\n";
        file << "ROM=roms/test.s19\n";
        file << "ROM_FORMAT=s19\n";
        file << "RAW_BASE=$C000\n";
        file << "CF=media/disk.img\n";
        file << "CF_SECTORS=4096\n";
        file << "CPU=6809\n";
        file << "LAYOUT_HEX=5B57696E646F775D0A\n";
        file << "MEMORY_START=$1230\n";
        file << "MEMORY_ROWS=48\n";
        file << "MEMORY_FOLLOW_PC=yes\n";
        file << "STACK_REGISTER=U\n";
        file << "STACK_START=0xD000\n";
        file << "STACK_ROWS=24\n";
        file << "STACK_FOLLOW=false\n";
        file << "SERIAL_HEX_VIEW=true\n";
        file << "SERIAL_RX_HEX=on\n";
        file << "OPERATIONS_PER_MINUTE=250\n";
        file << "RUN_MICRO_STEPS=true\n";
        file << "TRUE_CLOCK_HZ=2000000\n";
        file << "GUI_THEME=light\n";
        file << "SHOW_FILES=false\n";
        file << "SHOW_CONTROL=true\n";
        file << "SHOW_REGISTERS=false\n";
        file << "SHOW_DISASSEMBLY=true\n";
        file << "SHOW_MEMORY=false\n";
        file << "SHOW_STACK=true\n";
        file << "SHOW_MEMORY_MAP=false\n";
        file << "SHOW_MEMORY_MAPPER=true\n";
        file << "SHOW_PLD_LOGIC=false\n";
        file << "SHOW_COMPACT_FLASH=true\n";
        file << "SHOW_PARALLEL=false\n";
        file << "SHOW_VIDEO=true\n";
        file << "SHOW_BREAKPOINTS=false\n";
        file << "SHOW_WATCHPOINTS=true\n";
        file << "SHOW_TRACE=false\n";
        file << "SHOW_SERIAL=true\n";
        file << "SHOW_LOG=false\n";
        file << "BREAKPOINT=0xFF00;ENABLED=false;HITS=7;LABEL_HEX=456E747279\n";
        file << "WATCHPOINT=0xF430;TYPE=RW;ENABLED=1;HITS=3;LABEL_HEX=53657269616C\n";
    }

    std::string error;
    const auto loaded = microlind::app::load_session_definition(path, error);
    ASSERT_TRUE(loaded) << error;

    EXPECT_EQ(loaded->config_path, path.parent_path() / "configs/hw.cfg");
    EXPECT_EQ(loaded->rom_path, path.parent_path() / "roms/test.s19");
    EXPECT_EQ(loaded->cf_path, path.parent_path() / "media/disk.img");
    EXPECT_EQ(loaded->rom_format, microlind::cli::RomFormat::Srec);
    EXPECT_EQ(loaded->raw_base, 0xC000);
    EXPECT_EQ(loaded->cf_sectors, 4096u);
    ASSERT_TRUE(loaded->mode);
    EXPECT_EQ(*loaded->mode, microlind::CpuMode::MC6809);
    EXPECT_EQ(loaded->layout_ini, "[Window]\n");
    EXPECT_EQ(loaded->gui.memory_start, 0x1230);
    EXPECT_EQ(loaded->gui.memory_rows, 48);
    EXPECT_TRUE(loaded->gui.memory_follow_pc);
    EXPECT_EQ(loaded->gui.stack_register_index, 1);
    EXPECT_EQ(loaded->gui.stack_start, 0xD000);
    EXPECT_EQ(loaded->gui.stack_rows, 24);
    EXPECT_FALSE(loaded->gui.stack_follow_pointer);
    EXPECT_TRUE(loaded->gui.serial_hex_view);
    EXPECT_TRUE(loaded->gui.serial_rx_hex);
    EXPECT_EQ(loaded->gui.operations_per_minute, 250);
    EXPECT_TRUE(loaded->gui.run_micro_steps);
    EXPECT_EQ(loaded->gui.true_clock_hz, 2000000u);
    EXPECT_EQ(loaded->gui.theme, microlind::app::GuiTheme::Light);
    EXPECT_FALSE(loaded->gui.show_file_panel);
    EXPECT_TRUE(loaded->gui.show_control_panel);
    EXPECT_FALSE(loaded->gui.show_registers);
    EXPECT_TRUE(loaded->gui.show_disassembly);
    EXPECT_FALSE(loaded->gui.show_memory_viewer);
    EXPECT_TRUE(loaded->gui.show_stack);
    EXPECT_FALSE(loaded->gui.show_memory_map);
    EXPECT_TRUE(loaded->gui.show_mapper);
    EXPECT_FALSE(loaded->gui.show_pld_logic);
    EXPECT_TRUE(loaded->gui.show_compact_flash);
    EXPECT_FALSE(loaded->gui.show_parallel);
    EXPECT_TRUE(loaded->gui.show_video);
    EXPECT_FALSE(loaded->gui.show_breakpoints);
    EXPECT_TRUE(loaded->gui.show_watchpoints);
    EXPECT_FALSE(loaded->gui.show_trace);
    EXPECT_TRUE(loaded->gui.show_serial);
    EXPECT_FALSE(loaded->gui.show_log);

    ASSERT_THAT(loaded->breakpoints, SizeIs(1));
    EXPECT_EQ(loaded->breakpoints.front().address, 0xFF00);
    EXPECT_FALSE(loaded->breakpoints.front().enabled);
    EXPECT_EQ(loaded->breakpoints.front().hits, 7u);
    EXPECT_EQ(loaded->breakpoints.front().label, "Entry");

    ASSERT_THAT(loaded->watchpoints, SizeIs(1));
    EXPECT_EQ(loaded->watchpoints.front().address, 0xF430);
    EXPECT_EQ(loaded->watchpoints.front().type, microlind::app::WatchpointType::ReadWrite);
    EXPECT_TRUE(loaded->watchpoints.front().enabled);
    EXPECT_EQ(loaded->watchpoints.front().hits, 3u);
    EXPECT_EQ(loaded->watchpoints.front().label, "Serial");
}

TEST(SessionFileTest, LoadsLegacyStepsPerFrameAsOperationsPerMinute) {
    const auto path = test_output_path("legacy-steps.session");
    {
        std::ofstream file(path);
        file << "[Session]\n";
        file << "CONFIG=configs/hw.cfg\n";
        file << "ROM=roms/test.ihex\n";
        file << "STEPS_PER_FRAME=250\n";
    }

    std::string error;
    const auto loaded = microlind::app::load_session_definition(path, error);
    ASSERT_TRUE(loaded) << error;
    EXPECT_EQ(loaded->gui.operations_per_minute, 250);
}

TEST(SessionFileTest, SavesAndReloadsPersistedDebuggerState) {
    const auto dir = test_output_path("roundtrip-dir");
    std::filesystem::create_directories(dir);
    const auto path = dir / "roundtrip.session";

    microlind::app::SessionDefinition session;
    session.config_path = dir / "hw.cfg";
    session.rom_path = dir / "bios.ihex";
    session.cf_path = dir / "sim.img";
    session.rom_format = microlind::cli::RomFormat::Ihex;
    session.raw_base = 0x8000;
    session.cf_sectors = 512;
    session.mode = microlind::CpuMode::HD6309;
    session.layout_ini = "[Window][Memory]\n";
    session.gui.memory_start = 0x4000;
    session.gui.memory_rows = 32;
    session.gui.memory_follow_pc = false;
    session.gui.stack_register_index = 0;
    session.gui.stack_start = 0xDFFE;
    session.gui.stack_rows = 40;
    session.gui.stack_follow_pointer = true;
    session.gui.serial_hex_view = true;
    session.gui.serial_rx_hex = false;
    session.gui.operations_per_minute = 123;
    session.gui.run_micro_steps = true;
    session.gui.true_clock_hz = 3000000;
    session.gui.theme = microlind::app::GuiTheme::Light;
    session.gui.show_file_panel = false;
    session.gui.show_control_panel = true;
    session.gui.show_registers = false;
    session.gui.show_disassembly = true;
    session.gui.show_memory_viewer = false;
    session.gui.show_stack = true;
    session.gui.show_memory_map = false;
    session.gui.show_mapper = true;
    session.gui.show_pld_logic = false;
    session.gui.show_compact_flash = true;
    session.gui.show_parallel = false;
    session.gui.show_video = true;
    session.gui.show_breakpoints = false;
    session.gui.show_watchpoints = true;
    session.gui.show_trace = false;
    session.gui.show_serial = true;
    session.gui.show_log = false;
    session.breakpoints.push_back(microlind::app::Breakpoint{0xFF00, false, "Main entry", 11});
    session.watchpoints.push_back(
        microlind::app::Watchpoint{0xF433, microlind::app::WatchpointType::Write, true, "Serial TX", 4});

    std::string error;
    ASSERT_TRUE(microlind::app::save_session_definition(path, session, error)) << error;

    const auto loaded = microlind::app::load_session_definition(path, error);
    ASSERT_TRUE(loaded) << error;
    EXPECT_EQ(loaded->config_path, session.config_path);
    EXPECT_EQ(loaded->rom_path, session.rom_path);
    EXPECT_EQ(loaded->cf_path, session.cf_path);
    EXPECT_EQ(loaded->layout_ini, session.layout_ini);
    EXPECT_EQ(loaded->gui.memory_start, session.gui.memory_start);
    EXPECT_EQ(loaded->gui.stack_start, session.gui.stack_start);
    EXPECT_EQ(loaded->gui.operations_per_minute, session.gui.operations_per_minute);
    EXPECT_EQ(loaded->gui.run_micro_steps, session.gui.run_micro_steps);
    EXPECT_EQ(loaded->gui.true_clock_hz, session.gui.true_clock_hz);
    EXPECT_EQ(loaded->gui.theme, session.gui.theme);
    EXPECT_EQ(loaded->gui.show_file_panel, session.gui.show_file_panel);
    EXPECT_EQ(loaded->gui.show_control_panel, session.gui.show_control_panel);
    EXPECT_EQ(loaded->gui.show_registers, session.gui.show_registers);
    EXPECT_EQ(loaded->gui.show_disassembly, session.gui.show_disassembly);
    EXPECT_EQ(loaded->gui.show_memory_viewer, session.gui.show_memory_viewer);
    EXPECT_EQ(loaded->gui.show_stack, session.gui.show_stack);
    EXPECT_EQ(loaded->gui.show_memory_map, session.gui.show_memory_map);
    EXPECT_EQ(loaded->gui.show_mapper, session.gui.show_mapper);
    EXPECT_EQ(loaded->gui.show_pld_logic, session.gui.show_pld_logic);
    EXPECT_EQ(loaded->gui.show_compact_flash, session.gui.show_compact_flash);
    EXPECT_EQ(loaded->gui.show_parallel, session.gui.show_parallel);
    EXPECT_EQ(loaded->gui.show_video, session.gui.show_video);
    EXPECT_EQ(loaded->gui.show_breakpoints, session.gui.show_breakpoints);
    EXPECT_EQ(loaded->gui.show_watchpoints, session.gui.show_watchpoints);
    EXPECT_EQ(loaded->gui.show_trace, session.gui.show_trace);
    EXPECT_EQ(loaded->gui.show_serial, session.gui.show_serial);
    EXPECT_EQ(loaded->gui.show_log, session.gui.show_log);

    ASSERT_THAT(loaded->breakpoints, SizeIs(1));
    EXPECT_EQ(loaded->breakpoints.front().address, 0xFF00);
    EXPECT_FALSE(loaded->breakpoints.front().enabled);
    EXPECT_EQ(loaded->breakpoints.front().label, "Main entry");
    EXPECT_EQ(loaded->breakpoints.front().hits, 11u);

    ASSERT_THAT(loaded->watchpoints, SizeIs(1));
    EXPECT_EQ(loaded->watchpoints.front().address, 0xF433);
    EXPECT_EQ(loaded->watchpoints.front().type, microlind::app::WatchpointType::Write);
    EXPECT_TRUE(loaded->watchpoints.front().enabled);
    EXPECT_EQ(loaded->watchpoints.front().label, "Serial TX");
    EXPECT_EQ(loaded->watchpoints.front().hits, 4u);
}

} // namespace
