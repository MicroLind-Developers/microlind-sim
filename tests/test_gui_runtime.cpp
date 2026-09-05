#include "gui_runtime.hpp"

#include <algorithm>
#include <chrono>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

namespace {

using microlind::gui::GuiRuntime;
using microlind::gui::LogicCaptureState;
using microlind::gui::LogicSignal;
using microlind::gui::LogicTriggerMode;
using microlind::gui::RuntimeMode;

TEST(GuiRuntimeTest, TrueRunWorkerAdvancesCyclesAndStopsCleanly) {
    GuiRuntime runtime(microlind::CpuMode::HD6309);
    runtime.write_memory(0x0000, 0x12); // NOP
    runtime.write_memory(0x0001, 0x20); // BRA -3
    runtime.write_memory(0x0002, 0xFD);

    const auto before = runtime.status_snapshot();
    runtime.start_true_run(1000000);
    std::this_thread::sleep_for(std::chrono::milliseconds(25));

    const auto running = runtime.status_snapshot();
    EXPECT_EQ(RuntimeMode::TrueRun, running.mode);

    runtime.stop_true_run();
    const auto stopped = runtime.status_snapshot();
    EXPECT_EQ(RuntimeMode::Paused, stopped.mode);
    EXPECT_GT(stopped.total_cycles, before.total_cycles);
    EXPECT_GT(stopped.bus_cycles, before.bus_cycles);
}

TEST(GuiRuntimeTest, TrueRunCommandsExecuteInOrder) {
    GuiRuntime runtime(microlind::CpuMode::HD6309);
    runtime.write_memory(0x0000, 0x12); // NOP
    runtime.write_memory(0x0001, 0x20); // BRA -3
    runtime.write_memory(0x0002, 0xFD);

    runtime.start_true_run(1000000);
    runtime.add_log("queued one");
    runtime.set_true_run_target_hz(2000000);
    runtime.add_log("queued two");

    const auto running = runtime.status_snapshot();
    EXPECT_EQ(2000000u, running.true_target_hz);

    runtime.stop_true_run();
    const auto snapshot = runtime.debugger_snapshot();
    const auto one = std::find(snapshot.log.begin(), snapshot.log.end(), "queued one");
    const auto two = std::find(snapshot.log.begin(), snapshot.log.end(), "queued two");
    ASSERT_NE(snapshot.log.end(), one);
    ASSERT_NE(snapshot.log.end(), two);
    EXPECT_LT(std::distance(snapshot.log.begin(), one), std::distance(snapshot.log.begin(), two));
}

TEST(GuiRuntimeTest, SerialInjectionDuringTrueRunReachesDevice) {
    GuiRuntime runtime(microlind::CpuMode::HD6309);
    ASSERT_TRUE(runtime.load_hardware_config("tests/data/hw_test.cfg"));
    runtime.write_memory(0x0000, 0x12); // NOP
    runtime.write_memory(0x0001, 0x20); // BRA -3
    runtime.write_memory(0x0002, 0xFD);

    runtime.start_true_run(1000000);
    EXPECT_TRUE(runtime.inject_serial_bytes(std::vector<uint8_t>{'A'}));
    runtime.stop_true_run();

    EXPECT_EQ(runtime.peek_memory(0xF433), 'A');
}

TEST(GuiRuntimeTest, VdcSnapshotUpdatesDuringTrueRun) {
    GuiRuntime runtime(microlind::CpuMode::HD6309);
    ASSERT_TRUE(runtime.load_hardware_config("tests/data/hw_test.cfg"));

    const std::vector<uint8_t> program{
        0x86, 0x12,             // LDA #$12 (update address high register)
        0xB7, 0xF4, 0x40,       // STA $F440
        0x86, 0x20,             // LDA #$20
        0xB7, 0xF4, 0x41,       // STA $F441
        0x86, 0x13,             // LDA #$13 (update address low register)
        0xB7, 0xF4, 0x40,       // STA $F440
        0x86, 0x00,             // LDA #$00
        0xB7, 0xF4, 0x41,       // STA $F441
        0x86, 0x1F,             // LDA #$1F (VRAM data register)
        0xB7, 0xF4, 0x40,       // STA $F440
        0x86, 'A',
        0xB7, 0xF4, 0x41,       // STA $F441
        0x20, 0xFE,             // BRA *
    };
    for (std::size_t index = 0; index < program.size(); ++index) {
        runtime.write_memory(static_cast<uint16_t>(index), program[index]);
    }

    runtime.start_true_run(1000000);
    bool updated = false;
    for (int attempt = 0; attempt < 100 && !updated; ++attempt) {
        updated = runtime.vdc_snapshot().chars[0] == 'A';
        if (!updated) std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    EXPECT_TRUE(runtime.true_run_active());
    EXPECT_TRUE(updated);
    runtime.stop_true_run();
}

TEST(GuiRuntimeTest, DebugBatchRunsThroughRuntimeMode) {
    GuiRuntime runtime(microlind::CpuMode::HD6309);
    runtime.write_memory(0x0000, 0x12); // NOP
    runtime.write_memory(0x0001, 0x20); // BRA -3
    runtime.write_memory(0x0002, 0xFD);

    const auto before = runtime.status_snapshot();
    runtime.start_debug_run(false);
    const auto result = runtime.run_debug_batch(1);

    EXPECT_TRUE(runtime.debug_run_active());
    EXPECT_EQ(RuntimeMode::DebugRun, runtime.status_snapshot().mode);
    EXPECT_EQ(1u, result.executed);
    EXPECT_GT(runtime.status_snapshot().total_cycles, before.total_cycles);
}

TEST(GuiRuntimeTest, RunUntilUsesRuntimeTargetAndPausesOnHit) {
    GuiRuntime runtime(microlind::CpuMode::HD6309);
    runtime.write_memory(0x0000, 0x12); // NOP
    runtime.write_memory(0x0001, 0x12); // NOP

    runtime.start_run_until_address(0x0001);
    const auto result = runtime.run_debug_batch(10);

    EXPECT_TRUE(result.hit_target);
    EXPECT_FALSE(runtime.debug_run_active());
    EXPECT_EQ(RuntimeMode::Paused, runtime.status_snapshot().mode);
    EXPECT_EQ(0x0001, runtime.status_snapshot().pc);
}

TEST(GuiRuntimeTest, OwnsRunPreferences) {
    GuiRuntime runtime(microlind::CpuMode::HD6309);

    runtime.set_operations_per_minute(1200);
    runtime.set_run_micro_steps(true);
    runtime.set_true_run_target_hz(3000000);

    EXPECT_EQ(1200u, runtime.operations_per_minute());
    EXPECT_DOUBLE_EQ(20.0, runtime.operations_per_second());
    EXPECT_TRUE(runtime.run_micro_steps());
    EXPECT_EQ(3000000u, runtime.true_target_hz());

    const auto status = runtime.status_snapshot();
    EXPECT_EQ(1200u, status.operations_per_minute);
    EXPECT_TRUE(status.run_micro_steps);
    EXPECT_EQ(3000000u, status.true_target_hz);

    runtime.start_debug_run(false);
    EXPECT_FALSE(runtime.run_micro_steps());
    EXPECT_EQ(RuntimeMode::DebugRun, runtime.status_snapshot().mode);
}

TEST(GuiRuntimeTest, LogicAnalyserCapturesAndClearsInstructionSamples) {
    GuiRuntime runtime(microlind::CpuMode::HD6309);
    runtime.write_memory(0x0000, 0x12); // NOP
    runtime.write_memory(0x0001, 0x12); // NOP

    runtime.start_logic_analyser(false);
    runtime.run_instructions(2);

    const auto capture = runtime.logic_analyser_snapshot();
    EXPECT_EQ(LogicCaptureState::Capturing, capture.state);
    ASSERT_EQ(2u, capture.samples.size());
    EXPECT_LT(capture.samples[0].cycle, capture.samples[1].cycle);

    runtime.stop_logic_analyser();
    EXPECT_EQ(LogicCaptureState::Stopped, runtime.logic_analyser_snapshot().state);
    runtime.clear_logic_analyser();
    EXPECT_TRUE(runtime.logic_analyser_snapshot().samples.empty());
}

TEST(GuiRuntimeTest, LogicAnalyserWaitsForAndCapturesTriggerEdge) {
    GuiRuntime runtime(microlind::CpuMode::HD6309);
    ASSERT_TRUE(runtime.load_hardware_config("tests/data/hw_test.cfg"));
    runtime.write_memory(0x0000, 0x12); // NOP
    runtime.write_memory(0x0001, 0x12); // NOP

    runtime.start_logic_analyser(false, LogicSignal::ViaTimer1Running, LogicTriggerMode::Rising);
    runtime.run_instructions(1); // Establish the low trigger level.
    EXPECT_EQ(LogicCaptureState::WaitingForTrigger, runtime.logic_analyser_snapshot().state);

    runtime.write_memory(0xF424, 0x00);
    runtime.write_memory(0xF425, 0x10); // Start Timer 1.
    runtime.run_instructions(1);

    const auto capture = runtime.logic_analyser_snapshot();
    EXPECT_EQ(LogicCaptureState::Capturing, capture.state);
    ASSERT_FALSE(capture.samples.empty());
    EXPECT_TRUE(capture.samples.front().values[static_cast<std::size_t>(LogicSignal::ViaTimer1Running)]);
}

} // namespace
