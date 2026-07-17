#pragma once

#include <array>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "microlind/app/image_loader.hpp"
#include "microlind/app/sim_session.hpp"
#include "microlind/cpu.hpp"

namespace microlind::gui {

enum class RuntimeMode {
    Paused,
    DebugRun,
    DebugMicroRun,
    RunUntilAddress,
    RunUntilReturn,
    StepPending,
    MicroStepPending,
    StepOverPending,
    TrueRun,
    Stopping,
};

struct RuntimeStatusSnapshot {
    RuntimeMode mode{RuntimeMode::Paused};
    uint16_t pc{};
    uint64_t total_cycles{};
    uint64_t bus_cycles{};
    uint32_t operations_per_minute{600};
    bool run_micro_steps{};
    uint64_t true_target_hz{};
    double true_effective_hz{};
    bool pending_microcycles{};
};

struct RuntimeDebuggerSnapshot {
    Registers regs{};
    uint64_t total_cycles{};
    uint64_t clock_hz{};
    bool serial_mapped{};
    app::SerialSnapshot serial;
    std::vector<uint8_t> serial_tx;
    std::vector<std::string> log;
    std::vector<std::string> memory_map;
    app::MapperSnapshot mapper;
    app::CfSnapshot compact_flash;
    std::vector<app::Breakpoint> breakpoints;
    std::vector<app::Watchpoint> watchpoints;
    std::vector<app::InstructionTraceEntry> trace;
};

struct RuntimeDisassemblyLine {
    uint16_t address{};
    std::string bytes;
    std::string text;
    uint8_t length{1};
    bool current{};
};

struct RuntimeMemoryRow {
    uint16_t address{};
    std::array<uint8_t, 16> bytes{};
    std::array<bool, 16> at_pc{};
    std::array<bool, 16> at_s{};
    std::array<bool, 16> at_u{};
    std::array<bool, 16> watched{};
};

struct RuntimeStackRow {
    uint16_t address{};
    uint8_t high{};
    uint8_t low{};
    uint16_t word{};
    bool at_pointer{};
};

class GuiRuntime {
public:
    explicit GuiRuntime(CpuMode mode = CpuMode::HD6309);
    ~GuiRuntime();

    GuiRuntime(const GuiRuntime&) = delete;
    GuiRuntime& operator=(const GuiRuntime&) = delete;

    [[nodiscard]] RuntimeMode mode() const;
    void set_mode(RuntimeMode mode);
    [[nodiscard]] CpuMode cpu_mode() const;
    void set_cpu_mode(CpuMode mode);
    [[nodiscard]] RuntimeStatusSnapshot status_snapshot() const;
    [[nodiscard]] RuntimeDebuggerSnapshot debugger_snapshot() const;
    [[nodiscard]] std::vector<RuntimeDisassemblyLine> disassembly_snapshot(int line_count);
    [[nodiscard]] std::vector<RuntimeMemoryRow> memory_snapshot(uint16_t start, int rows);
    [[nodiscard]] std::vector<RuntimeStackRow> stack_snapshot(uint16_t start, int rows, int stack_register_index);
    [[nodiscard]] app::LogicDecodeSnapshot logic_snapshot(bool live_bus, uint16_t address, bool read);

    void stop();
    [[nodiscard]] uint32_t operations_per_minute() const;
    void set_operations_per_minute(uint32_t operations_per_minute);
    [[nodiscard]] double operations_per_second() const;
    [[nodiscard]] bool run_micro_steps() const;
    void set_run_micro_steps(bool run_micro_steps);
    void start_true_run(uint64_t target_hz);
    void stop_true_run();
    [[nodiscard]] uint64_t true_target_hz() const;
    void set_true_run_target_hz(uint64_t target_hz);
    [[nodiscard]] bool true_run_active() const;
    [[nodiscard]] bool debug_run_active() const;
    [[nodiscard]] bool run_until_active() const;
    [[nodiscard]] bool execution_active() const;
    void start_debug_run(bool micro_steps);
    void start_run_until_address(uint16_t address);
    void start_run_until_return(uint16_t address);
    void start_step_over(uint16_t address);
    app::RunResult run_debug_batch(uint32_t max_operations);
    [[nodiscard]] std::optional<uint16_t> step_over_target();
    [[nodiscard]] std::optional<uint16_t> return_address_from_stack();
    void add_log(std::string message);
    bool inject_serial_bytes(const std::vector<uint8_t>& bytes);
    void clear_serial_tx();
    void clear_log();
    void clear_trace();
    void write_memory(uint16_t address, uint8_t value);
    [[nodiscard]] uint8_t peek_memory(uint16_t address);
    [[nodiscard]] std::vector<app::Breakpoint> breakpoints() const;
    [[nodiscard]] std::vector<app::Watchpoint> watchpoints() const;
    void set_breakpoints(std::vector<app::Breakpoint> breakpoints);
    void set_watchpoints(std::vector<app::Watchpoint> watchpoints);
    bool add_breakpoint(uint16_t address, std::string label = {});
    bool remove_breakpoint(uint16_t address);
    void clear_breakpoints();
    bool set_breakpoint_enabled(uint16_t address, bool enabled);
    bool set_breakpoint_label(uint16_t address, std::string label);
    bool add_watchpoint(uint16_t address, app::WatchpointType type, std::string label = {});
    bool remove_watchpoint(uint16_t address);
    void clear_watchpoints();
    bool set_watchpoint_enabled(uint16_t address, bool enabled);
    bool set_watchpoint_label(uint16_t address, std::string label);
    bool load_rom(const std::filesystem::path& path, cli::RomFormat format, uint16_t raw_base);
    bool load_hardware_config(const std::filesystem::path& path);
    bool attach_cf_image(const std::filesystem::path& path, uint32_t minimum_sectors);
    bool remove_cf_image();
    bool set_logic_bus_mode(BusDecodeMode mode);
    void reset();

    app::RunResult run_instructions(uint32_t count);
    app::RunResult run_microcycles(uint32_t count);
    app::RealtimeRunResult run_realtime_cycles(uint64_t cycle_budget);
    app::RunResult run_until_address(uint16_t address, uint32_t max_instructions);
    app::RunResult run_until_return(uint32_t max_instructions);
    CpuTickResult step_instruction();
    SimulatorMicrocycleResult step_microcycle();

private:
    struct RuntimeCommand;

    bool enqueue_command_and_wait(std::shared_ptr<RuntimeCommand> command);
    void drain_commands();
    void complete_pending_commands(bool ok);
    void true_run_worker();

    app::SimSession session_;
    mutable std::mutex mutex_;
    std::mutex command_mutex_;
    std::condition_variable command_cv_;
    std::deque<std::shared_ptr<RuntimeCommand>> command_queue_;
    std::thread true_run_thread_;
    std::atomic_bool true_run_stop_{false};
    RuntimeMode mode_{RuntimeMode::Paused};
    uint16_t debug_target_address_{};
    uint32_t operations_per_minute_{600};
    bool run_micro_steps_{false};
    uint64_t true_target_hz_{1000000};
    double true_effective_hz_{0.0};
};

} // namespace microlind::gui
