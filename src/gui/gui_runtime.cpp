#include "gui_runtime.hpp"

#include "gui_thread_names.hpp"

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <deque>
#include <iomanip>
#include <memory>
#include <sstream>
#include <thread>
#include <utility>

#include "microlind/app/disassembler.hpp"

namespace microlind::gui {
namespace {

constexpr double kMaxTrueRunCatchupSeconds = 0.02;
constexpr double kMaxTrueRunWorkSeconds = 0.004;
constexpr uint64_t kMaxTrueRunBatchCycles = 4096;
constexpr std::size_t kLogicAnalyserCapacity = 8192;

constexpr std::size_t logic_signal_index(LogicSignal signal) {
    return static_cast<std::size_t>(signal);
}

bool is_debug_run_mode(RuntimeMode mode) {
    switch (mode) {
    case RuntimeMode::DebugRun:
    case RuntimeMode::DebugMicroRun:
    case RuntimeMode::RunUntilAddress:
    case RuntimeMode::RunUntilReturn:
    case RuntimeMode::StepOverPending:
        return true;
    default:
        return false;
    }
}

bool is_run_until_mode(RuntimeMode mode) {
    switch (mode) {
    case RuntimeMode::RunUntilAddress:
    case RuntimeMode::RunUntilReturn:
    case RuntimeMode::StepOverPending:
        return true;
    default:
        return false;
    }
}

std::string instruction_bytes(Bus& bus, uint16_t pc, uint8_t length) {
    std::ostringstream out;
    out << std::uppercase << std::hex << std::setfill('0');
    for (uint8_t i = 0; i < length; ++i) {
        if (i > 0) out << ' ';
        out << std::setw(2) << static_cast<int>(bus.peek8(static_cast<uint16_t>(pc + i)));
    }
    return out.str();
}

} // namespace

const char* logic_signal_name(LogicSignal signal) {
    switch (signal) {
    case LogicSignal::ClockE: return "Clock E";
    case LogicSignal::ClockQ: return "Clock Q";
    case LogicSignal::ReadWrite: return "R/W";
    case LogicSignal::BusAvailable: return "BA";
    case LogicSignal::BusStatus: return "BS";
    case LogicSignal::CpuIrq: return "CPU IRQ";
    case LogicSignal::CpuFirq: return "CPU FIRQ";
    case LogicSignal::ViaIrq: return "VIA IRQ";
    case LogicSignal::ViaTimer1Running: return "VIA T1 running";
    case LogicSignal::ViaPb7: return "VIA PB7";
    case LogicSignal::ViaTimer1Flag: return "VIA IFR6 (T1)";
    case LogicSignal::ViaTimer2Flag: return "VIA IFR5 (T2)";
    case LogicSignal::ParallelSelected: return "VIA selected";
    case LogicSignal::BusRead: return "Bus read";
    case LogicSignal::BusWrite: return "Bus write";
    case LogicSignal::Count: break;
    }
    return "Unknown";
}

GuiRuntime::GuiRuntime(CpuMode mode) : session_(mode) {}

struct GuiRuntime::RuntimeCommand {
    enum class Kind {
        SetTrueClockHz,
        AddLog,
        InjectSerialBytes,
        ClearSerialTx,
        ClearLog,
    };

    explicit RuntimeCommand(Kind kind) : kind(kind) {}

    Kind kind;
    uint64_t target_hz{};
    std::string message;
    std::vector<uint8_t> bytes;
    bool ok{true};
    bool completed{false};
    std::mutex completion_mutex;
    std::condition_variable completion_cv;
};

GuiRuntime::~GuiRuntime() {
    stop_true_run();
}

RuntimeMode GuiRuntime::mode() const {
    std::lock_guard lock(mutex_);
    return mode_;
}

void GuiRuntime::set_mode(RuntimeMode mode) {
    std::lock_guard lock(mutex_);
    mode_ = mode;
}

CpuMode GuiRuntime::cpu_mode() const {
    std::lock_guard lock(mutex_);
    return session_.mode();
}

void GuiRuntime::set_cpu_mode(CpuMode mode) {
    stop_true_run();
    std::lock_guard lock(mutex_);
    session_.set_mode(mode);
}

RuntimeStatusSnapshot GuiRuntime::status_snapshot() const {
    std::lock_guard lock(mutex_);
    const auto& sim = session_.simulator();
    return RuntimeStatusSnapshot{
        mode_,
        sim.cpu().regs().pc,
        sim.clock().total_cycles(),
        sim.bus().bus_cycle_count(),
        operations_per_minute_,
        run_micro_steps_,
        true_target_hz_,
        true_effective_hz_,
        sim.has_pending_microcycles(),
    };
}

RuntimeDebuggerSnapshot GuiRuntime::debugger_snapshot() const {
    std::lock_guard lock(mutex_);
    const auto& sim = session_.simulator();
    return RuntimeDebuggerSnapshot{
        sim.cpu().regs(),
        sim.clock().total_cycles(),
        sim.clock().frequency_hz(),
        session_.serial_mapped(),
        session_.serial_snapshot(),
        session_.serial_tx(),
        session_.log(),
        session_.memory_map(),
        session_.mapper_snapshot(),
        session_.cf_snapshot(),
        session_.parallel_snapshot(),
        session_.vdc_snapshot(),
        session_.breakpoints(),
        session_.watchpoints(),
        session_.trace(),
    };
}

app::ParallelSnapshot GuiRuntime::parallel_snapshot() const {
    std::lock_guard lock(mutex_);
    return session_.parallel_snapshot();
}

app::VdcSnapshot GuiRuntime::vdc_snapshot() const {
    std::lock_guard lock(mutex_);
    return session_.vdc_snapshot();
}

std::vector<RuntimeDisassemblyLine> GuiRuntime::disassembly_snapshot(int line_count) {
    std::lock_guard lock(mutex_);
    std::vector<RuntimeDisassemblyLine> lines;
    lines.reserve(static_cast<std::size_t>(std::max(line_count, 0)));

    auto& sim = session_.simulator();
    uint16_t pc = sim.cpu().regs().pc;
    const uint16_t current_pc = pc;
    for (int i = 0; i < line_count; ++i) {
        const auto disasm = cli::disassemble(sim.bus(), sim.cpu(), pc);
        const uint8_t length = std::max<uint8_t>(disasm.length, 1);
        lines.push_back(RuntimeDisassemblyLine{
            pc,
            instruction_bytes(sim.bus(), pc, length),
            disasm.text,
            length,
            pc == current_pc,
        });
        pc = static_cast<uint16_t>(pc + length);
    }
    return lines;
}

std::vector<RuntimeMemoryRow> GuiRuntime::memory_snapshot(uint16_t start, int rows) {
    std::lock_guard lock(mutex_);
    std::vector<RuntimeMemoryRow> out;
    out.reserve(static_cast<std::size_t>(std::max(rows, 0)));

    const auto& regs = session_.simulator().cpu().regs();
    const auto& watchpoints = session_.watchpoints();
    for (int row = 0; row < rows; ++row) {
        RuntimeMemoryRow snapshot;
        snapshot.address = static_cast<uint16_t>(start + row * 16);
        for (std::size_t col = 0; col < snapshot.bytes.size(); ++col) {
            const uint16_t address = static_cast<uint16_t>(snapshot.address + col);
            snapshot.bytes[col] = session_.peek_memory(address);
            snapshot.at_pc[col] = address == regs.pc;
            snapshot.at_s[col] = address == regs.s;
            snapshot.at_u[col] = address == regs.u;
            snapshot.watched[col] = std::any_of(
                watchpoints.begin(),
                watchpoints.end(),
                [address](const auto& watchpoint) { return watchpoint.address == address; });
        }
        out.push_back(snapshot);
    }
    return out;
}

std::vector<RuntimeStackRow> GuiRuntime::stack_snapshot(uint16_t start, int rows, int stack_register_index) {
    std::lock_guard lock(mutex_);
    std::vector<RuntimeStackRow> out;
    out.reserve(static_cast<std::size_t>(std::max(rows, 0)));

    const auto& regs = session_.simulator().cpu().regs();
    const uint16_t stack_pointer = stack_register_index == 0 ? regs.s : regs.u;
    for (int row = 0; row < rows; ++row) {
        const uint16_t address = static_cast<uint16_t>(start + row * 2);
        const uint8_t high = session_.peek_memory(address);
        const uint8_t low = session_.peek_memory(static_cast<uint16_t>(address + 1));
        out.push_back(RuntimeStackRow{
            address,
            high,
            low,
            static_cast<uint16_t>((high << 8) | low),
            address == stack_pointer,
        });
    }
    return out;
}

app::LogicDecodeSnapshot GuiRuntime::logic_snapshot(bool live_bus, uint16_t address, bool read) {
    std::lock_guard lock(mutex_);
    if (live_bus) {
        return session_.logic_decode_snapshot(session_.simulator().bus().last_signals());
    }
    return session_.logic_decode_snapshot(address, read);
}

LogicAnalyserSnapshot GuiRuntime::logic_analyser_snapshot() const {
    std::lock_guard lock(mutex_);
    return logic_analyser_;
}

void GuiRuntime::start_logic_analyser(
    bool microcycle_resolution,
    std::optional<LogicSignal> trigger,
    LogicTriggerMode trigger_mode) {
    std::lock_guard lock(mutex_);
    logic_analyser_.samples.clear();
    logic_analyser_.microcycle_resolution = microcycle_resolution;
    logic_analyser_.trigger = trigger;
    logic_analyser_.trigger_mode = trigger_mode;
    logic_analyser_.state = trigger ? LogicCaptureState::WaitingForTrigger : LogicCaptureState::Capturing;
    logic_trigger_previous_.reset();
}

void GuiRuntime::stop_logic_analyser() {
    std::lock_guard lock(mutex_);
    logic_analyser_.state = LogicCaptureState::Stopped;
    logic_trigger_previous_.reset();
}

void GuiRuntime::clear_logic_analyser() {
    std::lock_guard lock(mutex_);
    logic_analyser_.samples.clear();
    logic_trigger_previous_.reset();
}

void GuiRuntime::capture_logic_analyser_sample() {
    if (logic_analyser_.state == LogicCaptureState::Stopped) return;

    const auto& sim = session_.simulator();
    const auto& bus = sim.bus();
    const auto& signals = bus.last_signals();
    const auto parallel = session_.parallel_snapshot();

    LogicAnalyserSample sample;
    sample.cycle = sim.clock().total_cycles();
    sample.pc = sim.cpu().regs().pc;
    auto& values = sample.values;
    values[logic_signal_index(LogicSignal::ClockE)] = signals.e;
    values[logic_signal_index(LogicSignal::ClockQ)] = signals.q;
    values[logic_signal_index(LogicSignal::ReadWrite)] = signals.rw;
    values[logic_signal_index(LogicSignal::BusAvailable)] = signals.ba;
    values[logic_signal_index(LogicSignal::BusStatus)] = signals.bs;
    values[logic_signal_index(LogicSignal::CpuIrq)] = sim.cpu().irq_line_asserted();
    values[logic_signal_index(LogicSignal::CpuFirq)] = sim.cpu().firq_line_asserted();
    values[logic_signal_index(LogicSignal::ViaIrq)] = parallel.irq_asserted;
    values[logic_signal_index(LogicSignal::ViaTimer1Running)] = parallel.timer1_running;
    values[logic_signal_index(LogicSignal::ViaPb7)] = parallel.pb7_pin_level;
    values[logic_signal_index(LogicSignal::ViaTimer1Flag)] = (parallel.ifr & 0x40) != 0;
    values[logic_signal_index(LogicSignal::ViaTimer2Flag)] = (parallel.ifr & 0x20) != 0;
    values[logic_signal_index(LogicSignal::ParallelSelected)] = signals.mapped_select == BusDeviceSelect::Parallel;
    values[logic_signal_index(LogicSignal::BusRead)] = signals.apply_read;
    values[logic_signal_index(LogicSignal::BusWrite)] = signals.apply_write;

    if (logic_analyser_.state == LogicCaptureState::WaitingForTrigger) {
        const bool current = values[logic_signal_index(*logic_analyser_.trigger)];
        bool triggered = false;
        if (logic_trigger_previous_) {
            switch (logic_analyser_.trigger_mode) {
            case LogicTriggerMode::Rising: triggered = !*logic_trigger_previous_ && current; break;
            case LogicTriggerMode::Falling: triggered = *logic_trigger_previous_ && !current; break;
            case LogicTriggerMode::Either: triggered = *logic_trigger_previous_ != current; break;
            }
        }
        logic_trigger_previous_ = current;
        if (!triggered) return;
        logic_analyser_.state = LogicCaptureState::Capturing;
    }

    if (logic_analyser_.samples.size() == kLogicAnalyserCapacity) {
        logic_analyser_.samples.erase(
            logic_analyser_.samples.begin(),
            logic_analyser_.samples.begin() + static_cast<std::ptrdiff_t>(kLogicAnalyserCapacity / 4));
    }
    logic_analyser_.samples.push_back(sample);
}

void GuiRuntime::stop() {
    stop_true_run();
    std::lock_guard lock(mutex_);
    mode_ = RuntimeMode::Paused;
}

uint32_t GuiRuntime::operations_per_minute() const {
    std::lock_guard lock(mutex_);
    return operations_per_minute_;
}

void GuiRuntime::set_operations_per_minute(uint32_t operations_per_minute) {
    std::lock_guard lock(mutex_);
    operations_per_minute_ = operations_per_minute;
}

double GuiRuntime::operations_per_second() const {
    std::lock_guard lock(mutex_);
    return static_cast<double>(operations_per_minute_) / 60.0;
}

bool GuiRuntime::run_micro_steps() const {
    std::lock_guard lock(mutex_);
    return run_micro_steps_;
}

void GuiRuntime::set_run_micro_steps(bool run_micro_steps) {
    std::lock_guard lock(mutex_);
    run_micro_steps_ = run_micro_steps;
}

void GuiRuntime::start_true_run(uint64_t target_hz) {
    stop_true_run();

    {
        std::lock_guard lock(mutex_);
        true_target_hz_ = target_hz;
        true_effective_hz_ = 0.0;
        mode_ = RuntimeMode::TrueRun;
        session_.add_log("True running at " + std::to_string(target_hz / 1000000) + " MHz.");
    }

    true_run_stop_.store(false, std::memory_order_release);
    true_run_thread_ = std::thread(&GuiRuntime::true_run_worker, this);
    set_thread_name(true_run_thread_, "microlind-run");
}

void GuiRuntime::stop_true_run() {
    if (true_run_thread_.joinable()) {
        {
            std::lock_guard lock(mutex_);
            if (mode_ == RuntimeMode::TrueRun) {
                mode_ = RuntimeMode::Stopping;
            }
        }
        true_run_stop_.store(true, std::memory_order_release);
        command_cv_.notify_all();
        true_run_thread_.join();
    }
    complete_pending_commands(false);

    true_run_stop_.store(false, std::memory_order_release);
    std::lock_guard lock(mutex_);
    if (mode_ == RuntimeMode::TrueRun || mode_ == RuntimeMode::Stopping) {
        mode_ = RuntimeMode::Paused;
    }
    true_effective_hz_ = 0.0;
}

uint64_t GuiRuntime::true_target_hz() const {
    std::lock_guard lock(mutex_);
    return true_target_hz_;
}

void GuiRuntime::set_true_run_target_hz(uint64_t target_hz) {
    if (true_run_active()) {
        auto command = std::make_shared<RuntimeCommand>(RuntimeCommand::Kind::SetTrueClockHz);
        command->target_hz = target_hz;
        enqueue_command_and_wait(command);
        return;
    }

    std::lock_guard lock(mutex_);
    true_target_hz_ = target_hz;
}

bool GuiRuntime::true_run_active() const {
    std::lock_guard lock(mutex_);
    return mode_ == RuntimeMode::TrueRun;
}

bool GuiRuntime::debug_run_active() const {
    std::lock_guard lock(mutex_);
    return is_debug_run_mode(mode_);
}

bool GuiRuntime::run_until_active() const {
    std::lock_guard lock(mutex_);
    return is_run_until_mode(mode_);
}

bool GuiRuntime::execution_active() const {
    std::lock_guard lock(mutex_);
    return mode_ == RuntimeMode::TrueRun || is_debug_run_mode(mode_);
}

void GuiRuntime::start_debug_run(bool micro_steps) {
    stop_true_run();
    std::lock_guard lock(mutex_);
    run_micro_steps_ = micro_steps;
    mode_ = micro_steps ? RuntimeMode::DebugMicroRun : RuntimeMode::DebugRun;
}

void GuiRuntime::start_run_until_address(uint16_t address) {
    stop_true_run();
    std::lock_guard lock(mutex_);
    debug_target_address_ = address;
    mode_ = RuntimeMode::RunUntilAddress;
}

void GuiRuntime::start_run_until_return(uint16_t address) {
    stop_true_run();
    std::lock_guard lock(mutex_);
    debug_target_address_ = address;
    mode_ = RuntimeMode::RunUntilReturn;
}

void GuiRuntime::start_step_over(uint16_t address) {
    stop_true_run();
    std::lock_guard lock(mutex_);
    debug_target_address_ = address;
    mode_ = RuntimeMode::StepOverPending;
}

app::RunResult GuiRuntime::run_debug_batch(uint32_t max_operations) {
    stop_true_run();
    std::lock_guard lock(mutex_);

    app::RunResult result;
    switch (mode_) {
    case RuntimeMode::DebugRun:
        if (logic_analyser_.state != LogicCaptureState::Stopped && logic_analyser_.microcycle_resolution) {
            result = session_.run_microcycles(max_operations, [this] { capture_logic_analyser_sample(); });
        } else {
            result = session_.run_instructions(max_operations, [this] { capture_logic_analyser_sample(); });
        }
        if (result.hit_breakpoint || result.hit_watchpoint || max_operations == 0) {
            mode_ = RuntimeMode::Paused;
        }
        break;
    case RuntimeMode::DebugMicroRun:
        result = session_.run_microcycles(max_operations, [this] { capture_logic_analyser_sample(); });
        if (result.hit_breakpoint || result.hit_watchpoint || max_operations == 0) {
            mode_ = RuntimeMode::Paused;
        }
        break;
    case RuntimeMode::RunUntilAddress:
    case RuntimeMode::RunUntilReturn:
    case RuntimeMode::StepOverPending:
        result = session_.run_until_address(
            debug_target_address_, max_operations, [this] { capture_logic_analyser_sample(); });
        if (result.hit_target || result.hit_breakpoint || result.hit_watchpoint || max_operations == 0) {
            mode_ = RuntimeMode::Paused;
        }
        break;
    default:
        break;
    }
    return result;
}

std::optional<uint16_t> GuiRuntime::step_over_target() {
    std::lock_guard lock(mutex_);
    return session_.step_over_target();
}

std::optional<uint16_t> GuiRuntime::return_address_from_stack() {
    std::lock_guard lock(mutex_);
    return session_.return_address_from_stack();
}

void GuiRuntime::add_log(std::string message) {
    if (true_run_active()) {
        auto command = std::make_shared<RuntimeCommand>(RuntimeCommand::Kind::AddLog);
        command->message = std::move(message);
        enqueue_command_and_wait(command);
        return;
    }

    std::lock_guard lock(mutex_);
    session_.add_log(std::move(message));
}

bool GuiRuntime::inject_serial_bytes(const std::vector<uint8_t>& bytes) {
    if (true_run_active()) {
        auto command = std::make_shared<RuntimeCommand>(RuntimeCommand::Kind::InjectSerialBytes);
        command->bytes = bytes;
        return enqueue_command_and_wait(command);
    }

    std::lock_guard lock(mutex_);
    return session_.inject_serial_bytes(bytes);
}

void GuiRuntime::clear_serial_tx() {
    if (true_run_active()) {
        auto command = std::make_shared<RuntimeCommand>(RuntimeCommand::Kind::ClearSerialTx);
        enqueue_command_and_wait(command);
        return;
    }

    std::lock_guard lock(mutex_);
    session_.clear_serial_tx();
}

void GuiRuntime::clear_log() {
    if (true_run_active()) {
        auto command = std::make_shared<RuntimeCommand>(RuntimeCommand::Kind::ClearLog);
        enqueue_command_and_wait(command);
        return;
    }

    std::lock_guard lock(mutex_);
    session_.clear_log();
}

void GuiRuntime::clear_trace() {
    stop_true_run();
    std::lock_guard lock(mutex_);
    session_.clear_trace();
}

void GuiRuntime::write_memory(uint16_t address, uint8_t value) {
    stop_true_run();
    std::lock_guard lock(mutex_);
    session_.write_memory(address, value);
}

uint8_t GuiRuntime::peek_memory(uint16_t address) {
    std::lock_guard lock(mutex_);
    return session_.peek_memory(address);
}

std::vector<app::Breakpoint> GuiRuntime::breakpoints() const {
    std::lock_guard lock(mutex_);
    return session_.breakpoints();
}

std::vector<app::Watchpoint> GuiRuntime::watchpoints() const {
    std::lock_guard lock(mutex_);
    return session_.watchpoints();
}

void GuiRuntime::set_breakpoints(std::vector<app::Breakpoint> breakpoints) {
    stop_true_run();
    std::lock_guard lock(mutex_);
    session_.set_breakpoints(std::move(breakpoints));
}

void GuiRuntime::set_watchpoints(std::vector<app::Watchpoint> watchpoints) {
    stop_true_run();
    std::lock_guard lock(mutex_);
    session_.set_watchpoints(std::move(watchpoints));
}

bool GuiRuntime::add_breakpoint(uint16_t address, std::string label) {
    stop_true_run();
    std::lock_guard lock(mutex_);
    return session_.add_breakpoint(address, std::move(label));
}

bool GuiRuntime::remove_breakpoint(uint16_t address) {
    stop_true_run();
    std::lock_guard lock(mutex_);
    return session_.remove_breakpoint(address);
}

void GuiRuntime::clear_breakpoints() {
    stop_true_run();
    std::lock_guard lock(mutex_);
    session_.clear_breakpoints();
}

bool GuiRuntime::set_breakpoint_enabled(uint16_t address, bool enabled) {
    stop_true_run();
    std::lock_guard lock(mutex_);
    return session_.set_breakpoint_enabled(address, enabled);
}

bool GuiRuntime::set_breakpoint_label(uint16_t address, std::string label) {
    stop_true_run();
    std::lock_guard lock(mutex_);
    return session_.set_breakpoint_label(address, std::move(label));
}

bool GuiRuntime::add_watchpoint(uint16_t address, app::WatchpointType type, std::string label) {
    stop_true_run();
    std::lock_guard lock(mutex_);
    return session_.add_watchpoint(address, type, std::move(label));
}

bool GuiRuntime::remove_watchpoint(uint16_t address) {
    stop_true_run();
    std::lock_guard lock(mutex_);
    return session_.remove_watchpoint(address);
}

void GuiRuntime::clear_watchpoints() {
    stop_true_run();
    std::lock_guard lock(mutex_);
    session_.clear_watchpoints();
}

bool GuiRuntime::set_watchpoint_enabled(uint16_t address, bool enabled) {
    stop_true_run();
    std::lock_guard lock(mutex_);
    return session_.set_watchpoint_enabled(address, enabled);
}

bool GuiRuntime::set_watchpoint_label(uint16_t address, std::string label) {
    stop_true_run();
    std::lock_guard lock(mutex_);
    return session_.set_watchpoint_label(address, std::move(label));
}

bool GuiRuntime::load_rom(const std::filesystem::path& path, cli::RomFormat format, uint16_t raw_base) {
    stop_true_run();
    std::lock_guard lock(mutex_);
    return session_.load_rom(path, format, raw_base);
}

bool GuiRuntime::load_hardware_config(const std::filesystem::path& path) {
    stop_true_run();
    std::lock_guard lock(mutex_);
    return session_.load_hardware_config(path);
}

bool GuiRuntime::attach_cf_image(const std::filesystem::path& path, uint32_t minimum_sectors) {
    stop_true_run();
    std::lock_guard lock(mutex_);
    return session_.attach_cf_image(path, minimum_sectors);
}

bool GuiRuntime::remove_cf_image() {
    stop_true_run();
    std::lock_guard lock(mutex_);
    return session_.remove_cf_image();
}

bool GuiRuntime::set_logic_bus_mode(BusDecodeMode mode) {
    stop_true_run();
    std::lock_guard lock(mutex_);
    mode_ = RuntimeMode::Paused;
    return session_.set_logic_bus_mode(mode);
}

void GuiRuntime::reset() {
    stop_true_run();
    std::lock_guard lock(mutex_);
    session_.reset();
    mode_ = RuntimeMode::Paused;
}

app::RunResult GuiRuntime::run_instructions(uint32_t count) {
    stop_true_run();
    std::lock_guard lock(mutex_);
    mode_ = RuntimeMode::DebugRun;
    auto result = session_.run_instructions(count, [this] { capture_logic_analyser_sample(); });
    if (result.hit_breakpoint || result.hit_watchpoint || count == 0) {
        mode_ = RuntimeMode::Paused;
    }
    return result;
}

app::RunResult GuiRuntime::run_microcycles(uint32_t count) {
    stop_true_run();
    std::lock_guard lock(mutex_);
    mode_ = RuntimeMode::DebugMicroRun;
    auto result = session_.run_microcycles(count, [this] { capture_logic_analyser_sample(); });
    if (result.hit_breakpoint || result.hit_watchpoint || count == 0) {
        mode_ = RuntimeMode::Paused;
    }
    return result;
}

app::RealtimeRunResult GuiRuntime::run_realtime_cycles(uint64_t cycle_budget) {
    std::lock_guard lock(mutex_);
    mode_ = RuntimeMode::TrueRun;
    return session_.run_realtime_cycles(cycle_budget, [this] { capture_logic_analyser_sample(); });
}

app::RunResult GuiRuntime::run_until_address(uint16_t address, uint32_t max_instructions) {
    stop_true_run();
    std::lock_guard lock(mutex_);
    mode_ = RuntimeMode::RunUntilAddress;
    auto result = session_.run_until_address(address, max_instructions, [this] { capture_logic_analyser_sample(); });
    if (result.hit_target || result.hit_breakpoint || result.hit_watchpoint || max_instructions == 0) {
        mode_ = RuntimeMode::Paused;
    }
    return result;
}

app::RunResult GuiRuntime::run_until_return(uint32_t max_instructions) {
    stop_true_run();
    std::lock_guard lock(mutex_);
    mode_ = RuntimeMode::RunUntilReturn;
    auto result = session_.run_until_return(max_instructions, [this] { capture_logic_analyser_sample(); });
    if (result.hit_target || result.hit_breakpoint || result.hit_watchpoint || max_instructions == 0) {
        mode_ = RuntimeMode::Paused;
    }
    return result;
}

CpuTickResult GuiRuntime::step_instruction() {
    stop_true_run();
    std::lock_guard lock(mutex_);
    mode_ = RuntimeMode::StepPending;
    auto result = session_.step_instruction();
    capture_logic_analyser_sample();
    mode_ = RuntimeMode::Paused;
    return result;
}

SimulatorMicrocycleResult GuiRuntime::step_microcycle() {
    stop_true_run();
    std::lock_guard lock(mutex_);
    mode_ = RuntimeMode::MicroStepPending;
    auto result = session_.step_microcycle();
    capture_logic_analyser_sample();
    mode_ = RuntimeMode::Paused;
    return result;
}

bool GuiRuntime::enqueue_command_and_wait(std::shared_ptr<RuntimeCommand> command) {
    {
        std::lock_guard lock(command_mutex_);
        command_queue_.push_back(command);
    }
    command_cv_.notify_one();

    std::unique_lock lock(command->completion_mutex);
    command->completion_cv.wait(lock, [&command] { return command->completed; });
    return command->ok;
}

void GuiRuntime::drain_commands() {
    std::deque<std::shared_ptr<RuntimeCommand>> commands;
    {
        std::lock_guard lock(command_mutex_);
        commands.swap(command_queue_);
    }

    if (commands.empty()) return;

    {
        std::lock_guard lock(mutex_);
        for (const auto& command : commands) {
            switch (command->kind) {
            case RuntimeCommand::Kind::SetTrueClockHz:
                true_target_hz_ = command->target_hz;
                break;
            case RuntimeCommand::Kind::AddLog:
                session_.add_log(std::move(command->message));
                break;
            case RuntimeCommand::Kind::InjectSerialBytes:
                command->ok = session_.inject_serial_bytes(command->bytes);
                break;
            case RuntimeCommand::Kind::ClearSerialTx:
                session_.clear_serial_tx();
                break;
            case RuntimeCommand::Kind::ClearLog:
                session_.clear_log();
                break;
            }
        }
    }

    for (const auto& command : commands) {
        {
            std::lock_guard lock(command->completion_mutex);
            command->completed = true;
        }
        command->completion_cv.notify_all();
    }
}

void GuiRuntime::complete_pending_commands(bool ok) {
    std::deque<std::shared_ptr<RuntimeCommand>> commands;
    {
        std::lock_guard lock(command_mutex_);
        commands.swap(command_queue_);
    }

    for (const auto& command : commands) {
        {
            std::lock_guard lock(command->completion_mutex);
            command->ok = ok;
            command->completed = true;
        }
        command->completion_cv.notify_all();
    }
}

void GuiRuntime::true_run_worker() {
    set_current_thread_name("microlind-run");

    using clock = std::chrono::steady_clock;

    auto last = clock::now();
    double cycle_budget = 0.0;
    double stats_elapsed = 0.0;
    uint64_t stats_cycles = 0;

    while (!true_run_stop_.load(std::memory_order_acquire)) {
        drain_commands();

        const auto now = clock::now();
        const double elapsed = std::chrono::duration<double>(now - last).count();
        last = now;

        uint64_t target_hz = 0;
        {
            std::lock_guard lock(mutex_);
            target_hz = true_target_hz_;
        }

        if (target_hz == 0) {
            std::unique_lock lock(command_mutex_);
            command_cv_.wait_for(
                lock,
                std::chrono::milliseconds(1),
                [this] { return true_run_stop_.load(std::memory_order_acquire) || !command_queue_.empty(); });
            continue;
        }

        cycle_budget += elapsed * static_cast<double>(target_hz);
        cycle_budget = std::min(cycle_budget, static_cast<double>(target_hz) * kMaxTrueRunCatchupSeconds);

        bool did_work = false;
        const auto work_start = clock::now();
        while (cycle_budget >= 1.0 && !true_run_stop_.load(std::memory_order_acquire)) {
            const auto cycles_to_run = static_cast<uint64_t>(
                std::min<double>(cycle_budget, static_cast<double>(kMaxTrueRunBatchCycles)));

            app::RealtimeRunResult result;
            {
                std::lock_guard lock(mutex_);
                result = session_.run_realtime_cycles(cycles_to_run, [this] { capture_logic_analyser_sample(); });
            }
            drain_commands();

            if (result.cycles == 0) break;

            did_work = true;
            stats_cycles += result.cycles;
            cycle_budget = static_cast<double>(result.cycles) >= cycle_budget
                ? 0.0
                : cycle_budget - static_cast<double>(result.cycles);

            if (std::chrono::duration<double>(clock::now() - work_start).count() >= kMaxTrueRunWorkSeconds) {
                break;
            }
        }

        stats_elapsed += elapsed;
        if (stats_elapsed >= 0.25) {
            std::lock_guard lock(mutex_);
            true_effective_hz_ = static_cast<double>(stats_cycles) / stats_elapsed;
            stats_cycles = 0;
            stats_elapsed = 0.0;
        }

        if (!did_work || cycle_budget < 1.0) {
            std::unique_lock lock(command_mutex_);
            command_cv_.wait_for(
                lock,
                std::chrono::milliseconds(1),
                [this] { return true_run_stop_.load(std::memory_order_acquire) || !command_queue_.empty(); });
        } else {
            std::unique_lock lock(command_mutex_);
            command_cv_.wait_for(
                lock,
                std::chrono::microseconds(500),
                [this] { return true_run_stop_.load(std::memory_order_acquire) || !command_queue_.empty(); });
        }
    }

    drain_commands();

    std::lock_guard lock(mutex_);
    if (mode_ == RuntimeMode::TrueRun || mode_ == RuntimeMode::Stopping) {
        mode_ = RuntimeMode::Paused;
    }
    true_effective_hz_ = 0.0;
}

} // namespace microlind::gui
