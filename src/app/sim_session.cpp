#include "microlind/app/sim_session.hpp"

#include <algorithm>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <utility>

#include "microlind/app/disassembler.hpp"
#include "microlind/app/logic_validation.hpp"
#include "microlind/app/sim_builder.hpp"

#include "microlind/devices/compact_flash.hpp"
#include "microlind/devices/memory_mapper.hpp"
#include "microlind/devices/parallel.hpp"
#include "microlind/devices/serial.hpp"
#include "microlind/devices/vdc.hpp"

namespace microlind::app {

namespace {

const char* watchpoint_log_label(WatchpointType type) {
    switch (type) {
    case WatchpointType::Read: return "read";
    case WatchpointType::Write: return "write";
    case WatchpointType::ReadWrite: return "read/write";
    }
    return "unknown";
}

bool watchpoint_matches(WatchpointType configured, WatchpointType requested) {
    if (requested == WatchpointType::ReadWrite) {
        return configured == WatchpointType::ReadWrite;
    }
    return configured == WatchpointType::ReadWrite || configured == requested;
}

WatchpointType merge_watchpoint_types(WatchpointType a, WatchpointType b) {
    return a == b ? a : WatchpointType::ReadWrite;
}

class RealtimeBusScope {
public:
    explicit RealtimeBusScope(Bus& bus)
        : bus_(bus),
          detailed_bus_phases_(bus.detailed_bus_phases()),
          access_logging_(bus.access_logging()) {
        bus_.set_detailed_bus_phases(false);
        bus_.set_access_logging(false);
    }

    ~RealtimeBusScope() {
        bus_.set_detailed_bus_phases(detailed_bus_phases_);
        bus_.set_access_logging(access_logging_);
    }

    RealtimeBusScope(const RealtimeBusScope&) = delete;
    RealtimeBusScope& operator=(const RealtimeBusScope&) = delete;

private:
    Bus& bus_;
    bool detailed_bus_phases_{};
    bool access_logging_{};
};

uint8_t mapper_bits_for_address(
    const cli::HardwareConfig& cfg,
    const devices::MapperState* mapper_state,
    uint16_t address) {
    if (!cfg.mapper.present || mapper_state == nullptr) return 0;

    for (std::size_t i = 0; i < cfg.mapper.windows.size(); ++i) {
        const auto& window = cfg.mapper.windows[i];
        if (window.present && window.start <= address && address <= window.end) {
            return static_cast<uint8_t>(mapper_state->bank[i] & 0x07);
        }
    }

    if (!cfg.ram.present || cfg.ram.bank_size == 0 || address < cfg.ram.start || address > cfg.ram.end) {
        return 0;
    }

    const uint32_t offset = static_cast<uint32_t>(address - cfg.ram.start);
    const std::size_t window = static_cast<std::size_t>(offset / cfg.ram.bank_size);
    if (window >= 4) return 0;
    return static_cast<uint8_t>(mapper_state->bank[window] & 0x07);
}

} // namespace

SimSession::SimSession(CpuMode mode)
    : mode_(mode),
      sim_(cli::build_sim(
          mode_,
          nullptr,
          nullptr,
          &serial_dev_,
          [this](uint8_t value) { on_serial_tx(value); },
          &mapper_state_,
          &cf_dev_,
          &parallel_dev_,
          &vdc_dev_)) {
    add_log("Session ready.");
}

void SimSession::set_mode(CpuMode mode) {
    if (mode_ == mode) return;
    mode_ = mode;
    rebuild("Changed CPU mode.");
}

bool SimSession::load_rom(const std::filesystem::path& path, cli::RomFormat format, uint16_t raw_base) {
    if (path.empty()) {
        add_log("ROM path is empty.");
        return false;
    }

    auto loaded = cli::load_image(path, format, raw_base);
    if (!loaded || loaded->data.empty()) {
        add_log("Failed to load ROM: " + path.string());
        return false;
    }

    image_ = std::move(loaded);
    rebuild("Loaded ROM: " + path.string());
    return true;
}

bool SimSession::load_hardware_config(const std::filesystem::path& path) {
    if (path.empty()) {
        add_log("Hardware config path is empty.");
        return false;
    }

    std::string error;
    auto loaded = cli::load_hardware_config(path, error);
    if (!loaded) {
        add_log("Config error: " + error);
        return false;
    }

    hw_cfg_ = std::move(loaded);
    rebuild("Loaded hardware config: " + path.string());
    return true;
}

bool SimSession::attach_cf_image(const std::filesystem::path& path, uint32_t minimum_sectors) {
    if (!hw_cfg_ || !hw_cfg_->cf.present) {
        add_log("No CF device is configured.");
        return false;
    }
    if (path.empty()) {
        add_log("CF image path is empty.");
        return false;
    }

    std::ifstream image_file(path, std::ios::binary);
    if (!image_file) {
        add_log("Cannot open CF image: " + path.string());
        return false;
    }

    hw_cfg_->cf.image_path = path;
    hw_cfg_->cf.sectors = minimum_sectors;
    rebuild("Attached CF image: " + path.string());
    return true;
}

bool SimSession::remove_cf_image() {
    if (!hw_cfg_ || !hw_cfg_->cf.present) {
        add_log("No CF device is configured.");
        return false;
    }

    hw_cfg_->cf.image_path.clear();
    hw_cfg_->cf.sectors = 0;
    if (cf_dev_) {
        cf_dev_->unload_disk_image();
    }
    add_log("Removed CF image.");
    return true;
}

BusDecodeMode SimSession::logic_bus_mode() const {
    if (!hw_cfg_ || !hw_cfg_->logic.present) {
        return BusDecodeMode::RangeMap;
    }
    return hw_cfg_->logic.bus_mode;
}

bool SimSession::set_logic_bus_mode(BusDecodeMode mode) {
    if (!hw_cfg_ || !hw_cfg_->logic.present) {
        add_log("No PLD logic is configured.");
        return false;
    }
    if (hw_cfg_->logic.bus_mode == mode) {
        return true;
    }
    hw_cfg_->logic.bus_mode = mode;
    rebuild("Changed PLD bus mode.");
    return true;
}

void SimSession::reset() {
    sim_.reset_from_vector();
    sim_.reset_clock();
    clear_trace();
    add_log("Reset from vector.");
}

CpuTickResult SimSession::step_instruction() {
    const uint16_t pc = sim_.cpu().regs().pc;
    const auto disasm = cli::disassemble(sim_.bus(), sim_.cpu(), pc);
    sim_.bus().clear_access_log();
    sim_.bus().clear_decode_log();
    CpuTickResult result = sim_.tick();
    record_trace(pc, disasm.text, result);
    for (const auto& diagnostic : sim_.bus().decode_log()) {
        add_log(diagnostic);
    }
    return result;
}

SimulatorMicrocycleResult SimSession::step_microcycle() {
    const uint16_t pc = sim_.cpu().regs().pc;
    const auto disasm = cli::disassemble(sim_.bus(), sim_.cpu(), pc);
    sim_.bus().clear_access_log();
    sim_.bus().clear_decode_log();
    SimulatorMicrocycleResult result = sim_.tick_microcycle();
    if (result.instruction_started) {
        pending_micro_trace_pc_ = pc;
        pending_micro_trace_instruction_ = disasm.text;
    }
    if (result.instruction_complete && pending_micro_trace_pc_) {
        record_trace(*pending_micro_trace_pc_, pending_micro_trace_instruction_, result.instruction_result);
        pending_micro_trace_pc_.reset();
        pending_micro_trace_instruction_.clear();
    }
    for (const auto& diagnostic : sim_.bus().decode_log()) {
        add_log(diagnostic);
    }
    return result;
}

RunResult SimSession::run_instructions(uint32_t count, StepObserver after_step) {
    RunResult result;
    for (uint32_t i = 0; i < count; ++i) {
        if (check_breakpoint(result.executed, result)) {
            return result;
        }
        step_instruction();
        ++result.executed;
        if (after_step) after_step();
        if (check_watchpoints(result)) {
            return result;
        }
        if (check_breakpoint(result.executed, result)) {
            return result;
        }
    }
    return result;
}

RunResult SimSession::run_microcycles(uint32_t count, StepObserver after_step) {
    RunResult result;
    for (uint32_t i = 0; i < count; ++i) {
        if (!sim_.has_pending_microcycles() && check_breakpoint(result.executed, result)) {
            return result;
        }

        const auto step = step_microcycle();
        ++result.executed;
        if (after_step) after_step();
        if (check_watchpoints(result)) {
            return result;
        }
        if (step.instruction_complete && check_breakpoint(result.executed, result)) {
            return result;
        }
    }
    return result;
}

RealtimeRunResult SimSession::run_realtime_cycles(uint64_t cycle_budget, StepObserver after_step) {
    RealtimeRunResult result;
    if (cycle_budget == 0) return result;

    RealtimeBusScope realtime_bus(sim_.bus());
    sim_.bus().clear_access_log();
    sim_.bus().clear_decode_log();
    while (result.cycles < cycle_budget) {
        const CpuTickResult tick = sim_.tick();
        ++result.instructions;
        if (after_step) after_step();
        if (tick.cycles == 0) break;
        result.cycles += tick.cycles;
    }
    sim_.bus().clear_access_log();
    sim_.bus().clear_decode_log();
    return result;
}

RunResult SimSession::run_until_address(uint16_t address, uint32_t max_instructions, StepObserver after_step) {
    RunResult result;
    for (uint32_t i = 0; i < max_instructions; ++i) {
        if (check_target(address, result.executed, result)) {
            return result;
        }
        if (check_breakpoint(result.executed, result)) {
            return result;
        }
        step_instruction();
        ++result.executed;
        if (after_step) after_step();
        if (check_watchpoints(result)) {
            return result;
        }
        if (check_target(address, result.executed, result)) {
            return result;
        }
        if (check_breakpoint(result.executed, result)) {
            return result;
        }
    }
    return result;
}

RunResult SimSession::run_until_return(uint32_t max_instructions, StepObserver after_step) {
    const auto target = return_address_from_stack();
    if (!target) {
        add_log("No return address is available on S.");
        return {};
    }
    return run_until_address(*target, max_instructions, std::move(after_step));
}

void SimSession::tick_cycles(uint64_t cycles) {
    sim_.tick_clock(cycles);
}

std::optional<uint16_t> SimSession::step_over_target() {
    const uint16_t pc = sim_.cpu().regs().pc;
    const uint8_t op0 = sim_.bus().peek8(pc);
    switch (op0) {
    case 0x8D: // BSR
    case 0x9D: // JSR direct
    case 0xAD: // JSR indexed
    case 0xBD: // JSR extended
    case 0x17: { // LBSR
        const auto disasm = cli::disassemble(sim_.bus(), sim_.cpu(), pc);
        return static_cast<uint16_t>(pc + std::max<uint8_t>(disasm.length, 1));
    }
    default:
        return std::nullopt;
    }
}

std::optional<uint16_t> SimSession::return_address_from_stack() {
    const uint16_t s = sim_.cpu().regs().s;
    const uint16_t high = sim_.bus().peek8(s);
    const uint16_t low = sim_.bus().peek8(static_cast<uint16_t>(s + 1));
    return static_cast<uint16_t>((high << 8) | low);
}

uint8_t SimSession::read_memory(uint16_t address) {
    return sim_.bus().read8(address);
}

uint8_t SimSession::peek_memory(uint16_t address) {
    return sim_.bus().peek8(address);
}

void SimSession::write_memory(uint16_t address, uint8_t value) {
    sim_.bus().write8(address, value);
}

bool SimSession::inject_serial_text(std::string_view text) {
    return inject_serial_bytes(std::vector<uint8_t>(text.begin(), text.end()));
}

bool SimSession::inject_serial_bytes(const std::vector<uint8_t>& bytes) {
    if (!serial_dev_) {
        add_log("No serial device is mapped.");
        return false;
    }

    for (uint8_t ch : bytes) {
        serial_dev_->inject_rx(ch);
    }
    add_log("Injected " + std::to_string(bytes.size()) + " serial byte(s).");
    return true;
}

SerialSnapshot SimSession::serial_snapshot() const {
    SerialSnapshot snapshot;
    snapshot.present = serial_dev_ != nullptr;
    if (!serial_dev_) {
        return snapshot;
    }

    const auto led = serial_dev_->rgb_led();
    snapshot.output_port = serial_dev_->output_port();
    snapshot.led_red = led.red;
    snapshot.led_green = led.green;
    snapshot.led_blue = led.blue;
    snapshot.irq_asserted = serial_dev_->irq_asserted();
    return snapshot;
}

ParallelSnapshot SimSession::parallel_snapshot() const {
    ParallelSnapshot snapshot;
    snapshot.present = parallel_dev_ != nullptr;
    if (!hw_cfg_ || !hw_cfg_->parallel.present || !parallel_dev_) {
        return snapshot;
    }

    snapshot.start = hw_cfg_->parallel.start;
    snapshot.end = hw_cfg_->parallel.end;
    snapshot.input_a = parallel_dev_->input_a();
    snapshot.input_b = parallel_dev_->input_b();
    snapshot.output_a = parallel_dev_->output_a();
    snapshot.output_b = parallel_dev_->output_b();
    snapshot.ddr_a = parallel_dev_->ddr_a();
    snapshot.ddr_b = parallel_dev_->ddr_b();
    snapshot.port_a = parallel_dev_->port_a();
    snapshot.port_b = parallel_dev_->port_b();
    snapshot.acr = parallel_dev_->acr();
    snapshot.pcr = parallel_dev_->pcr();
    snapshot.ifr = parallel_dev_->ifr();
    snapshot.ier = parallel_dev_->ier();
    snapshot.irq_asserted = parallel_dev_->irq_asserted();
    snapshot.timer1_counter = parallel_dev_->timer1_counter();
    snapshot.timer1_latch = parallel_dev_->timer1_latch();
    snapshot.timer1_running = parallel_dev_->timer1_running();
    snapshot.timer1_free_running = parallel_dev_->timer1_free_running();
    snapshot.pb7_timer_output_enabled = parallel_dev_->timer1_pb7_output_enabled();
    snapshot.pb7_timer_level = parallel_dev_->timer1_pb7_level();
    snapshot.pb7_pin_level = parallel_dev_->pb7_pin_level();
    snapshot.pb7_transition_count = parallel_dev_->pb7_transition_count();
    return snapshot;
}

VdcSnapshot SimSession::vdc_snapshot() const {
    VdcSnapshot snapshot;
    snapshot.present = vdc_dev_ != nullptr;
    if (!hw_cfg_ || !hw_cfg_->video.present || !vdc_dev_) {
        return snapshot;
    }

    snapshot.start = hw_cfg_->video.start;
    snapshot.end = hw_cfg_->video.end;
    snapshot.selected_register = vdc_dev_->selected_register();
    snapshot.status = vdc_dev_->status();
    snapshot.registers = vdc_dev_->registers();
    snapshot.display_start = vdc_dev_->display_start();
    snapshot.attribute_start = vdc_dev_->attribute_start();
    snapshot.update_address = vdc_dev_->update_address();
    snapshot.cursor_position = vdc_dev_->cursor_position();
    snapshot.character_start = vdc_dev_->character_start();
    snapshot.frame_version = vdc_dev_->frame_version();
    snapshot.chars = vdc_dev_->display_chars();
    snapshot.attrs = vdc_dev_->display_attrs();
    snapshot.character_data = vdc_dev_->character_data();
    return snapshot;
}

bool SimSession::add_breakpoint(uint16_t address, std::string label) {
    const auto it = std::find_if(breakpoints_.begin(), breakpoints_.end(), [&](const Breakpoint& breakpoint) {
        return breakpoint.address == address;
    });
    if (it != breakpoints_.end()) return false;

    breakpoints_.push_back(Breakpoint{address, true, std::move(label), 0});
    std::sort(breakpoints_.begin(), breakpoints_.end(), [](const Breakpoint& a, const Breakpoint& b) {
        return a.address < b.address;
    });

    std::ostringstream out;
    out << "Added breakpoint at 0x" << std::uppercase << std::hex << std::setw(4) << std::setfill('0') << address << ".";
    add_log(out.str());
    return true;
}

bool SimSession::remove_breakpoint(uint16_t address) {
    const auto it = std::find_if(breakpoints_.begin(), breakpoints_.end(), [&](const Breakpoint& breakpoint) {
        return breakpoint.address == address;
    });
    if (it == breakpoints_.end()) return false;
    breakpoints_.erase(it);

    std::ostringstream out;
    out << "Removed breakpoint at 0x" << std::uppercase << std::hex << std::setw(4) << std::setfill('0') << address << ".";
    add_log(out.str());
    return true;
}

bool SimSession::is_breakpoint(uint16_t address) const {
    return std::any_of(breakpoints_.begin(), breakpoints_.end(), [&](const Breakpoint& breakpoint) {
        return breakpoint.address == address && breakpoint.enabled;
    });
}

bool SimSession::set_breakpoint_enabled(uint16_t address, bool enabled) {
    const auto it = std::find_if(breakpoints_.begin(), breakpoints_.end(), [&](const Breakpoint& breakpoint) {
        return breakpoint.address == address;
    });
    if (it == breakpoints_.end()) return false;
    it->enabled = enabled;
    return true;
}

bool SimSession::set_breakpoint_label(uint16_t address, std::string label) {
    const auto it = std::find_if(breakpoints_.begin(), breakpoints_.end(), [&](const Breakpoint& breakpoint) {
        return breakpoint.address == address;
    });
    if (it == breakpoints_.end()) return false;
    it->label = std::move(label);
    return true;
}

void SimSession::set_breakpoints(std::vector<Breakpoint> breakpoints) {
    breakpoints_ = std::move(breakpoints);
    std::sort(breakpoints_.begin(), breakpoints_.end(), [](const Breakpoint& a, const Breakpoint& b) {
        return a.address < b.address;
    });
}

bool SimSession::add_watchpoint(uint16_t address, WatchpointType type, std::string label) {
    auto it = std::find_if(watchpoints_.begin(), watchpoints_.end(), [&](const Watchpoint& watchpoint) {
        return watchpoint.address == address;
    });

    std::ostringstream out;
    if (it != watchpoints_.end()) {
        bool changed = false;
        if (!label.empty() && it->label != label) {
            it->label = std::move(label);
            changed = true;
        }
        if (!watchpoint_matches(it->type, type)) {
            it->type = merge_watchpoint_types(it->type, type);
            changed = true;
        }
        if (!changed) return false;
        out << "Updated watchpoint at 0x" << std::uppercase << std::hex << std::setw(4) << std::setfill('0') << address
            << " to " << watchpoint_log_label(it->type) << ".";
        add_log(out.str());
        return true;
    }

    watchpoints_.push_back(Watchpoint{address, type, true, std::move(label), 0});
    std::sort(watchpoints_.begin(), watchpoints_.end(), [](const Watchpoint& a, const Watchpoint& b) {
        return a.address < b.address;
    });

    out << "Added " << watchpoint_log_label(type) << " watchpoint at 0x" << std::uppercase << std::hex << std::setw(4)
        << std::setfill('0') << address << ".";
    add_log(out.str());
    return true;
}

bool SimSession::remove_watchpoint(uint16_t address) {
    const auto it = std::find_if(watchpoints_.begin(), watchpoints_.end(), [&](const Watchpoint& watchpoint) {
        return watchpoint.address == address;
    });
    if (it == watchpoints_.end()) return false;
    watchpoints_.erase(it);

    std::ostringstream out;
    out << "Removed watchpoint at 0x" << std::uppercase << std::hex << std::setw(4) << std::setfill('0') << address << ".";
    add_log(out.str());
    return true;
}

bool SimSession::remove_watchpoint(uint16_t address, WatchpointType type) {
    const auto it = std::find_if(watchpoints_.begin(), watchpoints_.end(), [&](const Watchpoint& watchpoint) {
        return watchpoint.address == address;
    });
    if (it == watchpoints_.end()) return false;

    if (!watchpoint_matches(it->type, type)) return false;
    if (it->type == WatchpointType::ReadWrite && type == WatchpointType::Read) {
        it->type = WatchpointType::Write;
    } else if (it->type == WatchpointType::ReadWrite && type == WatchpointType::Write) {
        it->type = WatchpointType::Read;
    } else {
        watchpoints_.erase(it);
    }

    std::ostringstream out;
    out << "Removed " << watchpoint_log_label(type) << " watchpoint at 0x" << std::uppercase << std::hex << std::setw(4)
        << std::setfill('0') << address << ".";
    add_log(out.str());
    return true;
}

bool SimSession::is_watchpoint(uint16_t address, WatchpointType type) const {
    return std::any_of(watchpoints_.begin(), watchpoints_.end(), [&](const Watchpoint& watchpoint) {
        return watchpoint.address == address && watchpoint.enabled && watchpoint_matches(watchpoint.type, type);
    });
}

bool SimSession::set_watchpoint_enabled(uint16_t address, bool enabled) {
    const auto it = std::find_if(watchpoints_.begin(), watchpoints_.end(), [&](const Watchpoint& watchpoint) {
        return watchpoint.address == address;
    });
    if (it == watchpoints_.end()) return false;
    it->enabled = enabled;
    return true;
}

bool SimSession::set_watchpoint_label(uint16_t address, std::string label) {
    const auto it = std::find_if(watchpoints_.begin(), watchpoints_.end(), [&](const Watchpoint& watchpoint) {
        return watchpoint.address == address;
    });
    if (it == watchpoints_.end()) return false;
    it->label = std::move(label);
    return true;
}

void SimSession::set_watchpoints(std::vector<Watchpoint> watchpoints) {
    watchpoints_ = std::move(watchpoints);
    std::sort(watchpoints_.begin(), watchpoints_.end(), [](const Watchpoint& a, const Watchpoint& b) {
        return a.address < b.address;
    });
}

void SimSession::clear_trace() {
    trace_.clear();
    pending_micro_trace_pc_.reset();
    pending_micro_trace_instruction_.clear();
    trace_cycle_base_ = sim_.clock().total_cycles();
}

std::vector<std::string> SimSession::memory_map() const {
    return sim_.bus().map_summary();
}

MapperSnapshot SimSession::mapper_snapshot() const {
    MapperSnapshot snapshot;
    if (!hw_cfg_ || !hw_cfg_->mapper.present || !mapper_state_) return snapshot;

    snapshot.present = true;
    snapshot.bank_size = hw_cfg_->ram.bank_size;
    snapshot.available = hw_cfg_->ram.available;
    for (int i = 0; i < 4; ++i) {
        snapshot.bank_registers[i] = hw_cfg_->mapper.bank_reg[i];
        snapshot.selected_banks[i] = mapper_state_->bank[i];
    }

    for (std::size_t i = 0; i < hw_cfg_->mapper.windows.size(); ++i) {
        const auto& window = hw_cfg_->mapper.windows[i];
        if (!window.present || window.start > window.end) continue;
        snapshot.windows.push_back(MapperWindowSnapshot{
            window.start,
            window.end,
            static_cast<uint8_t>(i),
            mapper_state_->bank[i],
        });
    }
    if (!snapshot.windows.empty()) return snapshot;

    if (hw_cfg_->ram.present && hw_cfg_->ram.bank_size > 0) {
        const uint32_t ram_size = static_cast<uint32_t>(hw_cfg_->ram.end - hw_cfg_->ram.start + 1);
        const uint32_t window_count = (ram_size + hw_cfg_->ram.bank_size - 1) / hw_cfg_->ram.bank_size;
        for (uint32_t i = 0; i < window_count && i < 4; ++i) {
            const uint32_t start = static_cast<uint32_t>(hw_cfg_->ram.start) + i * hw_cfg_->ram.bank_size;
            const uint32_t end = std::min<uint32_t>(start + hw_cfg_->ram.bank_size - 1, hw_cfg_->ram.end);
            snapshot.windows.push_back(MapperWindowSnapshot{
                static_cast<uint16_t>(start),
                static_cast<uint16_t>(end),
                static_cast<uint8_t>(i),
                mapper_state_->bank[i],
            });
        }
        if (hw_cfg_->mapper.bank_reg[3] != 0 && hw_cfg_->ram.bank_size == 0x4000 &&
            (hw_cfg_->ram.start > 0xC000 || hw_cfg_->ram.end < 0xDFFF)) {
            snapshot.windows.push_back(MapperWindowSnapshot{0xC000, 0xDFFF, 3, mapper_state_->bank[3]});
        }
    }

    return snapshot;
}

CfSnapshot SimSession::cf_snapshot() const {
    CfSnapshot snapshot;
    if (!hw_cfg_ || !hw_cfg_->cf.present || !cf_dev_) return snapshot;

    const auto device = cf_dev_->snapshot();
    snapshot.present = true;
    snapshot.image_loaded = device.image_loaded;
    snapshot.start = hw_cfg_->cf.start;
    snapshot.end = hw_cfg_->cf.end;
    snapshot.image_path = device.image_path;
    snapshot.sector_count = device.sector_count;
    snapshot.read_only = device.read_only;
    snapshot.error = device.error;
    snapshot.features = device.features;
    snapshot.sector_count_reg = device.sector_count_reg;
    snapshot.sector_number = device.sector_number;
    snapshot.cylinder_low = device.cylinder_low;
    snapshot.cylinder_high = device.cylinder_high;
    snapshot.drive_head = device.drive_head;
    snapshot.status = device.status;
    snapshot.command = device.command;
    snapshot.selected_lba = device.selected_lba;
    snapshot.requested_sector_count = device.requested_sector_count;
    switch (device.transfer_mode) {
    case devices::CompactFlash::TransferMode::Read:
        snapshot.transfer_mode = CfTransferMode::Read;
        break;
    case devices::CompactFlash::TransferMode::Write:
        snapshot.transfer_mode = CfTransferMode::Write;
        break;
    case devices::CompactFlash::TransferMode::None:
        snapshot.transfer_mode = CfTransferMode::None;
        break;
    }
    snapshot.transfer_size = device.transfer_size;
    snapshot.transfer_index = device.transfer_index;
    return snapshot;
}

LogicDecodeSnapshot SimSession::logic_decode_snapshot(uint16_t address, bool rw) const {
    BusSignals signals;
    signals.address = address;
    signals.rw = rw;
    signals.e = true;
    signals.q = false;
    signals.memory_enable = true;
    signals.mapper_enable = true;
    signals.data = 0xFF;
    return logic_decode_snapshot(signals);
}

LogicDecodeSnapshot SimSession::logic_decode_snapshot(const BusSignals& signals) const {
    LogicDecodeSnapshot snapshot;
    snapshot.phase = signals.phase;
    snapshot.cycle_kind = signals.cycle_kind;
    snapshot.address = signals.address;
    snapshot.data = signals.data;
    snapshot.rw = signals.rw;
    snapshot.e = signals.e;
    snapshot.q = signals.q;
    snapshot.ba = signals.ba;
    snapshot.bs = signals.bs;
    snapshot.breq = signals.breq;
    snapshot.memory_enable = signals.memory_enable;
    snapshot.mapper_enable = signals.mapper_enable;
    snapshot.apply_read = signals.apply_read;
    snapshot.apply_write = signals.apply_write;
    snapshot.log_access = signals.log_access;

    if (!hw_cfg_ || !hw_cfg_->logic.present) {
        snapshot.error = "No [PLD_LOGIC] section is configured.";
        return snapshot;
    }

    snapshot.configured = true;
    snapshot.bus_mode = hw_cfg_->logic.bus_mode;
    snapshot.signal_logic_path = hw_cfg_->logic.signal_logic_path;
    snapshot.memory_logic_path = hw_cfg_->logic.memory_logic_path;
    snapshot.address_logic_path = hw_cfg_->logic.address_logic_path;

    if (!logic_devices_) {
        snapshot.error = logic_error_.empty() ? "PLD logic is not loaded." : logic_error_;
        return snapshot;
    }

    snapshot.available = true;
    snapshot.mapper_bits = mapper_bits_for_address(*hw_cfg_, mapper_state_.get(), signals.address);
    snapshot.decoded = microlind::logic::decode_board_logic(*logic_devices_, microlind::logic::BoardSignals{
        .address = signals.address,
        .mapper_bits = snapshot.mapper_bits,
        .rw = signals.rw,
        .e = signals.e,
        .q = signals.q,
        .ba = signals.ba,
        .bs = signals.bs,
        .breq = signals.breq,
        .memory_enable = signals.memory_enable,
        .mapper_enable = signals.mapper_enable,
    });
    return snapshot;
}

void SimSession::add_log(std::string message) {
    log_.push_back(std::move(message));
    if (log_.size() > 256) {
        log_.erase(log_.begin());
    }
}

void SimSession::rebuild(std::string reason) {
    serial_dev_ = nullptr;
    cf_dev_ = nullptr;
    parallel_dev_ = nullptr;
    vdc_dev_ = nullptr;
    mapper_state_.reset();
    logic_devices_.reset();
    logic_error_.clear();
    if (hw_cfg_ && hw_cfg_->logic.present) {
        if (auto devices = cli::load_board_logic_devices(hw_cfg_->logic, logic_error_)) {
            logic_devices_ = std::move(devices);
        }
    }
    std::vector<std::string> diagnostics;
    sim_ = cli::build_sim(
        mode_,
        image_ ? &*image_ : nullptr,
        hw_cfg_ ? &*hw_cfg_ : nullptr,
        &serial_dev_,
        [this](uint8_t value) { on_serial_tx(value); },
        &mapper_state_,
        &cf_dev_,
        &parallel_dev_,
        &vdc_dev_,
        &diagnostics);
    for (const auto& diagnostic : diagnostics) {
        add_log(diagnostic);
    }
    add_log(std::move(reason));
}

void SimSession::on_serial_tx(uint8_t value) {
    serial_tx_.push_back(value);
    if (serial_tx_.size() > 8192) {
        serial_tx_.erase(
            serial_tx_.begin(),
            serial_tx_.begin() + static_cast<std::ptrdiff_t>(serial_tx_.size() - 8192));
    }
}

void SimSession::record_trace(uint16_t pc, std::string instruction, CpuTickResult result) {
    trace_.push_back(InstructionTraceEntry{
        pc,
        std::move(instruction),
        result.cycles,
        sim_.clock().total_cycles() - trace_cycle_base_,
    });
    if (trace_.size() > 256) {
        trace_.erase(trace_.begin());
    }
}

bool SimSession::check_breakpoint(uint32_t executed, RunResult& result) {
    const uint16_t pc = sim_.cpu().regs().pc;
    const auto it = std::find_if(breakpoints_.begin(), breakpoints_.end(), [&](const Breakpoint& breakpoint) {
        return breakpoint.address == pc && breakpoint.enabled;
    });
    if (it == breakpoints_.end()) return false;

    ++it->hits;
    result.executed = executed;
    result.hit_breakpoint = true;
    result.breakpoint_address = pc;

    std::ostringstream out;
    out << "Hit breakpoint at 0x" << std::uppercase << std::hex << std::setw(4) << std::setfill('0') << pc << ".";
    add_log(out.str());
    return true;
}

bool SimSession::check_target(uint16_t address, uint32_t executed, RunResult& result) {
    const uint16_t pc = sim_.cpu().regs().pc;
    if (pc != address) return false;

    result.executed = executed;
    result.hit_target = true;

    std::ostringstream out;
    out << "Reached 0x" << std::uppercase << std::hex << std::setw(4) << std::setfill('0') << address << ".";
    add_log(out.str());
    return true;
}

bool SimSession::check_watchpoints(RunResult& result) {
    if (watchpoints_.empty()) return false;

    for (const auto& access : sim_.bus().access_log()) {
        const WatchpointType type = access.type == BusAccessType::Read ? WatchpointType::Read : WatchpointType::Write;
        const auto it = std::find_if(watchpoints_.begin(), watchpoints_.end(), [&](const Watchpoint& watchpoint) {
            return watchpoint.address == access.address && watchpoint.enabled && watchpoint_matches(watchpoint.type, type);
        });
        if (it == watchpoints_.end()) continue;

        ++it->hits;
        result.hit_watchpoint = true;
        result.watchpoint_address = access.address;
        result.watchpoint_type = access.type;
        result.watchpoint_value = access.value;

        std::ostringstream out;
        out << "Hit " << watchpoint_log_label(type) << " watchpoint at 0x" << std::uppercase << std::hex << std::setw(4)
            << std::setfill('0') << access.address << " value 0x" << std::setw(2) << static_cast<int>(access.value)
            << ".";
        add_log(out.str());
        return true;
    }

    return false;
}

} // namespace microlind::app
