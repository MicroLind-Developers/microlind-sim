#include "microlind/app/sim_session.hpp"

#include <algorithm>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <utility>

#include "microlind/app/disassembler.hpp"
#include "microlind/app/sim_builder.hpp"

#include "microlind/devices/compact_flash.hpp"
#include "microlind/devices/memory_mapper.hpp"
#include "microlind/devices/serial.hpp"

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
          &cf_dev_)) {
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
    CpuTickResult result = sim_.tick();
    record_trace(pc, disasm.text, result);
    return result;
}

RunResult SimSession::run_instructions(uint32_t count) {
    RunResult result;
    for (uint32_t i = 0; i < count; ++i) {
        if (check_breakpoint(result.executed, result)) {
            return result;
        }
        step_instruction();
        ++result.executed;
        if (check_watchpoints(result)) {
            return result;
        }
        if (check_breakpoint(result.executed, result)) {
            return result;
        }
    }
    return result;
}

RunResult SimSession::run_until_address(uint16_t address, uint32_t max_instructions) {
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

RunResult SimSession::run_until_return(uint32_t max_instructions) {
    const auto target = return_address_from_stack();
    if (!target) {
        add_log("No return address is available on S.");
        return {};
    }
    return run_until_address(*target, max_instructions);
}

void SimSession::tick_cycles(uint64_t cycles) {
    sim_.tick_clock(cycles);
}

std::optional<uint16_t> SimSession::step_over_target() {
    const uint16_t pc = sim_.cpu().regs().pc;
    const uint8_t op0 = sim_.bus().read8(pc);
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
    const uint16_t high = sim_.bus().read8(s);
    const uint16_t low = sim_.bus().read8(static_cast<uint16_t>(s + 1));
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

void SimSession::add_log(std::string message) {
    log_.push_back(std::move(message));
    if (log_.size() > 256) {
        log_.erase(log_.begin());
    }
}

void SimSession::rebuild(std::string reason) {
    serial_dev_ = nullptr;
    cf_dev_ = nullptr;
    mapper_state_.reset();
    sim_ = cli::build_sim(
        mode_,
        image_ ? &*image_ : nullptr,
        hw_cfg_ ? &*hw_cfg_ : nullptr,
        &serial_dev_,
        [this](uint8_t value) { on_serial_tx(value); },
        &mapper_state_,
        &cf_dev_);
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
