#pragma once

#include <cstdint>
#include <cstddef>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "microlind/cpu.hpp"
#include "microlind/simulator.hpp"

#include "microlind/app/hardware_config.hpp"
#include "microlind/app/image_loader.hpp"

namespace microlind::devices {
class CompactFlash;
struct MapperState;
class XR88C92;
}

namespace microlind::app {

struct InstructionTraceEntry {
    uint16_t pc{};
    std::string instruction;
    uint32_t cycles{};
    uint64_t total_cycles{};
};

struct RunResult {
    uint32_t executed{};
    bool hit_breakpoint{};
    bool hit_target{};
    bool hit_watchpoint{};
    uint16_t breakpoint_address{};
    uint16_t watchpoint_address{};
    BusAccessType watchpoint_type{BusAccessType::Read};
    uint8_t watchpoint_value{};
};

enum class WatchpointType {
    Read,
    Write,
    ReadWrite,
};

struct Watchpoint {
    uint16_t address{};
    WatchpointType type{WatchpointType::Read};
    bool enabled{true};
    std::string label;
    uint64_t hits{};
};

struct Breakpoint {
    uint16_t address{};
    bool enabled{true};
    std::string label;
    uint64_t hits{};
};

struct MapperWindowSnapshot {
    uint16_t start{};
    uint16_t end{};
    uint8_t window{};
    uint8_t selected_bank{};
};

struct MapperSnapshot {
    bool present{};
    uint32_t bank_size{};
    uint32_t available{};
    uint16_t bank_registers[4]{};
    uint8_t selected_banks[4]{};
    std::vector<MapperWindowSnapshot> windows;
};

enum class CfTransferMode {
    None,
    Read,
    Write,
};

struct CfSnapshot {
    bool present{};
    uint16_t start{};
    uint16_t end{};
    std::filesystem::path image_path{};
    uint32_t sector_count{};
    bool read_only{};
    uint8_t error{};
    uint8_t features{};
    uint8_t sector_count_reg{};
    uint8_t sector_number{};
    uint8_t cylinder_low{};
    uint8_t cylinder_high{};
    uint8_t drive_head{};
    uint8_t status{};
    uint8_t command{};
    uint32_t selected_lba{};
    uint32_t requested_sector_count{};
    CfTransferMode transfer_mode{CfTransferMode::None};
    std::size_t transfer_size{};
    std::size_t transfer_index{};
};

class SimSession {
public:
    explicit SimSession(CpuMode mode = CpuMode::HD6309);

    [[nodiscard]] CpuMode mode() const { return mode_; }
    void set_mode(CpuMode mode);

    bool load_rom(const std::filesystem::path& path, cli::RomFormat format, uint16_t raw_base);
    bool load_hardware_config(const std::filesystem::path& path);
    bool attach_cf_image(const std::filesystem::path& path, uint32_t minimum_sectors);

    void reset();
    CpuTickResult step_instruction();
    RunResult run_instructions(uint32_t count);
    RunResult run_until_address(uint16_t address, uint32_t max_instructions);
    RunResult run_until_return(uint32_t max_instructions);
    void tick_cycles(uint64_t cycles);

    [[nodiscard]] std::optional<uint16_t> step_over_target();
    [[nodiscard]] std::optional<uint16_t> return_address_from_stack();

    [[nodiscard]] uint8_t read_memory(uint16_t address);
    [[nodiscard]] uint8_t peek_memory(uint16_t address);
    void write_memory(uint16_t address, uint8_t value);

    bool inject_serial_text(std::string_view text);
    bool inject_serial_bytes(const std::vector<uint8_t>& bytes);

    bool add_breakpoint(uint16_t address, std::string label = {});
    bool remove_breakpoint(uint16_t address);
    void clear_breakpoints() { breakpoints_.clear(); }
    [[nodiscard]] bool is_breakpoint(uint16_t address) const;
    bool set_breakpoint_enabled(uint16_t address, bool enabled);
    bool set_breakpoint_label(uint16_t address, std::string label);
    void set_breakpoints(std::vector<Breakpoint> breakpoints);
    [[nodiscard]] const std::vector<Breakpoint>& breakpoints() const { return breakpoints_; }

    bool add_watchpoint(uint16_t address, WatchpointType type, std::string label = {});
    bool remove_watchpoint(uint16_t address);
    bool remove_watchpoint(uint16_t address, WatchpointType type);
    void clear_watchpoints() { watchpoints_.clear(); }
    [[nodiscard]] bool is_watchpoint(uint16_t address, WatchpointType type) const;
    bool set_watchpoint_enabled(uint16_t address, bool enabled);
    bool set_watchpoint_label(uint16_t address, std::string label);
    void set_watchpoints(std::vector<Watchpoint> watchpoints);
    [[nodiscard]] const std::vector<Watchpoint>& watchpoints() const { return watchpoints_; }

    [[nodiscard]] const std::vector<InstructionTraceEntry>& trace() const { return trace_; }
    void clear_trace();

    [[nodiscard]] bool serial_mapped() const { return serial_dev_ != nullptr; }
    [[nodiscard]] std::vector<std::string> memory_map() const;
    [[nodiscard]] MapperSnapshot mapper_snapshot() const;
    [[nodiscard]] CfSnapshot cf_snapshot() const;

    [[nodiscard]] const std::vector<uint8_t>& serial_tx() const { return serial_tx_; }
    void clear_serial_tx() { serial_tx_.clear(); }

    [[nodiscard]] const std::vector<std::string>& log() const { return log_; }
    void clear_log() { log_.clear(); }
    void add_log(std::string message);

    [[nodiscard]] const Simulator& simulator() const { return sim_; }
    [[nodiscard]] Simulator& simulator() { return sim_; }

private:
    void rebuild(std::string reason);
    void on_serial_tx(uint8_t value);
    void record_trace(uint16_t pc, std::string instruction, CpuTickResult result);
    bool check_breakpoint(uint32_t executed, RunResult& result);
    bool check_target(uint16_t address, uint32_t executed, RunResult& result);
    bool check_watchpoints(RunResult& result);

    CpuMode mode_;
    std::optional<cli::LoadedImage> image_;
    std::optional<cli::HardwareConfig> hw_cfg_;
    std::shared_ptr<devices::MapperState> mapper_state_;
    devices::XR88C92* serial_dev_{nullptr};
    devices::CompactFlash* cf_dev_{nullptr};
    Simulator sim_;
    std::vector<uint8_t> serial_tx_;
    std::vector<Breakpoint> breakpoints_;
    std::vector<Watchpoint> watchpoints_;
    std::vector<InstructionTraceEntry> trace_;
    uint64_t trace_cycle_base_{};
    std::vector<std::string> log_;
};

} // namespace microlind::app
