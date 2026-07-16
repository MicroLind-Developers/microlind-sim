#pragma once

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "microlind/cpu.hpp"

#include "microlind/app/image_loader.hpp"
#include "microlind/app/sim_session.hpp"

namespace microlind::app {

enum class GuiTheme {
    Dark,
    Light,
};

struct GuiSessionState {
    uint16_t memory_start{};
    int memory_rows{16};
    bool memory_follow_pc{};
    int stack_register_index{};
    uint16_t stack_start{};
    int stack_rows{32};
    bool stack_follow_pointer{true};
    bool serial_hex_view{};
    bool serial_rx_hex{};
    int operations_per_minute{600};
    bool run_micro_steps{};
    uint32_t true_clock_hz{1000000};
    GuiTheme theme{GuiTheme::Dark};
    bool show_file_panel{true};
    bool show_control_panel{true};
    bool show_registers{true};
    bool show_disassembly{true};
    bool show_memory_viewer{true};
    bool show_stack{true};
    bool show_memory_map{true};
    bool show_mapper{true};
    bool show_pld_logic{true};
    bool show_compact_flash{true};
    bool show_breakpoints{true};
    bool show_watchpoints{true};
    bool show_trace{true};
    bool show_serial{true};
    bool show_log{true};
};

struct SessionDefinition {
    std::filesystem::path config_path;
    std::filesystem::path rom_path;
    std::filesystem::path cf_path;
    std::string layout_ini;
    cli::RomFormat rom_format{cli::RomFormat::Ihex};
    uint16_t raw_base{0x8000};
    uint32_t cf_sectors{};
    std::optional<CpuMode> mode;
    GuiSessionState gui;
    std::vector<Breakpoint> breakpoints;
    std::vector<Watchpoint> watchpoints;
};

[[nodiscard]] std::optional<cli::RomFormat> parse_rom_format(std::string value);
[[nodiscard]] const char* rom_format_name(cli::RomFormat format);
[[nodiscard]] const char* cpu_mode_name(CpuMode mode);
[[nodiscard]] std::string hex_encode(std::string_view value);
[[nodiscard]] std::optional<std::string> hex_decode(std::string_view value);

[[nodiscard]] std::filesystem::path session_relative_path(
    const std::filesystem::path& session_path,
    const std::filesystem::path& value);

[[nodiscard]] std::optional<SessionDefinition> load_session_definition(
    const std::filesystem::path& path,
    std::string& error);

bool save_session_definition(
    const std::filesystem::path& path,
    const SessionDefinition& session,
    std::string& error);

} // namespace microlind::app
