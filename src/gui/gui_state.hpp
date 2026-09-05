#pragma once

#include <algorithm>
#include <array>
#include <cstdint>
#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

#include <SDL.h>
#include "imgui.h"
#ifdef MICROLIND_HAS_PORTABLE_FILE_DIALOGS
#include "portable-file-dialogs.h"
#endif

#include "gui_runtime.hpp"

#include "microlind/app/image_loader.hpp"
#include "microlind/app/session_file.hpp"
#include "microlind/app/vdc_render.hpp"
#include "microlind/cpu.hpp"

namespace microlind::gui {

template <std::size_t N>
void set_buffer(std::array<char, N>& buffer, std::string_view value) {
    buffer.fill('\0');
    const std::size_t count = std::min(value.size(), N - 1);
    std::copy_n(value.data(), count, buffer.data());
}

template <std::size_t N>
std::string buffer_string(const std::array<char, N>& buffer) {
    return std::string(buffer.data());
}

struct TextureResource {
    SDL_Texture* texture{};
    int width{};
    int height{};
};

std::string hex_value(uint32_t value, int width);
std::string instruction_bytes(microlind::Bus& bus, uint16_t pc, uint8_t length);
TextureResource load_png_texture(SDL_Renderer* renderer, const std::filesystem::path& path);
SDL_Surface* load_png_surface(const std::filesystem::path& path);
bool save_vdc_screenshot_png(
    const std::filesystem::path& path,
    const microlind::app::VdcSnapshot& vdc,
    double elapsed_seconds,
    std::string& error);
bool update_rgba_texture(
    SDL_Renderer* renderer,
    TextureResource& texture,
    const microlind::app::VdcFramebuffer& framebuffer,
    std::string& error);
std::string serial_terminal_text(const std::vector<uint8_t>& bytes);
std::vector<uint8_t> parse_hex_bytes(std::string_view input, bool& ok);
const char* watchpoint_type_label(microlind::app::WatchpointType type);
const char* cf_transfer_label(microlind::app::CfTransferMode mode);
std::string cf_status_flags(uint8_t status);
std::string cf_command_name(uint8_t command);

#ifdef MICROLIND_HAS_PORTABLE_FILE_DIALOGS
std::string pick_file(const std::string& title, const std::vector<std::string>& filters);
std::string pick_save_file(const std::string& title, const std::string& default_path, const std::vector<std::string>& filters);
#endif

microlind::cli::RomFormat selected_rom_format(int index);
int rom_format_combo_index(microlind::cli::RomFormat format);

struct GuiState {
    GuiRuntime runtime{microlind::CpuMode::HD6309};

    std::array<char, 512> rom_path{};
    std::array<char, 512> config_path{};
    std::array<char, 512> cf_path{};
    std::array<char, 512> session_path{};
    std::array<char, 256> serial_input{};
    std::array<char, 128> breakpoint_label{};
    std::array<char, 128> watchpoint_label{};
    std::string pending_layout_ini;
    int rom_format_index{1};
    int raw_base{0x8000};
    int cf_min_sectors{0};
    int memory_start{0x0000};
    int memory_rows{16};
    bool memory_follow_pc{false};
    int breakpoint_address{0x0000};
    int run_until_address{0x0000};
    int watchpoint_address{0x0000};
    int pld_address{0x0000};
    int stack_register_index{0};
    int stack_start{0x0000};
    int stack_rows{32};
    bool stack_follow_pointer{true};
    bool serial_hex_view{false};
    bool serial_rx_hex{false};
    int vdc_scale_mode{};
    bool vdc_crt_aspect{true};
    bool pld_live_bus{false};
    bool pld_follow_pc{true};
    bool pld_read{true};
    bool quit_requested{false};
    bool help_open{false};
    bool about_open{false};
    microlind::app::GuiTheme theme{microlind::app::GuiTheme::Dark};
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
    bool show_parallel{true};
    bool show_logic_analyser{true};
    bool show_video{true};
    bool show_breakpoints{true};
    bool show_watchpoints{true};
    bool show_trace{true};
    bool show_serial{true};
    bool show_log{true};
    bool logic_analyser_microcycle{};
    bool logic_analyser_use_trigger{};
    int logic_analyser_trigger_signal{static_cast<int>(LogicSignal::ViaTimer2Flag)};
    int logic_analyser_trigger_mode{static_cast<int>(LogicTriggerMode::Rising)};
    int logic_analyser_samples_visible{160};
    int logic_analyser_history_offset{};
    std::array<bool, kLogicSignalCount> logic_analyser_signals{};
    std::array<ImVec4, kLogicSignalCount> logic_analyser_colors{};
    bool speaker_muted{false};
    bool speaker_audio_available{false};
    bool speaker_signal_active{false};
    float speaker_volume{0.12f};
    double speaker_frequency_hz{};
    uint64_t speaker_last_cycles{};
    uint64_t speaker_last_transitions{};
    TextureResource about_logo{};
    TextureResource vdc_display{};
    SDL_Renderer* renderer{};
    microlind::app::VdcSnapshot cached_vdc{};
    double last_vdc_refresh_time{-1.0};

    GuiState() {
        set_buffer(session_path, "examples/bios.session");
        set_buffer(rom_path, "examples/bios.ihex");
        set_buffer(config_path, "examples/hw.cfg");
        set_buffer(cf_path, "examples/sim.img");
        logic_analyser_signals[static_cast<std::size_t>(LogicSignal::ClockE)] = true;
        logic_analyser_signals[static_cast<std::size_t>(LogicSignal::ClockQ)] = true;
        logic_analyser_signals[static_cast<std::size_t>(LogicSignal::CpuIrq)] = true;
        logic_analyser_signals[static_cast<std::size_t>(LogicSignal::ViaTimer2Flag)] = true;
        logic_analyser_signals[static_cast<std::size_t>(LogicSignal::ViaPb7)] = true;
        const std::array<ImVec4, kLogicSignalCount> default_colors{{
            {0.35f, 0.75f, 1.00f, 1.00f}, {0.80f, 0.45f, 1.00f, 1.00f}, {0.45f, 0.90f, 0.60f, 1.00f},
            {1.00f, 0.75f, 0.30f, 1.00f}, {0.95f, 0.50f, 0.35f, 1.00f}, {1.00f, 0.35f, 0.35f, 1.00f},
            {1.00f, 0.55f, 0.25f, 1.00f}, {1.00f, 0.65f, 0.20f, 1.00f}, {0.30f, 0.90f, 0.85f, 1.00f},
            {0.95f, 0.85f, 0.25f, 1.00f}, {0.45f, 0.65f, 1.00f, 1.00f}, {0.95f, 0.45f, 0.75f, 1.00f},
            {0.60f, 0.90f, 0.35f, 1.00f}, {0.65f, 0.65f, 0.65f, 1.00f}, {0.85f, 0.85f, 0.85f, 1.00f},
        }};
        logic_analyser_colors = default_colors;
        runtime.add_log("GUI ready.");
    }

    bool running() const {
        const auto mode = runtime.mode();
        return mode == RuntimeMode::DebugRun || mode == RuntimeMode::DebugMicroRun;
    }

    bool run_until_active() const {
        return runtime.run_until_active();
    }

    bool true_running() const {
        return runtime.true_run_active();
    }

    bool debug_run_active() const {
        return runtime.debug_run_active();
    }

    void stop_execution() {
        runtime.stop();
    }

    void load_rom() {
        const std::string path = buffer_string(rom_path);
        runtime.load_rom(path, selected_rom_format(rom_format_index), static_cast<uint16_t>(raw_base));
    }

    void load_config() {
        const std::string path = buffer_string(config_path);
        runtime.load_hardware_config(path);
    }

    void attach_cf() {
        const std::string path = buffer_string(cf_path);
        runtime.attach_cf_image(path, static_cast<uint32_t>(std::max(cf_min_sectors, 0)));
    }

    void remove_cf() {
        if (runtime.remove_cf_image()) {
            set_buffer(cf_path, "");
            cf_min_sectors = 0;
        }
    }

    void load_session_file(const std::filesystem::path& path) {
        std::string error;
        const auto loaded = microlind::app::load_session_definition(path, error);
        if (!loaded) {
            runtime.add_log("Session error: " + error);
            return;
        }

        stop_execution();
        if (loaded->mode) {
            runtime.set_cpu_mode(*loaded->mode);
        }
        set_buffer(session_path, path.string());
        set_buffer(config_path, loaded->config_path.string());
        set_buffer(rom_path, loaded->rom_path.string());
        set_buffer(cf_path, loaded->cf_path.string());
        rom_format_index = rom_format_combo_index(loaded->rom_format);
        raw_base = loaded->raw_base;
        cf_min_sectors = static_cast<int>(loaded->cf_sectors);
        runtime.set_operations_per_minute(static_cast<uint32_t>(std::max(loaded->gui.operations_per_minute, 0)));
        runtime.set_true_run_target_hz(true_hz_for_index(true_clock_index_for_hz(loaded->gui.true_clock_hz)));
        memory_start = loaded->gui.memory_start;
        memory_rows = loaded->gui.memory_rows;
        memory_follow_pc = loaded->gui.memory_follow_pc;
        stack_register_index = loaded->gui.stack_register_index;
        stack_start = loaded->gui.stack_start;
        stack_rows = loaded->gui.stack_rows;
        stack_follow_pointer = loaded->gui.stack_follow_pointer;
        serial_hex_view = loaded->gui.serial_hex_view;
        serial_rx_hex = loaded->gui.serial_rx_hex;
        vdc_scale_mode = std::clamp(loaded->gui.vdc_scale_mode, 0, 4);
        vdc_crt_aspect = loaded->gui.vdc_crt_aspect;
        runtime.set_run_micro_steps(loaded->gui.run_micro_steps);
        theme = loaded->gui.theme;
        show_file_panel = loaded->gui.show_file_panel;
        show_control_panel = loaded->gui.show_control_panel;
        show_registers = loaded->gui.show_registers;
        show_disassembly = loaded->gui.show_disassembly;
        show_memory_viewer = loaded->gui.show_memory_viewer;
        show_stack = loaded->gui.show_stack;
        show_memory_map = loaded->gui.show_memory_map;
        show_mapper = loaded->gui.show_mapper;
        show_pld_logic = loaded->gui.show_pld_logic;
        show_compact_flash = loaded->gui.show_compact_flash;
        show_parallel = loaded->gui.show_parallel;
        show_logic_analyser = loaded->gui.show_logic_analyser;
        show_video = loaded->gui.show_video;
        show_breakpoints = loaded->gui.show_breakpoints;
        show_watchpoints = loaded->gui.show_watchpoints;
        show_trace = loaded->gui.show_trace;
        show_serial = loaded->gui.show_serial;
        show_log = loaded->gui.show_log;

        const bool config_ok = runtime.load_hardware_config(loaded->config_path);
        const bool rom_ok = config_ok && runtime.load_rom(loaded->rom_path, loaded->rom_format, loaded->raw_base);
        if (rom_ok) {
            if (!loaded->cf_path.empty()) {
                runtime.attach_cf_image(loaded->cf_path, loaded->cf_sectors);
            } else {
                runtime.remove_cf_image();
            }
        }
        if (config_ok && rom_ok) {
            runtime.set_breakpoints(loaded->breakpoints);
            runtime.set_watchpoints(loaded->watchpoints);
            pending_layout_ini = loaded->layout_ini;
            runtime.add_log("Loaded session: " + path.string());
        }
    }

    void load_session_from_field() {
        load_session_file(buffer_string(session_path));
    }

    bool save_session_file(const std::filesystem::path& path) {
        if (path.empty()) {
            runtime.add_log("Session path is empty.");
            return false;
        }

        stop_execution();

        microlind::app::SessionDefinition definition;
        definition.config_path = buffer_string(config_path);
        definition.rom_path = buffer_string(rom_path);
        definition.cf_path = buffer_string(cf_path);
        definition.rom_format = selected_rom_format(rom_format_index);
        definition.raw_base = static_cast<uint16_t>(raw_base);
        definition.cf_sectors = static_cast<uint32_t>(std::max(cf_min_sectors, 0));
        definition.mode = runtime.cpu_mode();
        definition.breakpoints = runtime.breakpoints();
        definition.watchpoints = runtime.watchpoints();
        definition.gui.operations_per_minute = static_cast<int>(runtime.operations_per_minute());
        definition.gui.run_micro_steps = runtime.run_micro_steps();
        definition.gui.true_clock_hz = static_cast<uint32_t>(runtime.true_target_hz());
        definition.gui.memory_start = static_cast<uint16_t>(memory_start);
        definition.gui.memory_rows = memory_rows;
        definition.gui.memory_follow_pc = memory_follow_pc;
        definition.gui.stack_register_index = stack_register_index;
        definition.gui.stack_start = static_cast<uint16_t>(stack_start);
        definition.gui.stack_rows = stack_rows;
        definition.gui.stack_follow_pointer = stack_follow_pointer;
        definition.gui.serial_hex_view = serial_hex_view;
        definition.gui.serial_rx_hex = serial_rx_hex;
        definition.gui.vdc_scale_mode = vdc_scale_mode;
        definition.gui.vdc_crt_aspect = vdc_crt_aspect;
        definition.gui.theme = theme;
        definition.gui.show_file_panel = show_file_panel;
        definition.gui.show_control_panel = show_control_panel;
        definition.gui.show_registers = show_registers;
        definition.gui.show_disassembly = show_disassembly;
        definition.gui.show_memory_viewer = show_memory_viewer;
        definition.gui.show_stack = show_stack;
        definition.gui.show_memory_map = show_memory_map;
        definition.gui.show_mapper = show_mapper;
        definition.gui.show_pld_logic = show_pld_logic;
        definition.gui.show_compact_flash = show_compact_flash;
        definition.gui.show_parallel = show_parallel;
        definition.gui.show_logic_analyser = show_logic_analyser;
        definition.gui.show_video = show_video;
        definition.gui.show_breakpoints = show_breakpoints;
        definition.gui.show_watchpoints = show_watchpoints;
        definition.gui.show_trace = show_trace;
        definition.gui.show_serial = show_serial;
        definition.gui.show_log = show_log;

        std::size_t layout_size = 0;
        const char* layout = ImGui::SaveIniSettingsToMemory(&layout_size);
        if (layout != nullptr && layout_size > 0) {
            definition.layout_ini = std::string(layout, layout_size);
        }

        std::string error;
        if (!microlind::app::save_session_definition(path, definition, error)) {
            runtime.add_log(error);
            return false;
        }

        set_buffer(session_path, path.string());
        runtime.add_log("Saved session: " + path.string());
        return true;
    }

    void save_session_from_field() {
        save_session_file(buffer_string(session_path));
    }

    void save_session_as() {
#ifdef MICROLIND_HAS_PORTABLE_FILE_DIALOGS
        const auto path = pick_save_file(
            "Save session",
            buffer_string(session_path),
            {"Session files", "*.session", "All files", "*"});
        if (!path.empty()) {
            save_session_file(path);
        }
#else
        save_session_from_field();
#endif
    }

    void send_serial_text(bool append_carriage_return = false) {
        const std::string text = buffer_string(serial_input);
        bool ok = true;
        std::vector<uint8_t> bytes = serial_rx_hex ? parse_hex_bytes(text, ok) : std::vector<uint8_t>(text.begin(), text.end());
        if (!ok) {
            runtime.add_log("Serial RX hex parse error.");
            return;
        }
        if (append_carriage_return) {
            bytes.push_back('\r');
        }
        if (runtime.inject_serial_bytes(bytes)) {
            set_buffer(serial_input, "");
        }
    }

    void step_once() {
        const auto result = runtime.run_instructions(1);
        runtime.stop();
        if (result.hit_breakpoint || result.hit_watchpoint) {
            stop_execution();
        }
    }

    void step_microcycle() {
        stop_execution();
        const auto result = runtime.step_microcycle();
        if (result.instruction_started) {
            runtime.add_log("Started micro-step instruction.");
        }
        if (result.instruction_complete) {
            runtime.add_log("Completed micro-step instruction.");
        }
    }

    void toggle_run() {
        if (running()) {
            runtime.stop();
        } else {
            runtime.start_debug_run(runtime.run_micro_steps());
        }
    }

    void toggle_true_run() {
        if (true_running()) {
            runtime.stop_true_run();
        } else {
            runtime.start_true_run(runtime.true_target_hz());
        }
    }

    void set_all_panels_visible(bool visible) {
        show_file_panel = visible;
        show_control_panel = visible;
        show_registers = visible;
        show_disassembly = visible;
        show_memory_viewer = visible;
        show_stack = visible;
        show_memory_map = visible;
        show_mapper = visible;
        show_pld_logic = visible;
        show_compact_flash = visible;
        show_parallel = visible;
        show_logic_analyser = visible;
        show_video = visible;
        show_breakpoints = visible;
        show_watchpoints = visible;
        show_trace = visible;
        show_serial = visible;
        show_log = visible;
    }

    static uint64_t true_hz_for_index(int index) {
        constexpr uint64_t clocks[] = {1000000, 2000000, 3000000};
        return clocks[std::clamp(index, 0, 2)];
    }

    static int true_clock_index_for_hz(uint64_t hz) {
        if (hz <= 1500000) return 0;
        if (hz <= 2500000) return 1;
        return 2;
    }

    void step_over() {
        stop_execution();
        const auto target = runtime.step_over_target();
        if (!target) {
            step_once();
            return;
        }
        run_until_address = *target;
        runtime.start_step_over(*target);
        runtime.add_log("Stepping over until " + hex_value(static_cast<uint16_t>(run_until_address), 4) + ".");
    }

    void run_until_return() {
        stop_execution();
        const auto target = runtime.return_address_from_stack();
        if (!target) {
            runtime.add_log("No return address is available on S.");
            return;
        }
        run_until_address = *target;
        runtime.start_run_until_return(*target);
        runtime.add_log("Running until return " + hex_value(static_cast<uint16_t>(run_until_address), 4) + ".");
    }

    void toggle_run_until_address() {
        if (run_until_active()) {
            runtime.stop();
            return;
        }
        runtime.start_run_until_address(static_cast<uint16_t>(run_until_address));
        runtime.add_log("Running until " + hex_value(static_cast<uint16_t>(run_until_address), 4) + ".");
    }
};

void set_next_window_defaults(float x, float y, float w, float h);

} // namespace microlind::gui
