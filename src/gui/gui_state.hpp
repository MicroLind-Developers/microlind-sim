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

#include "microlind/app/image_loader.hpp"
#include "microlind/app/session_file.hpp"
#include "microlind/app/sim_session.hpp"
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
    microlind::app::SimSession session{microlind::CpuMode::HD6309};

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
    int steps_per_frame{100};
    int memory_start{0x0000};
    int memory_rows{16};
    bool memory_follow_pc{false};
    int breakpoint_address{0x0000};
    int run_until_address{0x0000};
    int watchpoint_address{0x0000};
    int stack_register_index{0};
    int stack_start{0x0000};
    int stack_rows{32};
    bool stack_follow_pointer{true};
    bool serial_hex_view{false};
    bool serial_rx_hex{false};
    bool running{false};
    bool run_until_active{false};
    bool quit_requested{false};
    bool about_open{false};
    TextureResource about_logo{};

    GuiState() {
        set_buffer(session_path, "examples/bios.session");
        set_buffer(rom_path, "examples/bios.ihex");
        set_buffer(config_path, "examples/hw.cfg");
        set_buffer(cf_path, "examples/sim.img");
        session.add_log("GUI ready.");
    }

    void stop_execution() {
        running = false;
        run_until_active = false;
    }

    void load_rom() {
        const std::string path = buffer_string(rom_path);
        session.load_rom(path, selected_rom_format(rom_format_index), static_cast<uint16_t>(raw_base));
    }

    void load_config() {
        const std::string path = buffer_string(config_path);
        session.load_hardware_config(path);
    }

    void attach_cf() {
        const std::string path = buffer_string(cf_path);
        session.attach_cf_image(path, static_cast<uint32_t>(std::max(cf_min_sectors, 0)));
    }

    void load_session_file(const std::filesystem::path& path) {
        std::string error;
        const auto loaded = microlind::app::load_session_definition(path, error);
        if (!loaded) {
            session.add_log("Session error: " + error);
            return;
        }

        stop_execution();
        if (loaded->mode) {
            session.set_mode(*loaded->mode);
        }
        set_buffer(session_path, path.string());
        set_buffer(config_path, loaded->config_path.string());
        set_buffer(rom_path, loaded->rom_path.string());
        set_buffer(cf_path, loaded->cf_path.string());
        rom_format_index = rom_format_combo_index(loaded->rom_format);
        raw_base = loaded->raw_base;
        cf_min_sectors = static_cast<int>(loaded->cf_sectors);
        steps_per_frame = loaded->gui.steps_per_frame;
        memory_start = loaded->gui.memory_start;
        memory_rows = loaded->gui.memory_rows;
        memory_follow_pc = loaded->gui.memory_follow_pc;
        stack_register_index = loaded->gui.stack_register_index;
        stack_start = loaded->gui.stack_start;
        stack_rows = loaded->gui.stack_rows;
        stack_follow_pointer = loaded->gui.stack_follow_pointer;
        serial_hex_view = loaded->gui.serial_hex_view;
        serial_rx_hex = loaded->gui.serial_rx_hex;

        const bool config_ok = session.load_hardware_config(loaded->config_path);
        const bool rom_ok = config_ok && session.load_rom(loaded->rom_path, loaded->rom_format, loaded->raw_base);
        if (rom_ok && !loaded->cf_path.empty()) {
            session.attach_cf_image(loaded->cf_path, loaded->cf_sectors);
        }
        if (config_ok && rom_ok) {
            session.set_breakpoints(loaded->breakpoints);
            session.set_watchpoints(loaded->watchpoints);
            pending_layout_ini = loaded->layout_ini;
            session.add_log("Loaded session: " + path.string());
        }
    }

    void load_session_from_field() {
        load_session_file(buffer_string(session_path));
    }

    bool save_session_file(const std::filesystem::path& path) {
        if (path.empty()) {
            session.add_log("Session path is empty.");
            return false;
        }

        microlind::app::SessionDefinition definition;
        definition.config_path = buffer_string(config_path);
        definition.rom_path = buffer_string(rom_path);
        definition.cf_path = buffer_string(cf_path);
        definition.rom_format = selected_rom_format(rom_format_index);
        definition.raw_base = static_cast<uint16_t>(raw_base);
        definition.cf_sectors = static_cast<uint32_t>(std::max(cf_min_sectors, 0));
        definition.mode = session.mode();
        definition.breakpoints = session.breakpoints();
        definition.watchpoints = session.watchpoints();
        definition.gui.steps_per_frame = steps_per_frame;
        definition.gui.memory_start = static_cast<uint16_t>(memory_start);
        definition.gui.memory_rows = memory_rows;
        definition.gui.memory_follow_pc = memory_follow_pc;
        definition.gui.stack_register_index = stack_register_index;
        definition.gui.stack_start = static_cast<uint16_t>(stack_start);
        definition.gui.stack_rows = stack_rows;
        definition.gui.stack_follow_pointer = stack_follow_pointer;
        definition.gui.serial_hex_view = serial_hex_view;
        definition.gui.serial_rx_hex = serial_rx_hex;

        std::size_t layout_size = 0;
        const char* layout = ImGui::SaveIniSettingsToMemory(&layout_size);
        if (layout != nullptr && layout_size > 0) {
            definition.layout_ini = std::string(layout, layout_size);
        }

        std::string error;
        if (!microlind::app::save_session_definition(path, definition, error)) {
            session.add_log(error);
            return false;
        }

        set_buffer(session_path, path.string());
        session.add_log("Saved session: " + path.string());
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

    void send_serial_text() {
        const std::string text = buffer_string(serial_input);
        bool ok = true;
        const std::vector<uint8_t> bytes = serial_rx_hex ? parse_hex_bytes(text, ok) : std::vector<uint8_t>(text.begin(), text.end());
        if (!ok) {
            session.add_log("Serial RX hex parse error.");
            return;
        }
        if (session.inject_serial_bytes(bytes)) {
            set_buffer(serial_input, "");
        }
    }

    void step_once() {
        const auto result = session.run_instructions(1);
        if (result.hit_breakpoint || result.hit_watchpoint) {
            stop_execution();
        }
    }

    void toggle_run() {
        running = !running;
        if (running) {
            run_until_active = false;
        }
    }

    void step_over() {
        const auto target = session.step_over_target();
        if (!target) {
            step_once();
            return;
        }
        run_until_address = *target;
        running = false;
        run_until_active = true;
        session.add_log("Stepping over until " + hex_value(static_cast<uint16_t>(run_until_address), 4) + ".");
    }

    void run_until_return() {
        const auto target = session.return_address_from_stack();
        if (!target) {
            session.add_log("No return address is available on S.");
            return;
        }
        run_until_address = *target;
        running = false;
        run_until_active = true;
        session.add_log("Running until return " + hex_value(static_cast<uint16_t>(run_until_address), 4) + ".");
    }
};

void set_next_window_defaults(float x, float y, float w, float h);

} // namespace microlind::gui
