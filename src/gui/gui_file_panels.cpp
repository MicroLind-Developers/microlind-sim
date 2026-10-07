#include "gui_panel_decls.hpp"

#include <algorithm>
#include <cstdio>
#include <iterator>
#include <string>

#include "imgui.h"

namespace microlind::gui {
namespace {

ImVec4 serial_led_color(const app::SerialSnapshot& serial) {
    if (!serial.present) {
        return ImVec4(0.16f, 0.16f, 0.16f, 1.0f);
    }
    const float red = serial.led_red ? 0.95f : 0.08f;
    const float green = serial.led_green ? 0.95f : 0.08f;
    const float blue = serial.led_blue ? 0.95f : 0.08f;
    return ImVec4(red, green, blue, 1.0f);
}

void draw_power_led(const app::SerialSnapshot& serial) {
    ImGui::TextUnformatted("Power LED");
    ImGui::SameLine();
    ImGui::ColorButton(
        "##serial_power_led",
        serial_led_color(serial),
        ImGuiColorEditFlags_NoPicker | ImGuiColorEditFlags_NoDragDrop,
        ImVec2(20.0f, 20.0f));
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("OP4 red, OP5 green, OP6 blue");
    }
    ImGui::SameLine();
    ImGui::Text("OP $%02X", serial.output_port);
}

} // namespace

void draw_file_panel(GuiState& state) {
    set_next_window_defaults(8.0f, 28.0f, 360.0f, 340.0f);
    ImGui::Begin("Files", &state.show_file_panel);

    ImGui::InputText("Session", state.session_path.data(), state.session_path.size());
#ifdef MICROLIND_HAS_PORTABLE_FILE_DIALOGS
    ImGui::SameLine();
    if (ImGui::Button("Browse##session")) {
        const auto path = pick_file("Load session", {"Session files", "*.session *.ini *.cfg", "All files", "*"});
        if (!path.empty()) set_buffer(state.session_path, path);
    }
#endif
    if (ImGui::Button("Load Session")) {
        state.load_session_from_field();
    }

    ImGui::Separator();
    const char* formats[] = {"Raw", "Intel HEX", "S-record"};
    ImGui::Combo("ROM format", &state.rom_format_index, formats, static_cast<int>(std::size(formats)));
    if (state.rom_format_index == 0) {
        ImGui::InputInt("Raw base", &state.raw_base, 0x100, 0x1000, ImGuiInputTextFlags_CharsHexadecimal);
        state.raw_base = std::clamp(state.raw_base, 0, 0xFFFF);
    }
    ImGui::InputText("ROM", state.rom_path.data(), state.rom_path.size());
#ifdef MICROLIND_HAS_PORTABLE_FILE_DIALOGS
    ImGui::SameLine();
    if (ImGui::Button("Browse##rom")) {
        const auto path = pick_file("Load ROM", {"ROM files", "*.rom *.bin *.hex *.ihex *.s19 *.srec", "All files", "*"});
        if (!path.empty()) set_buffer(state.rom_path, path);
    }
#endif
    if (ImGui::Button("Load ROM")) {
        state.load_rom();
    }

    ImGui::Separator();
    ImGui::InputText("Config", state.config_path.data(), state.config_path.size());
#ifdef MICROLIND_HAS_PORTABLE_FILE_DIALOGS
    ImGui::SameLine();
    if (ImGui::Button("Browse##config")) {
        const auto path = pick_file("Load hardware config", {"Config files", "*.cfg *.ini", "All files", "*"});
        if (!path.empty()) set_buffer(state.config_path, path);
    }
#endif
    if (ImGui::Button("Load Config")) {
        state.load_config();
    }

    ImGui::Separator();
    ImGui::InputText("CF image", state.cf_path.data(), state.cf_path.size());
#ifdef MICROLIND_HAS_PORTABLE_FILE_DIALOGS
    ImGui::SameLine();
    if (ImGui::Button("Browse##cf")) {
        const auto path = pick_file("Attach CF image", {"Disk images", "*.img *.bin", "All files", "*"});
        if (!path.empty()) set_buffer(state.cf_path, path);
    }
#endif
    ImGui::InputInt("Min sectors", &state.cf_min_sectors);
    state.cf_min_sectors = std::max(state.cf_min_sectors, 0);
    if (ImGui::Button("Attach CF")) {
        state.attach_cf();
    }
    ImGui::SameLine();
    if (ImGui::Button("Remove CF")) {
        state.remove_cf();
    }

    ImGui::Separator();
    const char* modes[] = {"MC6809", "HD6309"};
    int mode_index = state.runtime.cpu_mode() == microlind::CpuMode::HD6309 ? 1 : 0;
    if (ImGui::Combo("CPU mode", &mode_index, modes, static_cast<int>(std::size(modes)))) {
        state.runtime.set_cpu_mode(mode_index == 1 ? microlind::CpuMode::HD6309 : microlind::CpuMode::MC6809);
    }

    ImGui::End();
}

void draw_control_panel(GuiState& state) {
    set_next_window_defaults(376.0f, 28.0f, 560.0f, 210.0f);
    ImGui::Begin("Control", &state.show_control_panel);
    if (ImGui::Button("Reset")) {
        state.stop_execution();
        state.runtime.reset();
    }
    ImGui::SameLine();
    ImGui::BeginDisabled(state.true_running());
    if (ImGui::Button("Step")) {
        state.step_once();
    }
    ImGui::SameLine();
    if (ImGui::Button("Micro Step")) {
        state.step_microcycle();
    }
    ImGui::SameLine();
    if (ImGui::Button("Step Over")) {
        state.step_over();
    }
    ImGui::SameLine();
    if (ImGui::Button(state.running() ? "Pause" : "Run")) {
        state.toggle_run();
    }
    ImGui::EndDisabled();
    int operations_per_minute = static_cast<int>(state.runtime.operations_per_minute());
    if (ImGui::SliderInt("Operations/min", &operations_per_minute, 10, 60000)) {
        state.runtime.set_operations_per_minute(static_cast<uint32_t>(std::max(operations_per_minute, 0)));
    }
    ImGui::Text("Frequency: %.2f operations/s", state.runtime.operations_per_second());
    ImGui::BeginDisabled(state.true_running());
    bool run_micro_steps = state.runtime.run_micro_steps();
    if (ImGui::Checkbox("Micro-step run", &run_micro_steps)) {
        state.runtime.set_run_micro_steps(run_micro_steps);
    }
    ImGui::InputInt("Run until", &state.run_until_address, 1, 256, ImGuiInputTextFlags_CharsHexadecimal);
    state.run_until_address = std::clamp(state.run_until_address, 0, 0xFFFF);
    if (ImGui::Button(state.run_until_active() ? "Stop Until" : "Run Until")) {
        state.toggle_run_until_address();
    }
    ImGui::SameLine();
    if (ImGui::Button("Until Return")) {
        state.run_until_return();
    }
    ImGui::EndDisabled();
    ImGui::Separator();
    const char* true_clocks[] = {"1 MHz", "2 MHz", "3 MHz"};
    int true_clock_index = GuiState::true_clock_index_for_hz(state.runtime.true_target_hz());
    if (ImGui::Combo("True clock", &true_clock_index, true_clocks, static_cast<int>(std::size(true_clocks)))) {
        state.runtime.set_true_run_target_hz(GuiState::true_hz_for_index(true_clock_index));
    }
    ImGui::SameLine();
    if (ImGui::Button(state.true_running() ? "Pause True" : "True Run")) {
        state.toggle_true_run();
    }
    const auto status = state.runtime.status_snapshot();
    ImGui::Text(
        "True frequency: %.2f MHz target, %.4f MHz effective",
        static_cast<double>(status.true_target_hz) / 1000000.0,
        status.true_effective_hz / 1000000.0);
    const auto snapshot = state.runtime.debugger_snapshot();
    ImGui::Text("Serial mapped: %s", snapshot.serial_mapped ? "yes" : "no");
    ImGui::End();
}

void draw_serial(GuiState& state) {
    set_next_window_defaults(944.0f, 694.0f, 480.0f, 80.0f);
    ImGui::Begin("Serial", &state.show_serial);
    const auto snapshot = state.runtime.debugger_snapshot();
    const auto& serial = snapshot.serial;
    draw_power_led(serial);
    ImGui::Separator();

    ImGui::BeginDisabled(!snapshot.serial_mapped);
    ImGui::Checkbox("Hex RX", &state.serial_rx_hex);
    ImGui::SameLine();
    const bool submitted = ImGui::InputText(
        "RX text",
        state.serial_input.data(),
        state.serial_input.size(),
        ImGuiInputTextFlags_EnterReturnsTrue);
    ImGui::SameLine();
    if (submitted && !buffer_string(state.serial_input).empty()) {
        state.send_serial_text(true);
    }
    ImGui::SameLine();
    if (ImGui::Button("Send")) {
        state.send_serial_text();
    }
    ImGui::EndDisabled();

    if (ImGui::Button("Clear TX")) {
        state.runtime.clear_serial_tx();
    }
    ImGui::SameLine();
    ImGui::Checkbox("Hex view", &state.serial_hex_view);

    ImGui::Separator();
    ImGui::TextUnformatted("TX");
    ImGui::BeginChild("serial_tx", ImVec2(0, 0), true, ImGuiWindowFlags_HorizontalScrollbar);
    const auto& tx = snapshot.serial_tx;
    if (state.serial_hex_view) {
        if (ImGui::BeginTable("serial_hex", 17, ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_RowBg)) {
            ImGui::TableSetupColumn("Offset");
            for (int i = 0; i < 16; ++i) {
                char label[4]{};
                std::snprintf(label, sizeof(label), "%X", i);
                ImGui::TableSetupColumn(label);
            }
            ImGui::TableHeadersRow();

            for (std::size_t row = 0; row < tx.size(); row += 16) {
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::Text("%04X", static_cast<unsigned>(row));
                for (std::size_t col = 0; col < 16; ++col) {
                    ImGui::TableNextColumn();
                    const std::size_t index = row + col;
                    if (index < tx.size()) {
                        ImGui::Text("%02X", tx[index]);
                    }
                }
            }
            ImGui::EndTable();
        }
    } else {
        const std::string text = serial_terminal_text(tx);
        ImGui::TextUnformatted(text.data(), text.data() + text.size());
    }
    ImGui::EndChild();
    ImGui::End();
}

void draw_log(GuiState& state) {
    set_next_window_defaults(944.0f, 780.0f, 480.0f, 86.0f);
    ImGui::Begin("Log", &state.show_log);
    const auto snapshot = state.runtime.debugger_snapshot();
    if (ImGui::Button("Clear")) {
        state.runtime.clear_log();
    }
    ImGui::Separator();
    for (const auto& line : snapshot.log) {
        ImGui::TextUnformatted(line.c_str());
    }
    if (ImGui::GetScrollY() >= ImGui::GetScrollMaxY()) {
        ImGui::SetScrollHereY(1.0f);
    }
    ImGui::End();
}

} // namespace microlind::gui
