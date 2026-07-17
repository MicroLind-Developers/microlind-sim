#include "gui_panels.hpp"

#include <algorithm>
#include <cstdio>
#include <string>
#include <string_view>

#include "gui_panel_decls.hpp"
#include "help_document.hpp"

#include "imgui.h"

#ifndef MICROLIND_REPOSITORY_URL
#define MICROLIND_REPOSITORY_URL "https://github.com/MicroLind-Developers/microlind-sim"
#endif

namespace microlind::gui {
namespace {

float status_bar_height() {
    return ImGui::GetFrameHeight() + ImGui::GetStyle().WindowPadding.y * 2.0f;
}

#ifdef IMGUI_HAS_DOCK
void draw_dockspace(float bottom_reserved_height) {
    const ImGuiViewport* viewport = ImGui::GetMainViewport();
    const ImVec2 size(
        viewport->WorkSize.x,
        std::max(0.0f, viewport->WorkSize.y - bottom_reserved_height));

    ImGui::SetNextWindowPos(viewport->WorkPos);
    ImGui::SetNextWindowSize(size);
    ImGui::SetNextWindowViewport(viewport->ID);

    ImGuiWindowFlags host_window_flags =
        ImGuiWindowFlags_NoTitleBar |
        ImGuiWindowFlags_NoCollapse |
        ImGuiWindowFlags_NoResize |
        ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoDocking |
        ImGuiWindowFlags_NoBringToFrontOnFocus |
        ImGuiWindowFlags_NoNavFocus;

    char label[32]{};
    std::snprintf(label, sizeof(label), "WindowOverViewport_%08X", static_cast<unsigned>(viewport->ID));

    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));
    ImGui::Begin(label, nullptr, host_window_flags);
    ImGui::PopStyleVar(3);

    ImGui::DockSpace(ImGui::GetID("DockSpace"));
    ImGui::End();
}
#endif

std::string trim_markdown_line(std::string_view line) {
    const auto begin = line.find_first_not_of(" \t");
    if (begin == std::string_view::npos) return {};
    const auto end = line.find_last_not_of(" \t");
    return std::string(line.substr(begin, end - begin + 1));
}

bool starts_with(std::string_view value, std::string_view prefix) {
    return value.size() >= prefix.size() && value.substr(0, prefix.size()) == prefix;
}

void draw_markdown_line(std::string_view line) {
    if (line.empty()) {
        ImGui::Spacing();
        return;
    }

    if (starts_with(line, "# ")) {
        ImGui::TextUnformatted(trim_markdown_line(line.substr(2)).c_str());
        ImGui::Separator();
        return;
    }

    if (starts_with(line, "## ")) {
        ImGui::Spacing();
        ImGui::TextColored(ImVec4(0.34f, 0.68f, 0.95f, 1.0f), "%s", trim_markdown_line(line.substr(3)).c_str());
        ImGui::Separator();
        return;
    }

    if (starts_with(line, "- ")) {
        ImGui::BulletText("%s", trim_markdown_line(line.substr(2)).c_str());
        return;
    }

    if (starts_with(line, "  ")) {
        ImGui::Indent();
        ImGui::TextWrapped("%s", trim_markdown_line(line).c_str());
        ImGui::Unindent();
        return;
    }

    ImGui::TextWrapped("%s", trim_markdown_line(line).c_str());
}

void draw_markdown_document(std::string_view text) {
    std::size_t start = 0;
    while (start <= text.size()) {
        const std::size_t end = text.find('\n', start);
        std::string_view line = end == std::string_view::npos
                                    ? text.substr(start)
                                    : text.substr(start, end - start);
        if (!line.empty() && line.back() == '\r') {
            line.remove_suffix(1);
        }
        draw_markdown_line(line);
        if (end == std::string_view::npos) break;
        start = end + 1;
    }
}

} // namespace

void draw_main_menu(GuiState& state) {
    if (!ImGui::BeginMainMenuBar()) return;

    if (ImGui::BeginMenu("File")) {
#ifdef MICROLIND_HAS_PORTABLE_FILE_DIALOGS
        if (ImGui::MenuItem("Load Session...", "Ctrl+O")) {
            const auto path = pick_file("Load session", {"Session files", "*.session *.ini *.cfg", "All files", "*"});
            if (!path.empty()) {
                state.load_session_file(path);
            }
        }
#else
        if (ImGui::MenuItem("Load Session", "Ctrl+O")) {
            state.load_session_from_field();
        }
#endif
        if (ImGui::MenuItem("Load Example Session")) {
            state.load_session_file("examples/bios.session");
        }
        ImGui::Separator();
        if (ImGui::MenuItem("Save Session", "Ctrl+S")) {
            state.save_session_from_field();
        }
        if (ImGui::MenuItem("Save Session As...", "Ctrl+Shift+S")) {
            state.save_session_as();
        }
        ImGui::Separator();
        if (ImGui::MenuItem("Exit", "Ctrl+Q")) {
            state.quit_requested = true;
        }
        ImGui::EndMenu();
    }

    if (ImGui::BeginMenu("Simulator")) {
        if (ImGui::MenuItem("Reset", "Ctrl+R")) {
            state.stop_execution();
            state.runtime.reset();
        }
        if (ImGui::MenuItem(state.true_running() ? "Pause True Run" : "True Run")) {
            state.toggle_true_run();
        }
        ImGui::Separator();
        ImGui::BeginDisabled(state.true_running());
        if (ImGui::MenuItem(state.running() ? "Pause" : "Run", "F5")) {
            state.toggle_run();
        }
        if (ImGui::MenuItem("Step", "F10")) {
            state.step_once();
        }
        if (ImGui::MenuItem("Micro Step", "F9")) {
            state.step_microcycle();
        }
        if (ImGui::MenuItem("Step Over", "F11")) {
            state.step_over();
        }
        if (ImGui::MenuItem("Run Until Return", "Shift+F11")) {
            state.run_until_return();
        }
        ImGui::EndDisabled();
        ImGui::EndMenu();
    }

    if (ImGui::BeginMenu("View")) {
        if (ImGui::BeginMenu("Theme")) {
            bool dark = state.theme == microlind::app::GuiTheme::Dark;
            if (ImGui::MenuItem("Dark", nullptr, dark)) {
                state.theme = microlind::app::GuiTheme::Dark;
            }
            bool light = state.theme == microlind::app::GuiTheme::Light;
            if (ImGui::MenuItem("Light", nullptr, light)) {
                state.theme = microlind::app::GuiTheme::Light;
            }
            ImGui::EndMenu();
        }
        ImGui::Separator();
        if (ImGui::MenuItem("Show All")) {
            state.set_all_panels_visible(true);
        }
        if (ImGui::MenuItem("Hide All")) {
            state.set_all_panels_visible(false);
        }
        ImGui::Separator();
        ImGui::MenuItem("Files", nullptr, &state.show_file_panel);
        ImGui::MenuItem("Control", nullptr, &state.show_control_panel);
        ImGui::MenuItem("Registers", nullptr, &state.show_registers);
        ImGui::MenuItem("Disassembly", nullptr, &state.show_disassembly);
        ImGui::MenuItem("Memory", nullptr, &state.show_memory_viewer);
        ImGui::MenuItem("Stack", nullptr, &state.show_stack);
        ImGui::MenuItem("Trace", nullptr, &state.show_trace);
        ImGui::Separator();
        ImGui::MenuItem("Memory Map", nullptr, &state.show_memory_map);
        ImGui::MenuItem("Memory Mapper", nullptr, &state.show_mapper);
        ImGui::MenuItem("PLD Logic", nullptr, &state.show_pld_logic);
        ImGui::MenuItem("CompactFlash", nullptr, &state.show_compact_flash);
        ImGui::MenuItem("Parallel I/O", nullptr, &state.show_parallel);
        ImGui::MenuItem("Serial", nullptr, &state.show_serial);
        ImGui::Separator();
        ImGui::MenuItem("Breakpoints", nullptr, &state.show_breakpoints);
        ImGui::MenuItem("Watchpoints", nullptr, &state.show_watchpoints);
        ImGui::MenuItem("Log", nullptr, &state.show_log);
        ImGui::EndMenu();
    }

    if (ImGui::BeginMenu("Help")) {
        if (ImGui::MenuItem("Help")) {
            state.help_open = true;
        }
        if (ImGui::MenuItem("About")) {
            state.about_open = true;
        }
        ImGui::EndMenu();
    }

    ImGui::EndMainMenuBar();
}

void draw_status_bar(GuiState& state) {
    const ImGuiViewport* viewport = ImGui::GetMainViewport();
    const float height = status_bar_height();
    ImGui::SetNextWindowPos(ImVec2(viewport->WorkPos.x, viewport->WorkPos.y + viewport->WorkSize.y - height));
    ImGui::SetNextWindowSize(ImVec2(viewport->WorkSize.x, height));

    ImGuiWindowFlags flags =
        ImGuiWindowFlags_NoDecoration |
        ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoSavedSettings |
        ImGuiWindowFlags_NoNav |
        ImGuiWindowFlags_NoBringToFrontOnFocus;
#ifdef IMGUI_HAS_DOCK
    flags |= ImGuiWindowFlags_NoDocking;
#endif

    if (ImGui::Begin("Status Bar", nullptr, flags)) {
        const auto status = state.runtime.status_snapshot();
        const char* run_state = "paused";
        switch (status.mode) {
        case RuntimeMode::DebugRun: run_state = "running"; break;
        case RuntimeMode::DebugMicroRun: run_state = "running micro"; break;
        case RuntimeMode::RunUntilAddress: run_state = "until"; break;
        case RuntimeMode::RunUntilReturn: run_state = "until return"; break;
        case RuntimeMode::StepOverPending: run_state = "step over"; break;
        case RuntimeMode::StepPending: run_state = "step"; break;
        case RuntimeMode::MicroStepPending: run_state = "micro step"; break;
        case RuntimeMode::TrueRun: run_state = "true running"; break;
        case RuntimeMode::Stopping: run_state = "stopping"; break;
        case RuntimeMode::Paused:
            run_state = status.pending_microcycles ? "micro" : "paused";
            break;
        }
        ImGui::Text("State: %s", run_state);
        ImGui::SameLine();
        ImGui::TextUnformatted("|");
        ImGui::SameLine();
        ImGui::Text("PC: %04X", status.pc);
        ImGui::SameLine();
        ImGui::TextUnformatted("|");
        ImGui::SameLine();
        ImGui::Text("Cycles: %llu", static_cast<unsigned long long>(status.total_cycles));
        ImGui::SameLine();
        ImGui::TextUnformatted("|");
        ImGui::SameLine();
        ImGui::Text("Bus: %llu", static_cast<unsigned long long>(status.bus_cycles));
        if (state.true_running()) {
            ImGui::SameLine();
            ImGui::TextUnformatted("|");
            ImGui::SameLine();
            ImGui::Text(
                "True: %.1f/%.4f MHz",
                static_cast<double>(status.true_target_hz) / 1000000.0,
                status.true_effective_hz / 1000000.0);
        }
        if (status.pending_microcycles) {
            ImGui::SameLine();
            ImGui::TextUnformatted("|");
            ImGui::SameLine();
            ImGui::TextUnformatted("Pending microcycles");
        }
        ImGui::SameLine();
        ImGui::TextUnformatted("|");
        ImGui::SameLine();
        ImGui::Text("Session: %s", buffer_string(state.session_path).c_str());
    }
    ImGui::End();
}

void draw_help_modal(GuiState& state) {
    if (state.help_open) {
        ImGui::OpenPopup("Help");
        state.help_open = false;
    }

    ImGui::SetNextWindowSize(ImVec2(760.0f, 640.0f), ImGuiCond_Appearing);
    const ImGuiWindowFlags flags = ImGuiWindowFlags_NoSavedSettings;
    if (!ImGui::BeginPopupModal("Help", nullptr, flags)) return;

    const float footer_height = ImGui::GetFrameHeightWithSpacing() + ImGui::GetStyle().ItemSpacing.y;
    ImGui::BeginChild("help_document", ImVec2(0.0f, -footer_height), true);
    draw_markdown_document(help_document_text());
    ImGui::EndChild();

    ImGui::SetItemDefaultFocus();
    if (ImGui::Button("Close", ImVec2(96.0f, 0.0f))) {
        ImGui::CloseCurrentPopup();
    }

    ImGui::EndPopup();
}

void draw_about_modal(GuiState& state) {
    if (state.about_open) {
        ImGui::OpenPopup("About Microlind Simulator");
        state.about_open = false;
    }

    const ImGuiWindowFlags flags = ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoSavedSettings;
    if (!ImGui::BeginPopupModal("About Microlind Simulator", nullptr, flags)) return;

    if (state.about_logo.texture != nullptr) {
        const float logo_size = 96.0f;
        ImGui::Image(
            reinterpret_cast<ImTextureID>(state.about_logo.texture),
            ImVec2(logo_size, logo_size));
        ImGui::SameLine();
    }

    ImGui::BeginGroup();
    ImGui::TextUnformatted("Microlind Simulator");
    ImGui::TextDisabled("HD6309/MC6809 simulator and debugger");
    ImGui::Spacing();
    ImGui::TextWrapped(
        "The GUI runs beside the command-line simulator and provides an interactive debugger "
        "for loading sessions, inspecting CPU state, stepping code, watching memory, and "
        "observing mapped devices.");
    ImGui::EndGroup();

    ImGui::Separator();
    ImGui::TextUnformatted("Copyright and license");
    ImGui::TextUnformatted("Copyright (c) 2026 Eric Lind");
    ImGui::TextUnformatted("Microlind Simulator is released under the MIT license.");

    ImGui::Separator();
    ImGui::TextUnformatted("Libraries and tools used");
    ImGui::BulletText("Dear ImGui - immediate-mode GUI and docking/table widgets");
    ImGui::BulletText("SDL2 - desktop windowing, renderer, input, and texture backend");
    ImGui::BulletText("libpng - loading the application logo texture");
    ImGui::BulletText("portable-file-dialogs - native file open/save dialogs when available");
    ImGui::BulletText("GoogleTest / GoogleMock - automated simulator and app-layer tests");
    ImGui::BulletText("CMake - build configuration and dependency wiring");

    ImGui::Separator();

    ImGui::SetItemDefaultFocus();
    if (ImGui::Button("Close", ImVec2(96.0f, 0.0f))) {
        ImGui::CloseCurrentPopup();
    }
    ImGui::SameLine();
    if (ImGui::Button("Open GitHub", ImVec2(120.0f, 0.0f))) {
        if (SDL_OpenURL(MICROLIND_REPOSITORY_URL) != 0) {
            state.runtime.add_log(std::string("Could not open GitHub URL: ") + SDL_GetError());
        }
    }

    ImGui::EndPopup();
}

void draw_workbench(GuiState& state) {
    if (!state.pending_layout_ini.empty()) {
        ImGui::LoadIniSettingsFromMemory(state.pending_layout_ini.data(), state.pending_layout_ini.size());
        state.pending_layout_ini.clear();
        state.runtime.add_log("Restored session window layout.");
    }

    draw_main_menu(state);
#ifdef IMGUI_HAS_DOCK
    draw_dockspace(status_bar_height());
#endif
    draw_help_modal(state);
    draw_about_modal(state);

    if (state.show_control_panel) draw_control_panel(state);
    if (state.show_serial) draw_serial(state);
    if (!state.true_running()) {
        if (state.show_file_panel) draw_file_panel(state);
        if (state.show_registers) draw_registers(state);
        if (state.show_disassembly) draw_disassembly(state);
        if (state.show_memory_viewer) draw_memory_viewer(state);
        if (state.show_stack) draw_stack(state);
        if (state.show_memory_map) draw_memory_map(state);
        if (state.show_mapper) draw_mapper(state);
        if (state.show_pld_logic) draw_pld_logic(state);
        if (state.show_compact_flash) draw_compact_flash(state);
        if (state.show_parallel) draw_parallel(state);
        if (state.show_breakpoints) draw_breakpoints(state);
        if (state.show_watchpoints) draw_watchpoints(state);
        if (state.show_trace) draw_trace(state);
        if (state.show_log) draw_log(state);
    }
    draw_status_bar(state);
}

void handle_shortcut(GuiState& state, SDL_Keycode key, SDL_Keymod mods) {
    const bool ctrl = (mods & KMOD_CTRL) != 0;
    const bool shift = (mods & KMOD_SHIFT) != 0;

    if (ctrl && key == SDLK_o) {
        state.load_session_from_field();
    } else if (ctrl && shift && key == SDLK_s) {
        state.save_session_as();
    } else if (ctrl && key == SDLK_s) {
        state.save_session_from_field();
    } else if (ctrl && key == SDLK_q) {
        state.quit_requested = true;
    } else if (ctrl && key == SDLK_r) {
        state.stop_execution();
        state.runtime.reset();
    } else if (key == SDLK_F5) {
        state.toggle_run();
    } else if (key == SDLK_F9) {
        state.step_microcycle();
    } else if (key == SDLK_F10) {
        state.step_once();
    } else if (shift && key == SDLK_F11) {
        state.run_until_return();
    } else if (key == SDLK_F11) {
        state.step_over();
    }
}

} // namespace microlind::gui
