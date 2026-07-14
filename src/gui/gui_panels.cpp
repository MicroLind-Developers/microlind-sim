#include "gui_panels.hpp"

#include <string>

#include "gui_panel_decls.hpp"

#include "imgui.h"

namespace microlind::gui {

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
            state.session.reset();
        }
        if (ImGui::MenuItem(state.running ? "Pause" : "Run", "F5")) {
            state.toggle_run();
        }
        if (ImGui::MenuItem("Step", "F10")) {
            state.step_once();
        }
        if (ImGui::MenuItem("Step Over", "F11")) {
            state.step_over();
        }
        if (ImGui::MenuItem("Run Until Return", "Shift+F11")) {
            state.run_until_return();
        }
        ImGui::EndMenu();
    }

    if (ImGui::BeginMenu("Help")) {
        if (ImGui::MenuItem("About")) {
            state.about_open = true;
        }
        ImGui::EndMenu();
    }

    ImGui::EndMainMenuBar();
}

void draw_status_bar(GuiState& state) {
    const ImGuiViewport* viewport = ImGui::GetMainViewport();
    const float height = ImGui::GetFrameHeight() + ImGui::GetStyle().WindowPadding.y * 2.0f;
    ImGui::SetNextWindowPos(ImVec2(viewport->WorkPos.x, viewport->WorkPos.y + viewport->WorkSize.y - height));
    ImGui::SetNextWindowSize(ImVec2(viewport->WorkSize.x, height));

    constexpr ImGuiWindowFlags flags =
        ImGuiWindowFlags_NoDecoration |
        ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoSavedSettings |
        ImGuiWindowFlags_NoNav |
        ImGuiWindowFlags_NoBringToFrontOnFocus;

    if (ImGui::Begin("Status Bar", nullptr, flags)) {
        const char* run_state = state.run_until_active ? "until" : (state.running ? "running" : "paused");
        const auto& sim = state.session.simulator();
        ImGui::Text("State: %s", run_state);
        ImGui::SameLine();
        ImGui::TextUnformatted("|");
        ImGui::SameLine();
        ImGui::Text("PC: %04X", sim.cpu().regs().pc);
        ImGui::SameLine();
        ImGui::TextUnformatted("|");
        ImGui::SameLine();
        ImGui::Text("Cycles: %llu", static_cast<unsigned long long>(sim.clock().total_cycles()));
        ImGui::SameLine();
        ImGui::TextUnformatted("|");
        ImGui::SameLine();
        ImGui::Text("Session: %s", buffer_string(state.session_path).c_str());
    }
    ImGui::End();
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
    ImGui::TextUnformatted("Included GUI features");
    ImGui::BulletText("Session loading and saving with persisted layout/debugger state");
    ImGui::BulletText("Run, pause, reset, step, step-over, and run-until-return controls");
    ImGui::BulletText("Registers, disassembly, editable memory, stack, trace, serial, mapper, and CompactFlash panels");
    ImGui::BulletText("Breakpoints and watchpoints with labels, enabled state, and hit counts");

    ImGui::Separator();
    ImGui::TextUnformatted("Libraries and tools used");
    ImGui::BulletText("Dear ImGui - immediate-mode GUI and docking/table widgets");
    ImGui::BulletText("SDL2 - desktop windowing, renderer, input, and texture backend");
    ImGui::BulletText("libpng - loading the application logo texture");
    ImGui::BulletText("portable-file-dialogs - native file open/save dialogs when available");
    ImGui::BulletText("GoogleTest / GoogleMock - automated simulator and app-layer tests");
    ImGui::BulletText("CMake - build configuration and dependency wiring");

    ImGui::Separator();
    ImGui::TextDisabled("Logo: resources/mlsim_small.png");

    if (ImGui::Button("Open GitHub", ImVec2(120.0f, 0.0f))) {
        constexpr const char* kRepositoryUrl = "https://github.com/MicroLind-Developers/microlind-sim";
        if (SDL_OpenURL(kRepositoryUrl) != 0) {
            state.session.add_log(std::string("Could not open GitHub URL: ") + SDL_GetError());
        }
    }
    ImGui::SameLine();
    ImGui::SetItemDefaultFocus();
    if (ImGui::Button("Close", ImVec2(96.0f, 0.0f))) {
        ImGui::CloseCurrentPopup();
    }

    ImGui::EndPopup();
}

void draw_workbench(GuiState& state) {
    if (!state.pending_layout_ini.empty()) {
        ImGui::LoadIniSettingsFromMemory(state.pending_layout_ini.data(), state.pending_layout_ini.size());
        state.pending_layout_ini.clear();
        state.session.add_log("Restored session window layout.");
    }

#ifdef IMGUI_HAS_DOCK
    ImGui::DockSpaceOverViewport(0, ImGui::GetMainViewport());
#endif

    draw_main_menu(state);
    draw_about_modal(state);

    draw_file_panel(state);
    draw_control_panel(state);
    draw_registers(state);
    draw_disassembly(state);
    draw_memory_viewer(state);
    draw_stack(state);
    draw_memory_map(state);
    draw_mapper(state);
    draw_compact_flash(state);
    draw_breakpoints(state);
    draw_watchpoints(state);
    draw_trace(state);
    draw_serial(state);
    draw_log(state);
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
        state.session.reset();
    } else if (key == SDLK_F5) {
        state.toggle_run();
    } else if (key == SDLK_F10) {
        state.step_once();
    } else if (shift && key == SDLK_F11) {
        state.run_until_return();
    } else if (key == SDLK_F11) {
        state.step_over();
    }
}

} // namespace microlind::gui
