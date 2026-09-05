#include <algorithm>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <string>

#include <SDL.h>
#include "imgui.h"
#include "imgui_impl_sdl2.h"
#include "imgui_impl_sdlrenderer2.h"

#include "gui_panels.hpp"
#include "gui_speaker.hpp"
#include "gui_state.hpp"
#include "gui_thread_names.hpp"

namespace {

using microlind::gui::GuiState;
using microlind::gui::PcSpeakerAudio;
using microlind::gui::draw_workbench;
using microlind::gui::handle_shortcut;
using microlind::gui::load_png_surface;
using microlind::gui::load_png_texture;

void apply_gui_theme(microlind::app::GuiTheme theme) {
    switch (theme) {
    case microlind::app::GuiTheme::Dark:
        ImGui::StyleColorsDark();
        break;
    case microlind::app::GuiTheme::Light:
        ImGui::StyleColorsLight();
        break;
    }
}

SDL_Color clear_color(microlind::app::GuiTheme theme) {
    switch (theme) {
    case microlind::app::GuiTheme::Dark: return SDL_Color{20, 22, 24, 255};
    case microlind::app::GuiTheme::Light: return SDL_Color{240, 240, 240, 255};
    }
    return SDL_Color{20, 22, 24, 255};
}

bool execution_active(microlind::gui::RuntimeMode mode) {
    using microlind::gui::RuntimeMode;
    switch (mode) {
    case RuntimeMode::DebugRun:
    case RuntimeMode::DebugMicroRun:
    case RuntimeMode::RunUntilAddress:
    case RuntimeMode::RunUntilReturn:
    case RuntimeMode::StepOverPending:
    case RuntimeMode::TrueRun:
        return true;
    default:
        return false;
    }
}

void update_pc_speaker(GuiState& state, PcSpeakerAudio& audio, double elapsed_seconds) {
    const auto status = state.runtime.status_snapshot();
    const auto parallel = state.runtime.parallel_snapshot();

    double frequency_hz = 0.0;
    const bool running = execution_active(status.mode);
    const bool timer_square_wave =
        parallel.present && parallel.pb7_timer_output_enabled && parallel.timer1_free_running &&
        parallel.timer1_running && (parallel.ddr_b & 0x80) != 0;

    if (running && status.mode == microlind::gui::RuntimeMode::TrueRun && timer_square_wave) {
        const double half_period_cycles = static_cast<double>(parallel.timer1_latch) + 1.0;
        frequency_hz = static_cast<double>(status.true_target_hz) / (2.0 * half_period_cycles);
    } else if (
        running && elapsed_seconds > 0.0 && status.total_cycles >= state.speaker_last_cycles &&
        parallel.pb7_transition_count >= state.speaker_last_transitions) {
        const uint64_t transitions = parallel.pb7_transition_count - state.speaker_last_transitions;
        frequency_hz = static_cast<double>(transitions) / (2.0 * elapsed_seconds);
    }

    state.speaker_frequency_hz = frequency_hz;
    state.speaker_signal_active = running && frequency_hz >= 20.0;
    state.speaker_last_cycles = status.total_cycles;
    state.speaker_last_transitions = parallel.pb7_transition_count;

    audio.set_tone(
        frequency_hz,
        state.speaker_volume,
        state.speaker_audio_available && state.speaker_signal_active && !state.speaker_muted);
}

void load_gui_fonts(ImGuiIO& io) {
    static constexpr ImWchar kGlyphRanges[] = {
        0x0020, 0x00FF,
        0x0192, 0x0192,
        0x0390, 0x03C9,
        0x207F, 0x20A7,
        0x2200, 0x22FF,
        0x2300, 0x23FF,
        0x2500, 0x259F,
        0,
    };

    static constexpr const char* kFontCandidates[] = {
        "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf",
        "/usr/share/fonts/dejavu/DejaVuSansMono.ttf",
        "/usr/share/fonts/TTF/DejaVuSansMono.ttf",
        "/usr/share/fonts/truetype/liberation2/LiberationMono-Regular.ttf",
        "C:/Windows/Fonts/consola.ttf",
        "/System/Library/Fonts/Supplemental/Andale Mono.ttf",
    };

    for (const char* path : kFontCandidates) {
        if (!std::filesystem::exists(path)) continue;
        if (io.Fonts->AddFontFromFileTTF(path, 15.0f, nullptr, kGlyphRanges) != nullptr) {
            return;
        }
    }

    io.Fonts->AddFontDefault();
}

int run_gui() {
    microlind::gui::set_current_thread_name("microlind-gui");

    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) != 0) {
        SDL_Log("SDL_Init failed: %s", SDL_GetError());
        return 1;
    }

    SDL_Window* window = SDL_CreateWindow(
        "microlind-sim GUI",
        SDL_WINDOWPOS_CENTERED,
        SDL_WINDOWPOS_CENTERED,
        1440,
        900,
        SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI);
    if (!window) {
        SDL_Log("SDL_CreateWindow failed: %s", SDL_GetError());
        SDL_Quit();
        return 1;
    }
    if (SDL_Surface* icon = load_png_surface("resources/mlsim_small.png")) {
        SDL_SetWindowIcon(window, icon);
        SDL_FreeSurface(icon);
    } else {
        SDL_Log("Could not load application icon: resources/mlsim_small.png");
    }

    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_PRESENTVSYNC | SDL_RENDERER_ACCELERATED);
    if (!renderer) {
        SDL_Log("SDL_CreateRenderer failed: %s", SDL_GetError());
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
#ifdef IMGUI_HAS_DOCK
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
#endif
    load_gui_fonts(io);

    ImGui_ImplSDL2_InitForSDLRenderer(window, renderer);
    ImGui_ImplSDLRenderer2_Init(renderer);

    GuiState state;
    state.renderer = renderer;
    PcSpeakerAudio speaker_audio;
    const bool audio_subsystem_initialized = SDL_InitSubSystem(SDL_INIT_AUDIO) == 0;
    if (audio_subsystem_initialized) {
        std::string error;
        state.speaker_audio_available = speaker_audio.start(error);
        if (!state.speaker_audio_available) {
            state.runtime.add_log("PC speaker audio unavailable: " + error);
        }
    } else {
        state.runtime.add_log(std::string("PC speaker audio unavailable: ") + SDL_GetError());
    }
    auto applied_theme = state.theme;
    apply_gui_theme(applied_theme);
    state.about_logo = load_png_texture(renderer, "resources/mlsim_logo.png");
    if (state.about_logo.texture == nullptr) {
        state.runtime.add_log("Could not load About logo: resources/mlsim_logo.png");
    }
    bool done = false;
    uint64_t last_counter = SDL_GetPerformanceCounter();
    double operation_budget = 0.0;

    while (!done) {
        const uint64_t now_counter = SDL_GetPerformanceCounter();
        const double elapsed_seconds =
            static_cast<double>(now_counter - last_counter) / static_cast<double>(SDL_GetPerformanceFrequency());
        last_counter = now_counter;

        SDL_Event event;
        while (SDL_PollEvent(&event) != 0) {
            ImGui_ImplSDL2_ProcessEvent(&event);
            if (event.type == SDL_QUIT) {
                done = true;
            }
            if (event.type == SDL_WINDOWEVENT && event.window.event == SDL_WINDOWEVENT_CLOSE &&
                event.window.windowID == SDL_GetWindowID(window)) {
                done = true;
            }
            if (event.type == SDL_KEYDOWN && event.key.repeat == 0 && !io.WantCaptureKeyboard) {
                handle_shortcut(state, event.key.keysym.sym, static_cast<SDL_Keymod>(event.key.keysym.mod));
            }
        }

        if (state.true_running()) {
            operation_budget = 0.0;
        } else {
            const bool timed_run_active = state.debug_run_active();
            if (timed_run_active) {
                operation_budget += elapsed_seconds * state.runtime.operations_per_second();
                operation_budget = std::min(operation_budget, 1000.0);
            } else {
                operation_budget = 0.0;
            }

            const auto operations_to_run = static_cast<uint32_t>(std::min(operation_budget, 1000.0));
            if (operations_to_run > 0) {
                operation_budget -= static_cast<double>(operations_to_run);
            }

            if (state.debug_run_active() && operations_to_run > 0) {
                state.runtime.run_debug_batch(operations_to_run);
                if (!state.debug_run_active()) {
                    operation_budget = 0.0;
                }
            }
        }

        if (state.theme != applied_theme) {
            applied_theme = state.theme;
            apply_gui_theme(applied_theme);
        }

        update_pc_speaker(state, speaker_audio, elapsed_seconds);

        ImGui_ImplSDLRenderer2_NewFrame();
        ImGui_ImplSDL2_NewFrame();
        ImGui::NewFrame();

        draw_workbench(state);
        if (state.quit_requested) {
            done = true;
        }

        ImGui::Render();
        const SDL_Color background = clear_color(applied_theme);
        SDL_SetRenderDrawColor(renderer, background.r, background.g, background.b, background.a);
        SDL_RenderClear(renderer);
        ImGui_ImplSDLRenderer2_RenderDrawData(ImGui::GetDrawData(), renderer);
        SDL_RenderPresent(renderer);
        if (state.true_running()) {
            SDL_Delay(1);
        }
    }

    ImGui_ImplSDLRenderer2_Shutdown();
    ImGui_ImplSDL2_Shutdown();
    ImGui::DestroyContext();
    speaker_audio.shutdown();
    if (audio_subsystem_initialized) {
        SDL_QuitSubSystem(SDL_INIT_AUDIO);
    }

    if (state.about_logo.texture != nullptr) {
        SDL_DestroyTexture(state.about_logo.texture);
        state.about_logo = {};
    }
    if (state.vdc_display.texture != nullptr) {
        SDL_DestroyTexture(state.vdc_display.texture);
        state.vdc_display = {};
    }
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}

} // namespace

int main() {
    try {
        return run_gui();
    } catch (const std::exception& ex) {
        SDL_Log("Unhandled exception: %s", ex.what());
        return 1;
    }
}
