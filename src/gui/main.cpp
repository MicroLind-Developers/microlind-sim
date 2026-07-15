#include <algorithm>
#include <cstdint>
#include <exception>

#include <SDL.h>
#include "imgui.h"
#include "imgui_impl_sdl2.h"
#include "imgui_impl_sdlrenderer2.h"

#include "gui_panels.hpp"
#include "gui_state.hpp"

namespace {

using microlind::gui::GuiState;
using microlind::gui::draw_workbench;
using microlind::gui::handle_shortcut;
using microlind::gui::load_png_surface;
using microlind::gui::load_png_texture;

int run_gui() {
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
    (void)io;
    ImGui::StyleColorsDark();

    ImGui_ImplSDL2_InitForSDLRenderer(window, renderer);
    ImGui_ImplSDLRenderer2_Init(renderer);

    GuiState state;
    state.about_logo = load_png_texture(renderer, "resources/mlsim_logo.png");
    if (state.about_logo.texture == nullptr) {
        state.session.add_log("Could not load About logo: resources/mlsim_logo.png");
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

        const bool timed_run_active = state.run_until_active || state.running;
        if (timed_run_active) {
            operation_budget += elapsed_seconds * state.operations_per_second();
            operation_budget = std::min(operation_budget, 1000.0);
        } else {
            operation_budget = 0.0;
        }

        const auto operations_to_run = static_cast<uint32_t>(std::min(operation_budget, 1000.0));
        if (operations_to_run > 0) {
            operation_budget -= static_cast<double>(operations_to_run);
        }

        if (state.run_until_active && operations_to_run > 0) {
            const auto result = state.session.run_until_address(
                static_cast<uint16_t>(state.run_until_address),
                operations_to_run);
            if (result.hit_target || result.hit_breakpoint || result.hit_watchpoint) {
                state.run_until_active = false;
                operation_budget = 0.0;
            }
        } else if (state.running && operations_to_run > 0) {
            const auto result = state.run_micro_steps
                ? state.session.run_microcycles(operations_to_run)
                : state.session.run_instructions(operations_to_run);
            if (result.hit_breakpoint || result.hit_watchpoint) {
                state.running = false;
                operation_budget = 0.0;
            }
        }

        ImGui_ImplSDLRenderer2_NewFrame();
        ImGui_ImplSDL2_NewFrame();
        ImGui::NewFrame();

        draw_workbench(state);
        if (state.quit_requested) {
            done = true;
        }

        ImGui::Render();
        SDL_SetRenderDrawColor(renderer, 20, 22, 24, 255);
        SDL_RenderClear(renderer);
        ImGui_ImplSDLRenderer2_RenderDrawData(ImGui::GetDrawData(), renderer);
        SDL_RenderPresent(renderer);
    }

    ImGui_ImplSDLRenderer2_Shutdown();
    ImGui_ImplSDL2_Shutdown();
    ImGui::DestroyContext();

    if (state.about_logo.texture != nullptr) {
        SDL_DestroyTexture(state.about_logo.texture);
        state.about_logo = {};
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
