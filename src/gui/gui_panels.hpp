#pragma once

#include <SDL.h>

#include "gui_state.hpp"

namespace microlind::gui {

void draw_workbench(GuiState& state);
void handle_shortcut(GuiState& state, SDL_Keycode key, SDL_Keymod mods);

} // namespace microlind::gui
