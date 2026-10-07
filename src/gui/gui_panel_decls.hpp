#pragma once

#include "gui_state.hpp"

namespace microlind::gui {

void draw_file_panel(GuiState& state);
void draw_control_panel(GuiState& state);
void draw_registers(GuiState& state);
void draw_disassembly(GuiState& state);
void draw_memory_map(GuiState& state);
void draw_memory_viewer(GuiState& state);
void draw_stack(GuiState& state);
void draw_breakpoints(GuiState& state);
void draw_watchpoints(GuiState& state);
void draw_mapper(GuiState& state);
void draw_pld_logic(GuiState& state);
void draw_compact_flash(GuiState& state);
void draw_parallel(GuiState& state);
void draw_logic_analyser(GuiState& state);
void draw_vdc_display(GuiState& state);
void draw_trace(GuiState& state);
void draw_serial(GuiState& state);
void draw_log(GuiState& state);
void draw_main_menu(GuiState& state);
void draw_status_bar(GuiState& state);
void draw_about_modal(GuiState& state);

} // namespace microlind::gui
