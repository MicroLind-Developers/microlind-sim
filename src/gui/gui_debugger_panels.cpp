#include "gui_panel_decls.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cfloat>
#include <cstdint>
#include <iterator>
#include <string>

#include "imgui.h"

#include "microlind/cpu.hpp"

namespace microlind::gui {

namespace {

constexpr uint8_t kFlagBits[] = {
    microlind::CC_E,
    microlind::CC_F,
    microlind::CC_H,
    microlind::CC_I,
    microlind::CC_N,
    microlind::CC_Z,
    microlind::CC_V,
    microlind::CC_C,
};

constexpr const char* kFlagNames[] = {"E", "F", "H", "I", "N", "Z", "V", "C"};

void draw_register_row(const char* name, uint32_t value, int width) {
    ImGui::TableNextRow();
    ImGui::TableNextColumn();
    ImGui::TextUnformatted(name);
    ImGui::TableNextColumn();
    const std::string text = hex_value(value, width);
    ImGui::TextUnformatted(text.c_str());
}

} // namespace

void draw_registers(GuiState& state) {
    const auto snapshot = state.runtime.debugger_snapshot();
    const auto& regs = snapshot.regs;

    set_next_window_defaults(8.0f, 376.0f, 240.0f, 360.0f);
    ImGui::Begin("Registers", &state.show_registers);
    if (ImGui::BeginTable("registers", 2, ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_RowBg)) {
        draw_register_row("PC", regs.pc, 4);
        draw_register_row("A", regs.a, 2);
        draw_register_row("B", regs.b, 2);
        draw_register_row("E", regs.e, 2);
        draw_register_row("F", regs.f, 2);
        draw_register_row("DP", regs.dp, 2);
        draw_register_row("CC", regs.cc, 2);
        draw_register_row("MD", regs.md, 2);
        draw_register_row("X", regs.x, 4);
        draw_register_row("Y", regs.y, 4);
        draw_register_row("U", regs.u, 4);
        draw_register_row("S", regs.s, 4);
        draw_register_row("V", regs.v, 4);
        ImGui::EndTable();
    }

    ImGui::Separator();
    ImGui::TextUnformatted("Flags");
    for (std::size_t i = 0; i < std::size(kFlagBits); ++i) {
        bool active = (regs.cc & kFlagBits[i]) != 0;
        ImGui::BeginDisabled();
        ImGui::Checkbox(kFlagNames[i], &active);
        ImGui::EndDisabled();
        if (i + 1 < std::size(kFlagBits)) {
            ImGui::SameLine();
        }
    }

    ImGui::Separator();
    ImGui::TextUnformatted("Clock");
    ImGui::Text("Cycles: %llu", static_cast<unsigned long long>(snapshot.total_cycles));
    ImGui::Text("Clock: %llu Hz", static_cast<unsigned long long>(snapshot.clock_hz));
    ImGui::End();
}

void draw_disassembly(GuiState& state) {
    set_next_window_defaults(376.0f, 186.0f, 560.0f, 340.0f);
    ImGui::Begin("Disassembly", &state.show_disassembly);
    const auto lines = state.runtime.disassembly_snapshot(18);

    if (ImGui::BeginTable("disassembly", 3, ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_RowBg | ImGuiTableFlags_Resizable)) {
        ImGui::TableSetupColumn("Address", ImGuiTableColumnFlags_WidthFixed, 72.0f);
        ImGui::TableSetupColumn("Bytes", ImGuiTableColumnFlags_WidthFixed, 160.0f);
        ImGui::TableSetupColumn("Disassembly");
        ImGui::TableHeadersRow();

        for (const auto& line : lines) {
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            if (line.current) {
                ImGui::TextColored(ImVec4(0.3f, 0.9f, 0.6f, 1.0f), "%04X", line.address);
            } else {
                ImGui::Text("%04X", line.address);
            }

            ImGui::TableNextColumn();
            if (line.current) {
                ImGui::TextColored(ImVec4(0.3f, 0.9f, 0.6f, 1.0f), "%s", line.bytes.c_str());
            } else {
                ImGui::TextUnformatted(line.bytes.c_str());
            }

            ImGui::TableNextColumn();
            if (line.current) {
                ImGui::TextColored(ImVec4(0.3f, 0.9f, 0.6f, 1.0f), "%s", line.text.c_str());
            } else {
                ImGui::TextUnformatted(line.text.c_str());
            }
        }
        ImGui::EndTable();
    }
    ImGui::End();
}

void draw_stack(GuiState& state) {
    const auto snapshot = state.runtime.debugger_snapshot();
    const auto& regs = snapshot.regs;
    const uint16_t stack_pointer = state.stack_register_index == 0 ? regs.s : regs.u;

    set_next_window_defaults(1188.0f, 28.0f, 236.0f, 490.0f);
    ImGui::Begin("Stack", &state.show_stack);

    const char* stack_names[] = {"S", "U"};
    ImGui::Combo("Register", &state.stack_register_index, stack_names, static_cast<int>(std::size(stack_names)));
    ImGui::Checkbox("Follow SP", &state.stack_follow_pointer);
    ImGui::SliderInt("Rows", &state.stack_rows, 8, 128);

    if (state.stack_follow_pointer) {
        state.stack_start = stack_pointer;
    } else {
        ImGui::InputInt("Start", &state.stack_start, 16, 256, ImGuiInputTextFlags_CharsHexadecimal);
        state.stack_start = std::clamp(state.stack_start, 0, 0xFFFF);
    }

    ImGui::Text("%s = %04X", stack_names[state.stack_register_index], stack_pointer);

    const auto stack_rows = state.runtime.stack_snapshot(static_cast<uint16_t>(state.stack_start), state.stack_rows, state.stack_register_index);
    if (ImGui::BeginTable("stack", 6, ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_RowBg | ImGuiTableFlags_Resizable)) {
        ImGui::TableSetupColumn("");
        ImGui::TableSetupColumn("Address");
        ImGui::TableSetupColumn("+0");
        ImGui::TableSetupColumn("+1");
        ImGui::TableSetupColumn("Word");
        ImGui::TableSetupColumn("ASCII");
        ImGui::TableHeadersRow();

        for (const auto& row : stack_rows) {
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            if (row.at_pointer) {
                ImGui::TextColored(ImVec4(0.3f, 0.9f, 0.6f, 1.0f), "SP");
            } else {
                ImGui::TextUnformatted("");
            }

            ImGui::TableNextColumn();
            if (row.at_pointer) {
                ImGui::TextColored(ImVec4(0.3f, 0.9f, 0.6f, 1.0f), "%04X", row.address);
            } else {
                ImGui::Text("%04X", row.address);
            }

            ImGui::TableNextColumn();
            ImGui::Text("%02X", row.high);
            ImGui::TableNextColumn();
            ImGui::Text("%02X", row.low);
            ImGui::TableNextColumn();
            ImGui::Text("%04X", row.word);
            ImGui::TableNextColumn();
            const char c0 = std::isprint(static_cast<unsigned char>(row.high)) ? static_cast<char>(row.high) : '.';
            const char c1 = std::isprint(static_cast<unsigned char>(row.low)) ? static_cast<char>(row.low) : '.';
            ImGui::Text("%c%c", c0, c1);
        }

        ImGui::EndTable();
    }

    ImGui::End();
}

void draw_breakpoints(GuiState& state) {
    set_next_window_defaults(252.0f, 376.0f, 360.0f, 220.0f);
    ImGui::Begin("Breakpoints", &state.show_breakpoints);
    ImGui::InputInt("Address", &state.breakpoint_address, 1, 256, ImGuiInputTextFlags_CharsHexadecimal);
    state.breakpoint_address = std::clamp(state.breakpoint_address, 0, 0xFFFF);
    ImGui::InputText("Label", state.breakpoint_label.data(), state.breakpoint_label.size());

    if (ImGui::Button("Add")) {
        if (state.runtime.add_breakpoint(static_cast<uint16_t>(state.breakpoint_address), buffer_string(state.breakpoint_label))) {
            set_buffer(state.breakpoint_label, "");
        }
    }
    ImGui::SameLine();
    if (ImGui::Button("Remove")) {
        state.runtime.remove_breakpoint(static_cast<uint16_t>(state.breakpoint_address));
    }
    ImGui::SameLine();
    if (ImGui::Button("Clear")) {
        state.runtime.clear_breakpoints();
        state.runtime.add_log("Cleared breakpoints.");
    }

    ImGui::Separator();
    if (ImGui::BeginTable("breakpoints", 5, ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_RowBg | ImGuiTableFlags_Resizable)) {
        ImGui::TableSetupColumn("On", ImGuiTableColumnFlags_WidthFixed, 34.0f);
        ImGui::TableSetupColumn("Address", ImGuiTableColumnFlags_WidthFixed, 70.0f);
        ImGui::TableSetupColumn("Hits", ImGuiTableColumnFlags_WidthFixed, 56.0f);
        ImGui::TableSetupColumn("Label");
        ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthFixed, 64.0f);
        ImGui::TableHeadersRow();

        const auto snapshot = state.runtime.debugger_snapshot();
        for (const auto& breakpoint : snapshot.breakpoints) {
            ImGui::PushID(static_cast<int>(breakpoint.address));
            ImGui::TableNextRow();

            ImGui::TableNextColumn();
            bool enabled = breakpoint.enabled;
            if (ImGui::Checkbox("##enabled", &enabled)) {
                state.runtime.set_breakpoint_enabled(breakpoint.address, enabled);
            }

            ImGui::TableNextColumn();
            ImGui::Text("%04X", breakpoint.address);

            ImGui::TableNextColumn();
            ImGui::Text("%llu", static_cast<unsigned long long>(breakpoint.hits));

            ImGui::TableNextColumn();
            std::array<char, 128> label{};
            set_buffer(label, breakpoint.label);
            ImGui::SetNextItemWidth(-FLT_MIN);
            if (ImGui::InputText("##label", label.data(), label.size())) {
                state.runtime.set_breakpoint_label(breakpoint.address, buffer_string(label));
            }

            ImGui::TableNextColumn();
            if (ImGui::SmallButton("Remove")) {
                state.runtime.remove_breakpoint(breakpoint.address);
                ImGui::PopID();
                break;
            }
            ImGui::PopID();
        }
        ImGui::EndTable();
    }
    ImGui::End();
}

void draw_watchpoints(GuiState& state) {
    set_next_window_defaults(252.0f, 582.0f, 400.0f, 240.0f);
    ImGui::Begin("Watchpoints", &state.show_watchpoints);
    ImGui::InputInt("Address", &state.watchpoint_address, 1, 256, ImGuiInputTextFlags_CharsHexadecimal);
    state.watchpoint_address = std::clamp(state.watchpoint_address, 0, 0xFFFF);
    ImGui::InputText("Label", state.watchpoint_label.data(), state.watchpoint_label.size());

    if (ImGui::Button("Add Read")) {
        if (state.runtime.add_watchpoint(
                static_cast<uint16_t>(state.watchpoint_address),
                microlind::app::WatchpointType::Read,
                buffer_string(state.watchpoint_label))) {
            set_buffer(state.watchpoint_label, "");
        }
    }
    ImGui::SameLine();
    if (ImGui::Button("Add Write")) {
        if (state.runtime.add_watchpoint(
                static_cast<uint16_t>(state.watchpoint_address),
                microlind::app::WatchpointType::Write,
                buffer_string(state.watchpoint_label))) {
            set_buffer(state.watchpoint_label, "");
        }
    }
    ImGui::SameLine();
    if (ImGui::Button("Clear")) {
        state.runtime.clear_watchpoints();
        state.runtime.add_log("Cleared watchpoints.");
    }

    ImGui::Separator();
    if (ImGui::BeginTable("watchpoints", 6, ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_RowBg | ImGuiTableFlags_Resizable)) {
        ImGui::TableSetupColumn("On", ImGuiTableColumnFlags_WidthFixed, 34.0f);
        ImGui::TableSetupColumn("Address", ImGuiTableColumnFlags_WidthFixed, 70.0f);
        ImGui::TableSetupColumn("Type", ImGuiTableColumnFlags_WidthFixed, 48.0f);
        ImGui::TableSetupColumn("Hits", ImGuiTableColumnFlags_WidthFixed, 56.0f);
        ImGui::TableSetupColumn("Label");
        ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthFixed, 64.0f);
        ImGui::TableHeadersRow();

        const auto snapshot = state.runtime.debugger_snapshot();
        for (const auto& watchpoint : snapshot.watchpoints) {
            ImGui::PushID(static_cast<int>(watchpoint.address));
            ImGui::TableNextRow();

            ImGui::TableNextColumn();
            bool enabled = watchpoint.enabled;
            if (ImGui::Checkbox("##enabled", &enabled)) {
                state.runtime.set_watchpoint_enabled(watchpoint.address, enabled);
            }

            ImGui::TableNextColumn();
            ImGui::Text("%04X", watchpoint.address);

            ImGui::TableNextColumn();
            ImGui::TextUnformatted(watchpoint_type_label(watchpoint.type));

            ImGui::TableNextColumn();
            ImGui::Text("%llu", static_cast<unsigned long long>(watchpoint.hits));

            ImGui::TableNextColumn();
            std::array<char, 128> label{};
            set_buffer(label, watchpoint.label);
            ImGui::SetNextItemWidth(-FLT_MIN);
            if (ImGui::InputText("##label", label.data(), label.size())) {
                state.runtime.set_watchpoint_label(watchpoint.address, buffer_string(label));
            }

            ImGui::TableNextColumn();
            if (ImGui::SmallButton("Remove")) {
                state.runtime.remove_watchpoint(watchpoint.address);
                ImGui::PopID();
                break;
            }
            ImGui::PopID();
        }
        ImGui::EndTable();
    }
    ImGui::End();
}

void draw_trace(GuiState& state) {
    set_next_window_defaults(480.0f, 706.0f, 456.0f, 160.0f);
    ImGui::Begin("Trace", &state.show_trace);
    const auto snapshot = state.runtime.debugger_snapshot();
    const auto& trace = snapshot.trace;
    if (ImGui::Button("Clear")) {
        state.runtime.clear_trace();
    }
    ImGui::SameLine();
    ImGui::Text("Entries: %llu", static_cast<unsigned long long>(trace.size()));
    ImGui::Separator();

    ImGui::BeginChild("trace_scroll", ImVec2(0, 0), false, ImGuiWindowFlags_HorizontalScrollbar);
    if (ImGui::BeginTable("trace", 4, ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_RowBg | ImGuiTableFlags_Resizable)) {
        ImGui::TableSetupColumn("PC");
        ImGui::TableSetupColumn("Instruction");
        ImGui::TableSetupColumn("Cycles");
        ImGui::TableSetupColumn("Total");
        ImGui::TableHeadersRow();

        for (auto it = trace.rbegin(); it != trace.rend(); ++it) {
            const auto& entry = *it;
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::Text("%04X", entry.pc);
            ImGui::TableNextColumn();
            ImGui::TextUnformatted(entry.instruction.c_str());
            ImGui::TableNextColumn();
            ImGui::Text("%u", entry.cycles);
            ImGui::TableNextColumn();
            ImGui::Text("%llu", static_cast<unsigned long long>(entry.total_cycles));
        }
        ImGui::EndTable();
    }
    ImGui::EndChild();
    ImGui::End();
}

} // namespace microlind::gui
