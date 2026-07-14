#include "gui_panel_decls.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

#include "imgui.h"

namespace microlind::gui {

void draw_memory_map(const GuiState& state) {
    set_next_window_defaults(944.0f, 28.0f, 240.0f, 190.0f);
    ImGui::Begin("Memory Map");
    const std::vector<std::string> summary = state.session.memory_map();
    if (summary.empty()) {
        ImGui::TextDisabled("No mapped devices.");
    } else {
        for (const auto& line : summary) {
            ImGui::TextUnformatted(line.c_str());
        }
    }
    ImGui::End();
}

void draw_memory_viewer(GuiState& state) {
    set_next_window_defaults(944.0f, 528.0f, 560.0f, 260.0f);
    ImGui::Begin("Memory");

    auto& sim = state.session.simulator();
    const auto& regs = sim.cpu().regs();

    if (state.memory_follow_pc) {
        state.memory_start = regs.pc & 0xFFF0;
    }

    ImGui::SetNextItemWidth(96.0f);
    ImGui::InputInt("Start", &state.memory_start, 16, 256, ImGuiInputTextFlags_CharsHexadecimal);
    state.memory_start = std::clamp(state.memory_start, 0, 0xFFFF);
    state.memory_start &= 0xFFF0;

    ImGui::SameLine();
    ImGui::SetNextItemWidth(86.0f);
    ImGui::SliderInt("Rows", &state.memory_rows, 4, 64);

    ImGui::SameLine();
    ImGui::Checkbox("Follow PC", &state.memory_follow_pc);

    ImGui::SameLine();
    if (ImGui::Button("PC")) {
        state.memory_follow_pc = false;
        state.memory_start = regs.pc & 0xFFF0;
    }
    ImGui::SameLine();
    if (ImGui::Button("S")) {
        state.memory_follow_pc = false;
        state.memory_start = regs.s & 0xFFF0;
    }
    ImGui::SameLine();
    if (ImGui::Button("U")) {
        state.memory_follow_pc = false;
        state.memory_start = regs.u & 0xFFF0;
    }

    ImGui::SameLine();
    if (ImGui::ArrowButton("mem_prev", ImGuiDir_Left)) {
        state.memory_follow_pc = false;
        state.memory_start = std::clamp(state.memory_start - 0x100, 0, 0xFFFF) & 0xFFF0;
    }
    ImGui::SameLine();
    if (ImGui::ArrowButton("mem_next", ImGuiDir_Right)) {
        state.memory_follow_pc = false;
        state.memory_start = std::clamp(state.memory_start + 0x100, 0, 0xFFFF) & 0xFFF0;
    }

    constexpr int kCols = 16;
    const ImGuiTableFlags flags = ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_RowBg |
                                  ImGuiTableFlags_Resizable | ImGuiTableFlags_ScrollY |
                                  ImGuiTableFlags_SizingFixedFit;
    const float line_height = ImGui::GetTextLineHeightWithSpacing();
    const float table_height = std::max(line_height * 6.0f, ImGui::GetContentRegionAvail().y);
    if (ImGui::BeginTable("memory_view", kCols + 2, flags, ImVec2(0.0f, table_height))) {
        ImGui::TableSetupScrollFreeze(1, 1);
        ImGui::TableSetupColumn("Address", ImGuiTableColumnFlags_WidthFixed, 64.0f);
        for (int col = 0; col < kCols; ++col) {
            char label[4]{};
            std::snprintf(label, sizeof(label), "%X", col);
            ImGui::TableSetupColumn(
                label,
                ImGuiTableColumnFlags_WidthFixed | ImGuiTableColumnFlags_NoResize,
                32.0f);
        }
        ImGui::TableSetupColumn("ASCII", ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableHeadersRow();

        for (int row = 0; row < state.memory_rows; ++row) {
            ImGui::TableNextRow();
            const uint16_t row_address = static_cast<uint16_t>(state.memory_start + row * kCols);
            ImGui::TableNextColumn();
            ImGui::Text("%04X", row_address);

            std::array<char, kCols + 1> ascii{};
            for (int col = 0; col < kCols; ++col) {
                ImGui::TableNextColumn();
                const uint16_t address = static_cast<uint16_t>(row_address + col);
                uint8_t value = state.session.peek_memory(address);
                ascii[static_cast<std::size_t>(col)] =
                    std::isprint(static_cast<unsigned char>(value)) ? static_cast<char>(value) : '.';

                const bool at_pc = address == regs.pc;
                const bool at_s = address == regs.s;
                const bool at_u = address == regs.u;
                const bool watched = state.session.is_watchpoint(address, microlind::app::WatchpointType::Read) ||
                                     state.session.is_watchpoint(address, microlind::app::WatchpointType::Write);
                if (at_pc) {
                    ImGui::TableSetBgColor(ImGuiTableBgTarget_CellBg, IM_COL32(38, 96, 56, 180));
                } else if (at_s || at_u) {
                    ImGui::TableSetBgColor(ImGuiTableBgTarget_CellBg, IM_COL32(72, 72, 116, 180));
                } else if (watched) {
                    ImGui::TableSetBgColor(ImGuiTableBgTarget_CellBg, IM_COL32(120, 92, 38, 180));
                }

                ImGui::PushID(static_cast<int>(address));
                ImGui::SetNextItemWidth(30.0f);
                if (ImGui::InputScalar(
                        "##byte",
                        ImGuiDataType_U8,
                        &value,
                        nullptr,
                        nullptr,
                        "%02X",
                        ImGuiInputTextFlags_CharsHexadecimal | ImGuiInputTextFlags_AutoSelectAll)) {
                    state.session.write_memory(address, value);
                    state.session.add_log("Wrote " + hex_value(value, 2) + " to " + hex_value(address, 4) + ".");
                }
                ImGui::PopID();
            }

            ImGui::TableNextColumn();
            ImGui::TextUnformatted(ascii.data());
        }
        ImGui::EndTable();
    }

    ImGui::End();
}

void draw_mapper(GuiState& state) {
    set_next_window_defaults(944.0f, 224.0f, 240.0f, 294.0f);
    ImGui::Begin("Memory Mapper");
    const auto mapper = state.session.mapper_snapshot();
    if (!mapper.present) {
        ImGui::TextDisabled("No memory mapper configured.");
        ImGui::End();
        return;
    }

    ImGui::Text("Bank size: %u", mapper.bank_size);
    ImGui::Text("Backing RAM: %u", mapper.available);

    if (ImGui::BeginTable("mapper_regs", 3, ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_RowBg)) {
        ImGui::TableSetupColumn("Window");
        ImGui::TableSetupColumn("Register");
        ImGui::TableSetupColumn("Selected bank");
        ImGui::TableHeadersRow();
        for (int i = 0; i < 4; ++i) {
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::Text("%d", i);
            ImGui::TableNextColumn();
            if (mapper.bank_registers[i] != 0) {
                ImGui::Text("%04X", mapper.bank_registers[i]);
            } else {
                ImGui::TextDisabled("-");
            }
            ImGui::TableNextColumn();
            ImGui::Text("%02X", mapper.selected_banks[i]);
        }
        ImGui::EndTable();
    }

    ImGui::Separator();
    if (ImGui::BeginTable("mapper_windows", 4, ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_RowBg)) {
        ImGui::TableSetupColumn("Range");
        ImGui::TableSetupColumn("Window");
        ImGui::TableSetupColumn("Bank");
        ImGui::TableSetupColumn("Physical");
        ImGui::TableHeadersRow();
        for (const auto& window : mapper.windows) {
            const uint32_t physical = static_cast<uint32_t>(window.selected_bank) * mapper.bank_size;
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::Text("%04X-%04X", window.start, window.end);
            ImGui::TableNextColumn();
            ImGui::Text("%u", window.window);
            ImGui::TableNextColumn();
            ImGui::Text("%02X", window.selected_bank);
            ImGui::TableNextColumn();
            ImGui::Text("%05X", physical);
        }
        ImGui::EndTable();
    }

    ImGui::End();
}

void draw_compact_flash(GuiState& state) {
    set_next_window_defaults(480.0f, 520.0f, 456.0f, 180.0f);
    ImGui::Begin("CompactFlash");
    const auto cf = state.session.cf_snapshot();
    if (!cf.present) {
        ImGui::TextDisabled("No CompactFlash device configured.");
        ImGui::End();
        return;
    }

    const std::string path = cf.image_path.empty() ? std::string("-") : cf.image_path.string();
    ImGui::Text("I/O: %04X-%04X", cf.start, cf.end);
    ImGui::Text("Image: %s", path.c_str());
    ImGui::Text("Sectors: %u", cf.sector_count);
    ImGui::Text("Mode: %s", cf.read_only ? "read-only" : "read/write");
    ImGui::Text("Transfer: %s", cf_transfer_label(cf.transfer_mode));
    if (cf.transfer_size > 0) {
        ImGui::SameLine();
        ImGui::Text("(%llu/%llu bytes)",
                    static_cast<unsigned long long>(cf.transfer_index),
                    static_cast<unsigned long long>(cf.transfer_size));
    }

    ImGui::Separator();
    if (ImGui::BeginTable("cf_registers", 3, ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_RowBg)) {
        ImGui::TableSetupColumn("Register");
        ImGui::TableSetupColumn("Value");
        ImGui::TableSetupColumn("Decoded");
        ImGui::TableHeadersRow();

        auto row = [](const char* name, uint32_t value, int width, const std::string& decoded = {}) {
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::TextUnformatted(name);
            ImGui::TableNextColumn();
            const std::string hex = hex_value(value, width);
            ImGui::TextUnformatted(hex.c_str());
            ImGui::TableNextColumn();
            ImGui::TextUnformatted(decoded.c_str());
        };

        row("Error", cf.error, 2);
        row("Features", cf.features, 2);
        row("Sector Count", cf.sector_count_reg, 2, std::to_string(cf.requested_sector_count));
        row("Sector Number", cf.sector_number, 2);
        row("Cylinder Low", cf.cylinder_low, 2);
        row("Cylinder High", cf.cylinder_high, 2);
        row("Drive/Head", cf.drive_head, 2, (cf.drive_head & 0x40) != 0 ? "LBA" : "CHS");
        row("Status", cf.status, 2, cf_status_flags(cf.status));
        row("Command", cf.command, 2, cf_command_name(cf.command));
        row("Selected LBA", cf.selected_lba, 8);

        ImGui::EndTable();
    }

    ImGui::End();
}

} // namespace microlind::gui
