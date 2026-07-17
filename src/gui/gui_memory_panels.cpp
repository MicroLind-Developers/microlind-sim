#include "gui_panel_decls.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cstdint>
#include <cstdio>
#include <initializer_list>
#include <iterator>
#include <string>
#include <utility>
#include <vector>

#include "imgui.h"

namespace microlind::gui {

void draw_memory_map(GuiState& state) {
    set_next_window_defaults(944.0f, 28.0f, 240.0f, 190.0f);
    ImGui::Begin("Memory Map", &state.show_memory_map);
    const auto snapshot = state.runtime.debugger_snapshot();
    const auto& summary = snapshot.memory_map;
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
    ImGui::Begin("Memory", &state.show_memory_viewer);

    const auto debug_snapshot = state.runtime.debugger_snapshot();
    const auto& regs = debug_snapshot.regs;

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
    const auto memory_rows = state.runtime.memory_snapshot(static_cast<uint16_t>(state.memory_start), state.memory_rows);
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

        for (const auto& row : memory_rows) {
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::Text("%04X", row.address);

            std::array<char, kCols + 1> ascii{};
            for (int col = 0; col < kCols; ++col) {
                ImGui::TableNextColumn();
                const auto index = static_cast<std::size_t>(col);
                const uint16_t address = static_cast<uint16_t>(row.address + col);
                uint8_t value = row.bytes[index];
                ascii[static_cast<std::size_t>(col)] =
                    std::isprint(static_cast<unsigned char>(value)) ? static_cast<char>(value) : '.';

                if (row.at_pc[index]) {
                    ImGui::TableSetBgColor(ImGuiTableBgTarget_CellBg, IM_COL32(38, 96, 56, 180));
                } else if (row.at_s[index] || row.at_u[index]) {
                    ImGui::TableSetBgColor(ImGuiTableBgTarget_CellBg, IM_COL32(72, 72, 116, 180));
                } else if (row.watched[index]) {
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
                    state.runtime.write_memory(address, value);
                    state.runtime.add_log("Wrote " + hex_value(value, 2) + " to " + hex_value(address, 4) + ".");
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
    ImGui::Begin("Memory Mapper", &state.show_mapper);
    const auto snapshot = state.runtime.debugger_snapshot();
    const auto& mapper = snapshot.mapper;
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
            if (mapper.bank_registers[i] != 0) {
                uint8_t bank = mapper.selected_banks[i];
                ImGui::PushID(i);
                ImGui::SetNextItemWidth(48.0f);
                if (ImGui::InputScalar(
                        "##mapper_bank",
                        ImGuiDataType_U8,
                        &bank,
                        nullptr,
                        nullptr,
                        "%02X",
                        ImGuiInputTextFlags_CharsHexadecimal | ImGuiInputTextFlags_AutoSelectAll)) {
                    const uint16_t reg = mapper.bank_registers[i];
                    state.runtime.write_memory(reg, bank);
                    state.runtime.add_log(
                        "Mapper window " + std::to_string(i) + " bank set to " +
                        hex_value(bank, 2) + " via " + hex_value(reg, 4) + ".");
                }
                ImGui::PopID();
            } else {
                ImGui::TextDisabled("-");
            }
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

namespace {

const char* bus_phase_label(microlind::BusPhase phase) {
    switch (phase) {
    case microlind::BusPhase::QHighELow: return "Q high / E low";
    case microlind::BusPhase::QHighEHigh: return "Q high / E high";
    case microlind::BusPhase::QLowEHigh: return "Q low / E high";
    case microlind::BusPhase::QLowELow: return "Q low / E low";
    }
    return "unknown";
}

const char* bus_cycle_kind_label(microlind::BusCycleKind kind) {
    switch (kind) {
    case microlind::BusCycleKind::Idle: return "Idle";
    case microlind::BusCycleKind::OpcodeFetch: return "Opcode fetch";
    case microlind::BusCycleKind::OperandRead: return "Operand read";
    case microlind::BusCycleKind::OperandWrite: return "Operand write";
    case microlind::BusCycleKind::StackRead: return "Stack read";
    case microlind::BusCycleKind::StackWrite: return "Stack write";
    case microlind::BusCycleKind::VectorRead: return "Vector read";
    case microlind::BusCycleKind::Internal: return "Internal";
    }
    return "Unknown";
}

const char* bus_decode_mode_label(microlind::BusDecodeMode mode) {
    switch (mode) {
    case microlind::BusDecodeMode::RangeMap: return "Range";
    case microlind::BusDecodeMode::Validate: return "Validate";
    case microlind::BusDecodeMode::Route: return "Route";
    }
    return "Unknown";
}

int bus_decode_mode_index(microlind::BusDecodeMode mode) {
    switch (mode) {
    case microlind::BusDecodeMode::RangeMap: return 0;
    case microlind::BusDecodeMode::Validate: return 1;
    case microlind::BusDecodeMode::Route: return 2;
    }
    return 0;
}

microlind::BusDecodeMode bus_decode_mode_from_index(int index) {
    switch (index) {
    case 1: return microlind::BusDecodeMode::Validate;
    case 2: return microlind::BusDecodeMode::Route;
    default: return microlind::BusDecodeMode::RangeMap;
    }
}

void draw_logic_signal_row(const char* name, bool asserted) {
    ImGui::TableNextRow();
    ImGui::TableNextColumn();
    ImGui::TextUnformatted(name);
    ImGui::TableNextColumn();
    if (asserted) {
        ImGui::TextColored(ImVec4(0.34f, 0.86f, 0.48f, 1.0f), "1");
    } else {
        ImGui::TextDisabled("0");
    }
}

void draw_logic_signal_table(const char* table_id, const std::initializer_list<std::pair<const char*, bool>>& rows) {
    if (!ImGui::BeginTable(table_id, 2, ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_RowBg)) return;
    ImGui::TableSetupColumn("Signal");
    ImGui::TableSetupColumn("State", ImGuiTableColumnFlags_WidthFixed, 48.0f);
    ImGui::TableHeadersRow();
    for (const auto& [name, asserted] : rows) {
        draw_logic_signal_row(name, asserted);
    }
    ImGui::EndTable();
}

} // namespace

void draw_pld_logic(GuiState& state) {
    set_next_window_defaults(1188.0f, 524.0f, 300.0f, 300.0f);
    ImGui::Begin("PLD Logic", &state.show_pld_logic);

    const auto debug_snapshot = state.runtime.debugger_snapshot();
    const auto& regs = debug_snapshot.regs;
    if (!state.pld_live_bus && state.pld_follow_pc) {
        state.pld_address = regs.pc;
    }

    ImGui::Checkbox("Live bus", &state.pld_live_bus);
    ImGui::BeginDisabled(state.pld_live_bus);
    ImGui::SameLine();
    ImGui::SetNextItemWidth(96.0f);
    ImGui::InputInt("Address", &state.pld_address, 16, 256, ImGuiInputTextFlags_CharsHexadecimal);
    state.pld_address = std::clamp(state.pld_address, 0, 0xFFFF);

    ImGui::SameLine();
    ImGui::Checkbox("Follow PC", &state.pld_follow_pc);

    if (ImGui::RadioButton("Read", state.pld_read)) {
        state.pld_read = true;
    }
    ImGui::SameLine();
    if (ImGui::RadioButton("Write", !state.pld_read)) {
        state.pld_read = false;
    }
    ImGui::EndDisabled();

    const auto snapshot = state.runtime.logic_snapshot(
        state.pld_live_bus,
        static_cast<uint16_t>(state.pld_address),
        state.pld_read);
    if (!snapshot.configured) {
        ImGui::TextDisabled("No [PLD_LOGIC] configured.");
        ImGui::End();
        return;
    }

    const char* bus_modes[] = {"Range", "Validate", "Route"};
    int bus_mode_index = bus_decode_mode_index(snapshot.bus_mode);
    ImGui::BeginDisabled(state.runtime.execution_active());
    ImGui::SetNextItemWidth(112.0f);
    if (ImGui::Combo("Bus mode", &bus_mode_index, bus_modes, static_cast<int>(std::size(bus_modes)))) {
        state.runtime.set_logic_bus_mode(bus_decode_mode_from_index(bus_mode_index));
    }
    ImGui::EndDisabled();
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled) && state.runtime.execution_active()) {
        ImGui::SetTooltip("Pause execution before changing PLD bus mode");
    }
    ImGui::SameLine();
    ImGui::TextDisabled("%s", bus_decode_mode_label(snapshot.bus_mode));

    ImGui::Text("SIGNAL: %s", snapshot.signal_logic_path.filename().string().c_str());
    ImGui::Text("MEMORY: %s", snapshot.memory_logic_path.filename().string().c_str());
    ImGui::Text("ADDRESS: %s", snapshot.address_logic_path.filename().string().c_str());

    if (!snapshot.available) {
        ImGui::Separator();
        ImGui::TextWrapped("%s", snapshot.error.c_str());
        ImGui::End();
        return;
    }

    ImGui::Separator();
    ImGui::Text("Input: %04X  %s  D=%02X  AM19..21=%u",
                snapshot.address,
                snapshot.rw ? "RD" : "WR",
                snapshot.data,
                snapshot.mapper_bits);
    ImGui::Text("Cycle: %s", bus_cycle_kind_label(snapshot.cycle_kind));
    ImGui::SameLine();
    ImGui::Text("| Phase: %s", bus_phase_label(snapshot.phase));
    ImGui::Text("Access: read=%u write=%u log=%u",
                snapshot.apply_read ? 1 : 0,
                snapshot.apply_write ? 1 : 0,
                snapshot.log_access ? 1 : 0);
    ImGui::Text("E:%u Q:%u BA:%u BS:%u BREQ:%u MEM_EN:%u MAP_EN:%u",
                snapshot.e ? 1 : 0,
                snapshot.q ? 1 : 0,
                snapshot.ba ? 1 : 0,
                snapshot.bs ? 1 : 0,
                snapshot.breq ? 1 : 0,
                snapshot.memory_enable ? 1 : 0,
                snapshot.mapper_enable ? 1 : 0);

    if (!snapshot.decoded.errors.empty()) {
        ImGui::TextColored(ImVec4(0.95f, 0.45f, 0.36f, 1.0f), "Decode errors");
        for (const auto& error : snapshot.decoded.errors) {
            ImGui::BulletText("%s", error.c_str());
        }
    }

    if (ImGui::CollapsingHeader("Control", ImGuiTreeNodeFlags_DefaultOpen)) {
        draw_logic_signal_table("pld_control", {
            {"RW1", snapshot.decoded.rw1},
            {"MEM_RD", snapshot.decoded.mem_rd},
            {"MEM_WR", snapshot.decoded.mem_wr},
            {"RD", snapshot.decoded.rd},
            {"WR", snapshot.decoded.wr},
            {"BAVAIL", snapshot.decoded.bavail},
        });
    }

    if (ImGui::CollapsingHeader("Memory", ImGuiTreeNodeFlags_DefaultOpen)) {
        draw_logic_signal_table("pld_memory", {
            {"ROM_EN", snapshot.decoded.rom_en},
            {"RAML_EN", snapshot.decoded.raml_en},
            {"RAMH_EN", snapshot.decoded.ramh_en},
            {"RAMX_EN", snapshot.decoded.ramx_en},
            {"IO_EN", snapshot.decoded.io_en},
            {"MAP_RD", snapshot.decoded.map_rd},
            {"BANK_SEL0", snapshot.decoded.bank_sel0},
            {"BANK_SEL1", snapshot.decoded.bank_sel1},
        });
        ImGui::Text("Bank select: %u", snapshot.decoded.bank_select);
    }

    if (ImGui::CollapsingHeader("I/O", ImGuiTreeNodeFlags_DefaultOpen)) {
        draw_logic_signal_table("pld_io", {
            {"MEM_EN", snapshot.decoded.mapper_register_en},
            {"IRQ_EN", snapshot.decoded.irq_en},
            {"PS2_EN", snapshot.decoded.ps2_en},
            {"CF_EN", snapshot.decoded.cf_en},
            {"PAR_EN", snapshot.decoded.par_en},
            {"SER_EN", snapshot.decoded.ser_en},
            {"VDC_EN", snapshot.decoded.vdc_en},
            {"SND_EN", snapshot.decoded.snd_en},
            {"EXP_EN", snapshot.decoded.exp_en},
        });
    }

    ImGui::End();
}

void draw_compact_flash(GuiState& state) {
    set_next_window_defaults(480.0f, 520.0f, 456.0f, 180.0f);
    ImGui::Begin("CompactFlash", &state.show_compact_flash);
    const auto snapshot = state.runtime.debugger_snapshot();
    const auto& cf = snapshot.compact_flash;
    if (!cf.present) {
        ImGui::TextDisabled("No CompactFlash device configured.");
        ImGui::End();
        return;
    }

    const std::string path = cf.image_path.empty() ? std::string("-") : cf.image_path.string();
    ImGui::Text("I/O: %04X-%04X", cf.start, cf.end);
    ImGui::Text("Image: %s", path.c_str());
    ImGui::Text("Loaded: %s", cf.image_loaded ? "yes" : "no");
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

void draw_parallel(GuiState& state) {
    set_next_window_defaults(480.0f, 706.0f, 456.0f, 180.0f);
    ImGui::Begin("Parallel I/O", &state.show_parallel);
    const auto snapshot = state.runtime.debugger_snapshot();
    const auto& parallel = snapshot.parallel;
    if (!parallel.present) {
        ImGui::TextDisabled("No W65C22 parallel device configured.");
        ImGui::End();
        return;
    }

    ImGui::Text("I/O: %04X-%04X", parallel.start, parallel.end);
    ImGui::Text("IRQ: %s", parallel.irq_asserted ? "asserted" : "idle");

    if (ImGui::BeginTable("parallel_registers", 4, ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_RowBg)) {
        ImGui::TableSetupColumn("Register");
        ImGui::TableSetupColumn("A");
        ImGui::TableSetupColumn("B");
        ImGui::TableSetupColumn("Notes");
        ImGui::TableHeadersRow();

        auto row = [](const char* name, uint8_t a, uint8_t b, const char* notes) {
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::TextUnformatted(name);
            ImGui::TableNextColumn();
            ImGui::Text("%02X", a);
            ImGui::TableNextColumn();
            ImGui::Text("%02X", b);
            ImGui::TableNextColumn();
            ImGui::TextUnformatted(notes);
        };

        row("Port", parallel.port_a, parallel.port_b, "effective pins");
        row("Output", parallel.output_a, parallel.output_b, "output latch");
        row("Input", parallel.input_a, parallel.input_b, "external pins");
        row("DDR", parallel.ddr_a, parallel.ddr_b, "1=output");

        ImGui::EndTable();
    }

    ImGui::Separator();
    if (ImGui::BeginTable("parallel_control", 2, ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_RowBg)) {
        ImGui::TableSetupColumn("Register");
        ImGui::TableSetupColumn("Value");
        ImGui::TableHeadersRow();

        auto row = [](const char* name, uint8_t value) {
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::TextUnformatted(name);
            ImGui::TableNextColumn();
            ImGui::Text("%02X", value);
        };

        row("ACR", parallel.acr);
        row("PCR", parallel.pcr);
        row("IFR", parallel.ifr);
        row("IER", parallel.ier);

        ImGui::EndTable();
    }

    ImGui::End();
}

} // namespace microlind::gui
