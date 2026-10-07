#include "gui_panel_decls.hpp"

#include <algorithm>
#include <array>
#include <cstring>
#include <limits>

#include "microlind/app/logic_validation.hpp"

namespace microlind::gui {
namespace {

constexpr std::array<const char*, 8> section_labels{
    "ROM regions", "RAM", "Memory mapper", "Serial", "Parallel I/O", "CompactFlash", "Video / VDC", "PLD routing"};
constexpr std::array<const char*, 8> section_keys{
    "ROM", "RAM", "MEMORY_MAPPER", "SERIAL", "PARALLEL", "CF", "VIDEO", "PLD_LOGIC"};

bool* enabled(app::ConfigEditor& editor, int section) {
    switch (section) {
    case 0: return &editor.rom_enabled;
    case 1: return &editor.config.ram.present;
    case 2: return &editor.config.mapper.present;
    case 3: return &editor.config.serial.present;
    case 4: return &editor.config.parallel.present;
    case 5: return &editor.config.cf.present;
    case 6: return &editor.config.video.present;
    default: return &editor.config.logic.present;
    }
}

bool address_input(const char* label, uint16_t& address, float width = 140) {
    uint32_t value = address;
    ImGui::SetNextItemWidth(width);
    if (!ImGui::InputScalar(label, ImGuiDataType_U32, &value, nullptr, nullptr, "%04X", ImGuiInputTextFlags_CharsHexadecimal)) return false;
    address = static_cast<uint16_t>(std::min(value, 0xFFFFu));
    return true;
}

bool number_input(const char* label, uint32_t& value) {
    ImGui::SetNextItemWidth(160);
    return ImGui::InputScalar(label, ImGuiDataType_U32, &value);
}

bool irq_input(uint8_t& level) {
    int value = level;
    ImGui::SetNextItemWidth(140);
    if (!ImGui::InputInt("IRQ level (0 disables)", &value)) return false;
    level = static_cast<uint8_t>(std::clamp(value, 0, 15));
    return true;
}

bool path_input(const char* label, std::filesystem::path& path, const std::filesystem::path& config_path,
                const std::vector<std::string>& filters) {
    std::vector<char> buffer(std::max<std::size_t>(4096, path.string().size() + 1024));
    std::memcpy(buffer.data(), path.string().c_str(), path.string().size());
    ImGui::TextUnformatted(label);
    ImGui::SetNextItemWidth(std::max(120.0f, ImGui::GetContentRegionAvail().x - 86.0f));
    const std::string id = "##" + std::string(label);
    bool changed = ImGui::InputText(id.c_str(), buffer.data(), buffer.size());
    if (changed) path = buffer.data();
#ifdef MICROLIND_HAS_PORTABLE_FILE_DIALOGS
    ImGui::SameLine();
    if (ImGui::Button(("Browse##" + std::string(label)).c_str())) {
        const auto selected = pick_file(label, filters);
        if (!selected.empty()) {
            const auto base = config_path.empty() ? std::filesystem::current_path() : config_path.parent_path();
            path = std::filesystem::absolute(selected).lexically_normal().lexically_relative(base);
            changed = true;
        }
    }
#else
    (void)config_path;
    (void)filters;
#endif
    return changed;
}

bool device_range(uint16_t& start, uint16_t& end) {
    bool changed = address_input("Start address (hex)", start);
    changed |= address_input("End address (hex)", end);
    if (start <= end) ImGui::TextDisabled("%u mapped registers", static_cast<unsigned>(end) - start + 1);
    return changed;
}

void edited(GuiState& state) {
    state.config_editor.modified = true;
    state.config_editor_error.clear();
    state.config_editor_status.clear();
    state.config_editor_pld_checked = false;
    state.config_editor_pld_issues.clear();
}

void check_pld(GuiState& state) {
    state.config_editor_pld_issues.clear();
    state.config_editor_pld_checked = true;
    const auto config = state.config_editor.effective_config();
    if (!config.logic.present) return;
    std::string error;
    const auto devices = cli::load_board_logic_devices(config.logic, error);
    if (!devices) {
        state.config_editor_pld_issues.push_back({"PLD_LOGIC", error, false});
        return;
    }
    for (const auto& issue : cli::validate_hardware_config_against_logic(config, *devices)) {
        state.config_editor_pld_issues.push_back({"PLD_LOGIC", cli::format_logic_validation_issue(issue),
            config.logic.bus_mode != BusDecodeMode::Route || issue.severity == cli::LogicValidationSeverity::Warning});
    }
}

void execute_action(GuiState& state) {
    if (state.config_editor_pending_action == 1) {
        state.config_editor.new_config();
        set_buffer(state.config_editor_path, "hw.cfg");
        state.config_editor_error.clear();
    } else if (state.config_editor.open(state.config_editor_pending_path, state.config_editor_error)) {
        set_buffer(state.config_editor_path, state.config_editor.path.string());
    }
    state.config_editor_status.clear();
    state.config_editor_pld_checked = false;
    state.config_editor_pld_issues.clear();
    state.config_editor_pending_action = 0;
}

void save_config(GuiState& state, bool save_as, bool apply) {
    auto destination = std::filesystem::path(buffer_string(state.config_editor_path));
#ifdef MICROLIND_HAS_PORTABLE_FILE_DIALOGS
    if (save_as || destination.empty()) {
        const auto selected = pick_save_file("Save hardware configuration", destination.empty() ? "hw.cfg" : destination.string(),
                                             {"Hardware configurations", "*.cfg *.ini", "All files", "*"});
        if (selected.empty()) return;
        destination = selected;
    }
#else
    (void)save_as;
#endif
    if (destination.empty()) {
        state.config_editor_error = "Choose a config file path.";
        return;
    }
    if (destination.extension().empty()) destination += ".cfg";
    if (apply && state.config_editor.config.logic.present) {
        check_pld(state);
        for (const auto& issue : state.config_editor_pld_issues) if (!issue.warning) {
            state.config_editor_error = "Cannot apply: " + issue.message;
            return;
        }
    }
    if (!state.config_editor.save(destination, state.config_editor_error)) return;
    set_buffer(state.config_editor_path, state.config_editor.path.string());
    state.config_editor_status = "Saved " + state.config_editor.path.string();
    state.runtime.add_log(state.config_editor_status);
    if (apply) {
        state.stop_execution();
        if (state.runtime.load_hardware_config(state.config_editor.path)) {
            set_buffer(state.config_path, state.config_editor.path.string());
            const auto config = state.config_editor.effective_config();
            set_buffer(state.cf_path, config.cf.present ? config.cf.image_path.string() : "");
            state.cf_min_sectors = config.cf.present
                ? static_cast<int>(std::min(config.cf.sectors, static_cast<uint32_t>(std::numeric_limits<int>::max()))) : 0;
            state.last_vdc_refresh_time = -1;
            state.config_editor_status += "; applied and simulation reset.";
        } else state.config_editor_error = "Saved, but applying failed. See the event log.";
    }
}

bool draw_settings(GuiState& state) {
    auto& config = state.config_editor.config;
    bool changed = false;
    switch (state.config_editor_section) {
    case 0:
        ImGui::TextWrapped("ROM image files are loaded separately in the Files window. Addresses here define their mapped regions.");
        if (ImGui::BeginTable("rom_regions", 4, ImGuiTableFlags_Borders | ImGuiTableFlags_SizingStretchProp)) {
            ImGui::TableSetupColumn("Region", ImGuiTableColumnFlags_WidthFixed, 55);
            ImGui::TableSetupColumn("Start (hex)");
            ImGui::TableSetupColumn("End (hex)");
            ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthFixed, 70);
            ImGui::TableHeadersRow();
            int remove = -1;
            for (std::size_t i = 0; i < config.roms.size(); ++i) {
                ImGui::PushID(static_cast<int>(i));
                ImGui::TableNextRow(); ImGui::TableNextColumn();
                ImGui::Text("%zu", i + 1);
                ImGui::TableNextColumn(); changed |= address_input("##start", config.roms[i].start, -1);
                ImGui::TableNextColumn(); changed |= address_input("##end", config.roms[i].end, -1);
                ImGui::TableNextColumn(); if (ImGui::SmallButton("Remove")) remove = static_cast<int>(i);
                ImGui::PopID();
            }
            if (remove >= 0) { config.roms.erase(config.roms.begin() + remove); changed = true; }
            ImGui::EndTable();
        }
        if (ImGui::Button("Add ROM region")) { config.roms.push_back({0xF800, 0xFFFF}); changed = true; }
        break;
    case 1:
        changed |= device_range(config.ram.start, config.ram.end);
        changed |= number_input("Bank size (bytes)", config.ram.bank_size);
        changed |= number_input("Backing RAM (bytes)", config.ram.available);
        ImGui::TextDisabled("%.0f KiB installed", config.ram.available / 1024.0);
        ImGui::TextWrapped("Banking requires the memory mapper. Without it, RAM is mapped as flat memory over the address range.");
        break;
    case 2:
        ImGui::TextWrapped("Four bank registers and optional CPU windows. Register address 0000 disables that register.");
        if (ImGui::BeginTable("mapper_windows", 5, ImGuiTableFlags_Borders | ImGuiTableFlags_SizingStretchProp)) {
            ImGui::TableSetupColumn("Bank", ImGuiTableColumnFlags_WidthFixed, 35);
            ImGui::TableSetupColumn("Register");
            ImGui::TableSetupColumn("Window", ImGuiTableColumnFlags_WidthFixed, 55);
            ImGui::TableSetupColumn("Start");
            ImGui::TableSetupColumn("End");
            ImGui::TableHeadersRow();
            for (int i = 0; i < 4; ++i) {
                ImGui::PushID(i);
                auto& window = config.mapper.windows[i];
                ImGui::TableNextRow(); ImGui::TableNextColumn();
                ImGui::Text("%d", i);
                ImGui::TableNextColumn(); changed |= address_input("##register", config.mapper.bank_reg[i], -1);
                ImGui::TableNextColumn(); changed |= ImGui::Checkbox("##window", &window.present);
                ImGui::BeginDisabled(!window.present);
                ImGui::TableNextColumn(); changed |= address_input("##start", window.start, -1);
                ImGui::TableNextColumn(); changed |= address_input("##end", window.end, -1);
                ImGui::EndDisabled();
                ImGui::PopID();
            }
            ImGui::EndTable();
        }
        ImGui::TextDisabled("All addresses are hexadecimal.");
        ImGui::TextWrapped("If all windows are disabled, the simulator derives them from RAM and bank size.");
        break;
    case 3:
        changed |= device_range(config.serial.start, config.serial.end);
        changed |= irq_input(config.serial.irq_level);
        break;
    case 4:
        changed |= device_range(config.parallel.start, config.parallel.end);
        changed |= irq_input(config.parallel.irq_level);
        break;
    case 5:
        changed |= device_range(config.cf.start, config.cf.end);
        ImGui::SeparatorText("Disk image");
        changed |= path_input("Image file", config.cf.image_path, state.config_editor.path,
                              {"Disk images", "*.img *.bin", "All files", "*"});
        changed |= number_input("Minimum sectors", config.cf.sectors);
        ImGui::TextDisabled("%.0f KiB minimum capacity (512 bytes per sector)", config.cf.sectors / 2.0);
        changed |= ImGui::Checkbox("Read-only image", &config.cf.read_only);
        ImGui::TextWrapped("Leave the image path empty to configure a CF device with no media. Paths are relative to the config file.");
        break;
    case 6:
        changed |= device_range(config.video.start, config.video.end);
        ImGui::Text("Private VRAM: 64 KiB (65536 bytes)");
        if (config.video.vram_size != 65536 && ImGui::Button("Use supported VRAM size")) {
            config.video.vram_size = 65536; changed = true;
        }
        ImGui::TextWrapped("Text and bitmap modes are selected by firmware through VDC registers.");
        break;
    case 7: {
        const std::vector<std::string> filters{"PLD source files", "*.pld", "All files", "*"};
        changed |= path_input("Signal logic", config.logic.signal_logic_path, state.config_editor.path, filters);
        changed |= path_input("Memory logic", config.logic.memory_logic_path, state.config_editor.path, filters);
        changed |= path_input("Address logic", config.logic.address_logic_path, state.config_editor.path, filters);
        int mode = static_cast<int>(config.logic.bus_mode);
        const char* labels[]{"Range map", "Validate", "PLD routing"};
        if (ImGui::Combo("Bus mode", &mode, labels, 3)) {
            config.logic.bus_mode = static_cast<BusDecodeMode>(mode); changed = true;
        }
        ImGui::TextWrapped("Range map uses configured addresses. Validate also reports PLD mismatches. PLD routing selects devices using the board decode.");
        break;
    }
    }
    return changed;
}

} // namespace

void draw_config_editor(GuiState& state) {
    if (state.config_editor_open_requested) {
        state.stop_execution();
        if (!state.config_editor_initialized) {
            state.config_editor_initialized = true;
            if (state.config_editor.open(buffer_string(state.config_path), state.config_editor_error))
                set_buffer(state.config_editor_path, state.config_editor.path.string());
            else set_buffer(state.config_editor_path, "hw.cfg");
        }
        ImGui::OpenPopup("Hardware Configuration");
        state.config_editor_open_requested = false;
    }
    ImGui::SetNextWindowSize(ImVec2(1160, 730), ImGuiCond_Appearing);
    ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(), ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    state.config_editor_modal_open = ImGui::IsPopupOpen("Hardware Configuration");
    bool open = true;
    if (!ImGui::BeginPopupModal("Hardware Configuration", &open, ImGuiWindowFlags_NoSavedSettings)) return;
    if (!open) {
        state.config_editor_modal_open = false;
        ImGui::CloseCurrentPopup(); ImGui::EndPopup(); return;
    }
    ImGui::TextUnformatted("Hardware configuration");
    ImGui::SameLine();
    if (state.config_editor.modified) ImGui::TextColored(ImVec4(1, 0.8f, 0.3f, 1), "Unsaved changes");
    ImGui::SetNextItemWidth(std::max(220.0f, ImGui::GetContentRegionAvail().x - 370));
    ImGui::InputText("Config file", state.config_editor_path.data(), state.config_editor_path.size());
    ImGui::SameLine();
    if (ImGui::Button("New")) state.config_editor_pending_action = 1;
    ImGui::SameLine();
    if (ImGui::Button("Open...")) {
        std::filesystem::path path = buffer_string(state.config_editor_path);
#ifdef MICROLIND_HAS_PORTABLE_FILE_DIALOGS
        path = pick_file("Open hardware configuration", {"Config files", "*.cfg *.ini", "All files", "*"});
#endif
        if (!path.empty()) { state.config_editor_pending_action = 2; state.config_editor_pending_path = path; }
    }
    ImGui::SameLine();
    if (ImGui::Button("Save as...")) save_config(state, true, false);
    ImGui::SameLine();
    if (ImGui::Button("Save")) save_config(state, false, false);
    ImGui::Separator();

    const float height = std::max(240.0f, ImGui::GetContentRegionAvail().y - 95);
    if (ImGui::BeginTable("editor_columns", 3, ImGuiTableFlags_Resizable | ImGuiTableFlags_SizingStretchProp)) {
        ImGui::TableSetupColumn("Devices", ImGuiTableColumnFlags_WidthFixed, 190);
        ImGui::TableSetupColumn("Settings", ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableSetupColumn("Address map", ImGuiTableColumnFlags_WidthFixed, 270);
        ImGui::TableNextRow(); ImGui::TableNextColumn();
        ImGui::BeginChild("device_list", ImVec2(0, height));
        ImGui::SeparatorText("Devices & memory");
        for (int i = 0; i < 8; ++i) {
            ImGui::PushID(i);
            if (ImGui::Checkbox("##enabled", enabled(state.config_editor, i))) edited(state);
            ImGui::SameLine();
            if (ImGui::Selectable(section_labels[i], state.config_editor_section == i)) state.config_editor_section = i;
            ImGui::PopID();
        }
        ImGui::Spacing();
        ImGui::TextWrapped("Disable a device to omit its section. Draft settings are retained.");
        ImGui::EndChild();

        ImGui::TableNextColumn();
        ImGui::BeginChild("device_settings", ImVec2(0, height));
        ImGui::SeparatorText(section_labels[state.config_editor_section]);
        ImGui::BeginDisabled(!*enabled(state.config_editor, state.config_editor_section));
        if (draw_settings(state)) edited(state);
        ImGui::EndDisabled();
        const auto issues = app::validate_config(state.config_editor.effective_config());
        for (const auto& issue : issues) if (issue.section == section_keys[state.config_editor_section]) {
            ImGui::PushStyleColor(ImGuiCol_Text, issue.warning ? ImVec4(1, 0.8f, 0.3f, 1) : ImVec4(1, 0.4f, 0.4f, 1));
            ImGui::TextWrapped("%s", issue.message.c_str());
            ImGui::PopStyleColor();
        }
        ImGui::EndChild();

        ImGui::TableNextColumn();
        ImGui::BeginChild("editor_map", ImVec2(0, height));
        ImGui::SeparatorText("Address map");
        for (const auto& range : app::config_address_map(state.config_editor.effective_config())) {
            ImGui::Text("%04X-%04X  %s", range.start, range.end, range.label.c_str());
        }
        ImGui::SeparatorText("Validation");
        bool valid = true;
        for (const auto& issue : issues) {
            valid &= issue.warning;
            ImGui::PushStyleColor(ImGuiCol_Text, issue.warning ? ImVec4(1, 0.8f, 0.3f, 1) : ImVec4(1, 0.4f, 0.4f, 1));
            ImGui::TextWrapped("%s: %s", issue.section.c_str(), issue.message.c_str());
            ImGui::PopStyleColor();
        }
        if (valid) ImGui::TextColored(ImVec4(0.4f, 0.9f, 0.6f, 1), "Address ranges and values valid");
        if (state.config_editor.config.logic.present) {
            if (ImGui::Button("Validate PLD")) check_pld(state);
            if (!state.config_editor_pld_checked) ImGui::TextDisabled("PLD has not been checked.");
            else if (state.config_editor_pld_issues.empty()) ImGui::TextColored(ImVec4(0.4f, 0.9f, 0.6f, 1), "PLD decode matches.");
            for (const auto& issue : state.config_editor_pld_issues) {
                ImGui::PushStyleColor(ImGuiCol_Text, issue.warning ? ImVec4(1, 0.8f, 0.3f, 1) : ImVec4(1, 0.4f, 0.4f, 1));
                ImGui::TextWrapped("%s", issue.message.c_str());
                ImGui::PopStyleColor();
            }
        }
        ImGui::EndChild();
        ImGui::EndTable();
    }
    ImGui::Separator();
    if (!state.config_editor_error.empty()) ImGui::TextColored(ImVec4(1, 0.4f, 0.4f, 1), "%s", state.config_editor_error.c_str());
    if (!state.config_editor_status.empty()) ImGui::TextWrapped("%s", state.config_editor_status.c_str());
    ImGui::TextDisabled("Simulation is paused. Applying resets the simulated hardware. Closing retains the draft.");
    if (ImGui::Button("Discard edits")) {
        state.config_editor.discard();
        state.config_editor_error.clear(); state.config_editor_status.clear();
        state.config_editor_pld_checked = false; state.config_editor_pld_issues.clear();
        set_buffer(state.config_editor_path, state.config_editor.path.empty() ? "hw.cfg" : state.config_editor.path.string());
    }
    ImGui::SameLine();
    const auto issues = app::validate_config(state.config_editor.effective_config());
    const bool invalid = std::any_of(issues.begin(), issues.end(), [](const auto& issue) { return !issue.warning; });
    ImGui::BeginDisabled(invalid);
    if (ImGui::Button("Save & apply")) save_config(state, false, true);
    ImGui::EndDisabled();
    ImGui::SameLine();
    if (ImGui::Button("Close")) {
        state.config_editor_modal_open = false;
        ImGui::CloseCurrentPopup();
    }

    if (state.config_editor_pending_action) {
        if (state.config_editor.modified) ImGui::OpenPopup("Discard unsaved configuration edits?");
        else execute_action(state);
    }
    if (ImGui::BeginPopupModal("Discard unsaved configuration edits?", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::TextUnformatted("Opening or creating a configuration will replace the current draft.");
        if (ImGui::Button("Discard and continue")) { execute_action(state); ImGui::CloseCurrentPopup(); }
        ImGui::SameLine();
        if (ImGui::Button("Cancel")) { state.config_editor_pending_action = 0; ImGui::CloseCurrentPopup(); }
        ImGui::EndPopup();
    }
    ImGui::EndPopup();
}

} // namespace microlind::gui
