#include "gui_panel_decls.hpp"

#include <algorithm>
#include <array>
#include <cstddef>

#include "imgui.h"

namespace microlind::gui {
namespace {

constexpr std::array<const char*, 3> kTriggerModeLabels{{"Rising edge", "Falling edge", "Either edge"}};

const char* capture_state_label(LogicCaptureState state) {
    switch (state) {
    case LogicCaptureState::Stopped: return "Stopped";
    case LogicCaptureState::WaitingForTrigger: return "Waiting for trigger";
    case LogicCaptureState::Capturing: return "Capturing";
    }
    return "Unknown";
}

void draw_waveforms(GuiState& state, const LogicAnalyserSnapshot& capture) {
    std::array<std::size_t, kLogicSignalCount> channels{};
    std::size_t channel_count = 0;
    for (std::size_t index = 0; index < kLogicSignalCount; ++index) {
        if (state.logic_analyser_signals[index]) channels[channel_count++] = index;
    }

    if (channel_count == 0) {
        ImGui::TextDisabled("Select one or more signals to plot.");
        return;
    }
    if (capture.samples.empty()) {
        ImGui::TextDisabled("No samples yet. Run the simulator or step it while capture is active.");
        return;
    }

    const int visible = std::clamp(state.logic_analyser_samples_visible, 16, 512);
    const std::size_t sample_count = std::min<std::size_t>(capture.samples.size(), static_cast<std::size_t>(visible));
    const int max_history_offset = static_cast<int>(capture.samples.size() - sample_count);
    state.logic_analyser_history_offset = std::clamp(state.logic_analyser_history_offset, 0, max_history_offset);
    const std::size_t first = capture.samples.size() - sample_count - static_cast<std::size_t>(state.logic_analyser_history_offset);
    const float lane_height = 40.5f;
    const float label_width = 142.0f;
    const float waveform_width = std::max(180.0f, ImGui::GetContentRegionAvail().x - label_width);
    const float height = lane_height * static_cast<float>(channel_count) + 26.0f;

    ImGui::BeginChild("LogicWaveforms", ImVec2(0.0f, height), true, ImGuiWindowFlags_HorizontalScrollbar);
    const ImVec2 origin = ImGui::GetCursorScreenPos();
    ImDrawList* draw = ImGui::GetWindowDrawList();
    const ImU32 grid = ImGui::GetColorU32(ImGuiCol_Border);
    const float x0 = origin.x + label_width;
    const float x1 = x0 + waveform_width;
    const float sample_width = sample_count > 1 ? waveform_width / static_cast<float>(sample_count - 1) : waveform_width;

    for (std::size_t lane = 0; lane < channel_count; ++lane) {
        const std::size_t signal_index = channels[lane];
        const auto signal = static_cast<LogicSignal>(signal_index);
        const float top = origin.y + lane_height * static_cast<float>(lane);
        const float high = top + 5.0f;
        const float low = top + lane_height - 6.0f;
        const ImU32 color = ImGui::ColorConvertFloat4ToU32(state.logic_analyser_colors[signal_index]);

        draw->AddText(ImVec2(origin.x + 4.0f, top + 5.0f), color, logic_signal_name(signal));
        draw->AddLine(ImVec2(x0, top + lane_height - 0.5f), ImVec2(x1, top + lane_height - 0.5f), grid);

        float previous_x = x0;
        bool previous_value = capture.samples[first].values[signal_index];
        float previous_y = previous_value ? high : low;
        if (sample_count == 1) {
            draw->AddLine(ImVec2(x0, previous_y), ImVec2(x1, previous_y), color, 1.5f);
            continue;
        }
        for (std::size_t offset = 1; offset < sample_count; ++offset) {
            const float x = x0 + sample_width * static_cast<float>(offset);
            const bool value = capture.samples[first + offset].values[signal_index];
            const float y = value ? high : low;
            draw->AddLine(ImVec2(previous_x, previous_y), ImVec2(x, previous_y), color, 1.5f);
            if (y != previous_y) draw->AddLine(ImVec2(x, previous_y), ImVec2(x, y), color, 1.5f);
            previous_x = x;
            previous_y = y;
        }
    }

    const uint64_t first_cycle = capture.samples[first].cycle;
    const uint64_t last_cycle = capture.samples.back().cycle;
    ImGui::SetCursorScreenPos(ImVec2(origin.x + 4.0f, origin.y + lane_height * static_cast<float>(channel_count) + 3.0f));
    ImGui::TextDisabled("cycles %llu to %llu  |  %zu samples", static_cast<unsigned long long>(first_cycle),
                        static_cast<unsigned long long>(last_cycle), sample_count);
    ImGui::EndChild();
}

} // namespace

void draw_logic_analyser(GuiState& state) {
    ImGui::SetNextWindowSize(ImVec2(800.0f, 500.0f), ImGuiCond_FirstUseEver);
    ImGui::Begin("Logic Analyser", &state.show_logic_analyser);

    const auto capture = state.runtime.logic_analyser_snapshot();
    ImGui::Text("State: %s", capture_state_label(capture.state));
    ImGui::SameLine();
    if (capture.state == LogicCaptureState::Stopped) {
        if (ImGui::Button("Run")) {
            const auto trigger = state.logic_analyser_use_trigger
                ? std::optional<LogicSignal>(static_cast<LogicSignal>(state.logic_analyser_trigger_signal))
                : std::nullopt;
            state.runtime.start_logic_analyser(
                state.logic_analyser_microcycle,
                trigger,
                static_cast<LogicTriggerMode>(state.logic_analyser_trigger_mode));
        }
    } else if (ImGui::Button("Stop")) {
        state.runtime.stop_logic_analyser();
    }
    ImGui::SameLine();
    if (ImGui::Button("Clear")) state.runtime.clear_logic_analyser();
    ImGui::SameLine();
    ImGui::TextDisabled("%zu / 8192 samples", capture.samples.size());

    ImGui::Separator();
    ImGui::Checkbox("Microcycle resolution", &state.logic_analyser_microcycle);
    ImGui::SameLine();
    ImGui::TextDisabled("(debug runs execute and sample individual bus cycles)");
    ImGui::Checkbox("Trigger", &state.logic_analyser_use_trigger);
    if (state.logic_analyser_use_trigger) {
        ImGui::SameLine();
        ImGui::SetNextItemWidth(180.0f);
        if (ImGui::BeginCombo("##trigger-signal", logic_signal_name(static_cast<LogicSignal>(state.logic_analyser_trigger_signal)))) {
            for (std::size_t index = 0; index < kLogicSignalCount; ++index) {
                const bool selected = state.logic_analyser_trigger_signal == static_cast<int>(index);
                if (ImGui::Selectable(logic_signal_name(static_cast<LogicSignal>(index)), selected)) {
                    state.logic_analyser_trigger_signal = static_cast<int>(index);
                }
                if (selected) ImGui::SetItemDefaultFocus();
            }
            ImGui::EndCombo();
        }
        ImGui::SameLine();
        ImGui::SetNextItemWidth(135.0f);
        ImGui::Combo("##trigger-mode", &state.logic_analyser_trigger_mode,
                     kTriggerModeLabels.data(), static_cast<int>(kTriggerModeLabels.size()));
    }
    ImGui::SetNextItemWidth(160.0f);
    ImGui::SliderInt("Samples visible", &state.logic_analyser_samples_visible, 16, 512);
    const int max_history_offset = std::max(0, static_cast<int>(capture.samples.size()) - state.logic_analyser_samples_visible);
    if (capture.state == LogicCaptureState::Capturing) state.logic_analyser_history_offset = 0;
    ImGui::SetNextItemWidth(240.0f);
    ImGui::SliderInt("History offset", &state.logic_analyser_history_offset, 0, max_history_offset,
                     max_history_offset == 0 ? "Latest" : "%d samples behind latest");

    ImGui::SeparatorText("Signals");
    if (ImGui::BeginTable("LogicSignals", 3, ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_RowBg)) {
        ImGui::TableSetupColumn("Plot", ImGuiTableColumnFlags_WidthFixed, 45.0f);
        ImGui::TableSetupColumn("Signal", ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableSetupColumn("Colour", ImGuiTableColumnFlags_WidthFixed, 130.0f);
        ImGui::TableHeadersRow();
        for (std::size_t index = 0; index < kLogicSignalCount; ++index) {
            ImGui::PushID(static_cast<int>(index));
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::Checkbox("##enabled", &state.logic_analyser_signals[index]);
            ImGui::TableSetColumnIndex(1);
            ImGui::TextUnformatted(logic_signal_name(static_cast<LogicSignal>(index)));
            ImGui::TableSetColumnIndex(2);
            ImGui::SetNextItemWidth(115.0f);
            ImGui::ColorEdit4("##color", &state.logic_analyser_colors[index].x,
                              ImGuiColorEditFlags_NoInputs | ImGuiColorEditFlags_AlphaBar);
            ImGui::PopID();
        }
        ImGui::EndTable();
    }

    ImGui::SeparatorText("Waveforms");
    draw_waveforms(state, capture);
    ImGui::End();
}

} // namespace microlind::gui
