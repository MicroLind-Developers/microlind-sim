#pragma once

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include "microlind/app/hardware_config.hpp"
#include "microlind/logic.hpp"

namespace microlind::cli {

enum class LogicValidationSeverity {
    Error,
    Warning,
};

struct LogicValidationIssue {
    LogicValidationSeverity severity{LogicValidationSeverity::Error};
    uint16_t address{};
    std::string message;
};

std::optional<microlind::logic::BoardLogicDevices> load_board_logic_devices(
    const std::filesystem::path& signal_path,
    const std::filesystem::path& memory_path,
    const std::filesystem::path& address_path,
    std::string& error);

std::optional<microlind::logic::BoardLogicDevices> load_board_logic_devices(
    const LogicConfig& cfg,
    std::string& error);

std::vector<LogicValidationIssue> validate_hardware_config_against_logic(
    const HardwareConfig& cfg,
    const microlind::logic::BoardLogicDevices& devices);

std::string generate_partial_hardware_config_from_logic(const microlind::logic::BoardLogicDevices& devices);
std::string format_logic_validation_issue(const LogicValidationIssue& issue);

} // namespace microlind::cli
