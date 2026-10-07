#pragma once

#include "microlind/app/hardware_config.hpp"

namespace microlind::app {

struct ConfigAddressRange {
    std::string section;
    std::string label;
    uint16_t start{};
    uint16_t end{};
};

struct ConfigIssue {
    std::string section;
    std::string message;
    bool warning{};
};

std::vector<ConfigAddressRange> config_address_map(const cli::HardwareConfig& config);
std::vector<ConfigIssue> validate_config(const cli::HardwareConfig& config);
std::string serialize_config(const cli::HardwareConfig& resolved_config,
                             const std::filesystem::path& destination,
                             const std::string& original_source = {});

// GUI-independent draft. Relative paths in config are based on path's directory.
class ConfigEditor {
public:
    ConfigEditor();
    void new_config();
    bool open(const std::filesystem::path& source, std::string& error);
    bool save(const std::filesystem::path& destination, std::string& error);
    void discard();
    [[nodiscard]] cli::HardwareConfig effective_config() const;

    cli::HardwareConfig config;
    bool rom_enabled{true};
    bool modified{};
    std::filesystem::path path;

private:
    cli::HardwareConfig saved_config_;
    bool saved_rom_enabled_{true};
    std::string source_;
};

} // namespace microlind::app
