#pragma once

#include <array>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include "microlind/bus.hpp"

namespace microlind::cli {

struct RomRegion {
    uint16_t start{};
    uint16_t end{};
};

struct RamConfig {
    uint16_t start{};
    uint16_t end{};
    uint32_t bank_size{0};
    uint32_t available{0};
    bool present{false};
};

struct SerialConfig {
    uint16_t start{};
    uint16_t end{};
    uint8_t irq_level{1};
    bool present{false};
};

struct CfConfig {
    uint16_t start{};
    uint16_t end{};
    std::filesystem::path image_path;
    uint32_t sectors{0};
    bool read_only{false};
    bool present{false};
};

struct MapperWindowConfig {
    uint16_t start{};
    uint16_t end{};
    bool present{false};
};

struct MapperConfig {
    uint16_t bank_reg[4]{};
    std::array<MapperWindowConfig, 4> windows{};
    bool present{false};
};

struct LogicConfig {
    std::filesystem::path signal_logic_path;
    std::filesystem::path memory_logic_path;
    std::filesystem::path address_logic_path;
    BusDecodeMode bus_mode{BusDecodeMode::Validate};
    bool present{false};
};

struct HardwareConfig {
    std::vector<RomRegion> roms;
    RamConfig ram;
    SerialConfig serial;
    CfConfig cf;
    MapperConfig mapper;
    LogicConfig logic;
};

std::optional<HardwareConfig> load_hardware_config(const std::filesystem::path& path, std::string& error);

} // namespace microlind::cli
