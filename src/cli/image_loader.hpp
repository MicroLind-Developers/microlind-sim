#pragma once

#include <cstdint>
#include <filesystem>
#include <optional>
#include <vector>

namespace microlind::cli {

enum class RomFormat { None, Raw, Ihex, Srec };

struct LoadedImage {
    std::vector<uint8_t> data;
    uint16_t base{0};
};

std::vector<uint8_t> load_file(const std::filesystem::path& path);
std::vector<uint8_t> load_ihex(const std::filesystem::path& path);
std::vector<uint8_t> load_srec(const std::filesystem::path& path);
std::optional<LoadedImage> load_image(
    const std::filesystem::path& path,
    RomFormat fmt,
    uint16_t base_override = 0x8000);

} // namespace microlind::cli
