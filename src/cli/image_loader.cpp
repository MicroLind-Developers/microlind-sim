#include "microlind/app/image_loader.hpp"

#include <fstream>
#include <iterator>
#include <string>
#include <string_view>

namespace microlind::cli {

std::vector<uint8_t> load_file(const std::filesystem::path& path) {
    std::ifstream file(path, std::ios::binary);
    return std::vector<uint8_t>(std::istreambuf_iterator<char>(file), {});
}

// Very small Intel HEX loader: only supports data records (00) and EOF (01).
// Address range is clipped to 64K.
std::vector<uint8_t> load_ihex(const std::filesystem::path& path) {
    std::vector<uint8_t> image(65536, 0xFF);
    std::ifstream file(path);
    std::string line;
    while (std::getline(file, line)) {
        if (line.empty() || line[0] != ':') continue;
        std::string_view sv(line);
        auto hex = [](std::string_view s) -> uint8_t {
            return static_cast<uint8_t>(std::stoul(std::string{s}, nullptr, 16));
        };
        const uint8_t count = hex(sv.substr(1, 2));
        const uint16_t addr = static_cast<uint16_t>(std::stoul(std::string{sv.substr(3, 4)}, nullptr, 16));
        const uint8_t type = hex(sv.substr(7, 2));
        if (type == 0x00) {
            for (uint8_t i = 0; i < count; ++i) {
                const size_t idx = 9 + i * 2;
                if (idx + 2 > sv.size()) break;
                uint8_t b = hex(sv.substr(idx, 2));
                const uint16_t a = static_cast<uint16_t>(addr + i);
                image[a] = b;
            }
        } else if (type == 0x01) {
            break; // EOF
        }
    }
    return image;
}

// Minimal S-Record (S1/S2/S3) loader. Clips addresses to 64K.
std::vector<uint8_t> load_srec(const std::filesystem::path& path) {
    std::vector<uint8_t> image(65536, 0xFF);
    std::ifstream file(path);
    std::string line;
    auto hex = [](std::string_view s) -> uint8_t {
        return static_cast<uint8_t>(std::stoul(std::string{s}, nullptr, 16));
    };
    while (std::getline(file, line)) {
        if (line.size() < 4 || line[0] != 'S') continue;
        const char t = line[1];
        if (t == '1' || t == '2' || t == '3') {
            std::string_view sv(line);
            const uint8_t count = hex(sv.substr(2, 2));
            size_t idx = 4;
            uint32_t addr = 0;
            if (t == '1') {
                addr = std::stoul(std::string{sv.substr(idx, 4)}, nullptr, 16);
                idx += 4;
            } else if (t == '2') {
                addr = std::stoul(std::string{sv.substr(idx, 6)}, nullptr, 16);
                idx += 6;
            } else {
                addr = std::stoul(std::string{sv.substr(idx, 8)}, nullptr, 16);
                idx += 8;
            }
            const uint8_t data_len = static_cast<uint8_t>(count - (idx / 2 - 2) - 1);
            for (uint8_t i = 0; i < data_len; ++i) {
                if (idx + 2 > sv.size()) break;
                uint8_t b = hex(sv.substr(idx, 2));
                idx += 2;
                image[static_cast<uint16_t>(addr + i)] = b;
            }
        }
    }
    return image;
}

std::optional<LoadedImage> load_image(
    const std::filesystem::path& path,
    RomFormat fmt,
    uint16_t base_override) {
    LoadedImage img;
    switch (fmt) {
    case RomFormat::Raw:
        img.data = load_file(path);
        img.base = base_override;
        break;
    case RomFormat::Ihex:
        img.data = load_ihex(path);
        img.base = 0x0000;
        break;
    case RomFormat::Srec:
        img.data = load_srec(path);
        img.base = 0x0000;
        break;
    case RomFormat::None:
        return std::nullopt;
    }
    return img;
}

} // namespace microlind::cli
