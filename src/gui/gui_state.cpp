#include "gui_state.hpp"

#include <cctype>
#include <cstdio>
#include <cstring>
#include <iomanip>
#include <iterator>
#include <optional>
#include <sstream>

#include <png.h>

#include "microlind/app/util.hpp"

namespace microlind::gui {

std::string hex_value(uint32_t value, int width) {
    std::ostringstream out;
    out << "0x" << std::uppercase << std::hex << std::setfill('0') << std::setw(width) << value;
    return out.str();
}

std::string instruction_bytes(microlind::Bus& bus, uint16_t pc, uint8_t length) {
    std::ostringstream out;
    out << std::uppercase << std::hex << std::setfill('0');
    for (uint8_t i = 0; i < length; ++i) {
        if (i > 0) out << ' ';
        out << std::setw(2) << static_cast<int>(bus.peek8(static_cast<uint16_t>(pc + i)));
    }
    return out.str();
}


TextureResource load_png_texture(SDL_Renderer* renderer, const std::filesystem::path& path) {
    png_image image{};
    image.version = PNG_IMAGE_VERSION;
    if (png_image_begin_read_from_file(&image, path.string().c_str()) == 0) {
        return {};
    }

    image.format = PNG_FORMAT_RGBA;
    std::vector<uint8_t> pixels(PNG_IMAGE_SIZE(image));
    if (png_image_finish_read(&image, nullptr, pixels.data(), 0, nullptr) == 0) {
        png_image_free(&image);
        return {};
    }

    SDL_Texture* texture = SDL_CreateTexture(
        renderer,
        SDL_PIXELFORMAT_RGBA32,
        SDL_TEXTUREACCESS_STATIC,
        static_cast<int>(image.width),
        static_cast<int>(image.height));
    if (!texture) {
        png_image_free(&image);
        return {};
    }

    SDL_UpdateTexture(texture, nullptr, pixels.data(), static_cast<int>(image.width * 4));
    SDL_SetTextureBlendMode(texture, SDL_BLENDMODE_BLEND);

    TextureResource result{texture, static_cast<int>(image.width), static_cast<int>(image.height)};
    png_image_free(&image);
    return result;
}

SDL_Surface* load_png_surface(const std::filesystem::path& path) {
    png_image image{};
    image.version = PNG_IMAGE_VERSION;
    if (png_image_begin_read_from_file(&image, path.string().c_str()) == 0) {
        return nullptr;
    }

    image.format = PNG_FORMAT_RGBA;
    std::vector<uint8_t> pixels(PNG_IMAGE_SIZE(image));
    if (png_image_finish_read(&image, nullptr, pixels.data(), 0, nullptr) == 0) {
        png_image_free(&image);
        return nullptr;
    }

    SDL_Surface* surface = SDL_CreateRGBSurfaceWithFormat(
        0,
        static_cast<int>(image.width),
        static_cast<int>(image.height),
        32,
        SDL_PIXELFORMAT_RGBA32);
    if (surface == nullptr) {
        png_image_free(&image);
        return nullptr;
    }

    const int source_pitch = static_cast<int>(image.width * 4);
    const int copy_rows = static_cast<int>(image.height);
    auto* destination = static_cast<uint8_t*>(surface->pixels);
    const auto* source = pixels.data();
    for (int y = 0; y < copy_rows; ++y) {
        std::memcpy(destination + y * surface->pitch, source + y * source_pitch, static_cast<std::size_t>(source_pitch));
    }

    png_image_free(&image);
    return surface;
}

std::string serial_terminal_text(const std::vector<uint8_t>& bytes) {
    std::string out;
    out.reserve(bytes.size());
    for (uint8_t value : bytes) {
        if (value == '\r') continue;
        if (value == '\n' || value == '\t' || std::isprint(static_cast<unsigned char>(value))) {
            out.push_back(static_cast<char>(value));
        } else {
            out.push_back('.');
        }
    }
    return out;
}

std::vector<uint8_t> parse_hex_bytes(std::string_view input, bool& ok) {
    std::vector<uint8_t> bytes;
    std::string token;
    ok = true;

    auto flush_token = [&]() {
        if (token.empty()) return;
        if (token.size() > 2) {
            ok = false;
            token.clear();
            return;
        }
        uint32_t value = 0;
        std::istringstream in(token);
        in >> std::hex >> value;
        if (!in || value > 0xFF) {
            ok = false;
        } else {
            bytes.push_back(static_cast<uint8_t>(value));
        }
        token.clear();
    };

    for (char ch : input) {
        if (std::isxdigit(static_cast<unsigned char>(ch))) {
            token.push_back(ch);
        } else if (std::isspace(static_cast<unsigned char>(ch)) || ch == ',' || ch == ';') {
            flush_token();
        } else {
            ok = false;
        }
    }
    flush_token();
    if (!ok) bytes.clear();
    return bytes;
}

const char* watchpoint_type_label(microlind::app::WatchpointType type) {
    switch (type) {
    case microlind::app::WatchpointType::Read: return "R";
    case microlind::app::WatchpointType::Write: return "W";
    case microlind::app::WatchpointType::ReadWrite: return "RW";
    }
    return "?";
}

const char* cf_transfer_label(microlind::app::CfTransferMode mode) {
    switch (mode) {
    case microlind::app::CfTransferMode::None: return "Idle";
    case microlind::app::CfTransferMode::Read: return "Read";
    case microlind::app::CfTransferMode::Write: return "Write";
    }
    return "?";
}

std::string cf_status_flags(uint8_t status) {
    std::vector<std::string_view> flags;
    if ((status & 0x01) != 0) flags.push_back("ERR");
    if ((status & 0x08) != 0) flags.push_back("DRQ");
    if ((status & 0x10) != 0) flags.push_back("DSC");
    if ((status & 0x40) != 0) flags.push_back("DRDY");
    if (flags.empty()) return "-";

    std::ostringstream out;
    for (std::size_t i = 0; i < flags.size(); ++i) {
        if (i > 0) out << ' ';
        out << flags[i];
    }
    return out.str();
}

std::string cf_command_name(uint8_t command) {
    switch (command) {
    case 0x00: return "-";
    case 0x10: return "Recalibrate";
    case 0x20:
    case 0x21: return "Read Sectors";
    case 0x30:
    case 0x31: return "Write Sectors";
    case 0x40:
    case 0x41: return "Read Verify";
    case 0x50: return "Format Track";
    case 0x90: return "Execute Diagnostic";
    case 0x91: return "Initialize Drive Parameters";
    case 0x95:
    case 0xE1: return "Idle Immediate";
    case 0x96:
    case 0xE2: return "Standby";
    case 0x97:
    case 0xE3: return "Idle";
    case 0x98:
    case 0xE5: return "Check Power Mode";
    case 0x99:
    case 0xE6: return "Sleep";
    case 0xC0: return "Erase Sectors";
    case 0xC4: return "Read Multiple";
    case 0xC5: return "Write Multiple";
    case 0xC6: return "Set Multiple Mode";
    case 0xEC: return "Identify";
    case 0xEF: return "Set Features";
    default: return "Unknown";
    }
}

#ifdef MICROLIND_HAS_PORTABLE_FILE_DIALOGS
std::string pick_file(const std::string& title, const std::vector<std::string>& filters) {
    const auto result = pfd::open_file(title, "", filters).result();
    if (result.empty()) return {};
    return result.front();
}

std::string pick_save_file(const std::string& title, const std::string& default_path, const std::vector<std::string>& filters) {
    return pfd::save_file(title, default_path, filters).result();
}
#endif

microlind::cli::RomFormat selected_rom_format(int index) {
    switch (index) {
    case 0: return microlind::cli::RomFormat::Raw;
    case 1: return microlind::cli::RomFormat::Ihex;
    case 2: return microlind::cli::RomFormat::Srec;
    default: return microlind::cli::RomFormat::Ihex;
    }
}

int rom_format_combo_index(microlind::cli::RomFormat format) {
    switch (format) {
    case microlind::cli::RomFormat::Raw: return 0;
    case microlind::cli::RomFormat::Ihex: return 1;
    case microlind::cli::RomFormat::Srec: return 2;
    case microlind::cli::RomFormat::None: return 1;
    }
    return 1;
}

void set_next_window_defaults(float x, float y, float w, float h) {
    ImGui::SetNextWindowPos(ImVec2(x, y), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(w, h), ImGuiCond_FirstUseEver);
}

} // namespace microlind::gui
