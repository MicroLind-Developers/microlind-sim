#include "gui_state.hpp"

#include <array>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <iomanip>
#include <iterator>
#include <optional>
#include <sstream>

#include <png.h>

#include "microlind/app/util.hpp"

namespace microlind::gui {
namespace {

constexpr std::array<uint16_t, 128> kCp437Unicode = {
    0x00C7, 0x00FC, 0x00E9, 0x00E2, 0x00E4, 0x00E0, 0x00E5, 0x00E7,
    0x00EA, 0x00EB, 0x00E8, 0x00EF, 0x00EE, 0x00EC, 0x00C4, 0x00C5,
    0x00C9, 0x00E6, 0x00C6, 0x00F4, 0x00F6, 0x00F2, 0x00FB, 0x00F9,
    0x00FF, 0x00D6, 0x00DC, 0x00A2, 0x00A3, 0x00A5, 0x20A7, 0x0192,
    0x00E1, 0x00ED, 0x00F3, 0x00FA, 0x00F1, 0x00D1, 0x00AA, 0x00BA,
    0x00BF, 0x2310, 0x00AC, 0x00BD, 0x00BC, 0x00A1, 0x00AB, 0x00BB,
    0x2591, 0x2592, 0x2593, 0x2502, 0x2524, 0x2561, 0x2562, 0x2556,
    0x2555, 0x2563, 0x2551, 0x2557, 0x255D, 0x255C, 0x255B, 0x2510,
    0x2514, 0x2534, 0x252C, 0x251C, 0x2500, 0x253C, 0x255E, 0x255F,
    0x255A, 0x2554, 0x2569, 0x2566, 0x2560, 0x2550, 0x256C, 0x2567,
    0x2568, 0x2564, 0x2565, 0x2559, 0x2558, 0x2552, 0x2553, 0x256B,
    0x256A, 0x2518, 0x250C, 0x2588, 0x2584, 0x258C, 0x2590, 0x2580,
    0x03B1, 0x00DF, 0x0393, 0x03C0, 0x03A3, 0x03C3, 0x00B5, 0x03C4,
    0x03A6, 0x0398, 0x03A9, 0x03B4, 0x221E, 0x03C6, 0x03B5, 0x2229,
    0x2261, 0x00B1, 0x2265, 0x2264, 0x2320, 0x2321, 0x00F7, 0x2248,
    0x00B0, 0x2219, 0x00B7, 0x221A, 0x207F, 0x00B2, 0x25A0, 0x00A0,
};

void append_utf8(std::string& out, uint32_t codepoint) {
    if (codepoint <= 0x7F) {
        out.push_back(static_cast<char>(codepoint));
    } else if (codepoint <= 0x7FF) {
        out.push_back(static_cast<char>(0xC0 | (codepoint >> 6)));
        out.push_back(static_cast<char>(0x80 | (codepoint & 0x3F)));
    } else {
        out.push_back(static_cast<char>(0xE0 | (codepoint >> 12)));
        out.push_back(static_cast<char>(0x80 | ((codepoint >> 6) & 0x3F)));
        out.push_back(static_cast<char>(0x80 | (codepoint & 0x3F)));
    }
}

char vdc_glyph(uint8_t value) {
    return value >= 0x20 && value <= 0x7E ? static_cast<char>(value) : '.';
}

void blend_pixel(uint8_t* pixel, const std::array<uint8_t, 3>& color, uint8_t alpha) {
    const unsigned inverse_alpha = 255u - alpha;
    for (std::size_t channel = 0; channel < color.size(); ++channel) {
        pixel[channel] = static_cast<uint8_t>(
            (static_cast<unsigned>(color[channel]) * alpha +
             static_cast<unsigned>(pixel[channel]) * inverse_alpha + 127u) /
            255u);
    }
}

} // namespace

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

bool save_vdc_screenshot_png(
    const std::filesystem::path& path,
    const microlind::app::VdcSnapshot& vdc,
    ImFont* font,
    std::string& error) {
    error.clear();
    if (!vdc.present) {
        error = "No VDC is configured.";
        return false;
    }
    if (font == nullptr || font->OwnerAtlas == nullptr) {
        error = "The VDC font is unavailable.";
        return false;
    }
    if (vdc.columns == 0 || vdc.rows == 0 ||
        static_cast<std::size_t>(vdc.columns) * vdc.rows > vdc.chars.size()) {
        error = "The VDC frame dimensions are invalid.";
        return false;
    }

    constexpr float font_size = 16.0f;
    ImFontBaked* baked = font->GetFontBaked(font_size);
    if (baked == nullptr) {
        error = "The VDC font could not be rasterized.";
        return false;
    }

    // Preload the complete VDC display range before retaining atlas pointers or UVs.
    for (ImWchar codepoint = 0x20; codepoint <= 0x7E; ++codepoint) {
        baked->FindGlyph(codepoint);
    }

    const ImTextureData* atlas = font->OwnerAtlas->TexData;
    if (atlas == nullptr || atlas->Pixels == nullptr || atlas->Width <= 0 || atlas->Height <= 0 ||
        (atlas->BytesPerPixel != 1 && atlas->BytesPerPixel != 4)) {
        error = "The VDC font atlas is unavailable.";
        return false;
    }

    const ImFontGlyph* widest = baked->FindGlyph('M');
    const int cell_width = std::max(1, static_cast<int>(std::ceil(widest != nullptr ? widest->AdvanceX : font_size / 2.0f)));
    const int cell_height = std::max(1, static_cast<int>(std::ceil(baked->Size)));
    const int width = static_cast<int>(vdc.columns) * cell_width;
    const int height = static_cast<int>(vdc.rows) * cell_height;

    constexpr std::array<uint8_t, 3> background{16, 18, 20};
    constexpr std::array<uint8_t, 3> foreground{226, 230, 234};
    constexpr std::array<uint8_t, 3> cursor_foreground{242, 224, 82};
    std::vector<uint8_t> pixels(static_cast<std::size_t>(width) * height * 4);
    for (std::size_t offset = 0; offset < pixels.size(); offset += 4) {
        pixels[offset] = background[0];
        pixels[offset + 1] = background[1];
        pixels[offset + 2] = background[2];
        pixels[offset + 3] = 255;
    }

    const uint16_t cursor_offset = static_cast<uint16_t>(vdc.cursor_position - vdc.display_start);
    const bool cursor_visible = cursor_offset < static_cast<std::size_t>(vdc.columns) * vdc.rows;

    for (std::size_t cell = 0; cell < static_cast<std::size_t>(vdc.columns) * vdc.rows; ++cell) {
        const ImFontGlyph* found = baked->FindGlyph(static_cast<ImWchar>(vdc_glyph(vdc.chars[cell])));
        if (found == nullptr || !found->Visible) continue;
        const ImFontGlyph glyph = *found;

        const int source_x = static_cast<int>(std::lround(glyph.U0 * atlas->Width));
        const int source_y = static_cast<int>(std::lround(glyph.V0 * atlas->Height));
        const int source_width = std::max(1, static_cast<int>(std::lround((glyph.U1 - glyph.U0) * atlas->Width)));
        const int source_height = std::max(1, static_cast<int>(std::lround((glyph.V1 - glyph.V0) * atlas->Height)));
        const int glyph_width = std::max(1, static_cast<int>(std::ceil(glyph.X1 - glyph.X0)));
        const int glyph_height = std::max(1, static_cast<int>(std::ceil(glyph.Y1 - glyph.Y0)));
        const int cell_x = static_cast<int>(cell % vdc.columns) * cell_width;
        const int cell_y = static_cast<int>(cell / vdc.columns) * cell_height;
        const int destination_x = cell_x + static_cast<int>(std::floor(glyph.X0));
        const int destination_y = cell_y + static_cast<int>(std::floor(glyph.Y0));
        const auto& color = cursor_visible && cell == cursor_offset ? cursor_foreground : foreground;

        for (int y = 0; y < glyph_height; ++y) {
            const int output_y = destination_y + y;
            if (output_y < 0 || output_y >= height) continue;
            const int atlas_y = source_y + y * source_height / glyph_height;
            if (atlas_y < 0 || atlas_y >= atlas->Height) continue;
            for (int x = 0; x < glyph_width; ++x) {
                const int output_x = destination_x + x;
                if (output_x < 0 || output_x >= width) continue;
                const int atlas_x = source_x + x * source_width / glyph_width;
                if (atlas_x < 0 || atlas_x >= atlas->Width) continue;

                const std::size_t atlas_offset =
                    (static_cast<std::size_t>(atlas_y) * atlas->Width + atlas_x) * atlas->BytesPerPixel;
                const uint8_t alpha = atlas->Pixels[atlas_offset + (atlas->BytesPerPixel == 4 ? 3 : 0)];
                uint8_t* output = pixels.data() +
                    (static_cast<std::size_t>(output_y) * width + output_x) * 4;
                blend_pixel(output, color, alpha);
            }
        }
    }

    png_image image{};
    image.version = PNG_IMAGE_VERSION;
    image.width = static_cast<png_uint_32>(width);
    image.height = static_cast<png_uint_32>(height);
    image.format = PNG_FORMAT_RGBA;
    if (png_image_write_to_file(&image, path.string().c_str(), 0, pixels.data(), 0, nullptr) == 0) {
        error = image.message[0] != '\0' ? image.message : "libpng could not write the file.";
        png_image_free(&image);
        return false;
    }
    png_image_free(&image);
    return true;
}

std::string serial_terminal_text(const std::vector<uint8_t>& bytes) {
    std::string out;
    out.reserve(bytes.size());

    for (std::size_t i = 0; i < bytes.size(); ++i) {
        const uint8_t value = bytes[i];
        if (value == '\r' || value == '\n') {
            out.push_back('\n');
            if (i + 1 < bytes.size()) {
                const uint8_t next = bytes[i + 1];
                if ((value == '\r' && next == '\n') || (value == '\n' && next == '\r')) {
                    ++i;
                }
            }
        } else if (value == '\t') {
            out.push_back('\t');
        } else if (value >= 0x20 && value <= 0x7E) {
            out.push_back(static_cast<char>(value));
        } else if (value >= 0x80) {
            append_utf8(out, kCp437Unicode[static_cast<std::size_t>(value - 0x80)]);
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
