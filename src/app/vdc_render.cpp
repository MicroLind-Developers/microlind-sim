#include "microlind/app/vdc_render.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <utility>

namespace microlind::app {
namespace {

constexpr uint8_t kRegCharacterTotalVertical = 0x09;
constexpr uint8_t kRegCursorStart = 0x0A;
constexpr uint8_t kRegCursorEnd = 0x0B;
constexpr uint8_t kRegCharacterHorizontal = 0x16;
constexpr uint8_t kRegCharacterVertical = 0x17;
constexpr uint8_t kRegVerticalScroll = 0x18;
constexpr uint8_t kRegHorizontalScroll = 0x19;
constexpr uint8_t kRegColor = 0x1A;
constexpr uint8_t kRegUnderlineScan = 0x1D;

constexpr uint8_t kGlobalReverse = 0x40;
constexpr uint8_t kSlowBlink = 0x20;
constexpr uint8_t kAttributesEnabled = 0x40;
constexpr uint8_t kAlternateCharset = 0x80;
constexpr uint8_t kReverse = 0x40;
constexpr uint8_t kUnderline = 0x20;
constexpr uint8_t kBlink = 0x10;
constexpr int kCharacterWidth = 8;
constexpr int kCharacterStride = 16;

constexpr std::array<VdcRgb, 16> kRgbiPalette = {{
    {0x00, 0x00, 0x00}, // black
    {0x55, 0x55, 0x55}, // dark gray
    {0x00, 0x00, 0xAA}, // dark blue
    {0x55, 0x55, 0xFF}, // light blue
    {0x00, 0xAA, 0x00}, // dark green
    {0x55, 0xFF, 0x55}, // light green
    {0x00, 0xAA, 0xAA}, // dark cyan
    {0x55, 0xFF, 0xFF}, // light cyan
    {0xAA, 0x00, 0x00}, // dark red
    {0xFF, 0x55, 0x55}, // light red
    {0xAA, 0x00, 0xAA}, // dark magenta
    {0xFF, 0x55, 0xFF}, // light magenta
    {0xAA, 0x55, 0x00}, // brown
    {0xFF, 0xFF, 0x55}, // yellow
    {0xAA, 0xAA, 0xAA}, // light gray
    {0xFF, 0xFF, 0xFF}, // white
}};

bool blink_phase(double elapsed_seconds, uint64_t frames_per_phase) {
    if (!std::isfinite(elapsed_seconds) || elapsed_seconds < 0.0) return true;

    constexpr double refresh_rate = 50.0;
    const uint64_t frame = static_cast<uint64_t>(std::floor(elapsed_seconds * refresh_rate));
    return ((frame / frames_per_phase) & 1u) == 0;
}

bool cursor_visible(const VdcSnapshot& snapshot, double elapsed_seconds) {
    const uint8_t mode = static_cast<uint8_t>((snapshot.registers[kRegCursorStart] >> 5) & 0x03);
    switch (mode) {
    case 0: return true;
    case 1: return false;
    case 2: return blink_phase(elapsed_seconds, 16);
    case 3: return blink_phase(elapsed_seconds, 32);
    }
    return false;
}

void set_pixel(std::vector<uint8_t>& rgba, int width, int x, int y, const VdcRgb& color) {
    const std::size_t offset = (static_cast<std::size_t>(y) * width + x) * 4;
    rgba[offset] = color.red;
    rgba[offset + 1] = color.green;
    rgba[offset + 2] = color.blue;
    rgba[offset + 3] = 0xFF;
}

} // namespace

VdcRgb vdc_rgb(uint8_t color) {
    return kRgbiPalette[color & 0x0F];
}

bool vdc_attributes_enabled(const VdcSnapshot& snapshot) {
    return (snapshot.registers[kRegHorizontalScroll] & kAttributesEnabled) != 0;
}

VdcCellStyle vdc_cell_style(const VdcSnapshot& snapshot, std::size_t cell) {
    const uint8_t color_register = snapshot.registers[kRegColor];
    uint8_t foreground = static_cast<uint8_t>(color_register >> 4);
    uint8_t background = static_cast<uint8_t>(color_register & 0x0F);

    VdcCellStyle style;
    const bool attributes_enabled = vdc_attributes_enabled(snapshot);
    const uint8_t attribute = attributes_enabled && cell < snapshot.attrs.size() ? snapshot.attrs[cell] : 0;
    if (attributes_enabled) {
        foreground = static_cast<uint8_t>(attribute & 0x0F);
        style.underline = (attribute & kUnderline) != 0;
        style.blink = (attribute & kBlink) != 0;
        style.alternate_charset = (attribute & kAlternateCharset) != 0;
    }

    const bool cell_reverse = attributes_enabled && (attribute & kReverse) != 0;
    style.reverse = ((snapshot.registers[kRegVerticalScroll] & kGlobalReverse) != 0) != cell_reverse;
    if (style.reverse) {
        std::swap(foreground, background);
    }

    style.foreground = vdc_rgb(foreground);
    style.background = vdc_rgb(background);
    return style;
}

bool vdc_blink_visible(const VdcSnapshot& snapshot, double elapsed_seconds) {
    const uint64_t frames_per_phase =
        (snapshot.registers[kRegVerticalScroll] & kSlowBlink) != 0 ? 32u : 16u;
    return blink_phase(elapsed_seconds, frames_per_phase);
}

int vdc_underline_row(const VdcSnapshot& snapshot, int cell_height) {
    if (cell_height <= 1) return 0;

    const int source_height = static_cast<int>(snapshot.registers[kRegCharacterTotalVertical] & 0x1F) + 1;
    const int source_row = std::min(
        static_cast<int>(snapshot.registers[kRegUnderlineScan] & 0x1F),
        source_height - 1);
    return std::clamp(source_row * cell_height / source_height, 0, cell_height - 1);
}

VdcFramebuffer render_vdc_framebuffer(const VdcSnapshot& snapshot, double elapsed_seconds) {
    VdcFramebuffer framebuffer;
    if (!snapshot.present || snapshot.columns == 0 || snapshot.rows == 0) return framebuffer;

    const int cell_width = static_cast<int>((snapshot.registers[kRegCharacterHorizontal] >> 4) & 0x0F) + 1;
    const int displayed_width = std::min({
        static_cast<int>(snapshot.registers[kRegCharacterHorizontal] & 0x0F),
        cell_width,
        kCharacterWidth});
    const int cell_height = static_cast<int>(snapshot.registers[kRegCharacterTotalVertical] & 0x1F) + 1;
    const int displayed_height = std::min(
        static_cast<int>(snapshot.registers[kRegCharacterVertical] & 0x1F),
        cell_height);
    framebuffer.width = static_cast<int>(snapshot.columns) * cell_width;
    framebuffer.height = static_cast<int>(snapshot.rows) * cell_height;
    framebuffer.rgba.resize(static_cast<std::size_t>(framebuffer.width) * framebuffer.height * 4);

    const std::size_t cell_count = std::min(
        static_cast<std::size_t>(snapshot.columns) * snapshot.rows,
        snapshot.chars.size());
    const uint16_t cursor_offset = static_cast<uint16_t>(snapshot.cursor_position - snapshot.display_start);
    const bool draw_cursor = cursor_offset < cell_count && cursor_visible(snapshot, elapsed_seconds);
    const int cursor_start = snapshot.registers[kRegCursorStart] & 0x1F;
    const int cursor_end = snapshot.registers[kRegCursorEnd] & 0x1F;
    const bool blink_visible = vdc_blink_visible(snapshot, elapsed_seconds);
    const int underline_row = vdc_underline_row(snapshot, cell_height);

    for (std::size_t cell = 0; cell < cell_count; ++cell) {
        const VdcCellStyle style = vdc_cell_style(snapshot, cell);
        const int cell_x = static_cast<int>(cell % snapshot.columns) * cell_width;
        const int cell_y = static_cast<int>(cell / snapshot.columns) * cell_height;
        const bool character_visible = !style.blink || blink_visible;
        const uint16_t character = static_cast<uint16_t>(snapshot.chars[cell]) +
            (style.alternate_charset ? 256u : 0u);
        const std::size_t character_offset = static_cast<std::size_t>(character) * kCharacterStride;

        for (int y = 0; y < cell_height; ++y) {
            uint8_t row_bits = 0;
            if (character_visible && y < displayed_height && y < kCharacterStride) {
                row_bits = snapshot.character_data[character_offset + static_cast<std::size_t>(y)];
            }
            const bool underline = character_visible && style.underline && y == underline_row;
            const bool cursor_scan = draw_cursor && cell == cursor_offset &&
                (cursor_start <= cursor_end
                    ? y >= cursor_start && y <= cursor_end
                    : y >= cursor_start || y <= cursor_end);

            for (int x = 0; x < cell_width; ++x) {
                const bool glyph_pixel = x < displayed_width && (row_bits & (0x80u >> x)) != 0;
                const bool foreground = glyph_pixel || (underline && x < displayed_width) ||
                    (cursor_scan && x < displayed_width);
                set_pixel(
                    framebuffer.rgba,
                    framebuffer.width,
                    cell_x + x,
                    cell_y + y,
                    foreground ? style.foreground : style.background);
            }
        }
    }

    return framebuffer;
}

} // namespace microlind::app
