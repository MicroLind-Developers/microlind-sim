#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "microlind/app/sim_session.hpp"

namespace microlind::app {

struct VdcRgb {
    uint8_t red{};
    uint8_t green{};
    uint8_t blue{};

    friend bool operator==(const VdcRgb&, const VdcRgb&) = default;
};

struct VdcCellStyle {
    VdcRgb foreground{};
    VdcRgb background{};
    bool reverse{};
    bool underline{};
    bool blink{};
    bool alternate_charset{};
};

struct VdcFramebuffer {
    int width{};
    int height{};
    std::vector<uint8_t> rgba;
};

struct VdcFrameGeometry {
    int width{};
    int height{};
    int byte_columns{};
    int row_groups{};
    int group_height{};
    int stride{};
    std::size_t bitmap_bytes{};
    std::size_t attribute_bytes{};
    const char* error{};
};

[[nodiscard]] bool vdc_bitmap_enabled(const VdcSnapshot& snapshot);
[[nodiscard]] VdcFrameGeometry vdc_frame_geometry(const VdcSnapshot& snapshot);
// nullptr means that the geometry and the mode-specific payload are valid.
[[nodiscard]] const char* vdc_frame_error(const VdcSnapshot& snapshot);

[[nodiscard]] VdcRgb vdc_rgb(uint8_t color);
[[nodiscard]] bool vdc_attributes_enabled(const VdcSnapshot& snapshot);
[[nodiscard]] VdcCellStyle vdc_cell_style(const VdcSnapshot& snapshot, std::size_t cell);
[[nodiscard]] bool vdc_blink_visible(const VdcSnapshot& snapshot, double elapsed_seconds);
[[nodiscard]] int vdc_underline_row(const VdcSnapshot& snapshot, int cell_height);
[[nodiscard]] VdcFramebuffer render_vdc_framebuffer(const VdcSnapshot& snapshot, double elapsed_seconds);

} // namespace microlind::app
