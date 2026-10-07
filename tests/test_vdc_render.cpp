#include <gtest/gtest.h>
#include <utility>

#include "microlind/app/vdc_render.hpp"

namespace {

using microlind::app::VdcRgb;
using microlind::app::VdcSnapshot;

VdcRgb pixel(const microlind::app::VdcFramebuffer& framebuffer, int x, int y) {
    const std::size_t offset = (static_cast<std::size_t>(y) * framebuffer.width + x) * 4;
    return {
        framebuffer.rgba[offset],
        framebuffer.rgba[offset + 1],
        framebuffer.rgba[offset + 2]};
}

TEST(VdcRenderTest, UsesColorRegisterWhenAttributesAreDisabled) {
    VdcSnapshot snapshot;
    snapshot.registers[0x1A] = 0xF2;
    snapshot.attrs[0] = 0x79;

    const auto style = microlind::app::vdc_cell_style(snapshot, 0);

    EXPECT_EQ(style.foreground, (VdcRgb{0xFF, 0xFF, 0xFF}));
    EXPECT_EQ(style.background, (VdcRgb{0x00, 0x00, 0xAA}));
    EXPECT_FALSE(style.reverse);
    EXPECT_FALSE(style.underline);
    EXPECT_FALSE(style.blink);
    EXPECT_FALSE(style.alternate_charset);
}

TEST(VdcRenderTest, DecodesEnabledCellAttributes) {
    VdcSnapshot snapshot;
    snapshot.registers[0x19] = 0x40;
    snapshot.registers[0x1A] = 0xF2;
    snapshot.attrs[0] = 0xF9;

    const auto style = microlind::app::vdc_cell_style(snapshot, 0);

    EXPECT_EQ(style.foreground, (VdcRgb{0x00, 0x00, 0xAA}));
    EXPECT_EQ(style.background, (VdcRgb{0xFF, 0x55, 0x55}));
    EXPECT_TRUE(style.reverse);
    EXPECT_TRUE(style.underline);
    EXPECT_TRUE(style.blink);
    EXPECT_TRUE(style.alternate_charset);
}

TEST(VdcRenderTest, GlobalAndCellReverseCancelEachOther) {
    VdcSnapshot snapshot;
    snapshot.registers[0x18] = 0x40;
    snapshot.registers[0x19] = 0x40;
    snapshot.registers[0x1A] = 0xF2;
    snapshot.attrs[0] = 0x49;

    const auto style = microlind::app::vdc_cell_style(snapshot, 0);

    EXPECT_FALSE(style.reverse);
    EXPECT_EQ(style.foreground, (VdcRgb{0xFF, 0x55, 0x55}));
    EXPECT_EQ(style.background, (VdcRgb{0x00, 0x00, 0xAA}));
}

TEST(VdcRenderTest, BlinkRateSelectsSixteenOrThirtyTwoFramePhases) {
    VdcSnapshot snapshot;

    EXPECT_TRUE(microlind::app::vdc_blink_visible(snapshot, 0.31));
    EXPECT_FALSE(microlind::app::vdc_blink_visible(snapshot, 0.32));
    EXPECT_TRUE(microlind::app::vdc_blink_visible(snapshot, 0.64));

    snapshot.registers[0x18] = 0x20;
    EXPECT_TRUE(microlind::app::vdc_blink_visible(snapshot, 0.63));
    EXPECT_FALSE(microlind::app::vdc_blink_visible(snapshot, 0.64));
}

TEST(VdcRenderTest, ScalesUnderlineScanLineToRenderedCell) {
    VdcSnapshot snapshot;
    snapshot.registers[0x09] = 0x0F;
    snapshot.registers[0x1D] = 0x0F;

    EXPECT_EQ(microlind::app::vdc_underline_row(snapshot, 16), 15);
    EXPECT_EQ(microlind::app::vdc_underline_row(snapshot, 8), 7);
}

TEST(VdcRenderTest, RendersCharacterRamBitsIntoNativeFramebuffer) {
    VdcSnapshot snapshot;
    snapshot.present = true;
    snapshot.columns = 1;
    snapshot.rows = 1;
    snapshot.registers[0x09] = 0x01;
    snapshot.registers[0x0A] = 0x20; // cursor disabled
    snapshot.registers[0x16] = 0x78;
    snapshot.registers[0x17] = 0x02;
    snapshot.registers[0x1A] = 0xF0;
    snapshot.chars[0] = 1;
    snapshot.character_data[16] = 0x81;
    snapshot.character_data[17] = 0x40;

    const auto framebuffer = microlind::app::render_vdc_framebuffer(snapshot, 0.0);

    ASSERT_EQ(framebuffer.width, 8);
    ASSERT_EQ(framebuffer.height, 2);
    EXPECT_EQ(pixel(framebuffer, 0, 0), (VdcRgb{0xFF, 0xFF, 0xFF}));
    EXPECT_EQ(pixel(framebuffer, 1, 0), (VdcRgb{0x00, 0x00, 0x00}));
    EXPECT_EQ(pixel(framebuffer, 7, 0), (VdcRgb{0xFF, 0xFF, 0xFF}));
    EXPECT_EQ(pixel(framebuffer, 1, 1), (VdcRgb{0xFF, 0xFF, 0xFF}));
}

TEST(VdcRenderTest, AlternateAttributeSelectsUpperCharacterRamBank) {
    VdcSnapshot snapshot;
    snapshot.present = true;
    snapshot.columns = 1;
    snapshot.rows = 1;
    snapshot.registers[0x09] = 0x00;
    snapshot.registers[0x0A] = 0x20;
    snapshot.registers[0x16] = 0x78;
    snapshot.registers[0x17] = 0x01;
    snapshot.registers[0x19] = 0x40;
    snapshot.registers[0x1A] = 0xF0;
    snapshot.chars[0] = 1;
    snapshot.attrs[0] = 0x8F;
    snapshot.character_data[(256 + 1) * 16] = 0x80;

    const auto framebuffer = microlind::app::render_vdc_framebuffer(snapshot, 0.0);

    EXPECT_EQ(pixel(framebuffer, 0, 0), (VdcRgb{0xFF, 0xFF, 0xFF}));
    EXPECT_EQ(pixel(framebuffer, 1, 0), (VdcRgb{0x00, 0x00, 0x00}));
}

TEST(VdcRenderTest, PreservesHorizontalAndVerticalCharacterSpacing) {
    VdcSnapshot snapshot;
    snapshot.present = true;
    snapshot.columns = 1;
    snapshot.rows = 1;
    snapshot.registers[0x09] = 0x02;
    snapshot.registers[0x0A] = 0x20;
    snapshot.registers[0x16] = 0x97;
    snapshot.registers[0x17] = 0x02;
    snapshot.registers[0x1A] = 0xF0;
    snapshot.chars[0] = 1;
    snapshot.character_data[16] = 0xFF;
    snapshot.character_data[17] = 0xFF;

    const auto framebuffer = microlind::app::render_vdc_framebuffer(snapshot, 0.0);

    ASSERT_EQ(framebuffer.width, 10);
    ASSERT_EQ(framebuffer.height, 3);
    EXPECT_EQ(pixel(framebuffer, 6, 0), (VdcRgb{0xFF, 0xFF, 0xFF}));
    EXPECT_EQ(pixel(framebuffer, 7, 0), (VdcRgb{0x00, 0x00, 0x00}));
    EXPECT_EQ(pixel(framebuffer, 0, 2), (VdcRgb{0x00, 0x00, 0x00}));
}

VdcSnapshot bitmap_snapshot(uint8_t columns = 2, uint8_t rows = 2, uint8_t group_height = 2) {
    VdcSnapshot snapshot;
    snapshot.present = true;
    snapshot.registers[0x01] = columns;
    snapshot.registers[0x06] = rows;
    snapshot.registers[0x09] = group_height - 1;
    snapshot.registers[0x16] = 0x78;
    snapshot.registers[0x17] = group_height;
    snapshot.registers[0x19] = 0x80;
    snapshot.registers[0x1A] = 0xF2;
    snapshot.bitmap_data.resize(static_cast<std::size_t>(columns) * rows * group_height);
    return snapshot;
}

TEST(VdcRenderTest, BitmapUsesScanLineOrderAndMostSignificantBitFirst) {
    auto snapshot = bitmap_snapshot();
    snapshot.bitmap_data = {0x80, 0x01, 0xAA, 0x55, 0xFF, 0x00, 0x00, 0xFF};
    // Text state and a solid cursor must not decorate the bitmap.
    snapshot.chars.fill(255);
    snapshot.attrs.fill(255);
    snapshot.character_data.fill(255);
    const auto frame = microlind::app::render_vdc_framebuffer(snapshot, 0.4);
    ASSERT_EQ(frame.width, 16);
    ASSERT_EQ(frame.height, 4);
    const auto foreground = microlind::app::vdc_rgb(15);
    const auto background = microlind::app::vdc_rgb(2);
    for (int y = 0; y < 4; ++y) {
        for (int x = 0; x < 16; ++x) {
            EXPECT_EQ(pixel(frame, x, y),
                (snapshot.bitmap_data[y * 2 + x / 8] & (0x80 >> (x % 8))) ? foreground : background);
            EXPECT_EQ(frame.rgba[(y * 16 + x) * 4 + 3], 255);
        }
    }
}

TEST(VdcRenderTest, BitmapAttributesUseBothColorNibblesPerRowGroup) {
    auto snapshot = bitmap_snapshot();
    snapshot.registers[0x19] = 0xC0;
    snapshot.bitmap_data.assign(8, 0xAA);
    snapshot.bitmap_attrs = {0x2F, 0xF2, 0x49, 0x94};
    const auto frame = microlind::app::render_vdc_framebuffer(snapshot, 0.4);
    ASSERT_EQ(frame.height, 4);
    for (int y = 0; y < 4; ++y) {
        for (int column = 0; column < 2; ++column) {
            const auto color = snapshot.bitmap_attrs[(y / 2) * 2 + column];
            EXPECT_EQ(pixel(frame, column * 8, y), microlind::app::vdc_rgb(color & 0x0F));
            EXPECT_EQ(pixel(frame, column * 8 + 1, y), microlind::app::vdc_rgb(color >> 4));
        }
    }
    snapshot.registers[0x18] = 0x40;
    const auto reversed = microlind::app::render_vdc_framebuffer(snapshot, 0.4);
    EXPECT_EQ(pixel(reversed, 0, 0), microlind::app::vdc_rgb(2));
    EXPECT_EQ(pixel(reversed, 1, 0), microlind::app::vdc_rgb(15));

    snapshot.registers[0x18] = 0;
    snapshot.registers[0x19] = 0x80;
    snapshot.bitmap_attrs.clear();
    const auto global = microlind::app::render_vdc_framebuffer(snapshot, 0.4);
    EXPECT_EQ(pixel(global, 8, 0), microlind::app::vdc_rgb(15));
    EXPECT_EQ(pixel(global, 9, 0), microlind::app::vdc_rgb(2));
}

TEST(VdcRenderTest, BitmapHasNative640By200DimensionsAndCanSwitchBackToText) {
    auto snapshot = bitmap_snapshot(80, 25, 8);
    snapshot.bitmap_data.front() = 0x80;
    snapshot.bitmap_data.back() = 0x01;
    const auto bitmap = microlind::app::render_vdc_framebuffer(snapshot, 0.0);
    ASSERT_EQ(bitmap.width, 640);
    ASSERT_EQ(bitmap.height, 200);
    EXPECT_EQ(bitmap.rgba.size(), 640u * 200u * 4u);
    EXPECT_EQ(pixel(bitmap, 0, 0), microlind::app::vdc_rgb(15));
    EXPECT_EQ(pixel(bitmap, 639, 199), microlind::app::vdc_rgb(15));
    snapshot.registers[0x19] = 0;
    snapshot.registers[0x0A] = 0x20;
    snapshot.chars[0] = 1;
    snapshot.character_data[16] = 0x40;
    const auto text = microlind::app::render_vdc_framebuffer(snapshot, 0.0);
    ASSERT_EQ(text.width, 640);
    ASSERT_EQ(text.height, 200);
    EXPECT_EQ(pixel(text, 0, 0), microlind::app::vdc_rgb(2));
    EXPECT_EQ(pixel(text, 1, 0), microlind::app::vdc_rgb(15));
    snapshot.registers[0x19] = 0x80;
    EXPECT_EQ(microlind::app::render_vdc_framebuffer(snapshot, 0.0).rgba, bitmap.rgba);
}

TEST(VdcRenderTest, RejectsInvalidBitmapProfilesAndIncompleteSnapshots) {
    const auto valid = bitmap_snapshot();
    for (auto [reg, value] : {std::pair{0x01, 0}, {0x06, 0}, {0x08, 1}, {0x08, 3},
             {0x18, 1}, {0x19, 0x81}, {0x19, 0x90}, {0x19, 0xA0},
             {0x16, 0x97}, {0x17, 1}, {0x06, 255}, {0x19, 0xC0}}) {
        auto snapshot = valid;
        snapshot.registers[reg] = static_cast<uint8_t>(value);
        // Excessive height and attribute payload are checked independently.
        if (reg == 0x06 && value == 255) {
            snapshot.registers[0x09] = 7;
            snapshot.registers[0x17] = 8;
        }
        EXPECT_NE(microlind::app::vdc_frame_error(snapshot), nullptr) << reg;
        EXPECT_TRUE(microlind::app::render_vdc_framebuffer(snapshot, 0.0).rgba.empty());
    }
    auto snapshot = valid;
    snapshot.bitmap_data.pop_back();
    EXPECT_NE(microlind::app::vdc_frame_error(snapshot), nullptr);
    snapshot = valid;
    snapshot.present = false;
    EXPECT_TRUE(microlind::app::render_vdc_framebuffer(snapshot, 0.0).rgba.empty());
    snapshot = valid;
    snapshot.registers[0x08] = 2; // also a non-interlaced register encoding
    EXPECT_EQ(microlind::app::vdc_frame_error(snapshot), nullptr);
}

} // namespace
