#include <gtest/gtest.h>

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

} // namespace
