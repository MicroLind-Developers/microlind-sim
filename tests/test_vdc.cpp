#include <gtest/gtest.h>

#include <cstdint>

#include "microlind/devices/vdc.hpp"

namespace {

class VdcBlockCopyTest : public testing::Test {
protected:
    void write_register(uint8_t reg, uint8_t value) {
        vdc.write8(0, reg);
        vdc.write8(1, value);
    }

    void write_address(uint8_t high_reg, uint16_t address) {
        write_register(high_reg, static_cast<uint8_t>(address >> 8));
        write_register(static_cast<uint8_t>(high_reg + 1), static_cast<uint8_t>(address));
    }

    void write_byte(uint16_t address, uint8_t value) {
        write_address(0x12, address);
        write_register(0x1F, value);
    }

    uint8_t peek_byte(uint16_t address) {
        write_address(0x12, address);
        vdc.write8(0, 0x1F);
        return vdc.peek8(1);
    }

    uint16_t source_address() const {
        return static_cast<uint16_t>((vdc.registers()[0x20] << 8) | vdc.registers()[0x21]);
    }

    void prepare_copy(uint16_t source, uint16_t destination) {
        write_address(0x20, source);
        write_address(0x12, destination);
        write_register(0x18, 0xD5); // copy bit together with other display control bits
    }

    microlind::devices::Vdc8568 vdc;
};

struct CopyCase {
    uint16_t source;
    uint16_t destination;
    uint16_t count;
};

class VdcBlockCopyCountTest : public VdcBlockCopyTest, public testing::WithParamInterface<CopyCase> {};

TEST_P(VdcBlockCopyCountTest, CopiesExactCountAndAdvancesBothAddresses) {
    const auto [source, destination, count] = GetParam();
    for (uint16_t i = 0; i < count; ++i) {
        write_byte(static_cast<uint16_t>(source + i), static_cast<uint8_t>(i ^ 0xA5));
    }
    write_byte(static_cast<uint16_t>(destination - 1), 0x5A);
    write_byte(static_cast<uint16_t>(destination + count), 0xC3);
    write_register(0x1B, 80); // display row stride must not affect block transfers
    prepare_copy(source, destination);

    const auto version = vdc.frame_version();
    // Selecting or reading the count register does not start a copy.
    vdc.write8(0, 0x1E);
    vdc.peek8(1);
    vdc.read8(1);
    EXPECT_EQ(vdc.update_address(), destination);
    EXPECT_EQ(source_address(), source);
    EXPECT_EQ(vdc.frame_version(), version);

    vdc.write8(1, static_cast<uint8_t>(count));

    EXPECT_EQ(vdc.update_address(), static_cast<uint16_t>(destination + count));
    EXPECT_EQ(source_address(), static_cast<uint16_t>(source + count));
    EXPECT_EQ(vdc.frame_version(), version + 1);
    EXPECT_EQ(vdc.status() & 0x90, 0x90);
    for (uint16_t i = 0; i < count; ++i) {
        SCOPED_TRACE(i);
        EXPECT_EQ(peek_byte(static_cast<uint16_t>(destination + i)), static_cast<uint8_t>(i ^ 0xA5));
        EXPECT_EQ(peek_byte(static_cast<uint16_t>(source + i)), static_cast<uint8_t>(i ^ 0xA5));
    }
    EXPECT_EQ(peek_byte(static_cast<uint16_t>(destination - 1)), 0x5A);
    EXPECT_EQ(peek_byte(static_cast<uint16_t>(destination + count)), 0xC3);
}

INSTANTIATE_TEST_SUITE_P(CountsAndWrapping, VdcBlockCopyCountTest, testing::Values(
    CopyCase{0x40FF, 0x20FF, 1},
    CopyCase{0x40FE, 0x20FE, 3},
    CopyCase{0x4000, 0x2000, 255},
    CopyCase{0x4000, 0x2000, 256},
    CopyCase{0xFFF0, 0x2000, 256},
    CopyCase{0x4000, 0xFFF0, 256}));

TEST_F(VdcBlockCopyTest, HigherOverlappingDestinationRepeatsEarlierWrites) {
    write_byte(0x2000, 'A');
    write_byte(0x2001, 'B');
    write_byte(0x2002, 'C');
    prepare_copy(0x2000, 0x2001);
    write_register(0x1E, 3);

    for (uint16_t address = 0x2000; address < 0x2004; ++address) {
        EXPECT_EQ(peek_byte(address), 'A');
    }
}

TEST_F(VdcBlockCopyTest, LowerOverlappingDestinationCopiesSuccessiveSourceBytes) {
    write_byte(0x2001, 'A');
    write_byte(0x2002, 'B');
    write_byte(0x2003, 'C');
    prepare_copy(0x2001, 0x2000);
    write_register(0x1E, 3);

    EXPECT_EQ(peek_byte(0x2000), 'A');
    EXPECT_EQ(peek_byte(0x2001), 'B');
    EXPECT_EQ(peek_byte(0x2002), 'C');
    EXPECT_EQ(peek_byte(0x2003), 'C');
}

TEST_F(VdcBlockCopyTest, CopyToSameAddressPreservesDataAndWrapsBothPointers) {
    write_byte(0xFFFF, 'A');
    write_byte(0x0000, 'B');
    prepare_copy(0xFFFF, 0xFFFF);
    write_register(0x1E, 2);

    EXPECT_EQ(source_address(), 0x0001);
    EXPECT_EQ(vdc.update_address(), 0x0001);
    EXPECT_EQ(peek_byte(0xFFFF), 'A');
    EXPECT_EQ(peek_byte(0x0000), 'B');
}

TEST_F(VdcBlockCopyTest, RepeatedCountsContinueCopyAndClearingCopyBitRestoresFill) {
    write_byte(0x4000, 'A');
    write_byte(0x4001, 'B');
    write_byte(0x4002, 'C'); // also latches the fill byte in register $1F
    prepare_copy(0x4000, 0x2000);
    write_register(0x1E, 1);
    write_register(0x1E, 2);

    EXPECT_EQ(source_address(), 0x4003);
    EXPECT_EQ(vdc.update_address(), 0x2003);
    EXPECT_EQ(vdc.registers()[0x1F], 'C');
    write_register(0x18, 0x55);
    write_register(0x1E, 2);
    EXPECT_EQ(source_address(), 0x4003);
    EXPECT_EQ(vdc.update_address(), 0x2005);
    EXPECT_EQ(peek_byte(0x2000), 'A');
    EXPECT_EQ(peek_byte(0x2001), 'B');
    EXPECT_EQ(peek_byte(0x2002), 'C');
    EXPECT_EQ(peek_byte(0x2003), 'C');
    EXPECT_EQ(peek_byte(0x2004), 'C');
    EXPECT_EQ(peek_byte(0x2005), 0x00);
}

TEST_F(VdcBlockCopyTest, VramCopyWrapsWithoutChangingDeviceState) {
    write_byte(0xFFFF, 0x81);
    write_byte(0x0000, 0x42);
    write_byte(0x0001, 0x24);
    const auto registers = vdc.registers();
    const auto version = vdc.frame_version();
    const auto selected = vdc.selected_register();
    std::array<uint8_t, 3> bytes{};
    vdc.copy_vram(0xFFFF, bytes);
    EXPECT_EQ(bytes, (std::array<uint8_t, 3>{0x81, 0x42, 0x24}));
    vdc.copy_vram(0xFFFF, {});
    EXPECT_EQ(vdc.registers(), registers);
    EXPECT_EQ(vdc.frame_version(), version);
    EXPECT_EQ(vdc.selected_register(), selected);
}

TEST_F(VdcBlockCopyTest, BitmapGeometryRegisterChangesInvalidateDisplay) {
    EXPECT_EQ(vdc.registers()[0x01], 80);
    EXPECT_EQ(vdc.registers()[0x1B], 0);
    for (uint8_t reg : {0x01, 0x06, 0x08, 0x1B}) {
        const auto version = vdc.frame_version();
        write_register(reg, 1);
        EXPECT_EQ(vdc.frame_version(), version + 1);
    }
}

} // namespace
