#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "microlind/bus.hpp"

namespace microlind::devices {

class Vdc8568 : public BusDevice {
public:
    static constexpr uint8_t RegisterCount = 0x25;
    static constexpr uint8_t Columns = 80;
    static constexpr uint8_t Rows = 25;
    static constexpr std::size_t DisplayCells = static_cast<std::size_t>(Columns) * Rows;
    static constexpr std::size_t CharacterBytes = 8192;
    static constexpr std::size_t VramSize = 65536;

    Vdc8568();

    uint8_t read8(uint16_t offset) override;
    uint8_t peek8(uint16_t offset) override;
    void write8(uint16_t offset, uint8_t value) override;

    [[nodiscard]] uint8_t selected_register() const { return selected_register_; }
    [[nodiscard]] uint8_t status() const;
    [[nodiscard]] const std::array<uint8_t, RegisterCount>& registers() const { return regs_; }
    [[nodiscard]] uint16_t display_start() const;
    [[nodiscard]] uint16_t attribute_start() const;
    [[nodiscard]] uint16_t update_address() const;
    [[nodiscard]] uint16_t cursor_position() const;
    [[nodiscard]] uint16_t character_start() const;
    [[nodiscard]] uint64_t frame_version() const { return frame_version_; }
    [[nodiscard]] std::array<uint8_t, DisplayCells> display_chars() const;
    [[nodiscard]] std::array<uint8_t, DisplayCells> display_attrs() const;
    [[nodiscard]] std::array<uint8_t, CharacterBytes> character_data() const;

private:
    static constexpr uint8_t RegDisplayStartHigh = 0x0C;
    static constexpr uint8_t RegDisplayStartLow = 0x0D;
    static constexpr uint8_t RegCursorHigh = 0x0E;
    static constexpr uint8_t RegCursorLow = 0x0F;
    static constexpr uint8_t RegUpdateAddressHigh = 0x12;
    static constexpr uint8_t RegUpdateAddressLow = 0x13;
    static constexpr uint8_t RegAttributeStartHigh = 0x14;
    static constexpr uint8_t RegAttributeStartLow = 0x15;
    static constexpr uint8_t RegCharacterHorizontal = 0x16;
    static constexpr uint8_t RegCharacterVertical = 0x17;
    static constexpr uint8_t RegBlockControl = 0x18;
    static constexpr uint8_t RegHorizontalScroll = 0x19;
    static constexpr uint8_t RegColor = 0x1A;
    static constexpr uint8_t RegAddressIncrement = 0x1B;
    static constexpr uint8_t RegCharacterBase = 0x1C;
    static constexpr uint8_t RegUnderlineScan = 0x1D;
    static constexpr uint8_t RegWordCount = 0x1E;
    static constexpr uint8_t RegData = 0x1F;

    [[nodiscard]] uint8_t read_selected_register(bool side_effects);
    void write_selected_register(uint8_t value);
    void perform_block_fill();
    void increment_update_address();
    void set_update_address(uint16_t address);

    std::array<uint8_t, VramSize> vram_{};
    std::array<uint8_t, RegisterCount> regs_{};
    uint8_t selected_register_{};
    uint64_t frame_version_{};
};

} // namespace microlind::devices
