#include "microlind/devices/vdc.hpp"

#include <algorithm>

namespace microlind::devices {
namespace {

constexpr uint8_t kStatusReady = 0x80;
constexpr uint8_t kStatusUpdateReady = 0x10;
constexpr uint8_t kStatusDisplayEnabled = 0x08;
constexpr uint8_t kBlockCopy = 0x80;

uint16_t word_from_regs(const std::array<uint8_t, Vdc8568::RegisterCount>& regs, uint8_t high, uint8_t low) {
    return static_cast<uint16_t>((static_cast<uint16_t>(regs[high]) << 8) | regs[low]);
}

} // namespace

Vdc8568::Vdc8568() {
    vram_.fill(0x00);
    std::fill_n(vram_.begin(), DisplayCells, 0x20);
    std::fill_n(vram_.begin() + 0x0800, DisplayCells, 0x0F);
    regs_[0x06] = Rows;
    regs_[0x09] = 0x07;
    regs_[0x0B] = 0x07;
    regs_[RegAttributeStartHigh] = 0x08;
    regs_[RegAttributeStartLow] = 0x00;
    regs_[RegCharacterHorizontal] = 0x78;
    regs_[RegCharacterVertical] = 0x08;
    regs_[RegAddressIncrement] = 0x01;
    regs_[RegCharacterBase] = 0x20;
    regs_[0x1A] = 0xF0;
}

uint8_t Vdc8568::status() const {
    return static_cast<uint8_t>(kStatusReady | kStatusUpdateReady | kStatusDisplayEnabled);
}

uint8_t Vdc8568::read8(uint16_t offset) {
    if ((offset & 0x01) == 0) {
        return status();
    }
    return read_selected_register(true);
}

uint8_t Vdc8568::peek8(uint16_t offset) {
    if ((offset & 0x01) == 0) {
        return status();
    }
    return read_selected_register(false);
}

void Vdc8568::write8(uint16_t offset, uint8_t value) {
    if ((offset & 0x01) == 0) {
        selected_register_ = value;
        return;
    }
    write_selected_register(value);
}

uint16_t Vdc8568::display_start() const {
    return word_from_regs(regs_, RegDisplayStartHigh, RegDisplayStartLow);
}

uint16_t Vdc8568::attribute_start() const {
    return word_from_regs(regs_, RegAttributeStartHigh, RegAttributeStartLow);
}

uint16_t Vdc8568::update_address() const {
    return word_from_regs(regs_, RegUpdateAddressHigh, RegUpdateAddressLow);
}

uint16_t Vdc8568::cursor_position() const {
    return word_from_regs(regs_, RegCursorHigh, RegCursorLow);
}

uint16_t Vdc8568::character_start() const {
    return static_cast<uint16_t>((regs_[RegCharacterBase] & 0xE0) << 8);
}

std::array<uint8_t, Vdc8568::DisplayCells> Vdc8568::display_chars() const {
    std::array<uint8_t, DisplayCells> chars{};
    const uint16_t start = display_start();
    for (std::size_t i = 0; i < chars.size(); ++i) {
        chars[i] = vram_[static_cast<uint16_t>(start + i)];
    }
    return chars;
}

std::array<uint8_t, Vdc8568::DisplayCells> Vdc8568::display_attrs() const {
    std::array<uint8_t, DisplayCells> attrs{};
    const uint16_t start = attribute_start();
    for (std::size_t i = 0; i < attrs.size(); ++i) {
        attrs[i] = vram_[static_cast<uint16_t>(start + i)];
    }
    return attrs;
}

std::array<uint8_t, Vdc8568::CharacterBytes> Vdc8568::character_data() const {
    std::array<uint8_t, CharacterBytes> data{};
    const uint16_t start = character_start();
    for (std::size_t i = 0; i < data.size(); ++i) {
        data[i] = vram_[static_cast<uint16_t>(start + i)];
    }
    return data;
}

uint8_t Vdc8568::read_selected_register(bool side_effects) {
    if (selected_register_ == RegData) {
        const uint8_t value = vram_[update_address()];
        if (side_effects) {
            increment_update_address();
        }
        return value;
    }
    if (selected_register_ < regs_.size()) {
        return regs_[selected_register_];
    }
    return 0xFF;
}

void Vdc8568::write_selected_register(uint8_t value) {
    if (selected_register_ == RegData) {
        regs_[RegData] = value;
        vram_[update_address()] = value;
        ++frame_version_;
        increment_update_address();
        return;
    }
    if (selected_register_ < regs_.size()) {
        regs_[selected_register_] = value;
        if (selected_register_ == RegWordCount && (regs_[RegBlockControl] & kBlockCopy) == 0) {
            perform_block_fill();
            return;
        }
        if (selected_register_ == RegDisplayStartHigh || selected_register_ == RegDisplayStartLow ||
            selected_register_ == RegAttributeStartHigh || selected_register_ == RegAttributeStartLow ||
            selected_register_ == RegCursorHigh || selected_register_ == RegCursorLow ||
            selected_register_ == 0x09 || selected_register_ == 0x0A || selected_register_ == 0x0B ||
            selected_register_ == RegCharacterHorizontal || selected_register_ == RegCharacterVertical ||
            selected_register_ == RegBlockControl || selected_register_ == RegHorizontalScroll ||
            selected_register_ == RegColor || selected_register_ == RegCharacterBase ||
            selected_register_ == RegUnderlineScan) {
            ++frame_version_;
        }
    }
}

void Vdc8568::perform_block_fill() {
    const std::size_t count = regs_[RegWordCount] == 0 ? 256 : regs_[RegWordCount];
    uint16_t address = update_address();
    for (std::size_t i = 0; i < count; ++i) {
        vram_[address] = regs_[RegData];
        address = static_cast<uint16_t>(address + 1);
    }
    set_update_address(address);
    ++frame_version_;
}

void Vdc8568::increment_update_address() {
    set_update_address(static_cast<uint16_t>(update_address() + 1));
}

void Vdc8568::set_update_address(uint16_t address) {
    regs_[RegUpdateAddressHigh] = static_cast<uint8_t>((address >> 8) & 0xFF);
    regs_[RegUpdateAddressLow] = static_cast<uint8_t>(address & 0xFF);
}

} // namespace microlind::devices
