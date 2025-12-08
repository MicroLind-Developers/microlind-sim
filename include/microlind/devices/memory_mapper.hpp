#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include "microlind/bus.hpp"

namespace microlind::devices {

// Shared state between the mapper registers and the banked memory.
struct MapperState {
    uint8_t bank[4]{0, 0, 0, 0};
};

// Four 8-bit registers that select the bank for each window.
class MemoryMapper : public microlind::BusDevice {
public:
    MemoryMapper(std::shared_ptr<MapperState> state, std::vector<int8_t> offset_map);

    uint8_t read8(uint16_t offset) override;
    void write8(uint16_t offset, uint8_t value) override;

private:
    std::shared_ptr<MapperState> state_;
    std::vector<int8_t> offset_map_; // offset->register index, -1 for unmapped.
};

// Banked RAM that uses mapper registers to select which bank is visible.
class BankedMemory : public microlind::BusDevice {
public:
    BankedMemory(std::shared_ptr<MapperState> state, std::size_t bank_size, std::size_t total_size, std::size_t window_count);

    uint8_t read8(uint16_t offset) override;
    void write8(uint16_t offset, uint8_t value) override;

private:
    std::shared_ptr<MapperState> state_;
    std::vector<uint8_t> data_;
    std::size_t bank_size_;
    std::size_t bank_mask_;
    std::size_t window_count_;
};

} // namespace microlind::devices
