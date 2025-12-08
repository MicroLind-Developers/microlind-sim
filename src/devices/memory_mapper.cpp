#include "microlind/devices/memory_mapper.hpp"

#include <algorithm>

namespace microlind::devices {

MemoryMapper::MemoryMapper(std::shared_ptr<MapperState> state, std::vector<int8_t> offset_map)
    : state_(std::move(state)), offset_map_(std::move(offset_map)) {}

uint8_t MemoryMapper::read8(uint16_t offset) {
    if (offset >= offset_map_.size()) return 0xFF;
    const int idx = offset_map_[offset];
    if (idx < 0 || idx >= 4) return 0xFF;
    return state_->bank[idx];
}

void MemoryMapper::write8(uint16_t offset, uint8_t value) {
    if (offset >= offset_map_.size()) return;
    const int idx = offset_map_[offset];
    if (idx < 0 || idx >= 4) return;
    state_->bank[idx] = value;
}

BankedMemory::BankedMemory(std::shared_ptr<MapperState> state, std::size_t bank_size, std::size_t total_size, std::size_t window_count)
    : state_(std::move(state)),
      data_(total_size, 0x00),
      bank_size_(bank_size ? bank_size : 1),
      window_count_(window_count ? window_count : 1) {
    const std::size_t bank_count = bank_size_ ? (total_size / bank_size_) : 0;
    bank_mask_ = bank_count ? (bank_count - 1) : 0;
}

uint8_t BankedMemory::read8(uint16_t offset) {
    const std::size_t window = offset / bank_size_;
    if (window >= window_count_) return 0xFF;
    const std::size_t bank = static_cast<std::size_t>(state_->bank[window]);
    std::size_t selected = bank_mask_ ? (bank & bank_mask_) : bank;
    const std::size_t phys = selected * bank_size_ + (offset % bank_size_);
    if (phys >= data_.size()) return 0xFF;
    return data_[phys];
}

void BankedMemory::write8(uint16_t offset, uint8_t value) {
    const std::size_t window = offset / bank_size_;
    if (window >= window_count_) return;
    const std::size_t bank = static_cast<std::size_t>(state_->bank[window]);
    std::size_t selected = bank_mask_ ? (bank & bank_mask_) : bank;
    const std::size_t phys = selected * bank_size_ + (offset % bank_size_);
    if (phys >= data_.size()) return;
    data_[phys] = value;
}

} // namespace microlind::devices
