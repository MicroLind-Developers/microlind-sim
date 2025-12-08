#include "microlind/devices/memory.hpp"

#include <algorithm>

namespace microlind::devices {

Memory::Memory(std::size_t size, bool writable) : data_(size, 0x00), writable_(writable) {}

uint8_t Memory::read8(uint16_t offset) {
    if (offset < data_.size()) {
        return data_[offset];
    }
    return 0xFF;
}

void Memory::write8(uint16_t offset, uint8_t value) {
    if (!writable_ || offset >= data_.size()) {
        return;
    }
    data_[offset] = value;
}

void Memory::load(std::size_t offset, const std::vector<uint8_t>& data) {
    const std::size_t end = std::min(data_.size(), offset + data.size());
    const std::size_t count = end > offset ? end - offset : 0;
    for (std::size_t i = 0; i < count; ++i) {
        data_[offset + i] = data[i];
    }
}

} // namespace microlind::devices
