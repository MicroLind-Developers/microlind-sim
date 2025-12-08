#pragma once

#include <cstdint>
#include <vector>

#include "microlind/bus.hpp"

namespace microlind::devices {

class Memory : public BusDevice {
public:
    Memory(std::size_t size, bool writable);

    uint8_t read8(uint16_t offset) override;
    void write8(uint16_t offset, uint8_t value) override;

    void load(std::size_t offset, const std::vector<uint8_t>& data);

private:
    std::vector<uint8_t> data_;
    bool writable_{};
};

} // namespace microlind::devices
