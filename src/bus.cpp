#include "microlind/bus.hpp"

#include <algorithm>
#include <cstdio>

namespace microlind {

namespace {
bool ranges_overlap(uint16_t a_start, uint16_t a_end, uint16_t b_start, uint16_t b_end) {
    return a_start <= b_end && b_start <= a_end;
}
}

std::optional<BusError> Bus::map_device(uint16_t start, uint16_t end, std::unique_ptr<BusDevice> device) {
    if (start > end) {
        return BusError{BusErrorType::EmptyRange, start, end};
    }

    const bool overlaps = std::any_of(devices_.begin(), devices_.end(), [&](const MappedDevice& m) {
        return ranges_overlap(start, end, m.start, m.end);
    });

    if (overlaps) {
        return BusError{BusErrorType::Overlap, start, end};
    }

    devices_.push_back(MappedDevice{start, end, std::move(device)});
    std::sort(devices_.begin(), devices_.end(), [](const MappedDevice& a, const MappedDevice& b) {
        if (a.start == b.start) {
            return a.end < b.end;
        }
        return a.start < b.start;
    });

    return std::nullopt;
}

uint8_t Bus::read8(uint16_t address) {
    for (auto& m : devices_) {
        if (m.contains(address)) {
            return m.device->read8(m.offset(address));
        }
    }
    return 0xFF;
}

void Bus::write8(uint16_t address, uint8_t value) {
    for (auto& m : devices_) {
        if (m.contains(address)) {
            m.device->write8(m.offset(address), value);
            return;
        }
    }
}

void Bus::tick_devices(uint32_t cycles) {
    for (auto& m : devices_) {
        m.device->tick(cycles);
    }
}

std::vector<std::string> Bus::map_summary() const {
    std::vector<std::string> out;
    out.reserve(devices_.size());
    for (const auto& m : devices_) {
        char buffer[32];
        std::snprintf(buffer, sizeof(buffer), "0x%04X-0x%04X", m.start, m.end);
        out.emplace_back(buffer);
    }
    return out;
}

} // namespace microlind
