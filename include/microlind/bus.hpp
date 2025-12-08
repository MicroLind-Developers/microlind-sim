#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace microlind {

class BusDevice {
public:
    virtual ~BusDevice() = default;
    virtual uint8_t read8(uint16_t offset) = 0;
    virtual void write8(uint16_t offset, uint8_t value) = 0;
    virtual void tick(uint32_t /*cycles*/) {}
};

enum class BusErrorType {
    EmptyRange,
    Overlap,
};

struct BusError {
    BusErrorType type;
    uint16_t start;
    uint16_t end;
};

class Bus {
public:
    Bus() = default;

    std::optional<BusError> map_device(uint16_t start, uint16_t end, std::unique_ptr<BusDevice> device);

    uint8_t read8(uint16_t address);
    void write8(uint16_t address, uint8_t value);
    void tick_devices(uint32_t cycles);

    std::vector<std::string> map_summary() const;

private:
    struct MappedDevice {
        uint16_t start{};
        uint16_t end{};
        std::unique_ptr<BusDevice> device;

        [[nodiscard]] bool contains(uint16_t address) const {
            return address >= start && address <= end;
        }

        [[nodiscard]] uint16_t offset(uint16_t address) const {
            return static_cast<uint16_t>(address - start);
        }
    };

    std::vector<MappedDevice> devices_{};
};

} // namespace microlind
