#pragma once

#include <cstdint>
#include <functional>

#include "microlind/bus.hpp"

namespace microlind::devices {

class InterruptController : public BusDevice {
public:
    using IrqLineCallback = std::function<void(bool)>;

    explicit InterruptController(IrqLineCallback on_irq_change = {});

    uint8_t read8(uint16_t offset) override;
    uint8_t peek8(uint16_t offset) override;
    void write8(uint16_t offset, uint8_t value) override;

    void request(uint8_t level);
    void clear();
    void clear(uint8_t level);

    [[nodiscard]] uint8_t pending_level() const { return pending_level_; }
    [[nodiscard]] uint8_t mask() const { return mask_; }
    [[nodiscard]] bool irq_asserted() const;

private:
    void update_irq_line();

    uint8_t pending_level_{};
    uint8_t mask_{0x0F};
    bool last_irq_asserted_{};
    IrqLineCallback on_irq_change_;
};

} // namespace microlind::devices
