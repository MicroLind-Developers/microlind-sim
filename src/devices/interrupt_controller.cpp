#include "microlind/devices/interrupt_controller.hpp"

#include <utility>

namespace microlind::devices {

InterruptController::InterruptController(IrqLineCallback on_irq_change)
    : on_irq_change_(std::move(on_irq_change)) {}

uint8_t InterruptController::read8(uint16_t /*offset*/) {
    return peek8(0);
}

uint8_t InterruptController::peek8(uint16_t /*offset*/) {
    return static_cast<uint8_t>((mask_ << 4) | (pending_level_ & 0x0F));
}

void InterruptController::write8(uint16_t /*offset*/, uint8_t value) {
    mask_ = static_cast<uint8_t>((value >> 4) & 0x0F);
    update_irq_line();
}

void InterruptController::request(uint8_t level) {
    const uint8_t normalized = static_cast<uint8_t>(level & 0x0F);
    if (normalized == 0) {
        return;
    }
    request_mask_ = static_cast<uint16_t>(request_mask_ | (1u << normalized));
    refresh_pending_level();
    update_irq_line();
}

void InterruptController::clear() {
    request_mask_ = 0;
    pending_level_ = 0;
    update_irq_line();
}

void InterruptController::clear(uint8_t level) {
    const uint8_t normalized = static_cast<uint8_t>(level & 0x0F);
    if (normalized == 0) {
        return;
    }
    request_mask_ = static_cast<uint16_t>(request_mask_ & ~(1u << normalized));
    refresh_pending_level();
    update_irq_line();
}

void InterruptController::refresh_pending_level() {
    pending_level_ = 0;
    for (int level = 15; level >= 1; --level) {
        if ((request_mask_ & (1u << level)) != 0) {
            pending_level_ = static_cast<uint8_t>(level);
            return;
        }
    }
}

bool InterruptController::irq_asserted() const {
    return pending_level_ > mask_;
}

void InterruptController::update_irq_line() {
    const bool asserted = irq_asserted();
    if (asserted == last_irq_asserted_) {
        return;
    }
    last_irq_asserted_ = asserted;
    if (on_irq_change_) {
        on_irq_change_(asserted);
    }
}

} // namespace microlind::devices
