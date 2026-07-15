#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <queue>
#include <string>

#include "microlind/bus.hpp"

namespace microlind::devices {

// Minimal XR88C92 dual UART model.
// Only channel A is functional for now (RXA/TXA). Other registers are stubbed.
class XR88C92 : public microlind::BusDevice {
public:
    using TxCallback = std::function<void(uint8_t)>;
    using IrqCallback = std::function<void(bool)>;

    struct RgbLed {
        bool red{};
        bool green{};
        bool blue{};
    };

    explicit XR88C92(TxCallback cb = nullptr, IrqCallback irq_cb = nullptr);

    uint8_t read8(uint16_t offset) override;
    uint8_t peek8(uint16_t offset) override;
    void write8(uint16_t offset, uint8_t value) override;

    // Push a byte into the RXA FIFO.
    void inject_rx(uint8_t value);

    // Current status bits for channel A.
    uint8_t status_a() const;
    [[nodiscard]] uint8_t interrupt_status() const;
    [[nodiscard]] bool irq_asserted() const { return irq_asserted_; }
    [[nodiscard]] uint8_t output_port() const { return output_port_; }
    [[nodiscard]] RgbLed rgb_led() const;

private:
    void refresh_irq();

    // Registers we care about.
    uint8_t mode_a_{0};
    uint8_t cmd_a_{0};
    uint8_t mode_b_{0};
    uint8_t cmd_b_{0};
    uint8_t acr_{0};
    uint8_t imr_{0};
    uint8_t gpr_{0};
    uint8_t opcr_{0};
    uint8_t output_port_{0};
    bool irq_asserted_{false};

    std::queue<uint8_t> rx_fifo_a_;
    TxCallback tx_cb_;
    IrqCallback irq_cb_;
};

} // namespace microlind::devices
