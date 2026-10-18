#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <queue>
#include <string>
#include <utility>

#include "microlind/bus.hpp"

namespace microlind::devices {

// Minimal XR88C92 dual UART model.
// Both channels and the MLOS crystal/16 periodic timer (ACR mode7) are functional.
// Other counter modes remain stubbed.
class XR88C92 : public microlind::BusDevice {
public:
    using TxCallback = std::function<void(uint8_t)>;
    using IrqCallback = std::function<void(bool)>;

    struct RgbLed {
        bool red{};
        bool green{};
        bool blue{};
    };

    explicit XR88C92(TxCallback cb = nullptr, IrqCallback irq_cb = nullptr, uint64_t cpu_hz = 1'000'000);

    uint8_t read8(uint16_t offset) override;
    uint8_t peek8(uint16_t offset) override;
    void write8(uint16_t offset, uint8_t value) override;
    void tick(uint32_t cycles) override;

    // Push a byte into the RXA FIFO.
    void inject_rx(uint8_t value);
    void inject_rx_b(uint8_t value);
    void set_tx_a_callback(TxCallback callback) { tx_cb_ = std::move(callback); }
    void set_tx_b_callback(TxCallback callback) { tx_b_cb_ = std::move(callback); }

    // Current status bits for channel A.
    uint8_t status_a() const;
    uint8_t status_b() const;
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
    uint64_t cpu_hz_;
    uint64_t timer_fraction_{};
    uint32_t timer_elapsed_{};
    uint16_t timer_preload_{};
    bool timer_running_{};
    bool timer_pending_{};

    std::queue<uint8_t> rx_fifo_a_;
    std::queue<uint8_t> rx_fifo_b_;
    TxCallback tx_cb_;
    TxCallback tx_b_cb_;
    IrqCallback irq_cb_;
};

} // namespace microlind::devices
