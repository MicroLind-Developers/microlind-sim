#include "microlind/devices/serial.hpp"

#include <iostream>
#include <utility>

namespace microlind::devices {

// Register offsets for channel A/B we care about.
static constexpr uint16_t MRA = 0x00;
static constexpr uint16_t SRA = 0x01; // also CSRA on write
static constexpr uint16_t CRA = 0x02;
static constexpr uint16_t RXA_TXA = 0x03;

static constexpr uint16_t ACR = 0x04; // also IPCR on read
static constexpr uint16_t IMR = 0x05; // also ISR on read
static constexpr uint16_t CUR = 0x06;
static constexpr uint16_t CLR = 0x07;

static constexpr uint16_t MRB = 0x08;
static constexpr uint16_t SRB = 0x09;
static constexpr uint16_t CRB = 0x0A;
static constexpr uint16_t RXB_TXB = 0x0B;

static constexpr uint16_t GPR = 0x0C;
static constexpr uint16_t IPR_OPCR = 0x0D;
static constexpr uint16_t STCR_SOPR = 0x0E;
static constexpr uint16_t SPCR_ROPR = 0x0F;

static constexpr uint8_t ISR_TX_READY_A = 0x01;
static constexpr uint8_t ISR_RX_READY_A = 0x02;
static constexpr uint8_t ISR_TX_READY_B = 0x10;
static constexpr uint8_t ISR_RX_READY_B = 0x20;
static constexpr uint8_t OUTPUT_RED = 0x10;
static constexpr uint8_t OUTPUT_GREEN = 0x20;
static constexpr uint8_t OUTPUT_BLUE = 0x40;

XR88C92::XR88C92(TxCallback cb, IrqCallback irq_cb, uint64_t cpu_hz)
    : cpu_hz_(cpu_hz ? cpu_hz : 1), tx_cb_(std::move(cb)), irq_cb_(std::move(irq_cb)) {
    refresh_irq();
}

uint8_t XR88C92::status_a() const {
    uint8_t status = 0x00;
    if (!rx_fifo_a_.empty()) status |= 0x01; // RxRDY
    status |= 0x04; // TxRDY always
    return status;
}

void XR88C92::inject_rx(uint8_t value) {
    rx_fifo_a_.push(value);
    refresh_irq();
}

uint8_t XR88C92::status_b() const {
    return static_cast<uint8_t>(0x04 | (rx_fifo_b_.empty() ? 0 : 0x01));
}

void XR88C92::inject_rx_b(uint8_t value) {
    rx_fifo_b_.push(value);
    refresh_irq();
}

uint8_t XR88C92::interrupt_status() const {
    uint8_t status = static_cast<uint8_t>(ISR_TX_READY_A | ISR_TX_READY_B | (timer_pending_ ? 0x08 : 0));
    if (!rx_fifo_a_.empty()) {
        status |= ISR_RX_READY_A;
    }
    if (!rx_fifo_b_.empty()) status |= ISR_RX_READY_B;
    return status;
}

XR88C92::RgbLed XR88C92::rgb_led() const {
    return RgbLed{
        (output_port_ & OUTPUT_RED) != 0,
        (output_port_ & OUTPUT_GREEN) != 0,
        (output_port_ & OUTPUT_BLUE) != 0,
    };
}

void XR88C92::refresh_irq() {
    const bool asserted = (interrupt_status() & imr_) != 0;
    if (asserted == irq_asserted_) {
        return;
    }
    irq_asserted_ = asserted;
    if (irq_cb_) {
        irq_cb_(asserted);
    }
}

void XR88C92::tick(uint32_t cycles) {
    if (!timer_running_ || (acr_ & 0x70) != 0x70) return;
    // 3.6864 MHz crystal, /16. Accumulate rational time without tick rounding.
    timer_fraction_ += uint64_t(cycles) * 3'686'400;
    const uint64_t divisor = cpu_hz_ * 16;
    const uint64_t counts = timer_fraction_ / divisor;
    timer_fraction_ %= divisor;
    const uint32_t period = 2u * (timer_preload_ ? uint32_t(timer_preload_) : 65536u);
    const uint64_t elapsed = timer_elapsed_ + counts;
    if (elapsed >= period) timer_pending_ = true;
    timer_elapsed_ = static_cast<uint32_t>(elapsed % period);
    refresh_irq();
}

uint8_t XR88C92::read8(uint16_t offset) {
    switch (offset) {
    case MRA: return mode_a_;
    case SRA: return status_a();
    case CRA: return cmd_a_;
    case RXA_TXA: {
        if (rx_fifo_a_.empty()) return 0x00;
        uint8_t v = rx_fifo_a_.front();
        rx_fifo_a_.pop();
        refresh_irq();
        return v;
    }
    case ACR: return acr_;
    case IMR: return interrupt_status(); // ISR on read
    case CUR:
    case CLR:
    case MRB: return mode_b_;
    case SRB: return status_b();
    case CRB: return cmd_b_;
    case RXB_TXB: {
        if (rx_fifo_b_.empty()) return 0x00;
        const uint8_t value = rx_fifo_b_.front();
        rx_fifo_b_.pop();
        refresh_irq();
        return value;
    }
    case GPR: return gpr_;
    case IPR_OPCR: return opcr_;
    case STCR_SOPR:
        timer_running_ = true;
        timer_elapsed_ = 0;
        timer_fraction_ = 0;
        return output_port_;
    case SPCR_ROPR:
        timer_pending_ = false;
        if ((acr_ & 0x70) != 0x70) timer_running_ = false;
        refresh_irq();
        return output_port_;
    default:
        return 0x00;
    }
}

uint8_t XR88C92::peek8(uint16_t offset) {
    switch (offset) {
    case MRA: return mode_a_;
    case SRA: return status_a();
    case CRA: return cmd_a_;
    case RXA_TXA:
        if (rx_fifo_a_.empty()) return 0x00;
        return rx_fifo_a_.front();
    case ACR: return acr_;
    case IMR: return interrupt_status();
    case CUR:
    case CLR:
    case MRB: return mode_b_;
    case SRB: return status_b();
    case CRB: return cmd_b_;
    case RXB_TXB: return rx_fifo_b_.empty() ? 0x00 : rx_fifo_b_.front();
    case GPR: return gpr_;
    case IPR_OPCR: return opcr_;
    case STCR_SOPR:
    case SPCR_ROPR:
        return output_port_;
    default:
        return 0x00;
    }
}

void XR88C92::write8(uint16_t offset, uint8_t value) {
    switch (offset) {
    case MRA: mode_a_ = value; break;
    case SRA: /* CSRA */ break;
    case CRA: cmd_a_ = value; break;
    case RXA_TXA:
        if (tx_cb_) tx_cb_(value);
        refresh_irq();
        break;
    case ACR: acr_ = value; break;
    case IMR:
        imr_ = value;
        refresh_irq();
        break;
    case CUR:
        timer_preload_ = static_cast<uint16_t>((timer_preload_ & 0xFF) | (uint16_t(value) << 8));
        break;
    case CLR:
        timer_preload_ = static_cast<uint16_t>((timer_preload_ & 0xFF00) | value);
        break;
    case MRB: mode_b_ = value; break;
    case SRB: /* CSRB */ break;
    case CRB: cmd_b_ = value; break;
    case RXB_TXB:
        if (tx_b_cb_) tx_b_cb_(value);
        refresh_irq();
        break;
    case GPR: gpr_ = value; break;
    case IPR_OPCR: opcr_ = value; break;
    case STCR_SOPR:
        output_port_ = static_cast<uint8_t>(output_port_ | value);
        break;
    case SPCR_ROPR:
        output_port_ = static_cast<uint8_t>(output_port_ & ~value);
        break;
    default:
        break;
    }
}

} // namespace microlind::devices
