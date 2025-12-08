#include "microlind/devices/serial.hpp"

#include <algorithm>
#include <iostream>

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

XR88C92::XR88C92(TxCallback cb) : tx_cb_(std::move(cb)) {}

uint8_t XR88C92::status_a() const {
    uint8_t status = 0x00;
    if (!rx_fifo_a_.empty()) status |= 0x01; // RxRDY
    status |= 0x04; // TxRDY always
    return status;
}

void XR88C92::inject_rx(uint8_t value) {
    rx_fifo_a_.push(value);
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
        return v;
    }
    case ACR: return acr_;
    case IMR: return imr_; // status not modeled
    case CUR:
    case CLR:
    case MRB: return mode_b_;
    case SRB: return 0x04; // Tx ready
    case CRB: return cmd_b_;
    case RXB_TXB: return 0x00; // channel B unused
    case GPR: return gpr_;
    case IPR_OPCR: return opcr_;
    case STCR_SOPR:
    case SPCR_ROPR:
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
        break;
    case ACR: acr_ = value; break;
    case IMR: imr_ = value; break;
    case CUR:
    case CLR:
        break;
    case MRB: mode_b_ = value; break;
    case SRB: /* CSRB */ break;
    case CRB: cmd_b_ = value; break;
    case RXB_TXB:
        if (tx_cb_) tx_cb_(value);
        break;
    case GPR: gpr_ = value; break;
    case IPR_OPCR: opcr_ = value; break;
    case STCR_SOPR:
    case SPCR_ROPR:
    default:
        break;
    }
}

} // namespace microlind::devices
