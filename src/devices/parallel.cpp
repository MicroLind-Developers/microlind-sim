#include "microlind/devices/parallel.hpp"

#include <algorithm>
#include <utility>

namespace microlind::devices {
namespace {

constexpr uint16_t ORB_IRB = 0x00;
constexpr uint16_t ORA_IRA = 0x01;
constexpr uint16_t DDRB = 0x02;
constexpr uint16_t DDRA = 0x03;
constexpr uint16_t T1CL = 0x04;
constexpr uint16_t T1CH = 0x05;
constexpr uint16_t T1LL = 0x06;
constexpr uint16_t T1LH = 0x07;
constexpr uint16_t T2CL = 0x08;
constexpr uint16_t T2CH = 0x09;
constexpr uint16_t SR = 0x0A;
constexpr uint16_t ACR = 0x0B;
constexpr uint16_t PCR = 0x0C;
constexpr uint16_t IFR = 0x0D;
constexpr uint16_t IER = 0x0E;
constexpr uint16_t ORA_NO_HANDSHAKE = 0x0F;

uint8_t low_byte(uint16_t value) {
    return static_cast<uint8_t>(value & 0xFF);
}

uint8_t high_byte(uint16_t value) {
    return static_cast<uint8_t>((value >> 8) & 0xFF);
}

} // namespace

W65C22::W65C22(IrqCallback irq_cb) : irq_cb_(std::move(irq_cb)) {}

uint8_t W65C22::read_port_a() const {
    return static_cast<uint8_t>((output_a_ & ddr_a_) | (input_a_ & ~ddr_a_));
}

uint8_t W65C22::effective_output_b() const {
    if ((acr_ & 0x80) == 0) return output_b_;
    return static_cast<uint8_t>((output_b_ & 0x7F) | (timer1_pb7_level_ ? 0x80 : 0x00));
}

uint8_t W65C22::read_port_b() const {
    return static_cast<uint8_t>((effective_output_b() & ddr_b_) | (input_b_ & ~ddr_b_));
}

void W65C22::set_port_b_input(uint8_t value) {
    const bool previous_pb7 = pb7_pin_level();
    input_b_ = value;
    record_pb7_transition(previous_pb7);
}

void W65C22::record_pb7_transition(bool previous_level) {
    if (previous_level != pb7_pin_level()) {
        ++pb7_transition_count_;
    }
}

uint8_t W65C22::ifr() const {
    const uint8_t pending = raw_ifr();
    return static_cast<uint8_t>(pending | (((pending & ier_) != 0) ? 0x80 : 0x00));
}

uint8_t W65C22::read8(uint16_t offset) {
    const uint16_t reg = static_cast<uint16_t>(offset & 0x0F);
    switch (reg) {
    case ORB_IRB:
        clear_ifr(0x18);
        return read_port_b();
    case ORA_IRA:
    case ORA_NO_HANDSHAKE:
        clear_ifr(0x03);
        return read_port_a();
    case DDRB: return ddr_b_;
    case DDRA: return ddr_a_;
    case T1CL:
        clear_ifr(IfrTimer1);
        return low_byte(timer1_counter_);
    case T1CH: return high_byte(timer1_counter_);
    case T1LL: return low_byte(timer1_latch_);
    case T1LH: return high_byte(timer1_latch_);
    case T2CL:
        clear_ifr(IfrTimer2);
        return low_byte(timer2_counter_);
    case T2CH: return high_byte(timer2_counter_);
    case SR: return shift_;
    case ACR: return acr_;
    case PCR: return pcr_;
    case IFR: return ifr();
    case IER: return ier();
    default: return 0xFF;
    }
}

uint8_t W65C22::peek8(uint16_t offset) {
    const uint16_t reg = static_cast<uint16_t>(offset & 0x0F);
    switch (reg) {
    case ORB_IRB: return read_port_b();
    case ORA_IRA:
    case ORA_NO_HANDSHAKE: return read_port_a();
    case DDRB: return ddr_b_;
    case DDRA: return ddr_a_;
    case T1CL: return low_byte(timer1_counter_);
    case T1CH: return high_byte(timer1_counter_);
    case T1LL: return low_byte(timer1_latch_);
    case T1LH: return high_byte(timer1_latch_);
    case T2CL: return low_byte(timer2_counter_);
    case T2CH: return high_byte(timer2_counter_);
    case SR: return shift_;
    case ACR: return acr_;
    case PCR: return pcr_;
    case IFR: return ifr();
    case IER: return ier();
    default: return 0xFF;
    }
}

void W65C22::write8(uint16_t offset, uint8_t value) {
    const uint16_t reg = static_cast<uint16_t>(offset & 0x0F);
    switch (reg) {
    case ORB_IRB: {
        const bool previous_pb7 = pb7_pin_level();
        output_b_ = value;
        clear_ifr(0x18);
        record_pb7_transition(previous_pb7);
        break;
    }
    case ORA_IRA:
    case ORA_NO_HANDSHAKE:
        output_a_ = value;
        clear_ifr(0x03);
        break;
    case DDRB: {
        const bool previous_pb7 = pb7_pin_level();
        ddr_b_ = value;
        record_pb7_transition(previous_pb7);
        break;
    }
    case DDRA:
        ddr_a_ = value;
        break;
    case T1CL:
        timer1_latch_ = static_cast<uint16_t>((timer1_latch_ & 0xFF00) | value);
        break;
    case T1CH: {
        const bool previous_pb7 = pb7_pin_level();
        timer1_latch_ = static_cast<uint16_t>((static_cast<uint16_t>(value) << 8) | (timer1_latch_ & 0x00FF));
        timer1_counter_ = timer1_latch_;
        timer1_running_ = true;
        if ((acr_ & 0x80) != 0) {
            timer1_pb7_level_ = false;
        }
        clear_ifr(IfrTimer1);
        record_pb7_transition(previous_pb7);
        break;
    }
    case T1LL:
        timer1_latch_ = static_cast<uint16_t>((timer1_latch_ & 0xFF00) | value);
        break;
    case T1LH:
        timer1_latch_ = static_cast<uint16_t>((static_cast<uint16_t>(value) << 8) | (timer1_latch_ & 0x00FF));
        clear_ifr(IfrTimer1);
        break;
    case T2CL:
        timer2_latch_ = static_cast<uint16_t>((timer2_latch_ & 0xFF00) | value);
        break;
    case T2CH:
        timer2_latch_ = static_cast<uint16_t>((static_cast<uint16_t>(value) << 8) | (timer2_latch_ & 0x00FF));
        timer2_counter_ = timer2_latch_;
        timer2_running_ = true;
        clear_ifr(IfrTimer2);
        break;
    case SR:
        shift_ = value;
        clear_ifr(0x04);
        break;
    case ACR: {
        const bool previous_pb7 = pb7_pin_level();
        acr_ = value;
        record_pb7_transition(previous_pb7);
        break;
    }
    case PCR:
        pcr_ = value;
        break;
    case IFR:
        clear_ifr(static_cast<uint8_t>(value & 0x7F));
        break;
    case IER:
        if ((value & 0x80) != 0) {
            ier_ = static_cast<uint8_t>(ier_ | (value & 0x7F));
        } else {
            ier_ = static_cast<uint8_t>(ier_ & ~(value & 0x7F));
        }
        refresh_irq();
        break;
    default:
        break;
    }
}

void W65C22::tick(uint32_t cycles) {
    if (cycles == 0) return;
    tick_timer(cycles, timer1_counter_, timer1_latch_, IfrTimer1, (acr_ & 0x40) != 0);
    tick_timer(cycles, timer2_counter_, timer2_latch_, IfrTimer2, false);
}

void W65C22::clear_ifr(uint8_t mask) {
    ifr_ = static_cast<uint8_t>(ifr_ & ~(mask & 0x7F));
    refresh_irq();
}

void W65C22::set_ifr(uint8_t mask) {
    ifr_ = static_cast<uint8_t>(ifr_ | (mask & 0x7F));
    refresh_irq();
}

void W65C22::refresh_irq() {
    const bool asserted = (raw_ifr() & ier_) != 0;
    if (asserted == irq_asserted_) return;
    irq_asserted_ = asserted;
    if (irq_cb_) {
        irq_cb_(asserted);
    }
}

void W65C22::tick_timer(uint32_t cycles, uint16_t& counter, uint16_t latch, uint8_t flag, bool continuous) {
    bool& running = flag == IfrTimer1 ? timer1_running_ : timer2_running_;
    if (!running) return;

    uint32_t remaining = cycles;
    while (running && remaining > 0) {
        if (remaining <= counter) {
            counter = static_cast<uint16_t>(counter - remaining);
            return;
        }
        remaining -= static_cast<uint32_t>(counter) + 1u;
        const bool previous_pb7 = pb7_pin_level();
        set_ifr(flag);
        if (flag == IfrTimer1 && (acr_ & 0x80) != 0) {
            timer1_pb7_level_ = continuous ? !timer1_pb7_level_ : true;
        }
        record_pb7_transition(previous_pb7);
        if (continuous) {
            counter = latch;
        } else {
            counter = 0xFFFF;
            running = false;
        }
    }
}

} // namespace microlind::devices
