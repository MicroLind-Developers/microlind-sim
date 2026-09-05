#pragma once

#include <cstdint>
#include <functional>

#include "microlind/bus.hpp"

namespace microlind::devices {

class W65C22 : public BusDevice {
public:
    using IrqCallback = std::function<void(bool)>;

    explicit W65C22(IrqCallback irq_cb = {});

    uint8_t read8(uint16_t offset) override;
    uint8_t peek8(uint16_t offset) override;
    void write8(uint16_t offset, uint8_t value) override;
    void tick(uint32_t cycles) override;

    void set_port_a_input(uint8_t value) { input_a_ = value; }
    void set_port_b_input(uint8_t value);

    [[nodiscard]] uint8_t port_a() const { return read_port_a(); }
    [[nodiscard]] uint8_t port_b() const { return read_port_b(); }
    [[nodiscard]] uint8_t output_a() const { return output_a_; }
    [[nodiscard]] uint8_t output_b() const { return output_b_; }
    [[nodiscard]] uint8_t input_a() const { return input_a_; }
    [[nodiscard]] uint8_t input_b() const { return input_b_; }
    [[nodiscard]] uint8_t ddr_a() const { return ddr_a_; }
    [[nodiscard]] uint8_t ddr_b() const { return ddr_b_; }
    [[nodiscard]] uint8_t acr() const { return acr_; }
    [[nodiscard]] uint8_t pcr() const { return pcr_; }
    [[nodiscard]] uint8_t ifr() const;
    [[nodiscard]] uint8_t ier() const { return static_cast<uint8_t>(ier_ | 0x80); }
    [[nodiscard]] bool irq_asserted() const { return irq_asserted_; }
    [[nodiscard]] uint16_t timer1_counter() const { return timer1_counter_; }
    [[nodiscard]] uint16_t timer1_latch() const { return timer1_latch_; }
    [[nodiscard]] bool timer1_running() const { return timer1_running_; }
    [[nodiscard]] bool timer1_pb7_output_enabled() const { return (acr_ & 0x80) != 0; }
    [[nodiscard]] bool timer1_free_running() const { return (acr_ & 0x40) != 0; }
    [[nodiscard]] bool timer1_pb7_level() const { return timer1_pb7_level_; }
    [[nodiscard]] bool pb7_pin_level() const { return (read_port_b() & 0x80) != 0; }
    [[nodiscard]] uint64_t pb7_transition_count() const { return pb7_transition_count_; }

private:
    static constexpr uint8_t IfrTimer1 = 0x40;
    static constexpr uint8_t IfrTimer2 = 0x20;

    [[nodiscard]] uint8_t read_port_a() const;
    [[nodiscard]] uint8_t read_port_b() const;
    [[nodiscard]] uint8_t effective_output_b() const;
    [[nodiscard]] uint8_t raw_ifr() const { return static_cast<uint8_t>(ifr_ & 0x7F); }
    void record_pb7_transition(bool previous_level);
    void clear_ifr(uint8_t mask);
    void set_ifr(uint8_t mask);
    void refresh_irq();
    void tick_timer(uint32_t cycles, uint16_t& counter, uint16_t latch, uint8_t flag, bool continuous);

    IrqCallback irq_cb_;
    uint8_t output_a_{};
    uint8_t output_b_{};
    uint8_t input_a_{0xFF};
    uint8_t input_b_{0xFF};
    uint8_t ddr_a_{};
    uint8_t ddr_b_{};
    uint8_t acr_{};
    uint8_t pcr_{};
    uint8_t shift_{};
    uint8_t ifr_{};
    uint8_t ier_{};
    uint16_t timer1_counter_{};
    uint16_t timer1_latch_{};
    uint16_t timer2_counter_{};
    uint16_t timer2_latch_{};
    bool timer1_running_{};
    bool timer2_running_{};
    bool timer1_pb7_level_{true};
    uint64_t pb7_transition_count_{};
    bool irq_asserted_{};
};

} // namespace microlind::devices
