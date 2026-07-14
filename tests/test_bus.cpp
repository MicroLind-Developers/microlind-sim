#include <cstddef>
#include <cstdint>
#include <algorithm>
#include <memory>
#include <vector>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "microlind/bus.hpp"
#include "microlind/simulator.hpp"

namespace {

class PhaseProbeMemory : public microlind::BusDevice {
public:
    uint8_t read8(uint16_t offset) override {
        return data[offset];
    }

    uint8_t peek8(uint16_t offset) override {
        return data[offset];
    }

    void write8(uint16_t offset, uint8_t value) override {
        data[offset] = value;
    }

    void tick(uint32_t cycles) override {
        ticked_cycles += cycles;
    }

    void tick_phase(const microlind::BusSignals& signals) override {
        phases.push_back(signals);
    }

    std::vector<uint8_t> data = std::vector<uint8_t>(0x10000, 0x00);
    uint32_t ticked_cycles{};
    std::vector<microlind::BusSignals> phases;
};

class SideEffectMemory : public microlind::BusDevice {
public:
    uint8_t read8(uint16_t offset) override {
        ++read_count;
        const uint8_t value = data[offset];
        data[offset] = 0x00;
        return value;
    }

    uint8_t peek8(uint16_t offset) override {
        return data[offset];
    }

    void write8(uint16_t offset, uint8_t value) override {
        ++write_count;
        data[offset] = value;
    }

    void tick_phase(const microlind::BusSignals& signals) override {
        phases.push_back(signals);
    }

    std::vector<uint8_t> data = std::vector<uint8_t>(0x10000, 0x00);
    uint32_t read_count{};
    uint32_t write_count{};
    std::vector<microlind::BusSignals> phases;
};

enum class MicroTargetRegister {
    A,
    B,
};

enum class MicroAddressMode {
    Immediate,
    Direct,
    Extended,
};

struct MicroLoadCase {
    const char* name;
    uint8_t opcode{};
    MicroTargetRegister target{};
    MicroAddressMode mode{};
    uint16_t address{};
    uint8_t value{};
};

struct MicroStoreCase {
    const char* name;
    uint8_t opcode{};
    MicroTargetRegister source{};
    MicroAddressMode mode{};
    uint16_t address{};
    uint8_t value{};
};

struct MicroWordLoadCase {
    const char* name;
    uint8_t opcode{};
    MicroAddressMode mode{};
    uint16_t address{};
    uint16_t value{};
};

struct MicroWordStoreCase {
    const char* name;
    uint8_t opcode{};
    MicroAddressMode mode{};
    uint16_t address{};
    uint16_t value{};
};

struct MicroBranchCase {
    const char* name;
    uint8_t opcode{};
    uint8_t cc{};
    uint8_t offset{};
    bool taken{};
    uint16_t expected_pc{};
};

struct MicroSubroutineBranchCase {
    const char* name;
    uint8_t opcode{};
    uint8_t md{};
    uint16_t offset{};
    uint8_t operand_bytes{};
    uint16_t return_pc{};
    uint16_t expected_pc{};
    uint8_t expected_cycles{};
};

struct MicroSubroutineJumpCase {
    const char* name;
    uint8_t opcode{};
    MicroAddressMode mode{};
    uint8_t dp{};
    uint16_t target{};
    uint16_t return_pc{};
    uint8_t expected_cycles{};
};

struct MicroJumpCase {
    const char* name;
    uint8_t opcode{};
    MicroAddressMode mode{};
    uint8_t dp{};
    uint8_t md{};
    uint16_t target{};
    uint8_t expected_cycles{};
};

struct MicroClearCase {
    const char* name;
    uint8_t opcode{};
    MicroTargetRegister target{};
    uint8_t md{};
    uint8_t expected_cycles{};
    uint8_t initial_target{};
    uint8_t initial_other{};
    uint8_t expected_target{};
    bool expect_z{};
    bool expect_n{};
    bool expect_v{};
    bool expect_carry{};
};

struct MicroAluImmediateCase {
    const char* name;
    uint8_t opcode{};
    MicroTargetRegister target{};
    uint8_t initial_target{};
    uint8_t initial_other{};
    uint8_t operand{};
    uint8_t initial_cc{};
    uint8_t expected_target{};
    bool expect_z{};
    bool expect_n{};
    bool expect_v{};
    bool expect_carry{};
    bool expect_half{};
};

uint8_t high_byte(uint16_t value) {
    return static_cast<uint8_t>((value >> 8) & 0xFF);
}

uint8_t low_byte(uint16_t value) {
    return static_cast<uint8_t>(value & 0xFF);
}

uint8_t address_byte_count(MicroAddressMode mode) {
    switch (mode) {
    case MicroAddressMode::Immediate: return 0;
    case MicroAddressMode::Direct: return 1;
    case MicroAddressMode::Extended: return 2;
    }
    return 0;
}

uint16_t load_data_address(const MicroLoadCase& test) {
    return test.mode == MicroAddressMode::Immediate ? 0x0101 : test.address;
}

uint16_t word_load_data_address(const MicroWordLoadCase& test) {
    return test.mode == MicroAddressMode::Immediate ? 0x0101 : test.address;
}

void write_load_program(SideEffectMemory& memory, const MicroLoadCase& test) {
    memory.data[0x0100] = test.opcode;
    switch (test.mode) {
    case MicroAddressMode::Immediate:
        memory.data[0x0101] = test.value;
        break;
    case MicroAddressMode::Direct:
        memory.data[0x0101] = low_byte(test.address);
        memory.data[test.address] = test.value;
        break;
    case MicroAddressMode::Extended:
        memory.data[0x0101] = high_byte(test.address);
        memory.data[0x0102] = low_byte(test.address);
        memory.data[test.address] = test.value;
        break;
    }
}

void write_word_load_program(SideEffectMemory& memory, const MicroWordLoadCase& test) {
    memory.data[0x0100] = test.opcode;
    switch (test.mode) {
    case MicroAddressMode::Immediate:
        memory.data[0x0101] = high_byte(test.value);
        memory.data[0x0102] = low_byte(test.value);
        break;
    case MicroAddressMode::Direct:
        memory.data[0x0101] = low_byte(test.address);
        memory.data[test.address] = high_byte(test.value);
        memory.data[static_cast<uint16_t>(test.address + 1)] = low_byte(test.value);
        break;
    case MicroAddressMode::Extended:
        memory.data[0x0101] = high_byte(test.address);
        memory.data[0x0102] = low_byte(test.address);
        memory.data[test.address] = high_byte(test.value);
        memory.data[static_cast<uint16_t>(test.address + 1)] = low_byte(test.value);
        break;
    }
}

void write_store_program(SideEffectMemory& memory, const MicroStoreCase& test) {
    memory.data[0x0100] = test.opcode;
    switch (test.mode) {
    case MicroAddressMode::Immediate:
        break;
    case MicroAddressMode::Direct:
        memory.data[0x0101] = low_byte(test.address);
        break;
    case MicroAddressMode::Extended:
        memory.data[0x0101] = high_byte(test.address);
        memory.data[0x0102] = low_byte(test.address);
        break;
    }
}

void write_word_store_program(SideEffectMemory& memory, const MicroWordStoreCase& test) {
    memory.data[0x0100] = test.opcode;
    switch (test.mode) {
    case MicroAddressMode::Immediate:
        break;
    case MicroAddressMode::Direct:
        memory.data[0x0101] = low_byte(test.address);
        break;
    case MicroAddressMode::Extended:
        memory.data[0x0101] = high_byte(test.address);
        memory.data[0x0102] = low_byte(test.address);
        break;
    }
}

uint8_t register_value(const microlind::Simulator& sim, MicroTargetRegister reg) {
    return reg == MicroTargetRegister::A ? sim.cpu().regs().a : sim.cpu().regs().b;
}

uint8_t other_register_value(const microlind::Simulator& sim, MicroTargetRegister reg) {
    return reg == MicroTargetRegister::A ? sim.cpu().regs().b : sim.cpu().regs().a;
}

void seed_registers(microlind::Simulator& sim, uint8_t a, uint8_t b) {
    sim.cpu().regs().a = a;
    sim.cpu().regs().b = b;
}

void expect_completed_or_drain_internal_cycle(microlind::Simulator& sim, bool complete_on_data_cycle) {
    if (complete_on_data_cycle) {
        EXPECT_FALSE(sim.has_pending_microcycles());
        return;
    }

    const auto internal_cycle = sim.tick_microcycle();
    EXPECT_TRUE(internal_cycle.emitted);
    EXPECT_FALSE(internal_cycle.instruction_started);
    EXPECT_TRUE(internal_cycle.instruction_complete);
    EXPECT_EQ(internal_cycle.signals.cycle_kind, microlind::BusCycleKind::Internal);
    EXPECT_FALSE(sim.has_pending_microcycles());
}

void expect_internal_cycles(microlind::Simulator& sim, uint8_t cycles) {
    for (uint8_t i = 0; i < cycles; ++i) {
        const auto internal_cycle = sim.tick_microcycle();
        EXPECT_TRUE(internal_cycle.emitted);
        EXPECT_FALSE(internal_cycle.instruction_started);
        EXPECT_EQ(internal_cycle.instruction_complete, i + 1 == cycles);
        EXPECT_EQ(internal_cycle.signals.cycle_kind, microlind::BusCycleKind::Internal);
    }
    EXPECT_FALSE(sim.has_pending_microcycles());
}

void run_load_microcycle_case(const MicroLoadCase& test) {
    microlind::Simulator sim(microlind::CpuMode::HD6309, 1'000'000);
    auto memory = std::make_unique<SideEffectMemory>();
    auto* probe = memory.get();
    write_load_program(*probe, test);
    ASSERT_FALSE(sim.map_device(0x0000, 0xFFFF, std::move(memory)));

    sim.cpu().set_pc(0x0100);
    seed_registers(sim, 0x11, 0x22);
    const uint8_t initial_target = register_value(sim, test.target);
    const uint8_t initial_other = other_register_value(sim, test.target);

    const auto opcode_fetch = sim.tick_microcycle();
    EXPECT_TRUE(opcode_fetch.emitted);
    EXPECT_TRUE(opcode_fetch.instruction_started);
    EXPECT_FALSE(opcode_fetch.instruction_complete);
    EXPECT_TRUE(sim.has_pending_microcycles());
    EXPECT_EQ(sim.cpu().regs().pc, 0x0101);
    EXPECT_EQ(register_value(sim, test.target), initial_target);
    EXPECT_EQ(probe->read_count, 1u);
    ASSERT_THAT(sim.bus().access_log(), testing::SizeIs(1));
    EXPECT_EQ(sim.bus().access_log().back().address, 0x0100);
    EXPECT_EQ(sim.bus().access_log().back().cycle_kind, microlind::BusCycleKind::OpcodeFetch);

    const uint8_t address_reads = address_byte_count(test.mode);
    for (uint8_t i = 0; i < address_reads; ++i) {
        const auto address_read = sim.tick_microcycle();
        EXPECT_TRUE(address_read.emitted);
        EXPECT_FALSE(address_read.instruction_started);
        EXPECT_FALSE(address_read.instruction_complete);
        EXPECT_EQ(sim.cpu().regs().pc, static_cast<uint16_t>(0x0102 + i));
        EXPECT_EQ(register_value(sim, test.target), initial_target);
        EXPECT_EQ(probe->read_count, static_cast<uint32_t>(2 + i));
        ASSERT_THAT(sim.bus().access_log(), testing::SizeIs(static_cast<std::size_t>(2 + i)));
        EXPECT_EQ(sim.bus().access_log().back().address, static_cast<uint16_t>(0x0101 + i));
        EXPECT_EQ(sim.bus().access_log().back().cycle_kind, microlind::BusCycleKind::OperandRead);
    }

    const auto data_read = sim.tick_microcycle();
    const bool complete_on_data_cycle = test.mode == MicroAddressMode::Immediate;
    EXPECT_TRUE(data_read.emitted);
    EXPECT_FALSE(data_read.instruction_started);
    EXPECT_EQ(data_read.instruction_complete, complete_on_data_cycle);
    EXPECT_EQ(register_value(sim, test.target), test.value);
    EXPECT_EQ(other_register_value(sim, test.target), initial_other);
    const uint16_t expected_data_pc = complete_on_data_cycle
        ? 0x0102
        : static_cast<uint16_t>(0x0101 + address_reads);
    EXPECT_EQ(sim.cpu().regs().pc, expected_data_pc);
    EXPECT_EQ(probe->read_count, static_cast<uint32_t>(2 + address_reads));
    ASSERT_THAT(sim.bus().access_log(), testing::SizeIs(static_cast<std::size_t>(2 + address_reads)));
    EXPECT_EQ(sim.bus().access_log().back().address, load_data_address(test));
    EXPECT_EQ(sim.bus().access_log().back().value, test.value);
    EXPECT_EQ(sim.bus().access_log().back().cycle_kind, microlind::BusCycleKind::OperandRead);
    EXPECT_EQ(probe->data[load_data_address(test)], 0x00);

    expect_completed_or_drain_internal_cycle(sim, complete_on_data_cycle);
}

void run_store_microcycle_case(const MicroStoreCase& test) {
    microlind::Simulator sim(microlind::CpuMode::HD6309, 1'000'000);
    auto memory = std::make_unique<SideEffectMemory>();
    auto* probe = memory.get();
    write_store_program(*probe, test);
    ASSERT_FALSE(sim.map_device(0x0000, 0xFFFF, std::move(memory)));

    sim.cpu().set_pc(0x0100);
    seed_registers(
        sim,
        test.source == MicroTargetRegister::A ? test.value : 0x11,
        test.source == MicroTargetRegister::B ? test.value : 0x22);

    const auto opcode_fetch = sim.tick_microcycle();
    EXPECT_TRUE(opcode_fetch.emitted);
    EXPECT_TRUE(opcode_fetch.instruction_started);
    EXPECT_FALSE(opcode_fetch.instruction_complete);
    EXPECT_EQ(probe->write_count, 0u);
    EXPECT_EQ(probe->data[test.address], 0x00);
    EXPECT_EQ(sim.cpu().regs().pc, 0x0101);

    const uint8_t address_reads = address_byte_count(test.mode);
    for (uint8_t i = 0; i < address_reads; ++i) {
        const auto address_read = sim.tick_microcycle();
        EXPECT_TRUE(address_read.emitted);
        EXPECT_FALSE(address_read.instruction_started);
        EXPECT_FALSE(address_read.instruction_complete);
        EXPECT_EQ(probe->write_count, 0u);
        EXPECT_EQ(probe->data[test.address], 0x00);
        EXPECT_EQ(sim.cpu().regs().pc, static_cast<uint16_t>(0x0102 + i));
        ASSERT_THAT(sim.bus().access_log(), testing::SizeIs(static_cast<std::size_t>(2 + i)));
        EXPECT_EQ(sim.bus().access_log().back().address, static_cast<uint16_t>(0x0101 + i));
        EXPECT_EQ(sim.bus().access_log().back().cycle_kind, microlind::BusCycleKind::OperandRead);
    }

    const auto write_cycle = sim.tick_microcycle();
    EXPECT_TRUE(write_cycle.emitted);
    EXPECT_FALSE(write_cycle.instruction_started);
    EXPECT_FALSE(write_cycle.instruction_complete);
    EXPECT_EQ(probe->write_count, 1u);
    EXPECT_EQ(probe->data[test.address], test.value);
    EXPECT_EQ(sim.cpu().regs().pc, static_cast<uint16_t>(0x0101 + address_reads));
    ASSERT_THAT(sim.bus().access_log(), testing::SizeIs(static_cast<std::size_t>(2 + address_reads)));
    EXPECT_EQ(sim.bus().access_log().back().type, microlind::BusAccessType::Write);
    EXPECT_EQ(sim.bus().access_log().back().address, test.address);
    EXPECT_EQ(sim.bus().access_log().back().value, test.value);
    EXPECT_EQ(sim.bus().access_log().back().cycle_kind, microlind::BusCycleKind::OperandWrite);

    expect_completed_or_drain_internal_cycle(sim, false);
}

void run_word_load_microcycle_case(const MicroWordLoadCase& test) {
    microlind::Simulator sim(microlind::CpuMode::HD6309, 1'000'000);
    auto memory = std::make_unique<SideEffectMemory>();
    auto* probe = memory.get();
    write_word_load_program(*probe, test);
    ASSERT_FALSE(sim.map_device(0x0000, 0xFFFF, std::move(memory)));

    sim.cpu().set_pc(0x0100);
    seed_registers(sim, 0x11, 0x22);

    const auto opcode_fetch = sim.tick_microcycle();
    EXPECT_TRUE(opcode_fetch.emitted);
    EXPECT_TRUE(opcode_fetch.instruction_started);
    EXPECT_FALSE(opcode_fetch.instruction_complete);
    EXPECT_TRUE(sim.has_pending_microcycles());
    EXPECT_EQ(sim.cpu().regs().pc, 0x0101);
    EXPECT_EQ(sim.cpu().regs().a, 0x11);
    EXPECT_EQ(sim.cpu().regs().b, 0x22);
    ASSERT_THAT(sim.bus().access_log(), testing::SizeIs(1));
    EXPECT_EQ(sim.bus().access_log().back().address, 0x0100);
    EXPECT_EQ(sim.bus().access_log().back().cycle_kind, microlind::BusCycleKind::OpcodeFetch);

    const uint8_t address_reads = address_byte_count(test.mode);
    for (uint8_t i = 0; i < address_reads; ++i) {
        const auto address_read = sim.tick_microcycle();
        EXPECT_TRUE(address_read.emitted);
        EXPECT_FALSE(address_read.instruction_started);
        EXPECT_FALSE(address_read.instruction_complete);
        EXPECT_EQ(sim.cpu().regs().pc, static_cast<uint16_t>(0x0102 + i));
        EXPECT_EQ(sim.cpu().regs().a, 0x11);
        EXPECT_EQ(sim.cpu().regs().b, 0x22);
        ASSERT_THAT(sim.bus().access_log(), testing::SizeIs(static_cast<std::size_t>(2 + i)));
        EXPECT_EQ(sim.bus().access_log().back().address, static_cast<uint16_t>(0x0101 + i));
        EXPECT_EQ(sim.bus().access_log().back().cycle_kind, microlind::BusCycleKind::OperandRead);
    }

    const auto high_read = sim.tick_microcycle();
    EXPECT_TRUE(high_read.emitted);
    EXPECT_FALSE(high_read.instruction_started);
    EXPECT_FALSE(high_read.instruction_complete);
    EXPECT_EQ(sim.cpu().regs().a, 0x11);
    EXPECT_EQ(sim.cpu().regs().b, 0x22);
    const uint16_t high_read_pc = test.mode == MicroAddressMode::Immediate
        ? 0x0102
        : static_cast<uint16_t>(0x0101 + address_reads);
    EXPECT_EQ(sim.cpu().regs().pc, high_read_pc);
    EXPECT_EQ(probe->read_count, static_cast<uint32_t>(2 + address_reads));
    ASSERT_THAT(sim.bus().access_log(), testing::SizeIs(static_cast<std::size_t>(2 + address_reads)));
    EXPECT_EQ(sim.bus().access_log().back().address, word_load_data_address(test));
    EXPECT_EQ(sim.bus().access_log().back().value, high_byte(test.value));
    EXPECT_EQ(sim.bus().access_log().back().cycle_kind, microlind::BusCycleKind::OperandRead);

    const auto low_read = sim.tick_microcycle();
    const bool complete_on_low_cycle = test.mode == MicroAddressMode::Immediate;
    EXPECT_TRUE(low_read.emitted);
    EXPECT_FALSE(low_read.instruction_started);
    EXPECT_EQ(low_read.instruction_complete, complete_on_low_cycle);
    EXPECT_EQ(sim.cpu().regs().a, high_byte(test.value));
    EXPECT_EQ(sim.cpu().regs().b, low_byte(test.value));
    const uint16_t low_read_pc = complete_on_low_cycle
        ? 0x0103
        : static_cast<uint16_t>(0x0101 + address_reads);
    EXPECT_EQ(sim.cpu().regs().pc, low_read_pc);
    EXPECT_EQ(probe->read_count, static_cast<uint32_t>(3 + address_reads));
    ASSERT_THAT(sim.bus().access_log(), testing::SizeIs(static_cast<std::size_t>(3 + address_reads)));
    EXPECT_EQ(sim.bus().access_log().back().address, static_cast<uint16_t>(word_load_data_address(test) + 1));
    EXPECT_EQ(sim.bus().access_log().back().value, low_byte(test.value));
    EXPECT_EQ(sim.bus().access_log().back().cycle_kind, microlind::BusCycleKind::OperandRead);
    EXPECT_EQ(probe->data[word_load_data_address(test)], 0x00);
    EXPECT_EQ(probe->data[static_cast<uint16_t>(word_load_data_address(test) + 1)], 0x00);

    expect_completed_or_drain_internal_cycle(sim, complete_on_low_cycle);
}

void run_word_store_microcycle_case(const MicroWordStoreCase& test) {
    microlind::Simulator sim(microlind::CpuMode::HD6309, 1'000'000);
    auto memory = std::make_unique<SideEffectMemory>();
    auto* probe = memory.get();
    write_word_store_program(*probe, test);
    ASSERT_FALSE(sim.map_device(0x0000, 0xFFFF, std::move(memory)));

    sim.cpu().set_pc(0x0100);
    seed_registers(sim, high_byte(test.value), low_byte(test.value));

    const auto opcode_fetch = sim.tick_microcycle();
    EXPECT_TRUE(opcode_fetch.emitted);
    EXPECT_TRUE(opcode_fetch.instruction_started);
    EXPECT_FALSE(opcode_fetch.instruction_complete);
    EXPECT_EQ(probe->write_count, 0u);
    EXPECT_EQ(probe->data[test.address], 0x00);
    EXPECT_EQ(probe->data[static_cast<uint16_t>(test.address + 1)], 0x00);
    EXPECT_EQ(sim.cpu().regs().pc, 0x0101);

    const uint8_t address_reads = address_byte_count(test.mode);
    for (uint8_t i = 0; i < address_reads; ++i) {
        const auto address_read = sim.tick_microcycle();
        EXPECT_TRUE(address_read.emitted);
        EXPECT_FALSE(address_read.instruction_started);
        EXPECT_FALSE(address_read.instruction_complete);
        EXPECT_EQ(probe->write_count, 0u);
        EXPECT_EQ(sim.cpu().regs().pc, static_cast<uint16_t>(0x0102 + i));
        ASSERT_THAT(sim.bus().access_log(), testing::SizeIs(static_cast<std::size_t>(2 + i)));
        EXPECT_EQ(sim.bus().access_log().back().address, static_cast<uint16_t>(0x0101 + i));
        EXPECT_EQ(sim.bus().access_log().back().cycle_kind, microlind::BusCycleKind::OperandRead);
    }

    const auto high_write = sim.tick_microcycle();
    EXPECT_TRUE(high_write.emitted);
    EXPECT_FALSE(high_write.instruction_started);
    EXPECT_FALSE(high_write.instruction_complete);
    EXPECT_EQ(probe->write_count, 1u);
    EXPECT_EQ(probe->data[test.address], high_byte(test.value));
    EXPECT_EQ(probe->data[static_cast<uint16_t>(test.address + 1)], 0x00);
    EXPECT_EQ(sim.cpu().regs().pc, static_cast<uint16_t>(0x0101 + address_reads));
    ASSERT_THAT(sim.bus().access_log(), testing::SizeIs(static_cast<std::size_t>(2 + address_reads)));
    EXPECT_EQ(sim.bus().access_log().back().type, microlind::BusAccessType::Write);
    EXPECT_EQ(sim.bus().access_log().back().address, test.address);
    EXPECT_EQ(sim.bus().access_log().back().value, high_byte(test.value));
    EXPECT_EQ(sim.bus().access_log().back().cycle_kind, microlind::BusCycleKind::OperandWrite);

    const auto low_write = sim.tick_microcycle();
    EXPECT_TRUE(low_write.emitted);
    EXPECT_FALSE(low_write.instruction_started);
    EXPECT_FALSE(low_write.instruction_complete);
    EXPECT_EQ(probe->write_count, 2u);
    EXPECT_EQ(probe->data[test.address], high_byte(test.value));
    EXPECT_EQ(probe->data[static_cast<uint16_t>(test.address + 1)], low_byte(test.value));
    EXPECT_EQ(sim.cpu().regs().pc, static_cast<uint16_t>(0x0101 + address_reads));
    ASSERT_THAT(sim.bus().access_log(), testing::SizeIs(static_cast<std::size_t>(3 + address_reads)));
    EXPECT_EQ(sim.bus().access_log().back().type, microlind::BusAccessType::Write);
    EXPECT_EQ(sim.bus().access_log().back().address, static_cast<uint16_t>(test.address + 1));
    EXPECT_EQ(sim.bus().access_log().back().value, low_byte(test.value));
    EXPECT_EQ(sim.bus().access_log().back().cycle_kind, microlind::BusCycleKind::OperandWrite);

    expect_completed_or_drain_internal_cycle(sim, false);
}

void run_branch_microcycle_case(const MicroBranchCase& test) {
    microlind::Simulator sim(microlind::CpuMode::HD6309, 1'000'000);
    auto memory = std::make_unique<SideEffectMemory>();
    auto* probe = memory.get();
    probe->data[0x0100] = test.opcode;
    probe->data[0x0101] = test.offset;
    ASSERT_FALSE(sim.map_device(0x0000, 0xFFFF, std::move(memory)));

    sim.cpu().set_pc(0x0100);
    sim.cpu().regs().cc = test.cc;

    const auto opcode_fetch = sim.tick_microcycle();
    EXPECT_TRUE(opcode_fetch.emitted);
    EXPECT_TRUE(opcode_fetch.instruction_started);
    EXPECT_FALSE(opcode_fetch.instruction_complete);
    EXPECT_TRUE(sim.has_pending_microcycles());
    EXPECT_EQ(sim.cpu().regs().pc, 0x0101);
    ASSERT_THAT(sim.bus().access_log(), testing::SizeIs(1));
    EXPECT_EQ(sim.bus().access_log().back().address, 0x0100);
    EXPECT_EQ(sim.bus().access_log().back().cycle_kind, microlind::BusCycleKind::OpcodeFetch);

    const auto offset_read = sim.tick_microcycle();
    EXPECT_TRUE(offset_read.emitted);
    EXPECT_FALSE(offset_read.instruction_started);
    EXPECT_EQ(offset_read.instruction_complete, !test.taken);
    EXPECT_EQ(sim.cpu().regs().pc, test.expected_pc);
    EXPECT_EQ(probe->read_count, 2u);
    ASSERT_THAT(sim.bus().access_log(), testing::SizeIs(2));
    EXPECT_EQ(sim.bus().access_log().back().address, 0x0101);
    EXPECT_EQ(sim.bus().access_log().back().value, test.offset);
    EXPECT_EQ(sim.bus().access_log().back().cycle_kind, microlind::BusCycleKind::OperandRead);

    expect_completed_or_drain_internal_cycle(sim, !test.taken);
}

void run_subroutine_branch_microcycle_case(const MicroSubroutineBranchCase& test) {
    microlind::Simulator sim(microlind::CpuMode::HD6309, 1'000'000);
    auto memory = std::make_unique<SideEffectMemory>();
    auto* probe = memory.get();
    probe->data[0x0100] = test.opcode;
    if (test.operand_bytes == 1) {
        probe->data[0x0101] = low_byte(test.offset);
    } else {
        probe->data[0x0101] = high_byte(test.offset);
        probe->data[0x0102] = low_byte(test.offset);
    }
    ASSERT_FALSE(sim.map_device(0x0000, 0xFFFF, std::move(memory)));

    sim.cpu().set_pc(0x0100);
    sim.cpu().regs().s = 0x9000;
    sim.cpu().regs().md = test.md;

    const auto opcode_fetch = sim.tick_microcycle();
    EXPECT_TRUE(opcode_fetch.emitted);
    EXPECT_TRUE(opcode_fetch.instruction_started);
    EXPECT_FALSE(opcode_fetch.instruction_complete);
    EXPECT_EQ(opcode_fetch.instruction_result.cycles, test.expected_cycles);
    EXPECT_EQ(sim.cpu().regs().pc, 0x0101);
    EXPECT_EQ(sim.cpu().regs().s, 0x9000);
    ASSERT_THAT(sim.bus().access_log(), testing::SizeIs(1));
    EXPECT_EQ(sim.bus().access_log().back().address, 0x0100);
    EXPECT_EQ(sim.bus().access_log().back().cycle_kind, microlind::BusCycleKind::OpcodeFetch);

    for (uint8_t i = 0; i < test.operand_bytes; ++i) {
        const auto operand_read = sim.tick_microcycle();
        EXPECT_TRUE(operand_read.emitted);
        EXPECT_FALSE(operand_read.instruction_started);
        EXPECT_FALSE(operand_read.instruction_complete);
        EXPECT_EQ(sim.cpu().regs().pc, static_cast<uint16_t>(0x0102 + i));
        EXPECT_EQ(sim.cpu().regs().s, 0x9000);
        ASSERT_THAT(sim.bus().access_log(), testing::SizeIs(static_cast<std::size_t>(2 + i)));
        EXPECT_EQ(sim.bus().access_log().back().address, static_cast<uint16_t>(0x0101 + i));
        EXPECT_EQ(sim.bus().access_log().back().cycle_kind, microlind::BusCycleKind::OperandRead);
    }

    const auto low_write = sim.tick_microcycle();
    EXPECT_TRUE(low_write.emitted);
    EXPECT_FALSE(low_write.instruction_started);
    EXPECT_FALSE(low_write.instruction_complete);
    EXPECT_EQ(sim.cpu().regs().pc, test.return_pc);
    EXPECT_EQ(sim.cpu().regs().s, 0x8FFF);
    EXPECT_EQ(probe->write_count, 1u);
    EXPECT_EQ(probe->data[0x8FFF], low_byte(test.return_pc));
    ASSERT_THAT(sim.bus().access_log(), testing::SizeIs(static_cast<std::size_t>(2 + test.operand_bytes)));
    EXPECT_EQ(sim.bus().access_log().back().type, microlind::BusAccessType::Write);
    EXPECT_EQ(sim.bus().access_log().back().address, 0x8FFF);
    EXPECT_EQ(sim.bus().access_log().back().value, low_byte(test.return_pc));
    EXPECT_EQ(sim.bus().access_log().back().cycle_kind, microlind::BusCycleKind::StackWrite);

    const auto high_write = sim.tick_microcycle();
    EXPECT_TRUE(high_write.emitted);
    EXPECT_FALSE(high_write.instruction_started);
    EXPECT_FALSE(high_write.instruction_complete);
    EXPECT_EQ(sim.cpu().regs().pc, test.expected_pc);
    EXPECT_EQ(sim.cpu().regs().s, 0x8FFE);
    EXPECT_EQ(probe->write_count, 2u);
    EXPECT_EQ(probe->data[0x8FFE], high_byte(test.return_pc));
    ASSERT_THAT(sim.bus().access_log(), testing::SizeIs(static_cast<std::size_t>(3 + test.operand_bytes)));
    EXPECT_EQ(sim.bus().access_log().back().type, microlind::BusAccessType::Write);
    EXPECT_EQ(sim.bus().access_log().back().address, 0x8FFE);
    EXPECT_EQ(sim.bus().access_log().back().value, high_byte(test.return_pc));
    EXPECT_EQ(sim.bus().access_log().back().cycle_kind, microlind::BusCycleKind::StackWrite);

    const uint8_t emitted_cycles = static_cast<uint8_t>(test.operand_bytes + 3);
    ASSERT_GE(test.expected_cycles, emitted_cycles);
    expect_internal_cycles(sim, static_cast<uint8_t>(test.expected_cycles - emitted_cycles));
}

void run_rts_microcycle_case() {
    microlind::Simulator sim(microlind::CpuMode::HD6309, 1'000'000);
    auto memory = std::make_unique<SideEffectMemory>();
    auto* probe = memory.get();
    probe->data[0x0100] = 0x39; // RTS
    probe->data[0x8FFE] = 0x20;
    probe->data[0x8FFF] = 0x40;
    ASSERT_FALSE(sim.map_device(0x0000, 0xFFFF, std::move(memory)));

    sim.cpu().set_pc(0x0100);
    sim.cpu().regs().s = 0x8FFE;

    const auto opcode_fetch = sim.tick_microcycle();
    EXPECT_TRUE(opcode_fetch.emitted);
    EXPECT_TRUE(opcode_fetch.instruction_started);
    EXPECT_FALSE(opcode_fetch.instruction_complete);
    EXPECT_EQ(opcode_fetch.instruction_result.cycles, 5u);
    EXPECT_EQ(sim.cpu().regs().pc, 0x0101);
    EXPECT_EQ(sim.cpu().regs().s, 0x8FFE);
    ASSERT_THAT(sim.bus().access_log(), testing::SizeIs(1));
    EXPECT_EQ(sim.bus().access_log().back().address, 0x0100);
    EXPECT_EQ(sim.bus().access_log().back().cycle_kind, microlind::BusCycleKind::OpcodeFetch);

    const auto high_read = sim.tick_microcycle();
    EXPECT_TRUE(high_read.emitted);
    EXPECT_FALSE(high_read.instruction_started);
    EXPECT_FALSE(high_read.instruction_complete);
    EXPECT_EQ(sim.cpu().regs().pc, 0x0101);
    EXPECT_EQ(sim.cpu().regs().s, 0x8FFF);
    ASSERT_THAT(sim.bus().access_log(), testing::SizeIs(2));
    EXPECT_EQ(sim.bus().access_log().back().address, 0x8FFE);
    EXPECT_EQ(sim.bus().access_log().back().value, 0x20);
    EXPECT_EQ(sim.bus().access_log().back().cycle_kind, microlind::BusCycleKind::StackRead);

    const auto low_read = sim.tick_microcycle();
    EXPECT_TRUE(low_read.emitted);
    EXPECT_FALSE(low_read.instruction_started);
    EXPECT_FALSE(low_read.instruction_complete);
    EXPECT_EQ(sim.cpu().regs().pc, 0x2040);
    EXPECT_EQ(sim.cpu().regs().s, 0x9000);
    ASSERT_THAT(sim.bus().access_log(), testing::SizeIs(3));
    EXPECT_EQ(sim.bus().access_log().back().address, 0x8FFF);
    EXPECT_EQ(sim.bus().access_log().back().value, 0x40);
    EXPECT_EQ(sim.bus().access_log().back().cycle_kind, microlind::BusCycleKind::StackRead);
    EXPECT_EQ(probe->data[0x8FFE], 0x00);
    EXPECT_EQ(probe->data[0x8FFF], 0x00);

    expect_internal_cycles(sim, 2);
}

void run_subroutine_jump_microcycle_case(const MicroSubroutineJumpCase& test) {
    microlind::Simulator sim(microlind::CpuMode::HD6309, 1'000'000);
    auto memory = std::make_unique<SideEffectMemory>();
    auto* probe = memory.get();
    probe->data[0x0100] = test.opcode;
    switch (test.mode) {
    case MicroAddressMode::Immediate:
        break;
    case MicroAddressMode::Direct:
        probe->data[0x0101] = low_byte(test.target);
        break;
    case MicroAddressMode::Extended:
        probe->data[0x0101] = high_byte(test.target);
        probe->data[0x0102] = low_byte(test.target);
        break;
    }
    ASSERT_FALSE(sim.map_device(0x0000, 0xFFFF, std::move(memory)));

    sim.cpu().set_pc(0x0100);
    sim.cpu().regs().s = 0x9000;
    sim.cpu().regs().dp = test.dp;

    const auto opcode_fetch = sim.tick_microcycle();
    EXPECT_TRUE(opcode_fetch.emitted);
    EXPECT_TRUE(opcode_fetch.instruction_started);
    EXPECT_FALSE(opcode_fetch.instruction_complete);
    EXPECT_EQ(opcode_fetch.instruction_result.cycles, test.expected_cycles);
    EXPECT_EQ(sim.cpu().regs().pc, 0x0101);
    EXPECT_EQ(sim.cpu().regs().s, 0x9000);
    ASSERT_THAT(sim.bus().access_log(), testing::SizeIs(1));
    EXPECT_EQ(sim.bus().access_log().back().address, 0x0100);
    EXPECT_EQ(sim.bus().access_log().back().cycle_kind, microlind::BusCycleKind::OpcodeFetch);

    const uint8_t address_reads = address_byte_count(test.mode);
    for (uint8_t i = 0; i < address_reads; ++i) {
        const auto address_read = sim.tick_microcycle();
        EXPECT_TRUE(address_read.emitted);
        EXPECT_FALSE(address_read.instruction_started);
        EXPECT_FALSE(address_read.instruction_complete);
        EXPECT_EQ(sim.cpu().regs().pc, static_cast<uint16_t>(0x0102 + i));
        EXPECT_EQ(sim.cpu().regs().s, 0x9000);
        ASSERT_THAT(sim.bus().access_log(), testing::SizeIs(static_cast<std::size_t>(2 + i)));
        EXPECT_EQ(sim.bus().access_log().back().address, static_cast<uint16_t>(0x0101 + i));
        EXPECT_EQ(sim.bus().access_log().back().cycle_kind, microlind::BusCycleKind::OperandRead);
    }

    const auto low_write = sim.tick_microcycle();
    EXPECT_TRUE(low_write.emitted);
    EXPECT_FALSE(low_write.instruction_started);
    EXPECT_FALSE(low_write.instruction_complete);
    EXPECT_EQ(sim.cpu().regs().pc, test.return_pc);
    EXPECT_EQ(sim.cpu().regs().s, 0x8FFF);
    EXPECT_EQ(probe->write_count, 1u);
    EXPECT_EQ(probe->data[0x8FFF], low_byte(test.return_pc));
    ASSERT_THAT(sim.bus().access_log(), testing::SizeIs(static_cast<std::size_t>(2 + address_reads)));
    EXPECT_EQ(sim.bus().access_log().back().type, microlind::BusAccessType::Write);
    EXPECT_EQ(sim.bus().access_log().back().address, 0x8FFF);
    EXPECT_EQ(sim.bus().access_log().back().value, low_byte(test.return_pc));
    EXPECT_EQ(sim.bus().access_log().back().cycle_kind, microlind::BusCycleKind::StackWrite);

    const auto high_write = sim.tick_microcycle();
    EXPECT_TRUE(high_write.emitted);
    EXPECT_FALSE(high_write.instruction_started);
    EXPECT_FALSE(high_write.instruction_complete);
    EXPECT_EQ(sim.cpu().regs().pc, test.target);
    EXPECT_EQ(sim.cpu().regs().s, 0x8FFE);
    EXPECT_EQ(probe->write_count, 2u);
    EXPECT_EQ(probe->data[0x8FFE], high_byte(test.return_pc));
    ASSERT_THAT(sim.bus().access_log(), testing::SizeIs(static_cast<std::size_t>(3 + address_reads)));
    EXPECT_EQ(sim.bus().access_log().back().type, microlind::BusAccessType::Write);
    EXPECT_EQ(sim.bus().access_log().back().address, 0x8FFE);
    EXPECT_EQ(sim.bus().access_log().back().value, high_byte(test.return_pc));
    EXPECT_EQ(sim.bus().access_log().back().cycle_kind, microlind::BusCycleKind::StackWrite);

    const uint8_t emitted_cycles = static_cast<uint8_t>(address_reads + 3);
    ASSERT_GE(test.expected_cycles, emitted_cycles);
    expect_internal_cycles(sim, static_cast<uint8_t>(test.expected_cycles - emitted_cycles));
}

void run_jump_microcycle_case(const MicroJumpCase& test) {
    microlind::Simulator sim(microlind::CpuMode::HD6309, 1'000'000);
    auto memory = std::make_unique<SideEffectMemory>();
    auto* probe = memory.get();
    probe->data[0x0100] = test.opcode;
    switch (test.mode) {
    case MicroAddressMode::Immediate:
        break;
    case MicroAddressMode::Direct:
        probe->data[0x0101] = low_byte(test.target);
        break;
    case MicroAddressMode::Extended:
        probe->data[0x0101] = high_byte(test.target);
        probe->data[0x0102] = low_byte(test.target);
        break;
    }
    ASSERT_FALSE(sim.map_device(0x0000, 0xFFFF, std::move(memory)));

    sim.cpu().set_pc(0x0100);
    sim.cpu().regs().dp = test.dp;
    sim.cpu().regs().md = test.md;

    const auto opcode_fetch = sim.tick_microcycle();
    EXPECT_TRUE(opcode_fetch.emitted);
    EXPECT_TRUE(opcode_fetch.instruction_started);
    EXPECT_FALSE(opcode_fetch.instruction_complete);
    EXPECT_EQ(opcode_fetch.instruction_result.cycles, test.expected_cycles);
    EXPECT_EQ(sim.cpu().regs().pc, 0x0101);
    ASSERT_THAT(sim.bus().access_log(), testing::SizeIs(1));
    EXPECT_EQ(sim.bus().access_log().back().address, 0x0100);
    EXPECT_EQ(sim.bus().access_log().back().cycle_kind, microlind::BusCycleKind::OpcodeFetch);

    const uint8_t address_reads = address_byte_count(test.mode);
    for (uint8_t i = 0; i < address_reads; ++i) {
        const auto address_read = sim.tick_microcycle();
        EXPECT_TRUE(address_read.emitted);
        EXPECT_FALSE(address_read.instruction_started);
        const bool final_operand = i + 1 == address_reads;
        EXPECT_EQ(address_read.instruction_complete, final_operand && test.expected_cycles == 1 + address_reads);
        const uint16_t expected_pc = final_operand ? test.target : static_cast<uint16_t>(0x0102 + i);
        EXPECT_EQ(sim.cpu().regs().pc, expected_pc);
        ASSERT_THAT(sim.bus().access_log(), testing::SizeIs(static_cast<std::size_t>(2 + i)));
        EXPECT_EQ(sim.bus().access_log().back().address, static_cast<uint16_t>(0x0101 + i));
        EXPECT_EQ(sim.bus().access_log().back().cycle_kind, microlind::BusCycleKind::OperandRead);
    }

    const uint8_t emitted_cycles = static_cast<uint8_t>(1 + address_reads);
    ASSERT_GE(test.expected_cycles, emitted_cycles);
    expect_internal_cycles(sim, static_cast<uint8_t>(test.expected_cycles - emitted_cycles));
}

void run_clear_microcycle_case(const MicroClearCase& test) {
    microlind::Simulator sim(microlind::CpuMode::HD6309, 1'000'000);
    auto memory = std::make_unique<SideEffectMemory>();
    auto* probe = memory.get();
    probe->data[0x0100] = test.opcode;
    ASSERT_FALSE(sim.map_device(0x0000, 0xFFFF, std::move(memory)));

    sim.cpu().set_pc(0x0100);
    seed_registers(
        sim,
        test.target == MicroTargetRegister::A ? test.initial_target : test.initial_other,
        test.target == MicroTargetRegister::B ? test.initial_target : test.initial_other);
    sim.cpu().regs().cc = static_cast<uint8_t>(microlind::CC_N | microlind::CC_V | microlind::CC_C);
    sim.cpu().regs().md = test.md;

    const auto opcode_fetch = sim.tick_microcycle();
    EXPECT_TRUE(opcode_fetch.emitted);
    EXPECT_TRUE(opcode_fetch.instruction_started);
    EXPECT_EQ(opcode_fetch.instruction_complete, test.expected_cycles == 1);
    EXPECT_EQ(opcode_fetch.instruction_result.cycles, test.expected_cycles);
    EXPECT_EQ(sim.cpu().regs().pc, 0x0101);
    EXPECT_EQ(register_value(sim, test.target), test.expected_target);
    EXPECT_EQ(other_register_value(sim, test.target), test.initial_other);
    EXPECT_EQ((sim.cpu().regs().cc & microlind::CC_Z) != 0, test.expect_z);
    EXPECT_EQ((sim.cpu().regs().cc & microlind::CC_N) != 0, test.expect_n);
    EXPECT_EQ((sim.cpu().regs().cc & microlind::CC_V) != 0, test.expect_v);
    EXPECT_EQ((sim.cpu().regs().cc & microlind::CC_C) != 0, test.expect_carry);
    EXPECT_EQ(probe->read_count, 1u);
    ASSERT_THAT(sim.bus().access_log(), testing::SizeIs(1));
    EXPECT_EQ(sim.bus().access_log().back().address, 0x0100);
    EXPECT_EQ(sim.bus().access_log().back().cycle_kind, microlind::BusCycleKind::OpcodeFetch);

    expect_internal_cycles(sim, static_cast<uint8_t>(test.expected_cycles - 1));
}

void run_alu_immediate_microcycle_case(const MicroAluImmediateCase& test) {
    microlind::Simulator sim(microlind::CpuMode::HD6309, 1'000'000);
    auto memory = std::make_unique<SideEffectMemory>();
    auto* probe = memory.get();
    probe->data[0x0100] = test.opcode;
    probe->data[0x0101] = test.operand;
    ASSERT_FALSE(sim.map_device(0x0000, 0xFFFF, std::move(memory)));

    sim.cpu().set_pc(0x0100);
    seed_registers(
        sim,
        test.target == MicroTargetRegister::A ? test.initial_target : test.initial_other,
        test.target == MicroTargetRegister::B ? test.initial_target : test.initial_other);
    sim.cpu().regs().cc = test.initial_cc;

    const auto opcode_fetch = sim.tick_microcycle();
    EXPECT_TRUE(opcode_fetch.emitted);
    EXPECT_TRUE(opcode_fetch.instruction_started);
    EXPECT_FALSE(opcode_fetch.instruction_complete);
    EXPECT_EQ(opcode_fetch.instruction_result.cycles, 2u);
    EXPECT_EQ(sim.cpu().regs().pc, 0x0101);
    EXPECT_EQ(register_value(sim, test.target), test.initial_target);
    EXPECT_EQ(other_register_value(sim, test.target), test.initial_other);

    const auto operand_read = sim.tick_microcycle();
    EXPECT_TRUE(operand_read.emitted);
    EXPECT_FALSE(operand_read.instruction_started);
    EXPECT_TRUE(operand_read.instruction_complete);
    EXPECT_EQ(operand_read.instruction_result.cycles, 2u);
    EXPECT_EQ(sim.cpu().regs().pc, 0x0102);
    EXPECT_EQ(register_value(sim, test.target), test.expected_target);
    EXPECT_EQ(other_register_value(sim, test.target), test.initial_other);
    EXPECT_EQ((sim.cpu().regs().cc & microlind::CC_Z) != 0, test.expect_z);
    EXPECT_EQ((sim.cpu().regs().cc & microlind::CC_N) != 0, test.expect_n);
    EXPECT_EQ((sim.cpu().regs().cc & microlind::CC_V) != 0, test.expect_v);
    EXPECT_EQ((sim.cpu().regs().cc & microlind::CC_C) != 0, test.expect_carry);
    EXPECT_EQ((sim.cpu().regs().cc & microlind::CC_H) != 0, test.expect_half);

    EXPECT_EQ(probe->read_count, 2u);
    ASSERT_THAT(sim.bus().access_log(), testing::SizeIs(2));
    EXPECT_EQ(sim.bus().access_log()[0].address, 0x0100);
    EXPECT_EQ(sim.bus().access_log()[0].cycle_kind, microlind::BusCycleKind::OpcodeFetch);
    EXPECT_EQ(sim.bus().access_log()[1].address, 0x0101);
    EXPECT_EQ(sim.bus().access_log()[1].cycle_kind, microlind::BusCycleKind::OperandRead);
}

TEST(BusPhaseTest, SimulatorExpandsInstructionCyclesIntoFourBusPhases) {
    microlind::Simulator sim(microlind::CpuMode::HD6309, 1'000'000);
    auto memory = std::make_unique<PhaseProbeMemory>();
    auto* probe = memory.get();
    probe->data[0x0100] = 0x12; // NOP
    ASSERT_FALSE(sim.map_device(0x0000, 0xFFFF, std::move(memory)));

    sim.cpu().set_pc(0x0100);
    const auto result = sim.tick();

    EXPECT_EQ(probe->ticked_cycles, result.cycles);
    ASSERT_EQ(probe->phases.size(), static_cast<std::size_t>(result.cycles) * 4u);
    ASSERT_EQ(sim.bus().phase_log().size(), probe->phases.size());

    EXPECT_EQ(probe->phases[0].phase, microlind::BusPhase::QHighELow);
    EXPECT_TRUE(probe->phases[0].q);
    EXPECT_FALSE(probe->phases[0].e);
    EXPECT_EQ(probe->phases[1].phase, microlind::BusPhase::QHighEHigh);
    EXPECT_TRUE(probe->phases[1].q);
    EXPECT_TRUE(probe->phases[1].e);
    EXPECT_EQ(probe->phases[2].phase, microlind::BusPhase::QLowEHigh);
    EXPECT_FALSE(probe->phases[2].q);
    EXPECT_TRUE(probe->phases[2].e);
    EXPECT_EQ(probe->phases[3].phase, microlind::BusPhase::QLowELow);
    EXPECT_FALSE(probe->phases[3].q);
    EXPECT_FALSE(probe->phases[3].e);

    for (std::size_t i = 0; i < 4; ++i) {
        EXPECT_TRUE(probe->phases[i].memory_enable);
        EXPECT_TRUE(probe->phases[i].rw);
        EXPECT_EQ(probe->phases[i].address, 0x0100);
        EXPECT_EQ(probe->phases[i].data, 0x12);
        EXPECT_EQ(probe->phases[i].cycle_kind, microlind::BusCycleKind::OpcodeFetch);
    }
    if (probe->phases.size() > 4) {
        EXPECT_FALSE(probe->phases[4].memory_enable);
        EXPECT_EQ(probe->phases[4].address, sim.cpu().regs().pc);
        EXPECT_EQ(probe->phases[4].cycle_kind, microlind::BusCycleKind::Internal);
    }
}

TEST(BusPhaseTest, TagsOpcodeAndOperandFetchCycles) {
    microlind::Simulator sim(microlind::CpuMode::HD6309, 1'000'000);
    auto memory = std::make_unique<PhaseProbeMemory>();
    auto* probe = memory.get();
    probe->data[0x0100] = 0x86; // LDA immediate
    probe->data[0x0101] = 0x42;
    ASSERT_FALSE(sim.map_device(0x0000, 0xFFFF, std::move(memory)));

    sim.cpu().set_pc(0x0100);
    const auto result = sim.tick();

    EXPECT_EQ(result.cycles, 2u);
    ASSERT_EQ(probe->phases.size(), 8u);
    for (std::size_t i = 0; i < 4; ++i) {
        EXPECT_EQ(probe->phases[i].cycle_kind, microlind::BusCycleKind::OpcodeFetch);
        EXPECT_EQ(probe->phases[i].address, 0x0100);
    }
    for (std::size_t i = 4; i < 8; ++i) {
        EXPECT_EQ(probe->phases[i].cycle_kind, microlind::BusCycleKind::OperandRead);
        EXPECT_EQ(probe->phases[i].address, 0x0101);
    }
}

TEST(BusPhaseTest, MicrocycleTickEmitsOneBusCycleAtATime) {
    microlind::Simulator sim(microlind::CpuMode::HD6309, 1'000'000);
    auto memory = std::make_unique<PhaseProbeMemory>();
    auto* probe = memory.get();
    probe->data[0x0100] = 0x12; // NOP
    ASSERT_FALSE(sim.map_device(0x0000, 0xFFFF, std::move(memory)));

    sim.cpu().set_pc(0x0100);
    const auto first = sim.tick_microcycle();

    EXPECT_TRUE(first.emitted);
    EXPECT_TRUE(first.instruction_started);
    EXPECT_FALSE(first.instruction_complete);
    EXPECT_EQ(first.instruction_result.cycles, 2u);
    EXPECT_EQ(first.pending_bus_cycles, 1u);
    EXPECT_TRUE(sim.has_pending_microcycles());
    EXPECT_EQ(sim.clock().total_cycles(), 1u);
    EXPECT_EQ(sim.bus().bus_cycle_count(), 1u);
    ASSERT_EQ(probe->phases.size(), 4u);
    EXPECT_EQ(probe->phases.front().cycle_kind, microlind::BusCycleKind::OpcodeFetch);
    EXPECT_EQ(probe->phases.front().address, 0x0100);

    const auto second = sim.tick_microcycle();

    EXPECT_TRUE(second.emitted);
    EXPECT_FALSE(second.instruction_started);
    EXPECT_TRUE(second.instruction_complete);
    EXPECT_EQ(second.pending_bus_cycles, 0u);
    EXPECT_FALSE(sim.has_pending_microcycles());
    EXPECT_EQ(sim.clock().total_cycles(), 2u);
    EXPECT_EQ(sim.bus().bus_cycle_count(), 2u);
    ASSERT_EQ(probe->phases.size(), 8u);
    EXPECT_EQ(probe->phases[4].cycle_kind, microlind::BusCycleKind::Internal);
    EXPECT_FALSE(probe->phases[4].memory_enable);
}

TEST(BusPhaseTest, ResumableMicrocycleLoadsUpdateOnDataRead) {
    const MicroLoadCase cases[] = {
        {"LDA immediate", 0x86, MicroTargetRegister::A, MicroAddressMode::Immediate, 0x0000, 0xA5},
        {"LDB immediate", 0xC6, MicroTargetRegister::B, MicroAddressMode::Immediate, 0x0000, 0x7E},
        {"LDA direct", 0x96, MicroTargetRegister::A, MicroAddressMode::Direct, 0x0020, 0xA5},
        {"LDB direct", 0xD6, MicroTargetRegister::B, MicroAddressMode::Direct, 0x0030, 0xC3},
        {"LDA extended", 0xB6, MicroTargetRegister::A, MicroAddressMode::Extended, 0x2040, 0x8C},
        {"LDB extended", 0xF6, MicroTargetRegister::B, MicroAddressMode::Extended, 0x2040, 0x8C},
    };

    for (const auto& test : cases) {
        SCOPED_TRACE(test.name);
        run_load_microcycle_case(test);
    }
}

TEST(BusPhaseTest, ResumableMicrocycleWordLoadsUpdateOnLowDataRead) {
    const MicroWordLoadCase cases[] = {
        {"LDD immediate", 0xCC, MicroAddressMode::Immediate, 0x0000, 0xA55A},
        {"LDD direct", 0xDC, MicroAddressMode::Direct, 0x0020, 0xC37E},
        {"LDD extended", 0xFC, MicroAddressMode::Extended, 0x2040, 0x8C42},
    };

    for (const auto& test : cases) {
        SCOPED_TRACE(test.name);
        run_word_load_microcycle_case(test);
    }
}

TEST(BusPhaseTest, FullTickDrainsPendingMicrocycles) {
    microlind::Simulator sim(microlind::CpuMode::HD6309, 1'000'000);
    auto memory = std::make_unique<PhaseProbeMemory>();
    auto* probe = memory.get();
    probe->data[0x0100] = 0x12; // NOP
    ASSERT_FALSE(sim.map_device(0x0000, 0xFFFF, std::move(memory)));

    sim.cpu().set_pc(0x0100);
    ASSERT_TRUE(sim.tick_microcycle().emitted);
    ASSERT_TRUE(sim.has_pending_microcycles());

    const auto result = sim.tick();

    EXPECT_EQ(result.cycles, 2u);
    EXPECT_FALSE(sim.has_pending_microcycles());
    EXPECT_EQ(sim.clock().total_cycles(), 2u);
    EXPECT_EQ(sim.bus().bus_cycle_count(), 2u);
    EXPECT_EQ(probe->ticked_cycles, 2u);
    ASSERT_EQ(probe->phases.size(), 8u);
    EXPECT_EQ(probe->phases[0].cycle_kind, microlind::BusCycleKind::OpcodeFetch);
    EXPECT_EQ(probe->phases[4].cycle_kind, microlind::BusCycleKind::Internal);
}

TEST(BusPhaseTest, ResumableMicrocycleStoresWriteOnDataCycle) {
    const MicroStoreCase cases[] = {
        {"STA direct", 0x97, MicroTargetRegister::A, MicroAddressMode::Direct, 0x0020, 0x5A},
        {"STB direct", 0xD7, MicroTargetRegister::B, MicroAddressMode::Direct, 0x0030, 0x9B},
        {"STA extended", 0xB7, MicroTargetRegister::A, MicroAddressMode::Extended, 0x2040, 0x5A},
        {"STB extended", 0xF7, MicroTargetRegister::B, MicroAddressMode::Extended, 0x2040, 0x6B},
    };

    for (const auto& test : cases) {
        SCOPED_TRACE(test.name);
        run_store_microcycle_case(test);
    }
}

TEST(BusPhaseTest, ResumableMicrocycleWordStoresWriteHighThenLow) {
    const MicroWordStoreCase cases[] = {
        {"STD direct", 0xDD, MicroAddressMode::Direct, 0x0020, 0x5A6B},
        {"STD extended", 0xFD, MicroAddressMode::Extended, 0x2040, 0x9BC3},
    };

    for (const auto& test : cases) {
        SCOPED_TRACE(test.name);
        run_word_store_microcycle_case(test);
    }
}

TEST(BusPhaseTest, ResumableMicrocycleShortBranchesUpdatePcAfterOffsetRead) {
    const MicroBranchCase cases[] = {
        {"BRA forward", 0x20, 0x00, 0x05, true, 0x0107},
        {"BRA backward", 0x20, 0x00, 0xFE, true, 0x0100},
        {"BNE taken", 0x26, 0x00, 0x03, true, 0x0105},
        {"BNE not taken", 0x26, microlind::CC_Z, 0x03, false, 0x0102},
        {"BEQ taken", 0x27, microlind::CC_Z, 0xFC, true, 0x00FE},
        {"BEQ not taken", 0x27, 0x00, 0xFC, false, 0x0102},
    };

    for (const auto& test : cases) {
        SCOPED_TRACE(test.name);
        run_branch_microcycle_case(test);
    }
}

TEST(BusPhaseTest, ResumableMicrocycleSubroutineBranchesPushReturnAddress) {
    const MicroSubroutineBranchCase cases[] = {
        {"BSR forward", 0x8D, 0x00, 0x03, 1, 0x0102, 0x0105, 7},
        {"BSR native timing", 0x8D, 0x01, 0xFE, 1, 0x0102, 0x0100, 6},
        {"LBSR forward", 0x17, 0x00, 0x0003, 2, 0x0103, 0x0106, 9},
        {"LBSR native timing", 0x17, 0x01, 0xFFFD, 2, 0x0103, 0x0100, 7},
    };

    for (const auto& test : cases) {
        SCOPED_TRACE(test.name);
        run_subroutine_branch_microcycle_case(test);
    }
}

TEST(BusPhaseTest, ResumableMicrocycleRtsPullsReturnAddress) {
    run_rts_microcycle_case();
}

TEST(BusPhaseTest, ResumableMicrocycleJsrPushesReturnAddressAndJumps) {
    const MicroSubroutineJumpCase cases[] = {
        {"JSR direct", 0x9D, MicroAddressMode::Direct, 0x12, 0x1234, 0x0102, 5},
        {"JSR extended", 0xBD, MicroAddressMode::Extended, 0x00, 0x2040, 0x0103, 7},
    };

    for (const auto& test : cases) {
        SCOPED_TRACE(test.name);
        run_subroutine_jump_microcycle_case(test);
    }
}

TEST(BusPhaseTest, ResumableMicrocycleJmpUpdatesPcAfterAddressOperand) {
    const MicroJumpCase cases[] = {
        {"JMP direct emulation", 0x0E, MicroAddressMode::Direct, 0x12, 0x00, 0x1234, 3},
        {"JMP direct native", 0x0E, MicroAddressMode::Direct, 0x12, 0x01, 0x1234, 2},
        {"JMP extended emulation", 0x7E, MicroAddressMode::Extended, 0x00, 0x00, 0x2040, 4},
        {"JMP extended native", 0x7E, MicroAddressMode::Extended, 0x00, 0x01, 0x2040, 3},
    };

    for (const auto& test : cases) {
        SCOPED_TRACE(test.name);
        run_jump_microcycle_case(test);
    }
}

TEST(BusPhaseTest, ResumableMicrocycleRegisterUnaryUpdatesAfterOpcodeFetch) {
    const MicroClearCase cases[] = {
        {"CLRA emulation", 0x4F, MicroTargetRegister::A, 0x00, 2, 0x80, 0x40, 0x00, true, false, false, true},
        {"CLRA native", 0x4F, MicroTargetRegister::A, 0x01, 1, 0x80, 0x40, 0x00, true, false, false, true},
        {"CLRB emulation", 0x5F, MicroTargetRegister::B, 0x00, 2, 0x40, 0x80, 0x00, true, false, false, true},
        {"CLRB native", 0x5F, MicroTargetRegister::B, 0x01, 1, 0x40, 0x80, 0x00, true, false, false, true},
        {"TSTA emulation", 0x4D, MicroTargetRegister::A, 0x00, 2, 0x80, 0x40, 0x80, false, true, false, false},
        {"TSTA native", 0x4D, MicroTargetRegister::A, 0x01, 1, 0x80, 0x40, 0x80, false, true, false, false},
        {"TSTB emulation", 0x5D, MicroTargetRegister::B, 0x00, 2, 0x00, 0x80, 0x00, true, false, false, false},
        {"TSTB native", 0x5D, MicroTargetRegister::B, 0x01, 1, 0x00, 0x80, 0x00, true, false, false, false},
        {"DECA emulation", 0x4A, MicroTargetRegister::A, 0x00, 2, 0x80, 0x40, 0x7F, false, false, true, true},
        {"DECA native", 0x4A, MicroTargetRegister::A, 0x01, 1, 0x01, 0x40, 0x00, true, false, false, true},
        {"DECB emulation", 0x5A, MicroTargetRegister::B, 0x00, 2, 0x80, 0x40, 0x7F, false, false, true, true},
        {"DECB native", 0x5A, MicroTargetRegister::B, 0x01, 1, 0x01, 0x40, 0x00, true, false, false, true},
        {"INCA emulation", 0x4C, MicroTargetRegister::A, 0x00, 2, 0x7F, 0x40, 0x80, false, true, true, true},
        {"INCA native", 0x4C, MicroTargetRegister::A, 0x01, 1, 0xFF, 0x40, 0x00, true, false, false, true},
        {"INCB emulation", 0x5C, MicroTargetRegister::B, 0x00, 2, 0x7F, 0x40, 0x80, false, true, true, true},
        {"INCB native", 0x5C, MicroTargetRegister::B, 0x01, 1, 0xFF, 0x40, 0x00, true, false, false, true},
        {"COMA emulation", 0x43, MicroTargetRegister::A, 0x00, 2, 0x00, 0x40, 0xFF, false, true, false, true},
        {"COMA native", 0x43, MicroTargetRegister::A, 0x01, 1, 0xFF, 0x40, 0x00, true, false, false, true},
        {"COMB emulation", 0x53, MicroTargetRegister::B, 0x00, 2, 0x00, 0x40, 0xFF, false, true, false, true},
        {"COMB native", 0x53, MicroTargetRegister::B, 0x01, 1, 0xFF, 0x40, 0x00, true, false, false, true},
        {"NEGA emulation", 0x40, MicroTargetRegister::A, 0x00, 2, 0x80, 0x40, 0x80, false, true, true, true},
        {"NEGA native", 0x40, MicroTargetRegister::A, 0x01, 1, 0x00, 0x40, 0x00, true, false, false, false},
        {"NEGB emulation", 0x50, MicroTargetRegister::B, 0x00, 2, 0x80, 0x40, 0x80, false, true, true, true},
        {"NEGB native", 0x50, MicroTargetRegister::B, 0x01, 1, 0x00, 0x40, 0x00, true, false, false, false},
        {"LSRA emulation", 0x44, MicroTargetRegister::A, 0x00, 2, 0x01, 0x40, 0x00, true, false, false, true},
        {"LSRA native", 0x44, MicroTargetRegister::A, 0x01, 1, 0x80, 0x40, 0x40, false, false, false, false},
        {"LSRB emulation", 0x54, MicroTargetRegister::B, 0x00, 2, 0x01, 0x40, 0x00, true, false, false, true},
        {"LSRB native", 0x54, MicroTargetRegister::B, 0x01, 1, 0x80, 0x40, 0x40, false, false, false, false},
        {"RORA emulation", 0x46, MicroTargetRegister::A, 0x00, 2, 0x02, 0x40, 0x81, false, true, true, false},
        {"RORA native", 0x46, MicroTargetRegister::A, 0x01, 1, 0x01, 0x40, 0x80, false, true, false, true},
        {"RORB emulation", 0x56, MicroTargetRegister::B, 0x00, 2, 0x02, 0x40, 0x81, false, true, true, false},
        {"RORB native", 0x56, MicroTargetRegister::B, 0x01, 1, 0x01, 0x40, 0x80, false, true, false, true},
        {"ASRA emulation", 0x47, MicroTargetRegister::A, 0x00, 2, 0x81, 0x40, 0xC0, false, true, false, true},
        {"ASRA native", 0x47, MicroTargetRegister::A, 0x01, 1, 0x02, 0x40, 0x01, false, false, false, false},
        {"ASRB emulation", 0x57, MicroTargetRegister::B, 0x00, 2, 0x81, 0x40, 0xC0, false, true, false, true},
        {"ASRB native", 0x57, MicroTargetRegister::B, 0x01, 1, 0x02, 0x40, 0x01, false, false, false, false},
        {"ASLA emulation", 0x48, MicroTargetRegister::A, 0x00, 2, 0x80, 0x40, 0x00, true, false, true, true},
        {"ASLA native", 0x48, MicroTargetRegister::A, 0x01, 1, 0x40, 0x20, 0x80, false, true, true, false},
        {"ASLB emulation", 0x58, MicroTargetRegister::B, 0x00, 2, 0x80, 0x40, 0x00, true, false, true, true},
        {"ASLB native", 0x58, MicroTargetRegister::B, 0x01, 1, 0x40, 0x20, 0x80, false, true, true, false},
        {"ROLA emulation", 0x49, MicroTargetRegister::A, 0x00, 2, 0x40, 0x20, 0x81, false, true, true, false},
        {"ROLA native", 0x49, MicroTargetRegister::A, 0x01, 1, 0x80, 0x40, 0x01, false, false, true, true},
        {"ROLB emulation", 0x59, MicroTargetRegister::B, 0x00, 2, 0x40, 0x20, 0x81, false, true, true, false},
        {"ROLB native", 0x59, MicroTargetRegister::B, 0x01, 1, 0x80, 0x40, 0x01, false, false, true, true},
    };

    for (const auto& test : cases) {
        SCOPED_TRACE(test.name);
        run_clear_microcycle_case(test);
    }
}

TEST(BusPhaseTest, ResumableMicrocycleImmediateAluUpdatesAfterOperandRead) {
    const MicroAluImmediateCase cases[] = {
        {"SUBA immediate", 0x80, MicroTargetRegister::A, 0x10, 0x42, 0x20, 0, 0xF0, false, true, false, true, false},
        {"CMPA immediate", 0x81, MicroTargetRegister::A, 0x20, 0x42, 0x20, 0, 0x20, true, false, false, false, false},
        {"SBCA immediate", 0x82, MicroTargetRegister::A, 0x00, 0x42, 0x00, microlind::CC_C, 0xFF, false, true, false, true, false},
        {"ANDA immediate", 0x84, MicroTargetRegister::A, 0xF0, 0x42, 0x0F, microlind::CC_C, 0x00, true, false, false, false, false},
        {"BITA immediate", 0x85, MicroTargetRegister::A, 0x80, 0x42, 0x80, microlind::CC_C, 0x80, false, true, false, true, false},
        {"EORA immediate", 0x88, MicroTargetRegister::A, 0xFF, 0x42, 0x0F, microlind::CC_C, 0xF0, false, true, false, false, false},
        {"ADCA immediate", 0x89, MicroTargetRegister::A, 0x7F, 0x42, 0x00, microlind::CC_C, 0x80, false, true, true, false, true},
        {"ORA immediate", 0x8A, MicroTargetRegister::A, 0x10, 0x42, 0x80, microlind::CC_C, 0x90, false, true, false, false, false},
        {"ADDA immediate", 0x8B, MicroTargetRegister::A, 0x0F, 0x42, 0x01, 0, 0x10, false, false, false, false, true},
        {"SUBB immediate", 0xC0, MicroTargetRegister::B, 0x10, 0x42, 0x20, 0, 0xF0, false, true, false, true, false},
        {"CMPB immediate", 0xC1, MicroTargetRegister::B, 0x20, 0x42, 0x20, 0, 0x20, true, false, false, false, false},
        {"SBCB immediate", 0xC2, MicroTargetRegister::B, 0x00, 0x42, 0x00, microlind::CC_C, 0xFF, false, true, false, true, false},
        {"ANDB immediate", 0xC4, MicroTargetRegister::B, 0xF0, 0x42, 0x0F, microlind::CC_C, 0x00, true, false, false, false, false},
        {"BITB immediate", 0xC5, MicroTargetRegister::B, 0x80, 0x42, 0x80, microlind::CC_C, 0x80, false, true, false, true, false},
        {"EORB immediate", 0xC8, MicroTargetRegister::B, 0xFF, 0x42, 0x0F, microlind::CC_C, 0xF0, false, true, false, false, false},
        {"ADCB immediate", 0xC9, MicroTargetRegister::B, 0x7F, 0x42, 0x00, microlind::CC_C, 0x80, false, true, true, false, true},
        {"ORB immediate", 0xCA, MicroTargetRegister::B, 0x10, 0x42, 0x80, microlind::CC_C, 0x90, false, true, false, false, false},
        {"ADDB immediate", 0xCB, MicroTargetRegister::B, 0x0F, 0x42, 0x01, 0, 0x10, false, false, false, false, true},
    };

    for (const auto& test : cases) {
        SCOPED_TRACE(test.name);
        run_alu_immediate_microcycle_case(test);
    }
}

TEST(BusPhaseTest, TagsStackWriteCycles) {
    microlind::Simulator sim(microlind::CpuMode::HD6309, 1'000'000);
    auto memory = std::make_unique<PhaseProbeMemory>();
    auto* probe = memory.get();
    probe->data[0x0100] = 0x8D; // BSR
    probe->data[0x0101] = 0x00;
    ASSERT_FALSE(sim.map_device(0x0000, 0xFFFF, std::move(memory)));

    sim.cpu().set_pc(0x0100);
    sim.cpu().regs().s = 0x9000;
    sim.tick();

    const auto is_stack_write = [](const microlind::BusSignals& signals) {
        return signals.cycle_kind == microlind::BusCycleKind::StackWrite &&
            !signals.rw &&
            (signals.address == 0x8FFE || signals.address == 0x8FFF);
    };
    EXPECT_TRUE(std::any_of(probe->phases.begin(), probe->phases.end(), is_stack_write));
}

TEST(BusPhaseTest, TickClockEmitsExternalIdleCycles) {
    microlind::Simulator sim(microlind::CpuMode::HD6309, 1'000'000);
    auto memory = std::make_unique<PhaseProbeMemory>();
    auto* probe = memory.get();
    ASSERT_FALSE(sim.map_device(0x0000, 0xFFFF, std::move(memory)));

    sim.cpu().set_pc(0x1234);
    sim.tick_clock(2);

    ASSERT_EQ(probe->phases.size(), 8u);
    for (const auto& phase : probe->phases) {
        EXPECT_FALSE(phase.memory_enable);
        EXPECT_EQ(phase.address, 0x1234);
        EXPECT_EQ(phase.cycle_kind, microlind::BusCycleKind::Idle);
    }
}

TEST(BusPhaseTest, ResetVectorFetchesAreVectorReads) {
    microlind::Simulator sim(microlind::CpuMode::HD6309, 1'000'000);
    auto memory = std::make_unique<PhaseProbeMemory>();
    auto* probe = memory.get();
    probe->data[0xFFFE] = 0x12;
    probe->data[0xFFFF] = 0x34;
    ASSERT_FALSE(sim.map_device(0x0000, 0xFFFF, std::move(memory)));

    sim.reset_from_vector();

    EXPECT_EQ(sim.cpu().regs().pc, 0x1234);
    ASSERT_EQ(probe->phases.size(), 8u);
    for (std::size_t i = 0; i < 4; ++i) {
        EXPECT_EQ(probe->phases[i].address, 0xFFFE);
        EXPECT_EQ(probe->phases[i].cycle_kind, microlind::BusCycleKind::VectorRead);
    }
    for (std::size_t i = 4; i < 8; ++i) {
        EXPECT_EQ(probe->phases[i].address, 0xFFFF);
        EXPECT_EQ(probe->phases[i].cycle_kind, microlind::BusCycleKind::VectorRead);
    }
}

TEST(BusPhaseTest, TagsUStackCycles) {
    microlind::Simulator sim(microlind::CpuMode::HD6309, 1'000'000);
    auto memory = std::make_unique<PhaseProbeMemory>();
    auto* probe = memory.get();
    probe->data[0x0100] = 0x36; // PSHU A
    probe->data[0x0101] = 0x02;
    probe->data[0x0102] = 0x37; // PULU A
    probe->data[0x0103] = 0x02;
    ASSERT_FALSE(sim.map_device(0x0000, 0xFFFF, std::move(memory)));

    sim.cpu().set_pc(0x0100);
    sim.cpu().regs().u = 0x9000;
    sim.cpu().regs().a = 0x5A;
    sim.tick();

    const auto is_u_stack_write = [](const microlind::BusSignals& signals) {
        return signals.cycle_kind == microlind::BusCycleKind::StackWrite &&
            !signals.rw &&
            signals.address == 0x8FFF &&
            signals.data == 0x5A;
    };
    EXPECT_TRUE(std::any_of(probe->phases.begin(), probe->phases.end(), is_u_stack_write));

    probe->phases.clear();
    sim.cpu().regs().a = 0x00;
    sim.tick();

    EXPECT_EQ(sim.cpu().regs().a, 0x5A);
    const auto is_u_stack_read = [](const microlind::BusSignals& signals) {
        return signals.cycle_kind == microlind::BusCycleKind::StackRead &&
            signals.rw &&
            signals.address == 0x8FFF &&
            signals.data == 0x5A;
    };
    EXPECT_TRUE(std::any_of(probe->phases.begin(), probe->phases.end(), is_u_stack_read));
}

TEST(BusDecodePolicyTest, ValidateModeKeepsRangeRoutingAndLogsMismatches) {
    microlind::Bus bus;
    auto memory = std::make_unique<PhaseProbeMemory>();
    memory->data[0x0000] = 0xA5;
    ASSERT_FALSE(bus.map_device(0x1000, 0x1000, microlind::BusDeviceSelect::Ram, std::move(memory)));

    bus.set_decoder(microlind::BusDecodeMode::Validate, [](const microlind::BusSignals&) {
        return microlind::BusDecodeResult{microlind::BusDeviceSelect::Rom, true, {}};
    });

    EXPECT_EQ(bus.read8(0x1000), 0xA5);
    ASSERT_THAT(bus.access_log(), testing::SizeIs(1));
    EXPECT_EQ(bus.access_log().front().decoded_select, microlind::BusDeviceSelect::Rom);
    EXPECT_EQ(bus.access_log().front().mapped_select, microlind::BusDeviceSelect::Ram);
    ASSERT_THAT(bus.decode_log(), testing::SizeIs(1));
    EXPECT_THAT(bus.decode_log().front(), testing::HasSubstr("PLD validation selected ROM"));
}

TEST(BusDecodePolicyTest, RouteModeUsesDecoderSelectedDeviceRole) {
    microlind::Bus bus;
    auto memory = std::make_unique<PhaseProbeMemory>();
    memory->data[0x0000] = 0xA5;
    ASSERT_FALSE(bus.map_device(0x1000, 0x1000, microlind::BusDeviceSelect::Ram, std::move(memory)));

    bus.set_decoder(microlind::BusDecodeMode::Route, [](const microlind::BusSignals&) {
        return microlind::BusDecodeResult{microlind::BusDeviceSelect::Rom, true, {}};
    });

    EXPECT_EQ(bus.read8(0x1000), 0xFF);
    ASSERT_THAT(bus.access_log(), testing::SizeIs(1));
    EXPECT_EQ(bus.access_log().front().decoded_select, microlind::BusDeviceSelect::Rom);
    EXPECT_EQ(bus.access_log().front().mapped_select, microlind::BusDeviceSelect::None);
    ASSERT_THAT(bus.decode_log(), testing::SizeIs(1));
    EXPECT_THAT(bus.decode_log().front(), testing::HasSubstr("PLD route selected ROM"));

    bus.set_decoder(microlind::BusDecodeMode::Route, [](const microlind::BusSignals&) {
        return microlind::BusDecodeResult{microlind::BusDeviceSelect::Ram, true, {}};
    });

    EXPECT_EQ(bus.read8(0x1000), 0xA5);
    EXPECT_EQ(bus.access_log().back().decoded_select, microlind::BusDeviceSelect::Ram);
    EXPECT_EQ(bus.access_log().back().mapped_select, microlind::BusDeviceSelect::Ram);
}

} // namespace
