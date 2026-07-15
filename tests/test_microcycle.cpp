#include <cstddef>
#include <cstdint>
#include <algorithm>
#include <iterator>
#include <memory>
#include <optional>
#include <vector>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "microlind/bus.hpp"
#include "microlind/simulator.hpp"

namespace {

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

struct MicroLongBranchCase {
    const char* name;
    uint8_t md{};
    uint16_t offset{};
    uint16_t expected_pc{};
    uint8_t expected_cycles{};
};

struct MicroPrefixedLongBranchCase {
    const char* name;
    uint8_t opcode{};
    uint8_t cc{};
    uint16_t offset{};
    bool taken{};
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
    EXPECT_EQ(opcode_fetch.instruction_result.cycles, test.taken ? 3u : 2u);
    EXPECT_TRUE(sim.has_pending_microcycles());
    EXPECT_EQ(sim.cpu().regs().pc, 0x0101);
    ASSERT_THAT(sim.bus().access_log(), testing::SizeIs(1));
    EXPECT_EQ(sim.bus().access_log().back().address, 0x0100);
    EXPECT_EQ(sim.bus().access_log().back().cycle_kind, microlind::BusCycleKind::OpcodeFetch);

    const auto offset_read = sim.tick_microcycle();
    EXPECT_TRUE(offset_read.emitted);
    EXPECT_FALSE(offset_read.instruction_started);
    EXPECT_EQ(offset_read.instruction_complete, !test.taken);
    EXPECT_EQ(offset_read.instruction_result.cycles, test.taken ? 3u : 2u);
    EXPECT_EQ(sim.cpu().regs().pc, test.expected_pc);
    EXPECT_EQ(probe->read_count, 2u);
    ASSERT_THAT(sim.bus().access_log(), testing::SizeIs(2));
    EXPECT_EQ(sim.bus().access_log().back().address, 0x0101);
    EXPECT_EQ(sim.bus().access_log().back().value, test.offset);
    EXPECT_EQ(sim.bus().access_log().back().cycle_kind, microlind::BusCycleKind::OperandRead);

    expect_completed_or_drain_internal_cycle(sim, !test.taken);
}

void run_long_branch_microcycle_case(const MicroLongBranchCase& test) {
    microlind::Simulator sim(microlind::CpuMode::HD6309, 1'000'000);
    auto memory = std::make_unique<SideEffectMemory>();
    auto* probe = memory.get();
    probe->data[0x0100] = 0x16;
    probe->data[0x0101] = high_byte(test.offset);
    probe->data[0x0102] = low_byte(test.offset);
    ASSERT_FALSE(sim.map_device(0x0000, 0xFFFF, std::move(memory)));

    sim.cpu().set_pc(0x0100);
    sim.cpu().regs().md = test.md;

    const auto opcode_fetch = sim.tick_microcycle();
    EXPECT_TRUE(opcode_fetch.emitted);
    EXPECT_TRUE(opcode_fetch.instruction_started);
    EXPECT_FALSE(opcode_fetch.instruction_complete);
    EXPECT_EQ(opcode_fetch.instruction_result.cycles, test.expected_cycles);
    EXPECT_EQ(sim.cpu().regs().pc, 0x0101);

    const auto high_read = sim.tick_microcycle();
    EXPECT_TRUE(high_read.emitted);
    EXPECT_FALSE(high_read.instruction_started);
    EXPECT_FALSE(high_read.instruction_complete);
    EXPECT_EQ(high_read.instruction_result.cycles, test.expected_cycles);
    EXPECT_EQ(sim.cpu().regs().pc, 0x0102);

    const auto low_read = sim.tick_microcycle();
    EXPECT_TRUE(low_read.emitted);
    EXPECT_FALSE(low_read.instruction_started);
    EXPECT_EQ(low_read.instruction_complete, test.expected_cycles == 3);
    EXPECT_EQ(low_read.instruction_result.cycles, test.expected_cycles);
    EXPECT_EQ(sim.cpu().regs().pc, test.expected_pc);
    EXPECT_EQ(probe->read_count, 3u);

    const uint8_t internal_cycles = static_cast<uint8_t>(test.expected_cycles - 3);
    if (internal_cycles > 0) {
        expect_internal_cycles(sim, internal_cycles);
    } else {
        EXPECT_FALSE(sim.has_pending_microcycles());
    }

    ASSERT_THAT(sim.bus().access_log(), testing::SizeIs(3));
    EXPECT_EQ(sim.bus().access_log()[0].address, 0x0100);
    EXPECT_EQ(sim.bus().access_log()[0].cycle_kind, microlind::BusCycleKind::OpcodeFetch);
    EXPECT_EQ(sim.bus().access_log()[1].address, 0x0101);
    EXPECT_EQ(sim.bus().access_log()[1].value, high_byte(test.offset));
    EXPECT_EQ(sim.bus().access_log()[1].cycle_kind, microlind::BusCycleKind::OperandRead);
    EXPECT_EQ(sim.bus().access_log()[2].address, 0x0102);
    EXPECT_EQ(sim.bus().access_log()[2].value, low_byte(test.offset));
    EXPECT_EQ(sim.bus().access_log()[2].cycle_kind, microlind::BusCycleKind::OperandRead);
}

void run_prefixed_long_branch_microcycle_case(const MicroPrefixedLongBranchCase& test) {
    microlind::Simulator sim(microlind::CpuMode::HD6309, 1'000'000);
    auto memory = std::make_unique<SideEffectMemory>();
    auto* probe = memory.get();
    probe->data[0x0100] = 0x10;
    probe->data[0x0101] = test.opcode;
    probe->data[0x0102] = high_byte(test.offset);
    probe->data[0x0103] = low_byte(test.offset);
    ASSERT_FALSE(sim.map_device(0x0000, 0xFFFF, std::move(memory)));

    sim.cpu().set_pc(0x0100);
    sim.cpu().regs().cc = test.cc;

    for (std::size_t i = 0; i < 4; ++i) {
        const auto result = sim.tick_microcycle();
        EXPECT_TRUE(result.emitted);
        EXPECT_EQ(result.instruction_started, i == 0);
        EXPECT_EQ(result.instruction_complete, i + 1 == test.expected_cycles);
        EXPECT_EQ(result.instruction_result.cycles, test.expected_cycles);
    }

    EXPECT_EQ(sim.cpu().regs().pc, test.expected_pc);
    EXPECT_EQ(probe->read_count, 4u);

    const uint8_t internal_cycles = static_cast<uint8_t>(test.expected_cycles - 4);
    if (internal_cycles > 0) {
        expect_internal_cycles(sim, internal_cycles);
    } else {
        EXPECT_FALSE(sim.has_pending_microcycles());
    }

    ASSERT_THAT(sim.bus().access_log(), testing::SizeIs(4));
    EXPECT_EQ(sim.bus().access_log()[0].address, 0x0100);
    EXPECT_EQ(sim.bus().access_log()[0].value, 0x10);
    EXPECT_EQ(sim.bus().access_log()[0].cycle_kind, microlind::BusCycleKind::OpcodeFetch);
    EXPECT_EQ(sim.bus().access_log()[1].address, 0x0101);
    EXPECT_EQ(sim.bus().access_log()[1].value, test.opcode);
    EXPECT_EQ(sim.bus().access_log()[1].cycle_kind, microlind::BusCycleKind::OpcodeFetch);
    EXPECT_EQ(sim.bus().access_log()[2].address, 0x0102);
    EXPECT_EQ(sim.bus().access_log()[2].value, high_byte(test.offset));
    EXPECT_EQ(sim.bus().access_log()[2].cycle_kind, microlind::BusCycleKind::OperandRead);
    EXPECT_EQ(sim.bus().access_log()[3].address, 0x0103);
    EXPECT_EQ(sim.bus().access_log()[3].value, low_byte(test.offset));
    EXPECT_EQ(sim.bus().access_log()[3].cycle_kind, microlind::BusCycleKind::OperandRead);
    EXPECT_EQ(test.taken, test.expected_cycles == 6);
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
        {"BRN never", 0x21, 0x00, 0x05, false, 0x0102},
        {"BHI taken", 0x22, 0x00, 0x03, true, 0x0105},
        {"BLS taken", 0x23, microlind::CC_C, 0x03, true, 0x0105},
        {"BCC taken", 0x24, 0x00, 0x03, true, 0x0105},
        {"BCS taken", 0x25, microlind::CC_C, 0x03, true, 0x0105},
        {"BNE taken", 0x26, 0x00, 0x03, true, 0x0105},
        {"BNE not taken", 0x26, microlind::CC_Z, 0x03, false, 0x0102},
        {"BEQ taken", 0x27, microlind::CC_Z, 0xFC, true, 0x00FE},
        {"BEQ not taken", 0x27, 0x00, 0xFC, false, 0x0102},
        {"BVC taken", 0x28, 0x00, 0x03, true, 0x0105},
        {"BVS taken", 0x29, microlind::CC_V, 0x03, true, 0x0105},
        {"BPL taken", 0x2A, 0x00, 0x03, true, 0x0105},
        {"BMI taken", 0x2B, microlind::CC_N, 0x03, true, 0x0105},
        {"BGE taken", 0x2C, 0x00, 0x03, true, 0x0105},
        {"BLT taken", 0x2D, microlind::CC_N, 0x03, true, 0x0105},
        {"BGT taken", 0x2E, 0x00, 0x03, true, 0x0105},
        {"BLE taken", 0x2F, microlind::CC_Z, 0x03, true, 0x0105},
    };

    for (const auto& test : cases) {
        SCOPED_TRACE(test.name);
        run_branch_microcycle_case(test);
    }
}

TEST(BusPhaseTest, ResumableMicrocycleLongBranchReadsSignedWordOffset) {
    const MicroLongBranchCase cases[] = {
        {"LBRA native forward", 0x01, 0x0003, 0x0106, 4},
        {"LBRA emulation backward", 0x00, 0xFFFD, 0x0100, 5},
        {"LBRA native wrap", 0x01, 0xFEEF, 0xFFF2, 4},
    };

    for (const auto& test : cases) {
        SCOPED_TRACE(test.name);
        run_long_branch_microcycle_case(test);
    }
}

TEST(BusPhaseTest, ResumableMicrocyclePrefixedLongBranchesReadSignedWordOffset) {
    const MicroPrefixedLongBranchCase cases[] = {
        {"LBRN never", 0x21, 0x00, 0x0003, false, 0x0104, 5},
        {"LBNE taken", 0x26, 0x00, 0x0003, true, 0x0107, 6},
        {"LBNE not taken", 0x26, microlind::CC_Z, 0x0003, false, 0x0104, 5},
        {"LBEQ taken backward", 0x27, microlind::CC_Z, 0xFFFC, true, 0x0100, 6},
        {"LBMI taken wrap", 0x2B, microlind::CC_N, 0xFEF0, true, 0xFFF4, 6},
        {"LBGT not taken", 0x2E, microlind::CC_Z, 0x0003, false, 0x0104, 5},
        {"LBLE taken", 0x2F, microlind::CC_Z, 0x0003, true, 0x0107, 6},
    };

    for (const auto& test : cases) {
        SCOPED_TRACE(test.name);
        run_prefixed_long_branch_microcycle_case(test);
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

TEST(BusPhaseTest, ResumableMicrocycleIndexedJmpAndJsrResolveAddressThenJump) {
    struct ExpectedAccess {
        microlind::BusAccessType type{};
        uint16_t address{};
        uint8_t value{};
        microlind::BusCycleKind cycle_kind{};
    };

    struct Case {
        const char* name;
        std::vector<uint8_t> program;
        uint16_t x{};
        uint16_t y{};
        uint16_t u{};
        uint16_t s{};
        std::vector<std::pair<uint16_t, uint8_t>> memory;
        uint8_t expected_cycles{};
        uint16_t expected_pc{};
        uint16_t expected_x{};
        uint16_t expected_y{};
        uint16_t expected_u{};
        uint16_t expected_s{};
        std::vector<ExpectedAccess> expected_accesses;
    };

    const Case cases[] = {
        {
            "JMP ,X",
            {0x6E, 0x84},
            0x2000,
            0x2100,
            0x4000,
            0x9000,
            {},
            3,
            0x2000,
            0x2000,
            0x2100,
            0x4000,
            0x9000,
            {
                {microlind::BusAccessType::Read, 0x0100, 0x6E, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0101, 0x84, microlind::BusCycleKind::OperandRead},
            },
        },
        {
            "JMP [8-bit offset,Y]",
            {0x6E, 0xB8, 0x02},
            0x2000,
            0x2100,
            0x4000,
            0x9000,
            {{0x2102, 0x30}, {0x2103, 0x00}},
            7,
            0x3000,
            0x2000,
            0x2100,
            0x4000,
            0x9000,
            {
                {microlind::BusAccessType::Read, 0x0100, 0x6E, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0101, 0xB8, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x0102, 0x02, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x2102, 0x30, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x2103, 0x00, microlind::BusCycleKind::OperandRead},
            },
        },
        {
            "JSR ,X",
            {0xAD, 0x84},
            0x2000,
            0x2100,
            0x4000,
            0x9000,
            {},
            7,
            0x2000,
            0x2000,
            0x2100,
            0x4000,
            0x8FFE,
            {
                {microlind::BusAccessType::Read, 0x0100, 0xAD, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0101, 0x84, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Write, 0x8FFF, 0x02, microlind::BusCycleKind::StackWrite},
                {microlind::BusAccessType::Write, 0x8FFE, 0x01, microlind::BusCycleKind::StackWrite},
            },
        },
        {
            "JSR [16-bit offset,U]",
            {0xAD, 0xD9, 0x00, 0x10},
            0x2000,
            0x2100,
            0x4000,
            0x9000,
            {{0x4010, 0x30}, {0x4011, 0x00}},
            14,
            0x3000,
            0x2000,
            0x2100,
            0x4000,
            0x8FFE,
            {
                {microlind::BusAccessType::Read, 0x0100, 0xAD, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0101, 0xD9, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x0102, 0x00, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x0103, 0x10, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x4010, 0x30, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x4011, 0x00, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Write, 0x8FFF, 0x04, microlind::BusCycleKind::StackWrite},
                {microlind::BusAccessType::Write, 0x8FFE, 0x01, microlind::BusCycleKind::StackWrite},
            },
        },
    };

    for (const auto& test : cases) {
        SCOPED_TRACE(test.name);
        microlind::Simulator sim(microlind::CpuMode::HD6309, 1'000'000);
        auto memory = std::make_unique<SideEffectMemory>();
        auto* probe = memory.get();
        for (std::size_t i = 0; i < test.program.size(); ++i) {
            probe->data[static_cast<uint16_t>(0x0100 + i)] = test.program[i];
        }
        for (const auto& [address, value] : test.memory) {
            probe->data[address] = value;
        }
        ASSERT_FALSE(sim.map_device(0x0000, 0xFFFF, std::move(memory)));

        sim.cpu().set_pc(0x0100);
        sim.cpu().regs().x = test.x;
        sim.cpu().regs().y = test.y;
        sim.cpu().regs().u = test.u;
        sim.cpu().regs().s = test.s;

        for (std::size_t i = 0; i < test.expected_accesses.size(); ++i) {
            const auto result = sim.tick_microcycle();
            EXPECT_TRUE(result.emitted);
            EXPECT_EQ(result.instruction_started, i == 0);
            EXPECT_EQ(result.instruction_complete, i + 1 == test.expected_cycles);
            EXPECT_EQ(result.instruction_result.cycles, test.expected_cycles);
        }

        const uint8_t internal_cycles = static_cast<uint8_t>(test.expected_cycles - test.expected_accesses.size());
        if (internal_cycles > 0) {
            expect_internal_cycles(sim, internal_cycles);
        } else {
            EXPECT_FALSE(sim.has_pending_microcycles());
        }

        EXPECT_EQ(sim.cpu().regs().pc, test.expected_pc);
        EXPECT_EQ(sim.cpu().regs().x, test.expected_x);
        EXPECT_EQ(sim.cpu().regs().y, test.expected_y);
        EXPECT_EQ(sim.cpu().regs().u, test.expected_u);
        EXPECT_EQ(sim.cpu().regs().s, test.expected_s);
        ASSERT_THAT(sim.bus().access_log(), testing::SizeIs(test.expected_accesses.size()));
        for (std::size_t i = 0; i < test.expected_accesses.size(); ++i) {
            SCOPED_TRACE(i);
            EXPECT_EQ(sim.bus().access_log()[i].type, test.expected_accesses[i].type);
            EXPECT_EQ(sim.bus().access_log()[i].address, test.expected_accesses[i].address);
            EXPECT_EQ(sim.bus().access_log()[i].value, test.expected_accesses[i].value);
            EXPECT_EQ(sim.bus().access_log()[i].cycle_kind, test.expected_accesses[i].cycle_kind);
        }
        if (test.program[0] == 0xAD) {
            const uint16_t return_pc = static_cast<uint16_t>(0x0100 + test.program.size());
            EXPECT_EQ(probe->data[0x8FFF], low_byte(return_pc));
            EXPECT_EQ(probe->data[0x8FFE], high_byte(return_pc));
        }
    }
}

TEST(BusPhaseTest, ResumableMicrocycleIndexedLeaWritesEffectiveAddress) {
    struct ExpectedAccess {
        microlind::BusAccessType type{};
        uint16_t address{};
        uint8_t value{};
        microlind::BusCycleKind cycle_kind{};
    };

    struct Case {
        const char* name;
        std::vector<uint8_t> program;
        uint16_t x{};
        uint16_t y{};
        uint16_t u{};
        uint16_t s{};
        uint8_t cc{};
        std::vector<std::pair<uint16_t, uint8_t>> memory;
        uint8_t expected_cycles{};
        uint16_t expected_pc{};
        uint16_t expected_x{};
        uint16_t expected_y{};
        uint16_t expected_u{};
        uint16_t expected_s{};
        bool expect_z{};
        bool expect_n{};
        std::vector<ExpectedAccess> expected_accesses;
    };

    const Case cases[] = {
        {
            "LEAX 5-bit offset,X",
            {0x30, 0x05},
            0x2000,
            0x2100,
            0x4000,
            0x9000,
            0x00,
            {},
            5,
            0x0102,
            0x2005,
            0x2100,
            0x4000,
            0x9000,
            false,
            false,
            {
                {microlind::BusAccessType::Read, 0x0100, 0x30, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0101, 0x05, microlind::BusCycleKind::OperandRead},
            },
        },
        {
            "LEAY 8-bit offset,X",
            {0x31, 0x88, 0xFE},
            0x2000,
            0x2100,
            0x4000,
            0x9000,
            0x00,
            {},
            5,
            0x0103,
            0x2000,
            0x1FFE,
            0x4000,
            0x9000,
            false,
            false,
            {
                {microlind::BusAccessType::Read, 0x0100, 0x31, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0101, 0x88, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x0102, 0xFE, microlind::BusCycleKind::OperandRead},
            },
        },
        {
            "LEAS ,--S leaves flags unchanged",
            {0x32, 0xE3},
            0x2000,
            0x2100,
            0x4000,
            0x9000,
            microlind::CC_N,
            {},
            7,
            0x0102,
            0x2000,
            0x2100,
            0x4000,
            0x8FFE,
            false,
            true,
            {
                {microlind::BusAccessType::Read, 0x0100, 0x32, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0101, 0xE3, microlind::BusCycleKind::OperandRead},
            },
        },
        {
            "LEAU [,X]",
            {0x33, 0x94},
            0x2000,
            0x2100,
            0x4000,
            0x9000,
            0x00,
            {{0x2000, 0x00}, {0x2001, 0x00}},
            7,
            0x0102,
            0x2000,
            0x2100,
            0x0000,
            0x9000,
            true,
            false,
            {
                {microlind::BusAccessType::Read, 0x0100, 0x33, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0101, 0x94, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x2000, 0x00, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x2001, 0x00, microlind::BusCycleKind::OperandRead},
            },
        },
        {
            "LEAX [16-bit offset,Y]",
            {0x30, 0xB9, 0x00, 0x10},
            0x2000,
            0x2100,
            0x4000,
            0x9000,
            0x00,
            {{0x2110, 0x80}, {0x2111, 0x00}},
            11,
            0x0104,
            0x8000,
            0x2100,
            0x4000,
            0x9000,
            false,
            true,
            {
                {microlind::BusAccessType::Read, 0x0100, 0x30, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0101, 0xB9, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x0102, 0x00, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x0103, 0x10, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x2110, 0x80, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x2111, 0x00, microlind::BusCycleKind::OperandRead},
            },
        },
    };

    for (const auto& test : cases) {
        SCOPED_TRACE(test.name);
        microlind::Simulator sim(microlind::CpuMode::HD6309, 1'000'000);
        auto memory = std::make_unique<SideEffectMemory>();
        auto* probe = memory.get();
        for (std::size_t i = 0; i < test.program.size(); ++i) {
            probe->data[static_cast<uint16_t>(0x0100 + i)] = test.program[i];
        }
        for (const auto& [address, value] : test.memory) {
            probe->data[address] = value;
        }
        ASSERT_FALSE(sim.map_device(0x0000, 0xFFFF, std::move(memory)));

        sim.cpu().set_pc(0x0100);
        sim.cpu().regs().x = test.x;
        sim.cpu().regs().y = test.y;
        sim.cpu().regs().u = test.u;
        sim.cpu().regs().s = test.s;
        sim.cpu().regs().cc = test.cc;

        for (std::size_t i = 0; i < test.expected_accesses.size(); ++i) {
            const auto result = sim.tick_microcycle();
            EXPECT_TRUE(result.emitted);
            EXPECT_EQ(result.instruction_started, i == 0);
            EXPECT_EQ(result.instruction_complete, i + 1 == test.expected_cycles);
            EXPECT_EQ(result.instruction_result.cycles, test.expected_cycles);
        }

        const uint8_t internal_cycles = static_cast<uint8_t>(test.expected_cycles - test.expected_accesses.size());
        if (internal_cycles > 0) {
            expect_internal_cycles(sim, internal_cycles);
        } else {
            EXPECT_FALSE(sim.has_pending_microcycles());
        }

        EXPECT_EQ(sim.cpu().regs().pc, test.expected_pc);
        EXPECT_EQ(sim.cpu().regs().x, test.expected_x);
        EXPECT_EQ(sim.cpu().regs().y, test.expected_y);
        EXPECT_EQ(sim.cpu().regs().u, test.expected_u);
        EXPECT_EQ(sim.cpu().regs().s, test.expected_s);
        EXPECT_EQ((sim.cpu().regs().cc & microlind::CC_Z) != 0, test.expect_z);
        EXPECT_EQ((sim.cpu().regs().cc & microlind::CC_N) != 0, test.expect_n);
        ASSERT_THAT(sim.bus().access_log(), testing::SizeIs(test.expected_accesses.size()));
        for (std::size_t i = 0; i < test.expected_accesses.size(); ++i) {
            SCOPED_TRACE(i);
            EXPECT_EQ(sim.bus().access_log()[i].type, test.expected_accesses[i].type);
            EXPECT_EQ(sim.bus().access_log()[i].address, test.expected_accesses[i].address);
            EXPECT_EQ(sim.bus().access_log()[i].value, test.expected_accesses[i].value);
            EXPECT_EQ(sim.bus().access_log()[i].cycle_kind, test.expected_accesses[i].cycle_kind);
        }
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

TEST(BusPhaseTest, ResumableMicrocyclePrefixedRegisterUnaryUpdatesAfterOpcodeFetch) {
    struct Case {
        const char* name;
        std::vector<uint8_t> program;
        uint8_t initial_a{};
        uint8_t initial_b{};
        uint8_t initial_e{};
        uint8_t initial_f{};
        uint8_t initial_cc{};
        uint8_t expected_a{};
        uint8_t expected_b{};
        uint8_t expected_e{};
        uint8_t expected_f{};
        bool expect_z{};
        bool expect_n{};
        bool expect_v{};
        bool expect_c{};
    };

    const Case cases[] = {
        {"NEGD", {0x10, 0x40}, 0x80, 0x00, 0x12, 0x34, 0x00, 0x80, 0x00, 0x12, 0x34, false, true, true, true},
        {"ROLD", {0x10, 0x49}, 0x80, 0x00, 0x12, 0x34, microlind::CC_C, 0x00, 0x01, 0x12, 0x34, false, false, true, true},
        {"CLRD", {0x10, 0x4F}, 0x12, 0x34, 0x56, 0x78, microlind::CC_N | microlind::CC_C, 0x00, 0x00, 0x56, 0x78, true, false, false, false},
        {"COMW", {0x10, 0x53}, 0x12, 0x34, 0x00, 0x00, 0x00, 0x12, 0x34, 0xFF, 0xFF, false, true, false, true},
        {"RORW", {0x10, 0x56}, 0x12, 0x34, 0x00, 0x01, microlind::CC_C, 0x12, 0x34, 0x80, 0x00, false, true, false, true},
        {"TSTW", {0x10, 0x5D}, 0x12, 0x34, 0x00, 0x00, microlind::CC_C | microlind::CC_V, 0x12, 0x34, 0x00, 0x00, true, false, false, false},
        {"COME", {0x11, 0x43}, 0x12, 0x34, 0x00, 0x56, 0x00, 0x12, 0x34, 0xFF, 0x56, false, true, false, true},
        {"DECF", {0x11, 0x5A}, 0x12, 0x34, 0x56, 0x80, microlind::CC_C, 0x12, 0x34, 0x56, 0x7F, false, false, true, true},
        {"INCE", {0x11, 0x4C}, 0x12, 0x34, 0x7F, 0x56, microlind::CC_C, 0x12, 0x34, 0x80, 0x56, false, true, true, true},
        {"CLRF", {0x11, 0x5F}, 0x12, 0x34, 0x56, 0x78, microlind::CC_N | microlind::CC_C, 0x12, 0x34, 0x56, 0x00, true, false, false, false},
    };

    for (const auto& test : cases) {
        SCOPED_TRACE(test.name);
        microlind::Simulator sim(microlind::CpuMode::HD6309, 1'000'000);
        auto memory = std::make_unique<SideEffectMemory>();
        auto* probe = memory.get();
        probe->data[0x0100] = test.program[0];
        probe->data[0x0101] = test.program[1];
        ASSERT_FALSE(sim.map_device(0x0000, 0xFFFF, std::move(memory)));

        sim.cpu().set_pc(0x0100);
        sim.cpu().regs().a = test.initial_a;
        sim.cpu().regs().b = test.initial_b;
        sim.cpu().regs().e = test.initial_e;
        sim.cpu().regs().f = test.initial_f;
        sim.cpu().regs().cc = test.initial_cc;

        for (std::size_t i = 0; i < test.program.size(); ++i) {
            const auto result = sim.tick_microcycle();
            EXPECT_TRUE(result.emitted);
            EXPECT_EQ(result.instruction_started, i == 0);
            EXPECT_FALSE(result.instruction_complete);
            EXPECT_EQ(result.instruction_result.cycles, 3);
        }

        expect_internal_cycles(sim, 1);

        EXPECT_EQ(sim.cpu().regs().pc, 0x0102);
        EXPECT_EQ(sim.cpu().regs().a, test.expected_a);
        EXPECT_EQ(sim.cpu().regs().b, test.expected_b);
        EXPECT_EQ(sim.cpu().regs().e, test.expected_e);
        EXPECT_EQ(sim.cpu().regs().f, test.expected_f);
        EXPECT_EQ((sim.cpu().regs().cc & microlind::CC_Z) != 0, test.expect_z);
        EXPECT_EQ((sim.cpu().regs().cc & microlind::CC_N) != 0, test.expect_n);
        EXPECT_EQ((sim.cpu().regs().cc & microlind::CC_V) != 0, test.expect_v);
        EXPECT_EQ((sim.cpu().regs().cc & microlind::CC_C) != 0, test.expect_c);
        EXPECT_EQ(probe->write_count, 0u);
        ASSERT_THAT(sim.bus().access_log(), testing::SizeIs(2));
        EXPECT_EQ(sim.bus().access_log()[0].type, microlind::BusAccessType::Read);
        EXPECT_EQ(sim.bus().access_log()[0].address, 0x0100);
        EXPECT_EQ(sim.bus().access_log()[0].value, test.program[0]);
        EXPECT_EQ(sim.bus().access_log()[0].cycle_kind, microlind::BusCycleKind::OpcodeFetch);
        EXPECT_EQ(sim.bus().access_log()[1].type, microlind::BusAccessType::Read);
        EXPECT_EQ(sim.bus().access_log()[1].address, 0x0101);
        EXPECT_EQ(sim.bus().access_log()[1].value, test.program[1]);
        EXPECT_EQ(sim.bus().access_log()[1].cycle_kind, microlind::BusCycleKind::OpcodeFetch);
    }
}

TEST(BusPhaseTest, ResumableMicrocycleEFAluModes) {
    struct ExpectedAccess {
        microlind::BusAccessType type{};
        uint16_t address{};
        uint8_t value{};
        microlind::BusCycleKind cycle_kind{};
    };

    struct Case {
        const char* name;
        std::vector<uint8_t> program;
        uint8_t dp{};
        uint16_t x{};
        uint8_t initial_e{};
        uint8_t initial_f{};
        std::vector<std::pair<uint16_t, uint8_t>> memory;
        uint8_t expected_cycles{};
        uint16_t expected_pc{};
        uint16_t expected_x{};
        uint8_t expected_e{};
        uint8_t expected_f{};
        bool expect_z{};
        bool expect_n{};
        bool expect_v{};
        bool expect_c{};
        std::vector<ExpectedAccess> expected_accesses;
    };

    const Case cases[] = {
        {
            "ADDE immediate",
            {0x11, 0x8B, 0x01},
            0x12,
            0x2000,
            0x7F,
            0x40,
            {},
            3,
            0x0103,
            0x2000,
            0x80,
            0x40,
            false,
            true,
            true,
            false,
            {
                {microlind::BusAccessType::Read, 0x0100, 0x11, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0101, 0x8B, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0102, 0x01, microlind::BusCycleKind::OperandRead},
            },
        },
        {
            "SUBF direct",
            {0x11, 0xD0, 0x20},
            0x12,
            0x2000,
            0x40,
            0x00,
            {{0x1220, 0x01}},
            5,
            0x0103,
            0x2000,
            0x40,
            0xFF,
            false,
            true,
            false,
            true,
            {
                {microlind::BusAccessType::Read, 0x0100, 0x11, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0101, 0xD0, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0102, 0x20, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x1220, 0x01, microlind::BusCycleKind::OperandRead},
            },
        },
        {
            "CMPE extended",
            {0x11, 0xB1, 0x20, 0x00},
            0x12,
            0x2000,
            0x42,
            0x40,
            {{0x2000, 0x42}},
            6,
            0x0104,
            0x2000,
            0x42,
            0x40,
            true,
            false,
            false,
            false,
            {
                {microlind::BusAccessType::Read, 0x0100, 0x11, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0101, 0xB1, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0102, 0x20, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x0103, 0x00, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x2000, 0x42, microlind::BusCycleKind::OperandRead},
            },
        },
        {
            "ADDF ,X",
            {0x11, 0xEB, 0x84},
            0x12,
            0x2000,
            0x40,
            0x01,
            {{0x2000, 0x01}},
            5,
            0x0103,
            0x2000,
            0x40,
            0x02,
            false,
            false,
            false,
            false,
            {
                {microlind::BusAccessType::Read, 0x0100, 0x11, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0101, 0xEB, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0102, 0x84, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x2000, 0x01, microlind::BusCycleKind::OperandRead},
            },
        },
        {
            "CMPF [,X]",
            {0x11, 0xE1, 0x94},
            0x12,
            0x2000,
            0x40,
            0x10,
            {{0x2000, 0x30}, {0x2001, 0x00}, {0x3000, 0x20}},
            8,
            0x0103,
            0x2000,
            0x40,
            0x10,
            false,
            true,
            false,
            true,
            {
                {microlind::BusAccessType::Read, 0x0100, 0x11, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0101, 0xE1, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0102, 0x94, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x2000, 0x30, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x2001, 0x00, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x3000, 0x20, microlind::BusCycleKind::OperandRead},
            },
        },
    };

    for (const auto& test : cases) {
        SCOPED_TRACE(test.name);
        microlind::Simulator sim(microlind::CpuMode::HD6309, 1'000'000);
        auto memory = std::make_unique<SideEffectMemory>();
        auto* probe = memory.get();
        for (std::size_t i = 0; i < test.program.size(); ++i) {
            probe->data[static_cast<uint16_t>(0x0100 + i)] = test.program[i];
        }
        for (const auto& [address, value] : test.memory) {
            probe->data[address] = value;
        }
        ASSERT_FALSE(sim.map_device(0x0000, 0xFFFF, std::move(memory)));

        sim.cpu().set_pc(0x0100);
        sim.cpu().regs().dp = test.dp;
        sim.cpu().regs().x = test.x;
        sim.cpu().regs().e = test.initial_e;
        sim.cpu().regs().f = test.initial_f;

        for (std::size_t i = 0; i < test.expected_accesses.size(); ++i) {
            const auto result = sim.tick_microcycle();
            EXPECT_TRUE(result.emitted);
            EXPECT_EQ(result.instruction_started, i == 0);
            EXPECT_EQ(result.instruction_complete, i + 1 == test.expected_cycles);
            EXPECT_EQ(result.instruction_result.cycles, test.expected_cycles);
        }

        const uint8_t internal_cycles = static_cast<uint8_t>(test.expected_cycles - test.expected_accesses.size());
        if (internal_cycles > 0) {
            expect_internal_cycles(sim, internal_cycles);
        } else {
            EXPECT_FALSE(sim.has_pending_microcycles());
        }

        EXPECT_EQ(sim.cpu().regs().pc, test.expected_pc);
        EXPECT_EQ(sim.cpu().regs().x, test.expected_x);
        EXPECT_EQ(sim.cpu().regs().e, test.expected_e);
        EXPECT_EQ(sim.cpu().regs().f, test.expected_f);
        EXPECT_EQ((sim.cpu().regs().cc & microlind::CC_Z) != 0, test.expect_z);
        EXPECT_EQ((sim.cpu().regs().cc & microlind::CC_N) != 0, test.expect_n);
        EXPECT_EQ((sim.cpu().regs().cc & microlind::CC_V) != 0, test.expect_v);
        EXPECT_EQ((sim.cpu().regs().cc & microlind::CC_C) != 0, test.expect_c);
        ASSERT_THAT(sim.bus().access_log(), testing::SizeIs(test.expected_accesses.size()));
        for (std::size_t i = 0; i < test.expected_accesses.size(); ++i) {
            SCOPED_TRACE(i);
            EXPECT_EQ(sim.bus().access_log()[i].type, test.expected_accesses[i].type);
            EXPECT_EQ(sim.bus().access_log()[i].address, test.expected_accesses[i].address);
            EXPECT_EQ(sim.bus().access_log()[i].value, test.expected_accesses[i].value);
            EXPECT_EQ(sim.bus().access_log()[i].cycle_kind, test.expected_accesses[i].cycle_kind);
        }
    }
}

TEST(BusPhaseTest, ResumableMicrocycleMiscInherentAndRegisterTransfers) {
    struct ExpectedAccess {
        microlind::BusAccessType type{};
        uint16_t address{};
        uint8_t value{};
        microlind::BusCycleKind cycle_kind{};
    };

    struct Case {
        const char* name;
        std::vector<uint8_t> program;
        uint8_t initial_a{};
        uint8_t initial_b{};
        uint8_t initial_cc{};
        uint16_t initial_x{};
        uint8_t expected_cycles{};
        uint8_t expected_a{};
        uint8_t expected_b{};
        uint8_t expected_cc{};
        uint16_t expected_x{};
        std::vector<ExpectedAccess> expected_accesses;
    };

    const Case cases[] = {
        {
            "MUL inherent",
            {0x3D},
            0x12,
            0x34,
            0x00,
            0x2000,
            11,
            0x03,
            0xA8,
            microlind::CC_C,
            0x2000,
            {{microlind::BusAccessType::Read, 0x0100, 0x3D, microlind::BusCycleKind::OpcodeFetch}},
        },
        {
            "ABX inherent",
            {0x3A},
            0x00,
            0x22,
            microlind::CC_C | microlind::CC_V,
            0x2000,
            3,
            0x00,
            0x22,
            microlind::CC_C | microlind::CC_V,
            0x2022,
            {{microlind::BusAccessType::Read, 0x0100, 0x3A, microlind::BusCycleKind::OpcodeFetch}},
        },
        {
            "SEX inherent",
            {0x1D},
            0x80,
            0x00,
            microlind::CC_C | microlind::CC_V,
            0x2000,
            2,
            0x80,
            0xFF,
            microlind::CC_N | microlind::CC_C | microlind::CC_V,
            0x2000,
            {{microlind::BusAccessType::Read, 0x0100, 0x1D, microlind::BusCycleKind::OpcodeFetch}},
        },
        {
            "DAA inherent",
            {0x19},
            0x0A,
            0x00,
            0x00,
            0x2000,
            2,
            0x10,
            0x00,
            0x00,
            0x2000,
            {{microlind::BusAccessType::Read, 0x0100, 0x19, microlind::BusCycleKind::OpcodeFetch}},
        },
        {
            "ORCC immediate",
            {0x1A, static_cast<uint8_t>(microlind::CC_I | microlind::CC_C)},
            0x12,
            0x34,
            0x00,
            0x2000,
            3,
            0x12,
            0x34,
            static_cast<uint8_t>(microlind::CC_I | microlind::CC_C),
            0x2000,
            {
                {microlind::BusAccessType::Read, 0x0100, 0x1A, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0101, static_cast<uint8_t>(microlind::CC_I | microlind::CC_C), microlind::BusCycleKind::OperandRead},
            },
        },
        {
            "ANDCC immediate",
            {0x1C, static_cast<uint8_t>(~microlind::CC_C)},
            0x12,
            0x34,
            static_cast<uint8_t>(microlind::CC_I | microlind::CC_C | microlind::CC_V),
            0x2000,
            3,
            0x12,
            0x34,
            static_cast<uint8_t>(microlind::CC_I | microlind::CC_V),
            0x2000,
            {
                {microlind::BusAccessType::Read, 0x0100, 0x1C, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0101, static_cast<uint8_t>(~microlind::CC_C), microlind::BusCycleKind::OperandRead},
            },
        },
        {
            "TFR A,B",
            {0x1F, 0x89},
            0x5A,
            0x00,
            microlind::CC_C,
            0x2000,
            6,
            0x5A,
            0x5A,
            microlind::CC_C,
            0x2000,
            {
                {microlind::BusAccessType::Read, 0x0100, 0x1F, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0101, 0x89, microlind::BusCycleKind::OperandRead},
            },
        },
        {
            "EXG D,X",
            {0x1E, 0x01},
            0x12,
            0x34,
            microlind::CC_C,
            0xABCD,
            8,
            0xAB,
            0xCD,
            microlind::CC_C,
            0x1234,
            {
                {microlind::BusAccessType::Read, 0x0100, 0x1E, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0101, 0x01, microlind::BusCycleKind::OperandRead},
            },
        },
    };

    for (const auto& test : cases) {
        SCOPED_TRACE(test.name);
        microlind::Simulator sim(microlind::CpuMode::HD6309, 1'000'000);
        auto memory = std::make_unique<SideEffectMemory>();
        auto* probe = memory.get();
        for (std::size_t i = 0; i < test.program.size(); ++i) {
            probe->data[static_cast<uint16_t>(0x0100 + i)] = test.program[i];
        }
        ASSERT_FALSE(sim.map_device(0x0000, 0xFFFF, std::move(memory)));

        sim.cpu().set_pc(0x0100);
        sim.cpu().regs().a = test.initial_a;
        sim.cpu().regs().b = test.initial_b;
        sim.cpu().regs().cc = test.initial_cc;
        sim.cpu().regs().x = test.initial_x;

        for (std::size_t i = 0; i < test.expected_accesses.size(); ++i) {
            const auto result = sim.tick_microcycle();
            EXPECT_TRUE(result.emitted);
            EXPECT_EQ(result.instruction_started, i == 0);
            EXPECT_EQ(result.instruction_complete, i + 1 == test.expected_cycles);
            EXPECT_EQ(result.instruction_result.cycles, test.expected_cycles);
        }

        const uint8_t internal_cycles = static_cast<uint8_t>(test.expected_cycles - test.expected_accesses.size());
        if (internal_cycles > 0) {
            expect_internal_cycles(sim, internal_cycles);
        } else {
            EXPECT_FALSE(sim.has_pending_microcycles());
        }

        EXPECT_EQ(sim.cpu().regs().pc, static_cast<uint16_t>(0x0100 + test.program.size()));
        EXPECT_EQ(sim.cpu().regs().a, test.expected_a);
        EXPECT_EQ(sim.cpu().regs().b, test.expected_b);
        EXPECT_EQ(sim.cpu().regs().cc, test.expected_cc);
        EXPECT_EQ(sim.cpu().regs().x, test.expected_x);
        EXPECT_EQ(probe->read_count, test.expected_accesses.size());
        EXPECT_EQ(probe->write_count, 0u);
        ASSERT_THAT(sim.bus().access_log(), testing::SizeIs(test.expected_accesses.size()));
        for (std::size_t i = 0; i < test.expected_accesses.size(); ++i) {
            SCOPED_TRACE(i);
            EXPECT_EQ(sim.bus().access_log()[i].type, test.expected_accesses[i].type);
            EXPECT_EQ(sim.bus().access_log()[i].address, test.expected_accesses[i].address);
            EXPECT_EQ(sim.bus().access_log()[i].value, test.expected_accesses[i].value);
            EXPECT_EQ(sim.bus().access_log()[i].cycle_kind, test.expected_accesses[i].cycle_kind);
        }
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

TEST(BusPhaseTest, ResumableMicrocycleDirectAluReadsDirectAddressThenMemory) {
    for (const bool native : {false, true}) {
        SCOPED_TRACE(native ? "native" : "emulation");
        microlind::Simulator sim(microlind::CpuMode::HD6309, 1'000'000);
        auto memory = std::make_unique<SideEffectMemory>();
        auto* probe = memory.get();
        probe->data[0x0100] = 0x9B; // ADDA direct
        probe->data[0x0101] = 0x34;
        probe->data[0x1234] = 0x01;
        ASSERT_FALSE(sim.map_device(0x0000, 0xFFFF, std::move(memory)));

        sim.cpu().set_pc(0x0100);
        sim.cpu().regs().dp = 0x12;
        sim.cpu().regs().md = native ? 0x01 : 0x00;
        seed_registers(sim, 0x0F, 0x42);

        const auto opcode_fetch = sim.tick_microcycle();
        EXPECT_TRUE(opcode_fetch.emitted);
        EXPECT_TRUE(opcode_fetch.instruction_started);
        EXPECT_FALSE(opcode_fetch.instruction_complete);
        EXPECT_EQ(opcode_fetch.instruction_result.cycles, native ? 3u : 4u);
        EXPECT_EQ(sim.cpu().regs().pc, 0x0101);
        EXPECT_EQ(sim.cpu().regs().a, 0x0F);

        const auto direct_operand = sim.tick_microcycle();
        EXPECT_TRUE(direct_operand.emitted);
        EXPECT_FALSE(direct_operand.instruction_started);
        EXPECT_FALSE(direct_operand.instruction_complete);
        EXPECT_EQ(sim.cpu().regs().pc, 0x0102);
        EXPECT_EQ(sim.cpu().regs().a, 0x0F);

        const auto memory_read = sim.tick_microcycle();
        EXPECT_TRUE(memory_read.emitted);
        EXPECT_FALSE(memory_read.instruction_started);
        EXPECT_EQ(memory_read.instruction_complete, native);
        EXPECT_EQ(sim.cpu().regs().pc, 0x0102);
        EXPECT_EQ(sim.cpu().regs().a, 0x10);
        EXPECT_NE(sim.cpu().regs().cc & microlind::CC_H, 0);

        if (!native) {
            expect_internal_cycles(sim, 1);
        } else {
            EXPECT_FALSE(sim.has_pending_microcycles());
        }

        EXPECT_EQ(probe->read_count, 3u);
        ASSERT_THAT(sim.bus().access_log(), testing::SizeIs(3));
        EXPECT_EQ(sim.bus().access_log()[0].address, 0x0100);
        EXPECT_EQ(sim.bus().access_log()[0].cycle_kind, microlind::BusCycleKind::OpcodeFetch);
        EXPECT_EQ(sim.bus().access_log()[1].address, 0x0101);
        EXPECT_EQ(sim.bus().access_log()[1].cycle_kind, microlind::BusCycleKind::OperandRead);
        EXPECT_EQ(sim.bus().access_log()[2].address, 0x1234);
        EXPECT_EQ(sim.bus().access_log()[2].cycle_kind, microlind::BusCycleKind::OperandRead);
    }
}

TEST(BusPhaseTest, ResumableMicrocycleExtendedAluReadsAddressThenMemory) {
    for (const bool native : {false, true}) {
        SCOPED_TRACE(native ? "native" : "emulation");
        microlind::Simulator sim(microlind::CpuMode::HD6309, 1'000'000);
        auto memory = std::make_unique<SideEffectMemory>();
        auto* probe = memory.get();
        probe->data[0x0100] = 0xBB; // ADDA extended
        probe->data[0x0101] = 0x12;
        probe->data[0x0102] = 0x34;
        probe->data[0x1234] = 0x01;
        ASSERT_FALSE(sim.map_device(0x0000, 0xFFFF, std::move(memory)));

        sim.cpu().set_pc(0x0100);
        sim.cpu().regs().md = native ? 0x01 : 0x00;
        seed_registers(sim, 0x0F, 0x42);

        const auto opcode_fetch = sim.tick_microcycle();
        EXPECT_TRUE(opcode_fetch.emitted);
        EXPECT_TRUE(opcode_fetch.instruction_started);
        EXPECT_FALSE(opcode_fetch.instruction_complete);
        EXPECT_EQ(opcode_fetch.instruction_result.cycles, native ? 4u : 5u);
        EXPECT_EQ(sim.cpu().regs().pc, 0x0101);
        EXPECT_EQ(sim.cpu().regs().a, 0x0F);

        const auto high_read = sim.tick_microcycle();
        EXPECT_TRUE(high_read.emitted);
        EXPECT_FALSE(high_read.instruction_started);
        EXPECT_FALSE(high_read.instruction_complete);
        EXPECT_EQ(sim.cpu().regs().pc, 0x0102);
        EXPECT_EQ(sim.cpu().regs().a, 0x0F);

        const auto low_read = sim.tick_microcycle();
        EXPECT_TRUE(low_read.emitted);
        EXPECT_FALSE(low_read.instruction_started);
        EXPECT_FALSE(low_read.instruction_complete);
        EXPECT_EQ(sim.cpu().regs().pc, 0x0103);
        EXPECT_EQ(sim.cpu().regs().a, 0x0F);

        const auto memory_read = sim.tick_microcycle();
        EXPECT_TRUE(memory_read.emitted);
        EXPECT_FALSE(memory_read.instruction_started);
        EXPECT_EQ(memory_read.instruction_complete, native);
        EXPECT_EQ(sim.cpu().regs().pc, 0x0103);
        EXPECT_EQ(sim.cpu().regs().a, 0x10);
        EXPECT_NE(sim.cpu().regs().cc & microlind::CC_H, 0);

        if (!native) {
            expect_internal_cycles(sim, 1);
        } else {
            EXPECT_FALSE(sim.has_pending_microcycles());
        }

        EXPECT_EQ(probe->read_count, 4u);
        ASSERT_THAT(sim.bus().access_log(), testing::SizeIs(4));
        EXPECT_EQ(sim.bus().access_log()[0].address, 0x0100);
        EXPECT_EQ(sim.bus().access_log()[0].cycle_kind, microlind::BusCycleKind::OpcodeFetch);
        EXPECT_EQ(sim.bus().access_log()[1].address, 0x0101);
        EXPECT_EQ(sim.bus().access_log()[1].cycle_kind, microlind::BusCycleKind::OperandRead);
        EXPECT_EQ(sim.bus().access_log()[2].address, 0x0102);
        EXPECT_EQ(sim.bus().access_log()[2].cycle_kind, microlind::BusCycleKind::OperandRead);
        EXPECT_EQ(sim.bus().access_log()[3].address, 0x1234);
        EXPECT_EQ(sim.bus().access_log()[3].cycle_kind, microlind::BusCycleKind::OperandRead);
    }
}

TEST(BusPhaseTest, ResumableMicrocycleDRegisterAddSubReadsWordOperands) {
    struct Case {
        const char* name;
        std::vector<uint8_t> program;
        std::optional<uint16_t> memory_address;
        uint16_t memory_operand{};
        uint16_t initial_d{};
        uint16_t expected_d{};
        uint8_t md{};
        uint8_t expected_cycles{};
        std::vector<uint16_t> read_addresses;
        bool expect_z{};
        bool expect_n{};
        bool expect_v{};
        bool expect_carry{};
    };

    const Case cases[] = {
        {
            "ADDD immediate native",
            {0xC3, 0x00, 0x01},
            std::nullopt,
            0x0000,
            0x0FFF,
            0x1000,
            0x01,
            3,
            {0x0100, 0x0101, 0x0102},
            false,
            false,
            false,
            false,
        },
        {
            "SUBD direct emulation",
            {0x93, 0x34},
            0x1234,
            0x0001,
            0x8000,
            0x7FFF,
            0x00,
            6,
            {0x0100, 0x0101, 0x1234, 0x1235},
            false,
            false,
            true,
            false,
        },
        {
            "ADDD extended native",
            {0xF3, 0x12, 0x34},
            0x1234,
            0x0001,
            0x7FFF,
            0x8000,
            0x01,
            5,
            {0x0100, 0x0101, 0x0102, 0x1234, 0x1235},
            false,
            true,
            true,
            false,
        },
        {
            "SUBD extended emulation",
            {0xB3, 0x12, 0x34},
            0x1234,
            0x1234,
            0x1234,
            0x0000,
            0x00,
            7,
            {0x0100, 0x0101, 0x0102, 0x1234, 0x1235},
            true,
            false,
            false,
            false,
        },
    };

    for (const auto& test : cases) {
        SCOPED_TRACE(test.name);
        microlind::Simulator sim(microlind::CpuMode::HD6309, 1'000'000);
        auto memory = std::make_unique<SideEffectMemory>();
        auto* probe = memory.get();
        for (std::size_t i = 0; i < test.program.size(); ++i) {
            probe->data[static_cast<uint16_t>(0x0100 + i)] = test.program[i];
        }
        if (test.memory_address.has_value()) {
            probe->data[*test.memory_address] = static_cast<uint8_t>((test.memory_operand >> 8) & 0xFF);
            probe->data[static_cast<uint16_t>(*test.memory_address + 1)] = static_cast<uint8_t>(test.memory_operand & 0xFF);
        }
        ASSERT_FALSE(sim.map_device(0x0000, 0xFFFF, std::move(memory)));

        sim.cpu().set_pc(0x0100);
        sim.cpu().regs().dp = 0x12;
        sim.cpu().regs().md = test.md;
        sim.cpu().regs().a = static_cast<uint8_t>((test.initial_d >> 8) & 0xFF);
        sim.cpu().regs().b = static_cast<uint8_t>(test.initial_d & 0xFF);

        for (std::size_t i = 0; i < test.read_addresses.size(); ++i) {
            const auto result = sim.tick_microcycle();
            EXPECT_TRUE(result.emitted);
            EXPECT_EQ(result.instruction_started, i == 0);
            EXPECT_EQ(result.instruction_complete, i + 1 == test.expected_cycles);
            EXPECT_EQ(result.instruction_result.cycles, test.expected_cycles);
        }

        const uint8_t internal_cycles = static_cast<uint8_t>(test.expected_cycles - test.read_addresses.size());
        if (internal_cycles > 0) {
            expect_internal_cycles(sim, internal_cycles);
        } else {
            EXPECT_FALSE(sim.has_pending_microcycles());
        }

        EXPECT_EQ(static_cast<uint16_t>((sim.cpu().regs().a << 8) | sim.cpu().regs().b), test.expected_d);
        EXPECT_EQ((sim.cpu().regs().cc & microlind::CC_Z) != 0, test.expect_z);
        EXPECT_EQ((sim.cpu().regs().cc & microlind::CC_N) != 0, test.expect_n);
        EXPECT_EQ((sim.cpu().regs().cc & microlind::CC_V) != 0, test.expect_v);
        EXPECT_EQ((sim.cpu().regs().cc & microlind::CC_C) != 0, test.expect_carry);
        EXPECT_EQ(probe->read_count, test.read_addresses.size());
        ASSERT_THAT(sim.bus().access_log(), testing::SizeIs(test.read_addresses.size()));
        for (std::size_t i = 0; i < test.read_addresses.size(); ++i) {
            EXPECT_EQ(sim.bus().access_log()[i].address, test.read_addresses[i]);
            EXPECT_EQ(
                sim.bus().access_log()[i].cycle_kind,
                i == 0 ? microlind::BusCycleKind::OpcodeFetch : microlind::BusCycleKind::OperandRead);
        }
    }
}

TEST(BusPhaseTest, ResumableMicrocyclePrefixedCmpdReadsWordOperands) {
    struct Case {
        const char* name;
        std::vector<uint8_t> program;
        std::optional<uint16_t> memory_address;
        uint16_t memory_operand{};
        uint16_t initial_d{};
        uint8_t md{};
        uint8_t expected_cycles{};
        std::vector<uint16_t> read_addresses;
        bool expect_z{};
        bool expect_n{};
        bool expect_v{};
        bool expect_carry{};
    };

    const Case cases[] = {
        {
            "CMPD immediate native",
            {0x10, 0x83, 0x12, 0x34},
            std::nullopt,
            0x0000,
            0x1234,
            0x01,
            4,
            {0x0100, 0x0101, 0x0102, 0x0103},
            true,
            false,
            false,
            false,
        },
        {
            "CMPD direct emulation",
            {0x10, 0x93, 0x34},
            0x1234,
            0x0001,
            0x8000,
            0x00,
            7,
            {0x0100, 0x0101, 0x0102, 0x1234, 0x1235},
            false,
            false,
            true,
            false,
        },
        {
            "CMPD extended native",
            {0x10, 0xB3, 0x12, 0x34},
            0x1234,
            0x0001,
            0x0000,
            0x01,
            6,
            {0x0100, 0x0101, 0x0102, 0x0103, 0x1234, 0x1235},
            false,
            true,
            false,
            true,
        },
    };

    for (const auto& test : cases) {
        SCOPED_TRACE(test.name);
        microlind::Simulator sim(microlind::CpuMode::HD6309, 1'000'000);
        auto memory = std::make_unique<SideEffectMemory>();
        auto* probe = memory.get();
        for (std::size_t i = 0; i < test.program.size(); ++i) {
            probe->data[static_cast<uint16_t>(0x0100 + i)] = test.program[i];
        }
        if (test.memory_address.has_value()) {
            probe->data[*test.memory_address] = static_cast<uint8_t>((test.memory_operand >> 8) & 0xFF);
            probe->data[static_cast<uint16_t>(*test.memory_address + 1)] = static_cast<uint8_t>(test.memory_operand & 0xFF);
        }
        ASSERT_FALSE(sim.map_device(0x0000, 0xFFFF, std::move(memory)));

        sim.cpu().set_pc(0x0100);
        sim.cpu().regs().dp = 0x12;
        sim.cpu().regs().md = test.md;
        sim.cpu().regs().a = static_cast<uint8_t>((test.initial_d >> 8) & 0xFF);
        sim.cpu().regs().b = static_cast<uint8_t>(test.initial_d & 0xFF);

        for (std::size_t i = 0; i < test.read_addresses.size(); ++i) {
            const auto result = sim.tick_microcycle();
            EXPECT_TRUE(result.emitted);
            EXPECT_EQ(result.instruction_started, i == 0);
            EXPECT_EQ(result.instruction_complete, i + 1 == test.expected_cycles);
            EXPECT_EQ(result.instruction_result.cycles, test.expected_cycles);
        }

        const uint8_t internal_cycles = static_cast<uint8_t>(test.expected_cycles - test.read_addresses.size());
        if (internal_cycles > 0) {
            expect_internal_cycles(sim, internal_cycles);
        } else {
            EXPECT_FALSE(sim.has_pending_microcycles());
        }

        EXPECT_EQ(static_cast<uint16_t>((sim.cpu().regs().a << 8) | sim.cpu().regs().b), test.initial_d);
        EXPECT_EQ((sim.cpu().regs().cc & microlind::CC_Z) != 0, test.expect_z);
        EXPECT_EQ((sim.cpu().regs().cc & microlind::CC_N) != 0, test.expect_n);
        EXPECT_EQ((sim.cpu().regs().cc & microlind::CC_V) != 0, test.expect_v);
        EXPECT_EQ((sim.cpu().regs().cc & microlind::CC_C) != 0, test.expect_carry);
        EXPECT_EQ(probe->read_count, test.read_addresses.size());
        ASSERT_THAT(sim.bus().access_log(), testing::SizeIs(test.read_addresses.size()));
        for (std::size_t i = 0; i < test.read_addresses.size(); ++i) {
            SCOPED_TRACE(i);
            EXPECT_EQ(sim.bus().access_log()[i].address, test.read_addresses[i]);
            EXPECT_EQ(
                sim.bus().access_log()[i].cycle_kind,
                i < 2 ? microlind::BusCycleKind::OpcodeFetch : microlind::BusCycleKind::OperandRead);
        }
    }
}

TEST(BusPhaseTest, ResumableMicrocycleIndexAndStackComparesReadWordOperands) {
    struct Case {
        const char* name;
        std::vector<uint8_t> program;
        std::optional<uint16_t> memory_address;
        uint16_t memory_operand{};
        char target{};
        uint16_t initial_value{};
        uint8_t md{};
        uint8_t expected_cycles{};
        std::vector<uint16_t> read_addresses;
        std::size_t opcode_fetch_count{};
        bool expect_z{};
        bool expect_n{};
        bool expect_v{};
        bool expect_carry{};
    };

    const Case cases[] = {
        {
            "CMPX immediate native",
            {0x8C, 0x12, 0x34},
            std::nullopt,
            0x0000,
            'x',
            0x1234,
            0x01,
            3,
            {0x0100, 0x0101, 0x0102},
            1,
            true,
            false,
            false,
            false,
        },
        {
            "CMPX direct emulation",
            {0x9C, 0x34},
            0x1234,
            0x0001,
            'x',
            0x8000,
            0x00,
            6,
            {0x0100, 0x0101, 0x1234, 0x1235},
            1,
            false,
            false,
            true,
            false,
        },
        {
            "CMPY extended native",
            {0x10, 0xBC, 0x12, 0x34},
            0x1234,
            0x0001,
            'y',
            0x0000,
            0x01,
            6,
            {0x0100, 0x0101, 0x0102, 0x0103, 0x1234, 0x1235},
            2,
            false,
            true,
            false,
            true,
        },
        {
            "CMPU direct native",
            {0x11, 0x93, 0x34},
            0x1234,
            0x0001,
            'u',
            0x0000,
            0x01,
            5,
            {0x0100, 0x0101, 0x0102, 0x1234, 0x1235},
            2,
            false,
            true,
            false,
            true,
        },
        {
            "CMPS immediate emulation",
            {0x11, 0x8C, 0x12, 0x34},
            std::nullopt,
            0x0000,
            's',
            0x1234,
            0x00,
            5,
            {0x0100, 0x0101, 0x0102, 0x0103},
            2,
            true,
            false,
            false,
            false,
        },
    };

    const auto set_target = [](microlind::Simulator& sim, char target, uint16_t value) {
        switch (target) {
        case 'x': sim.cpu().regs().x = value; break;
        case 'y': sim.cpu().regs().y = value; break;
        case 'u': sim.cpu().regs().u = value; break;
        case 's': sim.cpu().regs().s = value; break;
        }
    };
    const auto target_value = [](microlind::Simulator& sim, char target) {
        switch (target) {
        case 'x': return sim.cpu().regs().x;
        case 'y': return sim.cpu().regs().y;
        case 'u': return sim.cpu().regs().u;
        case 's': return sim.cpu().regs().s;
        }
        return uint16_t{};
    };

    for (const auto& test : cases) {
        SCOPED_TRACE(test.name);
        microlind::Simulator sim(microlind::CpuMode::HD6309, 1'000'000);
        auto memory = std::make_unique<SideEffectMemory>();
        auto* probe = memory.get();
        for (std::size_t i = 0; i < test.program.size(); ++i) {
            probe->data[static_cast<uint16_t>(0x0100 + i)] = test.program[i];
        }
        if (test.memory_address.has_value()) {
            probe->data[*test.memory_address] = static_cast<uint8_t>((test.memory_operand >> 8) & 0xFF);
            probe->data[static_cast<uint16_t>(*test.memory_address + 1)] = static_cast<uint8_t>(test.memory_operand & 0xFF);
        }
        ASSERT_FALSE(sim.map_device(0x0000, 0xFFFF, std::move(memory)));

        sim.cpu().set_pc(0x0100);
        sim.cpu().regs().dp = 0x12;
        sim.cpu().regs().md = test.md;
        set_target(sim, test.target, test.initial_value);

        for (std::size_t i = 0; i < test.read_addresses.size(); ++i) {
            const auto result = sim.tick_microcycle();
            EXPECT_TRUE(result.emitted);
            EXPECT_EQ(result.instruction_started, i == 0);
            EXPECT_EQ(result.instruction_complete, i + 1 == test.expected_cycles);
            EXPECT_EQ(result.instruction_result.cycles, test.expected_cycles);
        }

        const uint8_t internal_cycles = static_cast<uint8_t>(test.expected_cycles - test.read_addresses.size());
        if (internal_cycles > 0) {
            expect_internal_cycles(sim, internal_cycles);
        } else {
            EXPECT_FALSE(sim.has_pending_microcycles());
        }

        EXPECT_EQ(target_value(sim, test.target), test.initial_value);
        EXPECT_EQ((sim.cpu().regs().cc & microlind::CC_Z) != 0, test.expect_z);
        EXPECT_EQ((sim.cpu().regs().cc & microlind::CC_N) != 0, test.expect_n);
        EXPECT_EQ((sim.cpu().regs().cc & microlind::CC_V) != 0, test.expect_v);
        EXPECT_EQ((sim.cpu().regs().cc & microlind::CC_C) != 0, test.expect_carry);
        EXPECT_EQ(probe->read_count, test.read_addresses.size());
        ASSERT_THAT(sim.bus().access_log(), testing::SizeIs(test.read_addresses.size()));
        for (std::size_t i = 0; i < test.read_addresses.size(); ++i) {
            SCOPED_TRACE(i);
            EXPECT_EQ(sim.bus().access_log()[i].address, test.read_addresses[i]);
            EXPECT_EQ(
                sim.bus().access_log()[i].cycle_kind,
                i < test.opcode_fetch_count ? microlind::BusCycleKind::OpcodeFetch : microlind::BusCycleKind::OperandRead);
        }
    }
}

TEST(BusPhaseTest, ResumableMicrocycleIndexAndStackWordLoadsStoresUseWordAccesses) {
    struct ExpectedAccess {
        microlind::BusAccessType type{};
        uint16_t address{};
        uint8_t value{};
        microlind::BusCycleKind cycle_kind{};
    };

    struct Case {
        const char* name;
        std::vector<uint8_t> program;
        bool store{};
        std::optional<uint16_t> memory_address;
        uint16_t memory_value{};
        char target{};
        uint16_t initial_value{};
        uint16_t expected_value{};
        uint8_t md{};
        uint8_t expected_cycles{};
        std::vector<ExpectedAccess> expected_accesses;
        bool expect_z{};
        bool expect_n{};
    };

    const Case cases[] = {
        {
            "LDX direct native",
            {0x9E, 0x34},
            false,
            0x1234,
            0x8000,
            'x',
            0x1111,
            0x8000,
            0x01,
            4,
            {
                {microlind::BusAccessType::Read, 0x0100, 0x9E, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0101, 0x34, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x1234, 0x80, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x1235, 0x00, microlind::BusCycleKind::OperandRead},
            },
            false,
            true,
        },
        {
            "LDY immediate native",
            {0x10, 0x8E, 0x00, 0x00},
            false,
            std::nullopt,
            0x0000,
            'y',
            0x1111,
            0x0000,
            0x01,
            4,
            {
                {microlind::BusAccessType::Read, 0x0100, 0x10, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0101, 0x8E, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0102, 0x00, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x0103, 0x00, microlind::BusCycleKind::OperandRead},
            },
            true,
            false,
        },
        {
            "LDU extended emulation",
            {0xFE, 0x12, 0x34},
            false,
            0x1234,
            0x1234,
            'u',
            0x1111,
            0x1234,
            0x00,
            6,
            {
                {microlind::BusAccessType::Read, 0x0100, 0xFE, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0101, 0x12, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x0102, 0x34, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x1234, 0x12, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x1235, 0x34, microlind::BusCycleKind::OperandRead},
            },
            false,
            false,
        },
        {
            "STX direct native",
            {0x9F, 0x34},
            true,
            0x1234,
            0x0000,
            'x',
            0x1234,
            0x1234,
            0x01,
            4,
            {
                {microlind::BusAccessType::Read, 0x0100, 0x9F, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0101, 0x34, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Write, 0x1234, 0x12, microlind::BusCycleKind::OperandWrite},
                {microlind::BusAccessType::Write, 0x1235, 0x34, microlind::BusCycleKind::OperandWrite},
            },
            false,
            false,
        },
        {
            "STY extended native",
            {0x10, 0xBF, 0x12, 0x34},
            true,
            0x1234,
            0x0000,
            'y',
            0x8000,
            0x8000,
            0x01,
            6,
            {
                {microlind::BusAccessType::Read, 0x0100, 0x10, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0101, 0xBF, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0102, 0x12, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x0103, 0x34, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Write, 0x1234, 0x80, microlind::BusCycleKind::OperandWrite},
                {microlind::BusAccessType::Write, 0x1235, 0x00, microlind::BusCycleKind::OperandWrite},
            },
            false,
            true,
        },
        {
            "STS direct emulation",
            {0x10, 0xDF, 0x34},
            true,
            0x1234,
            0x0000,
            's',
            0x0000,
            0x0000,
            0x00,
            6,
            {
                {microlind::BusAccessType::Read, 0x0100, 0x10, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0101, 0xDF, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0102, 0x34, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Write, 0x1234, 0x00, microlind::BusCycleKind::OperandWrite},
                {microlind::BusAccessType::Write, 0x1235, 0x00, microlind::BusCycleKind::OperandWrite},
            },
            true,
            false,
        },
    };

    const auto set_target = [](microlind::Simulator& sim, char target, uint16_t value) {
        switch (target) {
        case 'x': sim.cpu().regs().x = value; break;
        case 'y': sim.cpu().regs().y = value; break;
        case 'u': sim.cpu().regs().u = value; break;
        case 's': sim.cpu().regs().s = value; break;
        }
    };
    const auto target_value = [](microlind::Simulator& sim, char target) {
        switch (target) {
        case 'x': return sim.cpu().regs().x;
        case 'y': return sim.cpu().regs().y;
        case 'u': return sim.cpu().regs().u;
        case 's': return sim.cpu().regs().s;
        }
        return uint16_t{};
    };

    for (const auto& test : cases) {
        SCOPED_TRACE(test.name);
        microlind::Simulator sim(microlind::CpuMode::HD6309, 1'000'000);
        auto memory = std::make_unique<SideEffectMemory>();
        auto* probe = memory.get();
        for (std::size_t i = 0; i < test.program.size(); ++i) {
            probe->data[static_cast<uint16_t>(0x0100 + i)] = test.program[i];
        }
        if (test.memory_address.has_value() && !test.store) {
            probe->data[*test.memory_address] = static_cast<uint8_t>((test.memory_value >> 8) & 0xFF);
            probe->data[static_cast<uint16_t>(*test.memory_address + 1)] = static_cast<uint8_t>(test.memory_value & 0xFF);
        }
        ASSERT_FALSE(sim.map_device(0x0000, 0xFFFF, std::move(memory)));

        sim.cpu().set_pc(0x0100);
        sim.cpu().regs().dp = 0x12;
        sim.cpu().regs().md = test.md;
        set_target(sim, test.target, test.initial_value);

        for (std::size_t i = 0; i < test.expected_accesses.size(); ++i) {
            const auto result = sim.tick_microcycle();
            EXPECT_TRUE(result.emitted);
            EXPECT_EQ(result.instruction_started, i == 0);
            EXPECT_EQ(result.instruction_complete, i + 1 == test.expected_cycles);
            EXPECT_EQ(result.instruction_result.cycles, test.expected_cycles);
        }

        const uint8_t internal_cycles = static_cast<uint8_t>(test.expected_cycles - test.expected_accesses.size());
        if (internal_cycles > 0) {
            expect_internal_cycles(sim, internal_cycles);
        } else {
            EXPECT_FALSE(sim.has_pending_microcycles());
        }

        EXPECT_EQ(target_value(sim, test.target), test.expected_value);
        EXPECT_EQ((sim.cpu().regs().cc & microlind::CC_Z) != 0, test.expect_z);
        EXPECT_EQ((sim.cpu().regs().cc & microlind::CC_N) != 0, test.expect_n);
        ASSERT_THAT(sim.bus().access_log(), testing::SizeIs(test.expected_accesses.size()));
        for (std::size_t i = 0; i < test.expected_accesses.size(); ++i) {
            SCOPED_TRACE(i);
            EXPECT_EQ(sim.bus().access_log()[i].type, test.expected_accesses[i].type);
            EXPECT_EQ(sim.bus().access_log()[i].address, test.expected_accesses[i].address);
            EXPECT_EQ(sim.bus().access_log()[i].value, test.expected_accesses[i].value);
            EXPECT_EQ(sim.bus().access_log()[i].cycle_kind, test.expected_accesses[i].cycle_kind);
        }
    }
}

TEST(BusPhaseTest, ResumableMicrocycleStackPushPullUsesMaskOrder) {
    struct ExpectedAccess {
        microlind::BusAccessType type{};
        uint16_t address{};
        uint8_t value{};
        microlind::BusCycleKind cycle_kind{};
    };

    struct Case {
        const char* name;
        std::vector<uint8_t> program;
        uint8_t md{};
        uint16_t s{};
        uint16_t u{};
        uint8_t a{};
        uint8_t b{};
        std::vector<std::pair<uint16_t, uint8_t>> memory;
        uint8_t expected_cycles{};
        uint16_t expected_pc{};
        uint16_t expected_s{};
        uint16_t expected_u{};
        uint8_t expected_a{};
        uint8_t expected_b{};
        std::vector<ExpectedAccess> expected_accesses;
    };

    const Case cases[] = {
        {
            "PSHS B,PC emulation",
            {0x34, 0x84},
            0x00,
            0x9000,
            0xA000,
            0x00,
            0x7A,
            {},
            8,
            0x0102,
            0x8FFD,
            0xA000,
            0x00,
            0x7A,
            {
                {microlind::BusAccessType::Read, 0x0100, 0x34, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0101, 0x84, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Write, 0x8FFF, 0x02, microlind::BusCycleKind::StackWrite},
                {microlind::BusAccessType::Write, 0x8FFE, 0x01, microlind::BusCycleKind::StackWrite},
                {microlind::BusAccessType::Write, 0x8FFD, 0x7A, microlind::BusCycleKind::StackWrite},
            },
        },
        {
            "PULS B,PC emulation",
            {0x35, 0x84},
            0x00,
            0x9000,
            0xA000,
            0x00,
            0x00,
            {{0x9000, 0x7A}, {0x9001, 0x12}, {0x9002, 0x34}},
            8,
            0x1234,
            0x9003,
            0xA000,
            0x00,
            0x7A,
            {
                {microlind::BusAccessType::Read, 0x0100, 0x35, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0101, 0x84, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x9000, 0x7A, microlind::BusCycleKind::StackRead},
                {microlind::BusAccessType::Read, 0x9001, 0x12, microlind::BusCycleKind::StackRead},
                {microlind::BusAccessType::Read, 0x9002, 0x34, microlind::BusCycleKind::StackRead},
            },
        },
        {
            "PSHU A native",
            {0x36, 0x02},
            0x01,
            0x9000,
            0xA000,
            0x5A,
            0x00,
            {},
            5,
            0x0102,
            0x9000,
            0x9FFF,
            0x5A,
            0x00,
            {
                {microlind::BusAccessType::Read, 0x0100, 0x36, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0101, 0x02, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Write, 0x9FFF, 0x5A, microlind::BusCycleKind::StackWrite},
            },
        },
        {
            "PULU A native",
            {0x37, 0x02},
            0x01,
            0x9000,
            0xA000,
            0x00,
            0x00,
            {{0xA000, 0x5A}},
            5,
            0x0102,
            0x9000,
            0xA001,
            0x5A,
            0x00,
            {
                {microlind::BusAccessType::Read, 0x0100, 0x37, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0101, 0x02, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0xA000, 0x5A, microlind::BusCycleKind::StackRead},
            },
        },
    };

    for (const auto& test : cases) {
        SCOPED_TRACE(test.name);
        microlind::Simulator sim(microlind::CpuMode::HD6309, 1'000'000);
        auto memory = std::make_unique<SideEffectMemory>();
        auto* probe = memory.get();
        for (std::size_t i = 0; i < test.program.size(); ++i) {
            probe->data[static_cast<uint16_t>(0x0100 + i)] = test.program[i];
        }
        for (const auto& [address, value] : test.memory) {
            probe->data[address] = value;
        }
        ASSERT_FALSE(sim.map_device(0x0000, 0xFFFF, std::move(memory)));

        sim.cpu().set_pc(0x0100);
        sim.cpu().regs().md = test.md;
        sim.cpu().regs().s = test.s;
        sim.cpu().regs().u = test.u;
        sim.cpu().regs().a = test.a;
        sim.cpu().regs().b = test.b;

        for (std::size_t i = 0; i < test.expected_accesses.size(); ++i) {
            const auto result = sim.tick_microcycle();
            EXPECT_TRUE(result.emitted);
            EXPECT_EQ(result.instruction_started, i == 0);
            EXPECT_EQ(result.instruction_complete, i + 1 == test.expected_cycles);
            EXPECT_EQ(result.instruction_result.cycles, test.expected_cycles);
        }

        const uint8_t internal_cycles = static_cast<uint8_t>(test.expected_cycles - test.expected_accesses.size());
        if (internal_cycles > 0) {
            expect_internal_cycles(sim, internal_cycles);
        } else {
            EXPECT_FALSE(sim.has_pending_microcycles());
        }

        EXPECT_EQ(sim.cpu().regs().pc, test.expected_pc);
        EXPECT_EQ(sim.cpu().regs().s, test.expected_s);
        EXPECT_EQ(sim.cpu().regs().u, test.expected_u);
        EXPECT_EQ(sim.cpu().regs().a, test.expected_a);
        EXPECT_EQ(sim.cpu().regs().b, test.expected_b);
        ASSERT_THAT(sim.bus().access_log(), testing::SizeIs(test.expected_accesses.size()));
        for (std::size_t i = 0; i < test.expected_accesses.size(); ++i) {
            SCOPED_TRACE(i);
            EXPECT_EQ(sim.bus().access_log()[i].type, test.expected_accesses[i].type);
            EXPECT_EQ(sim.bus().access_log()[i].address, test.expected_accesses[i].address);
            EXPECT_EQ(sim.bus().access_log()[i].value, test.expected_accesses[i].value);
            EXPECT_EQ(sim.bus().access_log()[i].cycle_kind, test.expected_accesses[i].cycle_kind);
        }
    }
}

TEST(BusPhaseTest, ResumableMicrocycleHD6309WStackShortcuts) {
    struct ExpectedAccess {
        microlind::BusAccessType type{};
        uint16_t address{};
        uint8_t value{};
        microlind::BusCycleKind cycle_kind{};
    };

    struct Case {
        const char* name{};
        std::vector<uint8_t> program;
        uint16_t s{};
        uint16_t u{};
        uint16_t initial_w{};
        std::vector<std::pair<uint16_t, uint8_t>> memory;
        uint16_t expected_s{};
        uint16_t expected_u{};
        uint16_t expected_w{};
        std::vector<ExpectedAccess> expected_accesses;
    };

    const Case cases[] = {
        {
            "PSHSW",
            {0x10, 0x38},
            0x9000,
            0xA000,
            0xBEEF,
            {},
            0x8FFE,
            0xA000,
            0xBEEF,
            {
                {microlind::BusAccessType::Read, 0x0100, 0x10, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0101, 0x38, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Write, 0x8FFF, 0xEF, microlind::BusCycleKind::StackWrite},
                {microlind::BusAccessType::Write, 0x8FFE, 0xBE, microlind::BusCycleKind::StackWrite},
            },
        },
        {
            "PULSW",
            {0x10, 0x39},
            0x9000,
            0xA000,
            0x0000,
            {{0x9000, 0xBE}, {0x9001, 0xEF}},
            0x9002,
            0xA000,
            0xBEEF,
            {
                {microlind::BusAccessType::Read, 0x0100, 0x10, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0101, 0x39, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x9000, 0xBE, microlind::BusCycleKind::StackRead},
                {microlind::BusAccessType::Read, 0x9001, 0xEF, microlind::BusCycleKind::StackRead},
            },
        },
        {
            "PSHUW",
            {0x10, 0x3A},
            0x9000,
            0xA000,
            0xBEEF,
            {},
            0x9000,
            0x9FFE,
            0xBEEF,
            {
                {microlind::BusAccessType::Read, 0x0100, 0x10, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0101, 0x3A, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Write, 0x9FFE, 0xBE, microlind::BusCycleKind::StackWrite},
                {microlind::BusAccessType::Write, 0x9FFF, 0xEF, microlind::BusCycleKind::StackWrite},
            },
        },
        {
            "PULUW",
            {0x10, 0x3B},
            0x9000,
            0xA000,
            0x0000,
            {{0xA000, 0xBE}, {0xA001, 0xEF}},
            0x9000,
            0xA002,
            0xBEEF,
            {
                {microlind::BusAccessType::Read, 0x0100, 0x10, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0101, 0x3B, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0xA000, 0xBE, microlind::BusCycleKind::StackRead},
                {microlind::BusAccessType::Read, 0xA001, 0xEF, microlind::BusCycleKind::StackRead},
            },
        },
    };

    for (const auto& test : cases) {
        SCOPED_TRACE(test.name);
        microlind::Simulator sim(microlind::CpuMode::HD6309, 1'000'000);
        auto memory = std::make_unique<SideEffectMemory>();
        auto* probe = memory.get();
        for (std::size_t i = 0; i < test.program.size(); ++i) {
            probe->data[static_cast<uint16_t>(0x0100 + i)] = test.program[i];
        }
        for (const auto& [address, value] : test.memory) {
            probe->data[address] = value;
        }
        ASSERT_FALSE(sim.map_device(0x0000, 0xFFFF, std::move(memory)));

        sim.cpu().set_pc(0x0100);
        sim.cpu().regs().s = test.s;
        sim.cpu().regs().u = test.u;
        sim.cpu().regs().e = high_byte(test.initial_w);
        sim.cpu().regs().f = low_byte(test.initial_w);

        constexpr uint8_t expected_cycles = 6;
        for (std::size_t i = 0; i < test.expected_accesses.size(); ++i) {
            const auto result = sim.tick_microcycle();
            EXPECT_TRUE(result.emitted);
            EXPECT_EQ(result.instruction_started, i == 0);
            EXPECT_EQ(result.instruction_complete, i + 1 == expected_cycles);
            EXPECT_EQ(result.instruction_result.cycles, expected_cycles);
        }

        expect_internal_cycles(sim, static_cast<uint8_t>(expected_cycles - test.expected_accesses.size()));

        const uint16_t actual_w =
            static_cast<uint16_t>((static_cast<uint16_t>(sim.cpu().regs().e) << 8) | sim.cpu().regs().f);
        EXPECT_EQ(sim.cpu().regs().pc, 0x0102);
        EXPECT_EQ(sim.cpu().regs().s, test.expected_s);
        EXPECT_EQ(sim.cpu().regs().u, test.expected_u);
        EXPECT_EQ(actual_w, test.expected_w);
        ASSERT_THAT(sim.bus().access_log(), testing::SizeIs(test.expected_accesses.size()));
        for (std::size_t i = 0; i < test.expected_accesses.size(); ++i) {
            SCOPED_TRACE(i);
            EXPECT_EQ(sim.bus().access_log()[i].type, test.expected_accesses[i].type);
            EXPECT_EQ(sim.bus().access_log()[i].address, test.expected_accesses[i].address);
            EXPECT_EQ(sim.bus().access_log()[i].value, test.expected_accesses[i].value);
            EXPECT_EQ(sim.bus().access_log()[i].cycle_kind, test.expected_accesses[i].cycle_kind);
        }
    }
}

TEST(BusPhaseTest, ResumableMicrocycleMemoryUnaryDirectAndExtendedReadWriteShape) {
    struct ExpectedAccess {
        microlind::BusAccessType type{};
        uint16_t address{};
        uint8_t value{};
        microlind::BusCycleKind cycle_kind{};
    };

    struct Case {
        const char* name;
        std::vector<uint8_t> program;
        uint8_t dp{};
        uint8_t cc{};
        uint16_t memory_address{};
        uint8_t memory_value{};
        uint8_t expected_cycles{};
        uint8_t expected_memory_value{};
        uint8_t expected_read_count{};
        uint8_t expected_write_count{};
        bool expect_n{};
        bool expect_z{};
        bool expect_v{};
        bool expect_c{};
        std::vector<ExpectedAccess> expected_accesses;
    };

    const Case cases[] = {
        {
            "NEG direct reads then writes",
            {0x00, 0x34},
            0x12,
            0x00,
            0x1234,
            0x01,
            6,
            0xFF,
            3,
            1,
            true,
            false,
            false,
            true,
            {
                {microlind::BusAccessType::Read, 0x0100, 0x00, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0101, 0x34, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x1234, 0x01, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Write, 0x1234, 0xFF, microlind::BusCycleKind::OperandWrite},
            },
        },
        {
            "TST direct reads without write",
            {0x0D, 0x34},
            0x12,
            microlind::CC_C,
            0x1234,
            0x80,
            6,
            0x00,
            3,
            0,
            true,
            false,
            false,
            false,
            {
                {microlind::BusAccessType::Read, 0x0100, 0x0D, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0101, 0x34, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x1234, 0x80, microlind::BusCycleKind::OperandRead},
            },
        },
        {
            "CLR extended writes without target read",
            {0x7F, 0x12, 0x34},
            0x00,
            microlind::CC_N | microlind::CC_C | microlind::CC_V,
            0x1234,
            0xAA,
            7,
            0x00,
            3,
            1,
            false,
            true,
            false,
            false,
            {
                {microlind::BusAccessType::Read, 0x0100, 0x7F, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0101, 0x12, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x0102, 0x34, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Write, 0x1234, 0x00, microlind::BusCycleKind::OperandWrite},
            },
        },
        {
            "ROL extended reads then writes using carry",
            {0x79, 0x12, 0x34},
            0x00,
            microlind::CC_C,
            0x1234,
            0x80,
            7,
            0x01,
            4,
            1,
            false,
            false,
            true,
            true,
            {
                {microlind::BusAccessType::Read, 0x0100, 0x79, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0101, 0x12, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x0102, 0x34, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x1234, 0x80, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Write, 0x1234, 0x01, microlind::BusCycleKind::OperandWrite},
            },
        },
    };

    for (const auto& test : cases) {
        SCOPED_TRACE(test.name);
        microlind::Simulator sim(microlind::CpuMode::HD6309, 1'000'000);
        auto memory = std::make_unique<SideEffectMemory>();
        auto* probe = memory.get();
        for (std::size_t i = 0; i < test.program.size(); ++i) {
            probe->data[static_cast<uint16_t>(0x0100 + i)] = test.program[i];
        }
        probe->data[test.memory_address] = test.memory_value;
        ASSERT_FALSE(sim.map_device(0x0000, 0xFFFF, std::move(memory)));

        sim.cpu().set_pc(0x0100);
        sim.cpu().regs().dp = test.dp;
        sim.cpu().regs().cc = test.cc;

        for (std::size_t i = 0; i < test.expected_accesses.size(); ++i) {
            const auto result = sim.tick_microcycle();
            EXPECT_TRUE(result.emitted);
            EXPECT_EQ(result.instruction_started, i == 0);
            EXPECT_EQ(result.instruction_complete, i + 1 == test.expected_cycles);
            EXPECT_EQ(result.instruction_result.cycles, test.expected_cycles);
        }

        const uint8_t internal_cycles = static_cast<uint8_t>(test.expected_cycles - test.expected_accesses.size());
        if (internal_cycles > 0) {
            expect_internal_cycles(sim, internal_cycles);
        } else {
            EXPECT_FALSE(sim.has_pending_microcycles());
        }

        EXPECT_EQ(probe->data[test.memory_address], test.expected_memory_value);
        EXPECT_EQ(probe->read_count, test.expected_read_count);
        EXPECT_EQ(probe->write_count, test.expected_write_count);
        EXPECT_EQ((sim.cpu().regs().cc & microlind::CC_N) != 0, test.expect_n);
        EXPECT_EQ((sim.cpu().regs().cc & microlind::CC_Z) != 0, test.expect_z);
        EXPECT_EQ((sim.cpu().regs().cc & microlind::CC_V) != 0, test.expect_v);
        EXPECT_EQ((sim.cpu().regs().cc & microlind::CC_C) != 0, test.expect_c);
        ASSERT_THAT(sim.bus().access_log(), testing::SizeIs(test.expected_accesses.size()));
        for (std::size_t i = 0; i < test.expected_accesses.size(); ++i) {
            SCOPED_TRACE(i);
            EXPECT_EQ(sim.bus().access_log()[i].type, test.expected_accesses[i].type);
            EXPECT_EQ(sim.bus().access_log()[i].address, test.expected_accesses[i].address);
            EXPECT_EQ(sim.bus().access_log()[i].value, test.expected_accesses[i].value);
            EXPECT_EQ(sim.bus().access_log()[i].cycle_kind, test.expected_accesses[i].cycle_kind);
        }
    }
}

TEST(BusPhaseTest, ResumableMicrocycleIndexedLoadStoreConventionalModes) {
    struct ExpectedAccess {
        microlind::BusAccessType type{};
        uint16_t address{};
        uint8_t value{};
        microlind::BusCycleKind cycle_kind{};
    };

    struct Case {
        const char* name;
        std::vector<uint8_t> program;
        uint8_t md{};
        uint16_t x{};
        uint16_t y{};
        uint16_t u{};
        uint16_t s{};
        uint8_t a{};
        uint8_t b{};
        std::vector<std::pair<uint16_t, uint8_t>> memory;
        uint8_t expected_cycles{};
        uint16_t expected_pc{};
        uint16_t expected_x{};
        uint16_t expected_y{};
        uint16_t expected_u{};
        uint16_t expected_s{};
        uint8_t expected_a{};
        uint8_t expected_b{};
        bool expect_z{};
        bool expect_n{};
        std::vector<ExpectedAccess> expected_accesses;
    };

    const Case cases[] = {
        {
            "LDA ,X",
            {0xA6, 0x84},
            0x00,
            0x2000,
            0x2100,
            0x3000,
            0x4000,
            0x00,
            0x00,
            {{0x2000, 0x5A}},
            4,
            0x0102,
            0x2000,
            0x2100,
            0x3000,
            0x4000,
            0x5A,
            0x00,
            false,
            false,
            {
                {microlind::BusAccessType::Read, 0x0100, 0xA6, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0101, 0x84, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x2000, 0x5A, microlind::BusCycleKind::OperandRead},
            },
        },
        {
            "LDB 5-bit offset,X",
            {0xE6, 0x05},
            0x00,
            0x2000,
            0x2100,
            0x3000,
            0x4000,
            0x00,
            0x00,
            {{0x2005, 0x80}},
            5,
            0x0102,
            0x2000,
            0x2100,
            0x3000,
            0x4000,
            0x00,
            0x80,
            false,
            true,
            {
                {microlind::BusAccessType::Read, 0x0100, 0xE6, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0101, 0x05, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x2005, 0x80, microlind::BusCycleKind::OperandRead},
            },
        },
        {
            "LDD 8-bit offset,Y",
            {0xEC, 0xA8, 0xFE},
            0x00,
            0x2000,
            0x2100,
            0x3000,
            0x4000,
            0x00,
            0x00,
            {{0x20FE, 0x12}, {0x20FF, 0x34}},
            6,
            0x0103,
            0x2000,
            0x2100,
            0x3000,
            0x4000,
            0x12,
            0x34,
            false,
            false,
            {
                {microlind::BusAccessType::Read, 0x0100, 0xEC, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0101, 0xA8, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x0102, 0xFE, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x20FE, 0x12, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x20FF, 0x34, microlind::BusCycleKind::OperandRead},
            },
        },
        {
            "STA ,X+ native",
            {0xA7, 0x80},
            0x01,
            0x2000,
            0x2100,
            0x3000,
            0x4000,
            0x44,
            0x00,
            {},
            5,
            0x0102,
            0x2001,
            0x2100,
            0x3000,
            0x4000,
            0x44,
            0x00,
            false,
            false,
            {
                {microlind::BusAccessType::Read, 0x0100, 0xA7, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0101, 0x80, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Write, 0x2000, 0x44, microlind::BusCycleKind::OperandWrite},
            },
        },
        {
            "STD ,--U",
            {0xED, 0xC3},
            0x00,
            0x2000,
            0x2100,
            0x3000,
            0x4000,
            0x12,
            0x34,
            {},
            8,
            0x0102,
            0x2000,
            0x2100,
            0x2FFE,
            0x4000,
            0x12,
            0x34,
            false,
            false,
            {
                {microlind::BusAccessType::Read, 0x0100, 0xED, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0101, 0xC3, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Write, 0x2FFE, 0x12, microlind::BusCycleKind::OperandWrite},
                {microlind::BusAccessType::Write, 0x2FFF, 0x34, microlind::BusCycleKind::OperandWrite},
            },
        },
        {
            "STB 16-bit offset,S",
            {0xE7, 0xE9, 0x00, 0x10},
            0x00,
            0x2000,
            0x2100,
            0x3000,
            0x4000,
            0x00,
            0x7F,
            {},
            8,
            0x0104,
            0x2000,
            0x2100,
            0x3000,
            0x4000,
            0x00,
            0x7F,
            false,
            false,
            {
                {microlind::BusAccessType::Read, 0x0100, 0xE7, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0101, 0xE9, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x0102, 0x00, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x0103, 0x10, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Write, 0x4010, 0x7F, microlind::BusCycleKind::OperandWrite},
            },
        },
        {
            "LDA 8-bit PC-relative",
            {0xA6, 0x8C, 0x02},
            0x00,
            0x2000,
            0x2100,
            0x3000,
            0x4000,
            0x00,
            0x00,
            {{0x0105, 0x66}},
            5,
            0x0103,
            0x2000,
            0x2100,
            0x3000,
            0x4000,
            0x66,
            0x00,
            false,
            false,
            {
                {microlind::BusAccessType::Read, 0x0100, 0xA6, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0101, 0x8C, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x0102, 0x02, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x0105, 0x66, microlind::BusCycleKind::OperandRead},
            },
        },
    };

    for (const auto& test : cases) {
        SCOPED_TRACE(test.name);
        microlind::Simulator sim(microlind::CpuMode::HD6309, 1'000'000);
        auto memory = std::make_unique<SideEffectMemory>();
        auto* probe = memory.get();
        for (std::size_t i = 0; i < test.program.size(); ++i) {
            probe->data[static_cast<uint16_t>(0x0100 + i)] = test.program[i];
        }
        for (const auto& [address, value] : test.memory) {
            probe->data[address] = value;
        }
        ASSERT_FALSE(sim.map_device(0x0000, 0xFFFF, std::move(memory)));

        sim.cpu().set_pc(0x0100);
        sim.cpu().regs().md = test.md;
        sim.cpu().regs().x = test.x;
        sim.cpu().regs().y = test.y;
        sim.cpu().regs().u = test.u;
        sim.cpu().regs().s = test.s;
        sim.cpu().regs().a = test.a;
        sim.cpu().regs().b = test.b;

        for (std::size_t i = 0; i < test.expected_accesses.size(); ++i) {
            const auto result = sim.tick_microcycle();
            EXPECT_TRUE(result.emitted);
            EXPECT_EQ(result.instruction_started, i == 0);
            EXPECT_EQ(result.instruction_complete, i + 1 == test.expected_cycles);
            EXPECT_EQ(result.instruction_result.cycles, test.expected_cycles);
        }

        const uint8_t internal_cycles = static_cast<uint8_t>(test.expected_cycles - test.expected_accesses.size());
        if (internal_cycles > 0) {
            expect_internal_cycles(sim, internal_cycles);
        } else {
            EXPECT_FALSE(sim.has_pending_microcycles());
        }

        EXPECT_EQ(sim.cpu().regs().pc, test.expected_pc);
        EXPECT_EQ(sim.cpu().regs().x, test.expected_x);
        EXPECT_EQ(sim.cpu().regs().y, test.expected_y);
        EXPECT_EQ(sim.cpu().regs().u, test.expected_u);
        EXPECT_EQ(sim.cpu().regs().s, test.expected_s);
        EXPECT_EQ(sim.cpu().regs().a, test.expected_a);
        EXPECT_EQ(sim.cpu().regs().b, test.expected_b);
        EXPECT_EQ((sim.cpu().regs().cc & microlind::CC_Z) != 0, test.expect_z);
        EXPECT_EQ((sim.cpu().regs().cc & microlind::CC_N) != 0, test.expect_n);
        ASSERT_THAT(sim.bus().access_log(), testing::SizeIs(test.expected_accesses.size()));
        for (std::size_t i = 0; i < test.expected_accesses.size(); ++i) {
            SCOPED_TRACE(i);
            EXPECT_EQ(sim.bus().access_log()[i].type, test.expected_accesses[i].type);
            EXPECT_EQ(sim.bus().access_log()[i].address, test.expected_accesses[i].address);
            EXPECT_EQ(sim.bus().access_log()[i].value, test.expected_accesses[i].value);
            EXPECT_EQ(sim.bus().access_log()[i].cycle_kind, test.expected_accesses[i].cycle_kind);
        }
    }
}

TEST(BusPhaseTest, ResumableMicrocycleIndexedIndirectLoadStoreConventionalModes) {
    struct ExpectedAccess {
        microlind::BusAccessType type{};
        uint16_t address{};
        uint8_t value{};
        microlind::BusCycleKind cycle_kind{};
    };

    struct Case {
        const char* name;
        std::vector<uint8_t> program;
        uint16_t x{};
        uint16_t y{};
        uint16_t u{};
        uint16_t s{};
        uint8_t a{};
        uint8_t b{};
        std::vector<std::pair<uint16_t, uint8_t>> memory;
        uint8_t expected_cycles{};
        uint16_t expected_pc{};
        uint16_t expected_x{};
        uint16_t expected_y{};
        uint16_t expected_u{};
        uint16_t expected_s{};
        uint8_t expected_a{};
        uint8_t expected_b{};
        std::vector<ExpectedAccess> expected_accesses;
    };

    const Case cases[] = {
        {
            "LDA [,X]",
            {0xA6, 0x94},
            0x2000,
            0x2100,
            0x3000,
            0x4000,
            0x00,
            0x00,
            {{0x2000, 0x30}, {0x2001, 0x00}, {0x3000, 0x5A}},
            7,
            0x0102,
            0x2000,
            0x2100,
            0x3000,
            0x4000,
            0x5A,
            0x00,
            {
                {microlind::BusAccessType::Read, 0x0100, 0xA6, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0101, 0x94, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x2000, 0x30, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x2001, 0x00, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x3000, 0x5A, microlind::BusCycleKind::OperandRead},
            },
        },
        {
            "LDD [8-bit offset,Y]",
            {0xEC, 0xB8, 0x02},
            0x2000,
            0x2100,
            0x3000,
            0x4000,
            0x00,
            0x00,
            {{0x2102, 0x30}, {0x2103, 0x00}, {0x3000, 0x12}, {0x3001, 0x34}},
            9,
            0x0103,
            0x2000,
            0x2100,
            0x3000,
            0x4000,
            0x12,
            0x34,
            {
                {microlind::BusAccessType::Read, 0x0100, 0xEC, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0101, 0xB8, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x0102, 0x02, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x2102, 0x30, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x2103, 0x00, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x3000, 0x12, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x3001, 0x34, microlind::BusCycleKind::OperandRead},
            },
        },
        {
            "STA [,X++]",
            {0xA7, 0x91},
            0x2000,
            0x2100,
            0x3000,
            0x4000,
            0x44,
            0x00,
            {{0x2000, 0x30}, {0x2001, 0x00}},
            10,
            0x0102,
            0x2002,
            0x2100,
            0x3000,
            0x4000,
            0x44,
            0x00,
            {
                {microlind::BusAccessType::Read, 0x0100, 0xA7, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0101, 0x91, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x2000, 0x30, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x2001, 0x00, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Write, 0x3000, 0x44, microlind::BusCycleKind::OperandWrite},
            },
        },
        {
            "STD [,--U]",
            {0xED, 0xD3},
            0x2000,
            0x2100,
            0x2002,
            0x4000,
            0x12,
            0x34,
            {{0x2000, 0x30}, {0x2001, 0x00}},
            11,
            0x0102,
            0x2000,
            0x2100,
            0x2000,
            0x4000,
            0x12,
            0x34,
            {
                {microlind::BusAccessType::Read, 0x0100, 0xED, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0101, 0xD3, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x2000, 0x30, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x2001, 0x00, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Write, 0x3000, 0x12, microlind::BusCycleKind::OperandWrite},
                {microlind::BusAccessType::Write, 0x3001, 0x34, microlind::BusCycleKind::OperandWrite},
            },
        },
        {
            "STB [16-bit offset,S]",
            {0xE7, 0xF9, 0x00, 0x10},
            0x2000,
            0x2100,
            0x3000,
            0x4000,
            0x00,
            0x7F,
            {{0x4010, 0x30}, {0x4011, 0x00}},
            11,
            0x0104,
            0x2000,
            0x2100,
            0x3000,
            0x4000,
            0x00,
            0x7F,
            {
                {microlind::BusAccessType::Read, 0x0100, 0xE7, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0101, 0xF9, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x0102, 0x00, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x0103, 0x10, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x4010, 0x30, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x4011, 0x00, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Write, 0x3000, 0x7F, microlind::BusCycleKind::OperandWrite},
            },
        },
    };

    for (const auto& test : cases) {
        SCOPED_TRACE(test.name);
        microlind::Simulator sim(microlind::CpuMode::HD6309, 1'000'000);
        auto memory = std::make_unique<SideEffectMemory>();
        auto* probe = memory.get();
        for (std::size_t i = 0; i < test.program.size(); ++i) {
            probe->data[static_cast<uint16_t>(0x0100 + i)] = test.program[i];
        }
        for (const auto& [address, value] : test.memory) {
            probe->data[address] = value;
        }
        ASSERT_FALSE(sim.map_device(0x0000, 0xFFFF, std::move(memory)));

        sim.cpu().set_pc(0x0100);
        sim.cpu().regs().x = test.x;
        sim.cpu().regs().y = test.y;
        sim.cpu().regs().u = test.u;
        sim.cpu().regs().s = test.s;
        sim.cpu().regs().a = test.a;
        sim.cpu().regs().b = test.b;

        for (std::size_t i = 0; i < test.expected_accesses.size(); ++i) {
            const auto result = sim.tick_microcycle();
            EXPECT_TRUE(result.emitted);
            EXPECT_EQ(result.instruction_started, i == 0);
            EXPECT_EQ(result.instruction_complete, i + 1 == test.expected_cycles);
            EXPECT_EQ(result.instruction_result.cycles, test.expected_cycles);
        }

        const uint8_t internal_cycles = static_cast<uint8_t>(test.expected_cycles - test.expected_accesses.size());
        if (internal_cycles > 0) {
            expect_internal_cycles(sim, internal_cycles);
        } else {
            EXPECT_FALSE(sim.has_pending_microcycles());
        }

        EXPECT_EQ(sim.cpu().regs().pc, test.expected_pc);
        EXPECT_EQ(sim.cpu().regs().x, test.expected_x);
        EXPECT_EQ(sim.cpu().regs().y, test.expected_y);
        EXPECT_EQ(sim.cpu().regs().u, test.expected_u);
        EXPECT_EQ(sim.cpu().regs().s, test.expected_s);
        EXPECT_EQ(sim.cpu().regs().a, test.expected_a);
        EXPECT_EQ(sim.cpu().regs().b, test.expected_b);
        ASSERT_THAT(sim.bus().access_log(), testing::SizeIs(test.expected_accesses.size()));
        for (std::size_t i = 0; i < test.expected_accesses.size(); ++i) {
            SCOPED_TRACE(i);
            EXPECT_EQ(sim.bus().access_log()[i].type, test.expected_accesses[i].type);
            EXPECT_EQ(sim.bus().access_log()[i].address, test.expected_accesses[i].address);
            EXPECT_EQ(sim.bus().access_log()[i].value, test.expected_accesses[i].value);
            EXPECT_EQ(sim.bus().access_log()[i].cycle_kind, test.expected_accesses[i].cycle_kind);
        }
    }
}

TEST(BusPhaseTest, ResumableMicrocycleIndexedAlu8ConventionalModes) {
    struct ExpectedAccess {
        microlind::BusAccessType type{};
        uint16_t address{};
        uint8_t value{};
        microlind::BusCycleKind cycle_kind{};
    };

    struct Case {
        const char* name;
        std::vector<uint8_t> program;
        uint16_t x{};
        uint16_t y{};
        uint8_t a{};
        uint8_t b{};
        std::vector<std::pair<uint16_t, uint8_t>> memory;
        uint8_t expected_cycles{};
        uint16_t expected_pc{};
        uint16_t expected_x{};
        uint16_t expected_y{};
        uint8_t expected_a{};
        uint8_t expected_b{};
        bool expect_z{};
        bool expect_n{};
        bool expect_v{};
        bool expect_c{};
        std::vector<ExpectedAccess> expected_accesses;
    };

    const Case cases[] = {
        {
            "SUBA ,X",
            {0xA0, 0x84},
            0x2000,
            0x2100,
            0x10,
            0x00,
            {{0x2000, 0x01}},
            4,
            0x0102,
            0x2000,
            0x2100,
            0x0F,
            0x00,
            false,
            false,
            false,
            false,
            {
                {microlind::BusAccessType::Read, 0x0100, 0xA0, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0101, 0x84, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x2000, 0x01, microlind::BusCycleKind::OperandRead},
            },
        },
        {
            "CMPA 5-bit offset,X",
            {0xA1, 0x05},
            0x2000,
            0x2100,
            0x20,
            0x00,
            {{0x2005, 0x20}},
            5,
            0x0102,
            0x2000,
            0x2100,
            0x20,
            0x00,
            true,
            false,
            false,
            false,
            {
                {microlind::BusAccessType::Read, 0x0100, 0xA1, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0101, 0x05, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x2005, 0x20, microlind::BusCycleKind::OperandRead},
            },
        },
        {
            "ADDB 8-bit offset,Y",
            {0xEB, 0xA8, 0x02},
            0x2000,
            0x2100,
            0x00,
            0x7F,
            {{0x2102, 0x01}},
            5,
            0x0103,
            0x2000,
            0x2100,
            0x00,
            0x80,
            false,
            true,
            true,
            false,
            {
                {microlind::BusAccessType::Read, 0x0100, 0xEB, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0101, 0xA8, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x0102, 0x02, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x2102, 0x01, microlind::BusCycleKind::OperandRead},
            },
        },
        {
            "BITB [,X]",
            {0xE5, 0x94},
            0x2000,
            0x2100,
            0x00,
            0x0F,
            {{0x2000, 0x30}, {0x2001, 0x00}, {0x3000, 0xF0}},
            7,
            0x0102,
            0x2000,
            0x2100,
            0x00,
            0x0F,
            true,
            false,
            false,
            false,
            {
                {microlind::BusAccessType::Read, 0x0100, 0xE5, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0101, 0x94, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x2000, 0x30, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x2001, 0x00, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x3000, 0xF0, microlind::BusCycleKind::OperandRead},
            },
        },
    };

    for (const auto& test : cases) {
        SCOPED_TRACE(test.name);
        microlind::Simulator sim(microlind::CpuMode::HD6309, 1'000'000);
        auto memory = std::make_unique<SideEffectMemory>();
        auto* probe = memory.get();
        for (std::size_t i = 0; i < test.program.size(); ++i) {
            probe->data[static_cast<uint16_t>(0x0100 + i)] = test.program[i];
        }
        for (const auto& [address, value] : test.memory) {
            probe->data[address] = value;
        }
        ASSERT_FALSE(sim.map_device(0x0000, 0xFFFF, std::move(memory)));

        sim.cpu().set_pc(0x0100);
        sim.cpu().regs().x = test.x;
        sim.cpu().regs().y = test.y;
        sim.cpu().regs().a = test.a;
        sim.cpu().regs().b = test.b;

        for (std::size_t i = 0; i < test.expected_accesses.size(); ++i) {
            const auto result = sim.tick_microcycle();
            EXPECT_TRUE(result.emitted);
            EXPECT_EQ(result.instruction_started, i == 0);
            EXPECT_EQ(result.instruction_complete, i + 1 == test.expected_cycles);
            EXPECT_EQ(result.instruction_result.cycles, test.expected_cycles);
        }

        const uint8_t internal_cycles = static_cast<uint8_t>(test.expected_cycles - test.expected_accesses.size());
        if (internal_cycles > 0) {
            expect_internal_cycles(sim, internal_cycles);
        } else {
            EXPECT_FALSE(sim.has_pending_microcycles());
        }

        EXPECT_EQ(sim.cpu().regs().pc, test.expected_pc);
        EXPECT_EQ(sim.cpu().regs().x, test.expected_x);
        EXPECT_EQ(sim.cpu().regs().y, test.expected_y);
        EXPECT_EQ(sim.cpu().regs().a, test.expected_a);
        EXPECT_EQ(sim.cpu().regs().b, test.expected_b);
        EXPECT_EQ((sim.cpu().regs().cc & microlind::CC_Z) != 0, test.expect_z);
        EXPECT_EQ((sim.cpu().regs().cc & microlind::CC_N) != 0, test.expect_n);
        EXPECT_EQ((sim.cpu().regs().cc & microlind::CC_V) != 0, test.expect_v);
        EXPECT_EQ((sim.cpu().regs().cc & microlind::CC_C) != 0, test.expect_c);
        ASSERT_THAT(sim.bus().access_log(), testing::SizeIs(test.expected_accesses.size()));
        for (std::size_t i = 0; i < test.expected_accesses.size(); ++i) {
            SCOPED_TRACE(i);
            EXPECT_EQ(sim.bus().access_log()[i].type, test.expected_accesses[i].type);
            EXPECT_EQ(sim.bus().access_log()[i].address, test.expected_accesses[i].address);
            EXPECT_EQ(sim.bus().access_log()[i].value, test.expected_accesses[i].value);
            EXPECT_EQ(sim.bus().access_log()[i].cycle_kind, test.expected_accesses[i].cycle_kind);
        }
    }
}

TEST(BusPhaseTest, ResumableMicrocycleIndexedWordAluAndCompareModes) {
    struct ExpectedAccess {
        microlind::BusAccessType type{};
        uint16_t address{};
        uint8_t value{};
        microlind::BusCycleKind cycle_kind{};
    };

    struct Case {
        const char* name;
        std::vector<uint8_t> program;
        uint16_t x{};
        uint16_t y{};
        uint16_t u{};
        uint16_t s{};
        uint8_t a{};
        uint8_t b{};
        std::vector<std::pair<uint16_t, uint8_t>> memory;
        uint8_t expected_cycles{};
        uint16_t expected_pc{};
        uint16_t expected_x{};
        uint16_t expected_y{};
        uint16_t expected_u{};
        uint16_t expected_s{};
        uint8_t expected_a{};
        uint8_t expected_b{};
        bool expect_z{};
        bool expect_n{};
        bool expect_v{};
        bool expect_c{};
        std::vector<ExpectedAccess> expected_accesses;
    };

    const Case cases[] = {
        {
            "ADDD ,X",
            {0xE3, 0x84},
            0x2000,
            0x2100,
            0x3000,
            0x4000,
            0x00,
            0x01,
            {{0x2000, 0x00}, {0x2001, 0x01}},
            6,
            0x0102,
            0x2000,
            0x2100,
            0x3000,
            0x4000,
            0x00,
            0x02,
            false,
            false,
            false,
            false,
            {
                {microlind::BusAccessType::Read, 0x0100, 0xE3, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0101, 0x84, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x2000, 0x00, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x2001, 0x01, microlind::BusCycleKind::OperandRead},
            },
        },
        {
            "SUBD 8-bit offset,Y",
            {0xA3, 0xA8, 0x02},
            0x2000,
            0x2100,
            0x3000,
            0x4000,
            0x01,
            0x00,
            {{0x2102, 0x00}, {0x2103, 0x01}},
            7,
            0x0103,
            0x2000,
            0x2100,
            0x3000,
            0x4000,
            0x00,
            0xFF,
            false,
            false,
            false,
            false,
            {
                {microlind::BusAccessType::Read, 0x0100, 0xA3, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0101, 0xA8, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x0102, 0x02, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x2102, 0x00, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x2103, 0x01, microlind::BusCycleKind::OperandRead},
            },
        },
        {
            "CMPX ,X",
            {0xAC, 0x84},
            0x2000,
            0x2100,
            0x3000,
            0x4000,
            0x00,
            0x00,
            {{0x2000, 0x20}, {0x2001, 0x00}},
            6,
            0x0102,
            0x2000,
            0x2100,
            0x3000,
            0x4000,
            0x00,
            0x00,
            true,
            false,
            false,
            false,
            {
                {microlind::BusAccessType::Read, 0x0100, 0xAC, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0101, 0x84, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x2000, 0x20, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x2001, 0x00, microlind::BusCycleKind::OperandRead},
            },
        },
        {
            "CMPD [,X]",
            {0x10, 0xA3, 0x94},
            0x2000,
            0x2100,
            0x3000,
            0x4000,
            0x12,
            0x35,
            {{0x2000, 0x30}, {0x2001, 0x00}, {0x3000, 0x12}, {0x3001, 0x34}},
            10,
            0x0103,
            0x2000,
            0x2100,
            0x3000,
            0x4000,
            0x12,
            0x35,
            false,
            false,
            false,
            false,
            {
                {microlind::BusAccessType::Read, 0x0100, 0x10, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0101, 0xA3, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0102, 0x94, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x2000, 0x30, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x2001, 0x00, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x3000, 0x12, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x3001, 0x34, microlind::BusCycleKind::OperandRead},
            },
        },
        {
            "CMPU 5-bit offset,X",
            {0x11, 0xA3, 0x05},
            0x2000,
            0x2100,
            0x1234,
            0x4000,
            0x00,
            0x00,
            {{0x2005, 0x12}, {0x2006, 0x35}},
            8,
            0x0103,
            0x2000,
            0x2100,
            0x1234,
            0x4000,
            0x00,
            0x00,
            false,
            true,
            false,
            true,
            {
                {microlind::BusAccessType::Read, 0x0100, 0x11, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0101, 0xA3, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0102, 0x05, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x2005, 0x12, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x2006, 0x35, microlind::BusCycleKind::OperandRead},
            },
        },
    };

    for (const auto& test : cases) {
        SCOPED_TRACE(test.name);
        microlind::Simulator sim(microlind::CpuMode::HD6309, 1'000'000);
        auto memory = std::make_unique<SideEffectMemory>();
        auto* probe = memory.get();
        for (std::size_t i = 0; i < test.program.size(); ++i) {
            probe->data[static_cast<uint16_t>(0x0100 + i)] = test.program[i];
        }
        for (const auto& [address, value] : test.memory) {
            probe->data[address] = value;
        }
        ASSERT_FALSE(sim.map_device(0x0000, 0xFFFF, std::move(memory)));

        sim.cpu().set_pc(0x0100);
        sim.cpu().regs().x = test.x;
        sim.cpu().regs().y = test.y;
        sim.cpu().regs().u = test.u;
        sim.cpu().regs().s = test.s;
        sim.cpu().regs().a = test.a;
        sim.cpu().regs().b = test.b;

        for (std::size_t i = 0; i < test.expected_accesses.size(); ++i) {
            const auto result = sim.tick_microcycle();
            EXPECT_TRUE(result.emitted);
            EXPECT_EQ(result.instruction_started, i == 0);
            EXPECT_EQ(result.instruction_complete, i + 1 == test.expected_cycles);
            EXPECT_EQ(result.instruction_result.cycles, test.expected_cycles);
        }

        const uint8_t internal_cycles = static_cast<uint8_t>(test.expected_cycles - test.expected_accesses.size());
        if (internal_cycles > 0) {
            expect_internal_cycles(sim, internal_cycles);
        } else {
            EXPECT_FALSE(sim.has_pending_microcycles());
        }

        EXPECT_EQ(sim.cpu().regs().pc, test.expected_pc);
        EXPECT_EQ(sim.cpu().regs().x, test.expected_x);
        EXPECT_EQ(sim.cpu().regs().y, test.expected_y);
        EXPECT_EQ(sim.cpu().regs().u, test.expected_u);
        EXPECT_EQ(sim.cpu().regs().s, test.expected_s);
        EXPECT_EQ(sim.cpu().regs().a, test.expected_a);
        EXPECT_EQ(sim.cpu().regs().b, test.expected_b);
        EXPECT_EQ((sim.cpu().regs().cc & microlind::CC_Z) != 0, test.expect_z);
        EXPECT_EQ((sim.cpu().regs().cc & microlind::CC_N) != 0, test.expect_n);
        EXPECT_EQ((sim.cpu().regs().cc & microlind::CC_V) != 0, test.expect_v);
        EXPECT_EQ((sim.cpu().regs().cc & microlind::CC_C) != 0, test.expect_c);
        ASSERT_THAT(sim.bus().access_log(), testing::SizeIs(test.expected_accesses.size()));
        for (std::size_t i = 0; i < test.expected_accesses.size(); ++i) {
            SCOPED_TRACE(i);
            EXPECT_EQ(sim.bus().access_log()[i].type, test.expected_accesses[i].type);
            EXPECT_EQ(sim.bus().access_log()[i].address, test.expected_accesses[i].address);
            EXPECT_EQ(sim.bus().access_log()[i].value, test.expected_accesses[i].value);
            EXPECT_EQ(sim.bus().access_log()[i].cycle_kind, test.expected_accesses[i].cycle_kind);
        }
    }
}

TEST(BusPhaseTest, ResumableMicrocycleIndexedWordLoadStoreModes) {
    struct ExpectedAccess {
        microlind::BusAccessType type{};
        uint16_t address{};
        uint8_t value{};
        microlind::BusCycleKind cycle_kind{};
    };

    struct Case {
        const char* name;
        std::vector<uint8_t> program;
        uint16_t x{};
        uint16_t y{};
        uint16_t u{};
        uint16_t s{};
        std::vector<std::pair<uint16_t, uint8_t>> memory;
        std::vector<std::pair<uint16_t, uint8_t>> expected_memory;
        uint8_t expected_cycles{};
        uint16_t expected_pc{};
        uint16_t expected_x{};
        uint16_t expected_y{};
        uint16_t expected_u{};
        uint16_t expected_s{};
        bool expect_z{};
        bool expect_n{};
        std::vector<ExpectedAccess> expected_accesses;
    };

    const Case cases[] = {
        {
            "LDX ,X",
            {0xAE, 0x84},
            0x2000,
            0x2100,
            0x3000,
            0x4000,
            {{0x2000, 0x12}, {0x2001, 0x34}},
            {},
            5,
            0x0102,
            0x1234,
            0x2100,
            0x3000,
            0x4000,
            false,
            false,
            {
                {microlind::BusAccessType::Read, 0x0100, 0xAE, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0101, 0x84, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x2000, 0x12, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x2001, 0x34, microlind::BusCycleKind::OperandRead},
            },
        },
        {
            "LDU 8-bit offset,Y",
            {0xEE, 0xA8, 0x02},
            0x2000,
            0x2100,
            0x3000,
            0x4000,
            {{0x2102, 0xAB}, {0x2103, 0xCD}},
            {},
            6,
            0x0103,
            0x2000,
            0x2100,
            0xABCD,
            0x4000,
            false,
            true,
            {
                {microlind::BusAccessType::Read, 0x0100, 0xEE, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0101, 0xA8, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x0102, 0x02, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x2102, 0xAB, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x2103, 0xCD, microlind::BusCycleKind::OperandRead},
            },
        },
        {
            "LDY [,X]",
            {0x10, 0xAE, 0x94},
            0x2000,
            0x2100,
            0x3000,
            0x4000,
            {{0x2000, 0x30}, {0x2001, 0x00}, {0x3000, 0x80}, {0x3001, 0x00}},
            {},
            9,
            0x0103,
            0x2000,
            0x8000,
            0x3000,
            0x4000,
            false,
            true,
            {
                {microlind::BusAccessType::Read, 0x0100, 0x10, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0101, 0xAE, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0102, 0x94, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x2000, 0x30, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x2001, 0x00, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x3000, 0x80, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x3001, 0x00, microlind::BusCycleKind::OperandRead},
            },
        },
        {
            "STX ,X",
            {0xAF, 0x84},
            0x2000,
            0x2100,
            0x3000,
            0x4000,
            {},
            {{0x2000, 0x20}, {0x2001, 0x00}},
            5,
            0x0102,
            0x2000,
            0x2100,
            0x3000,
            0x4000,
            false,
            false,
            {
                {microlind::BusAccessType::Read, 0x0100, 0xAF, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0101, 0x84, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Write, 0x2000, 0x20, microlind::BusCycleKind::OperandWrite},
                {microlind::BusAccessType::Write, 0x2001, 0x00, microlind::BusCycleKind::OperandWrite},
            },
        },
        {
            "STY [16-bit offset,S]",
            {0x10, 0xAF, 0xF9, 0x00, 0x10},
            0x2000,
            0xBEEF,
            0x3000,
            0x4000,
            {{0x4010, 0x30}, {0x4011, 0x00}},
            {{0x3000, 0xBE}, {0x3001, 0xEF}},
            13,
            0x0105,
            0x2000,
            0xBEEF,
            0x3000,
            0x4000,
            false,
            true,
            {
                {microlind::BusAccessType::Read, 0x0100, 0x10, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0101, 0xAF, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0102, 0xF9, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x0103, 0x00, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x0104, 0x10, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x4010, 0x30, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x4011, 0x00, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Write, 0x3000, 0xBE, microlind::BusCycleKind::OperandWrite},
                {microlind::BusAccessType::Write, 0x3001, 0xEF, microlind::BusCycleKind::OperandWrite},
            },
        },
    };

    for (const auto& test : cases) {
        SCOPED_TRACE(test.name);
        microlind::Simulator sim(microlind::CpuMode::HD6309, 1'000'000);
        auto memory = std::make_unique<SideEffectMemory>();
        auto* probe = memory.get();
        for (std::size_t i = 0; i < test.program.size(); ++i) {
            probe->data[static_cast<uint16_t>(0x0100 + i)] = test.program[i];
        }
        for (const auto& [address, value] : test.memory) {
            probe->data[address] = value;
        }
        ASSERT_FALSE(sim.map_device(0x0000, 0xFFFF, std::move(memory)));

        sim.cpu().set_pc(0x0100);
        sim.cpu().regs().x = test.x;
        sim.cpu().regs().y = test.y;
        sim.cpu().regs().u = test.u;
        sim.cpu().regs().s = test.s;

        for (std::size_t i = 0; i < test.expected_accesses.size(); ++i) {
            const auto result = sim.tick_microcycle();
            EXPECT_TRUE(result.emitted);
            EXPECT_EQ(result.instruction_started, i == 0);
            EXPECT_EQ(result.instruction_complete, i + 1 == test.expected_cycles);
            EXPECT_EQ(result.instruction_result.cycles, test.expected_cycles);
        }

        const uint8_t internal_cycles = static_cast<uint8_t>(test.expected_cycles - test.expected_accesses.size());
        if (internal_cycles > 0) {
            expect_internal_cycles(sim, internal_cycles);
        } else {
            EXPECT_FALSE(sim.has_pending_microcycles());
        }

        EXPECT_EQ(sim.cpu().regs().pc, test.expected_pc);
        EXPECT_EQ(sim.cpu().regs().x, test.expected_x);
        EXPECT_EQ(sim.cpu().regs().y, test.expected_y);
        EXPECT_EQ(sim.cpu().regs().u, test.expected_u);
        EXPECT_EQ(sim.cpu().regs().s, test.expected_s);
        EXPECT_EQ((sim.cpu().regs().cc & microlind::CC_Z) != 0, test.expect_z);
        EXPECT_EQ((sim.cpu().regs().cc & microlind::CC_N) != 0, test.expect_n);
        for (const auto& [address, value] : test.expected_memory) {
            EXPECT_EQ(probe->data[address], value);
        }
        ASSERT_THAT(sim.bus().access_log(), testing::SizeIs(test.expected_accesses.size()));
        for (std::size_t i = 0; i < test.expected_accesses.size(); ++i) {
            SCOPED_TRACE(i);
            EXPECT_EQ(sim.bus().access_log()[i].type, test.expected_accesses[i].type);
            EXPECT_EQ(sim.bus().access_log()[i].address, test.expected_accesses[i].address);
            EXPECT_EQ(sim.bus().access_log()[i].value, test.expected_accesses[i].value);
            EXPECT_EQ(sim.bus().access_log()[i].cycle_kind, test.expected_accesses[i].cycle_kind);
        }
    }
}

TEST(BusPhaseTest, ResumableMicrocycleWLoadStoreArithmeticModes) {
    struct ExpectedAccess {
        microlind::BusAccessType type{};
        uint16_t address{};
        uint8_t value{};
        microlind::BusCycleKind cycle_kind{};
    };

    struct Case {
        const char* name;
        std::vector<uint8_t> program;
        uint8_t dp{};
        uint16_t initial_w{};
        std::vector<std::pair<uint16_t, uint8_t>> memory;
        std::vector<std::pair<uint16_t, uint8_t>> expected_memory;
        uint8_t expected_cycles{};
        uint16_t expected_pc{};
        uint16_t expected_w{};
        bool expect_z{};
        bool expect_n{};
        bool expect_v{};
        bool expect_c{};
        std::vector<ExpectedAccess> expected_accesses;
    };

    const Case cases[] = {
        {
            "LDW immediate",
            {0x10, 0x86, 0x80, 0x01},
            0x12,
            0x0000,
            {},
            {},
            4,
            0x0104,
            0x8001,
            false,
            true,
            false,
            false,
            {
                {microlind::BusAccessType::Read, 0x0100, 0x10, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0101, 0x86, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0102, 0x80, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x0103, 0x01, microlind::BusCycleKind::OperandRead},
            },
        },
        {
            "STW direct",
            {0x10, 0x97, 0x20},
            0x12,
            0xBEEF,
            {},
            {{0x1220, 0xBE}, {0x1221, 0xEF}},
            6,
            0x0103,
            0xBEEF,
            false,
            true,
            false,
            false,
            {
                {microlind::BusAccessType::Read, 0x0100, 0x10, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0101, 0x97, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0102, 0x20, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Write, 0x1220, 0xBE, microlind::BusCycleKind::OperandWrite},
                {microlind::BusAccessType::Write, 0x1221, 0xEF, microlind::BusCycleKind::OperandWrite},
            },
        },
        {
            "ADDW immediate",
            {0x10, 0x8B, 0x00, 0x01},
            0x12,
            0x7FFF,
            {},
            {},
            5,
            0x0104,
            0x8000,
            false,
            true,
            true,
            false,
            {
                {microlind::BusAccessType::Read, 0x0100, 0x10, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0101, 0x8B, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0102, 0x00, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x0103, 0x01, microlind::BusCycleKind::OperandRead},
            },
        },
        {
            "SUBW direct",
            {0x10, 0x90, 0x20},
            0x12,
            0x0100,
            {{0x1220, 0x00}, {0x1221, 0x01}},
            {},
            7,
            0x0103,
            0x00FF,
            false,
            false,
            false,
            false,
            {
                {microlind::BusAccessType::Read, 0x0100, 0x10, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0101, 0x90, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0102, 0x20, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x1220, 0x00, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x1221, 0x01, microlind::BusCycleKind::OperandRead},
            },
        },
        {
            "CMPW extended",
            {0x10, 0xB1, 0x20, 0x00},
            0x12,
            0x1234,
            {{0x2000, 0x12}, {0x2001, 0x34}},
            {},
            8,
            0x0104,
            0x1234,
            true,
            false,
            false,
            false,
            {
                {microlind::BusAccessType::Read, 0x0100, 0x10, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0101, 0xB1, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0102, 0x20, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x0103, 0x00, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x2000, 0x12, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x2001, 0x34, microlind::BusCycleKind::OperandRead},
            },
        },
        {
            "LDW extended",
            {0x10, 0xB6, 0x20, 0x00},
            0x12,
            0x0000,
            {{0x2000, 0x00}, {0x2001, 0x00}},
            {},
            7,
            0x0104,
            0x0000,
            true,
            false,
            false,
            false,
            {
                {microlind::BusAccessType::Read, 0x0100, 0x10, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0101, 0xB6, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0102, 0x20, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x0103, 0x00, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x2000, 0x00, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x2001, 0x00, microlind::BusCycleKind::OperandRead},
            },
        },
        {
            "STW extended",
            {0x10, 0xB7, 0x20, 0x00},
            0x12,
            0x1234,
            {},
            {{0x2000, 0x12}, {0x2001, 0x34}},
            7,
            0x0104,
            0x1234,
            false,
            false,
            false,
            false,
            {
                {microlind::BusAccessType::Read, 0x0100, 0x10, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0101, 0xB7, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0102, 0x20, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x0103, 0x00, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Write, 0x2000, 0x12, microlind::BusCycleKind::OperandWrite},
                {microlind::BusAccessType::Write, 0x2001, 0x34, microlind::BusCycleKind::OperandWrite},
            },
        },
    };

    for (const auto& test : cases) {
        SCOPED_TRACE(test.name);
        microlind::Simulator sim(microlind::CpuMode::HD6309, 1'000'000);
        auto memory = std::make_unique<SideEffectMemory>();
        auto* probe = memory.get();
        for (std::size_t i = 0; i < test.program.size(); ++i) {
            probe->data[static_cast<uint16_t>(0x0100 + i)] = test.program[i];
        }
        for (const auto& [address, value] : test.memory) {
            probe->data[address] = value;
        }
        ASSERT_FALSE(sim.map_device(0x0000, 0xFFFF, std::move(memory)));

        sim.cpu().set_pc(0x0100);
        sim.cpu().regs().dp = test.dp;
        sim.cpu().regs().e = static_cast<uint8_t>(test.initial_w >> 8);
        sim.cpu().regs().f = static_cast<uint8_t>(test.initial_w & 0xFF);

        for (std::size_t i = 0; i < test.expected_accesses.size(); ++i) {
            const auto result = sim.tick_microcycle();
            EXPECT_TRUE(result.emitted);
            EXPECT_EQ(result.instruction_started, i == 0);
            EXPECT_EQ(result.instruction_complete, i + 1 == test.expected_cycles);
            EXPECT_EQ(result.instruction_result.cycles, test.expected_cycles);
        }

        const uint8_t internal_cycles = static_cast<uint8_t>(test.expected_cycles - test.expected_accesses.size());
        if (internal_cycles > 0) {
            expect_internal_cycles(sim, internal_cycles);
        } else {
            EXPECT_FALSE(sim.has_pending_microcycles());
        }

        const uint16_t actual_w = static_cast<uint16_t>((static_cast<uint16_t>(sim.cpu().regs().e) << 8) | sim.cpu().regs().f);
        EXPECT_EQ(sim.cpu().regs().pc, test.expected_pc);
        EXPECT_EQ(actual_w, test.expected_w);
        EXPECT_EQ((sim.cpu().regs().cc & microlind::CC_Z) != 0, test.expect_z);
        EXPECT_EQ((sim.cpu().regs().cc & microlind::CC_N) != 0, test.expect_n);
        EXPECT_EQ((sim.cpu().regs().cc & microlind::CC_V) != 0, test.expect_v);
        EXPECT_EQ((sim.cpu().regs().cc & microlind::CC_C) != 0, test.expect_c);
        for (const auto& [address, value] : test.expected_memory) {
            EXPECT_EQ(probe->data[address], value);
        }
        ASSERT_THAT(sim.bus().access_log(), testing::SizeIs(test.expected_accesses.size()));
        for (std::size_t i = 0; i < test.expected_accesses.size(); ++i) {
            SCOPED_TRACE(i);
            EXPECT_EQ(sim.bus().access_log()[i].type, test.expected_accesses[i].type);
            EXPECT_EQ(sim.bus().access_log()[i].address, test.expected_accesses[i].address);
            EXPECT_EQ(sim.bus().access_log()[i].value, test.expected_accesses[i].value);
            EXPECT_EQ(sim.bus().access_log()[i].cycle_kind, test.expected_accesses[i].cycle_kind);
        }
    }
}

TEST(BusPhaseTest, ResumableMicrocycleQLoadStoreModes) {
    struct ExpectedAccess {
        microlind::BusAccessType type{};
        uint16_t address{};
        uint8_t value{};
        microlind::BusCycleKind cycle_kind{};
    };

    struct Case {
        const char* name;
        std::vector<uint8_t> program;
        uint8_t dp{};
        uint32_t initial_q{};
        std::vector<std::pair<uint16_t, uint8_t>> memory;
        std::vector<std::pair<uint16_t, uint8_t>> expected_memory;
        uint8_t expected_cycles{};
        uint16_t expected_pc{};
        uint32_t expected_q{};
        bool expect_z{};
        bool expect_n{};
        std::vector<ExpectedAccess> expected_accesses;
    };

    const Case cases[] = {
        {
            "LDQ immediate",
            {0xCD, 0x12, 0x34, 0x56, 0x78},
            0x12,
            0x00000000u,
            {},
            {},
            5,
            0x0105,
            0x12345678u,
            false,
            false,
            {
                {microlind::BusAccessType::Read, 0x0100, 0xCD, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0101, 0x12, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x0102, 0x34, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x0103, 0x56, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x0104, 0x78, microlind::BusCycleKind::OperandRead},
            },
        },
        {
            "LDQ direct zero",
            {0x10, 0xDC, 0x20},
            0x12,
            0xFFFFFFFFu,
            {{0x1220, 0x00}, {0x1221, 0x00}, {0x1222, 0x00}, {0x1223, 0x00}},
            {},
            8,
            0x0103,
            0x00000000u,
            true,
            false,
            {
                {microlind::BusAccessType::Read, 0x0100, 0x10, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0101, 0xDC, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0102, 0x20, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x1220, 0x00, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x1221, 0x00, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x1222, 0x00, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x1223, 0x00, microlind::BusCycleKind::OperandRead},
            },
        },
        {
            "STQ direct negative",
            {0x10, 0xDD, 0x20},
            0x12,
            0x80000001u,
            {},
            {{0x1220, 0x80}, {0x1221, 0x00}, {0x1222, 0x00}, {0x1223, 0x01}},
            8,
            0x0103,
            0x80000001u,
            false,
            true,
            {
                {microlind::BusAccessType::Read, 0x0100, 0x10, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0101, 0xDD, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0102, 0x20, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Write, 0x1220, 0x80, microlind::BusCycleKind::OperandWrite},
                {microlind::BusAccessType::Write, 0x1221, 0x00, microlind::BusCycleKind::OperandWrite},
                {microlind::BusAccessType::Write, 0x1222, 0x00, microlind::BusCycleKind::OperandWrite},
                {microlind::BusAccessType::Write, 0x1223, 0x01, microlind::BusCycleKind::OperandWrite},
            },
        },
        {
            "LDQ extended negative",
            {0x10, 0xFC, 0x20, 0x00},
            0x12,
            0x00000000u,
            {{0x2000, 0x80}, {0x2001, 0x00}, {0x2002, 0x00}, {0x2003, 0x00}},
            {},
            9,
            0x0104,
            0x80000000u,
            false,
            true,
            {
                {microlind::BusAccessType::Read, 0x0100, 0x10, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0101, 0xFC, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0102, 0x20, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x0103, 0x00, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x2000, 0x80, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x2001, 0x00, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x2002, 0x00, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x2003, 0x00, microlind::BusCycleKind::OperandRead},
            },
        },
        {
            "STQ extended zero",
            {0x10, 0xFD, 0x20, 0x00},
            0x12,
            0x00000000u,
            {},
            {{0x2000, 0x00}, {0x2001, 0x00}, {0x2002, 0x00}, {0x2003, 0x00}},
            9,
            0x0104,
            0x00000000u,
            true,
            false,
            {
                {microlind::BusAccessType::Read, 0x0100, 0x10, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0101, 0xFD, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0102, 0x20, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x0103, 0x00, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Write, 0x2000, 0x00, microlind::BusCycleKind::OperandWrite},
                {microlind::BusAccessType::Write, 0x2001, 0x00, microlind::BusCycleKind::OperandWrite},
                {microlind::BusAccessType::Write, 0x2002, 0x00, microlind::BusCycleKind::OperandWrite},
                {microlind::BusAccessType::Write, 0x2003, 0x00, microlind::BusCycleKind::OperandWrite},
            },
        },
    };

    for (const auto& test : cases) {
        SCOPED_TRACE(test.name);
        microlind::Simulator sim(microlind::CpuMode::HD6309, 1'000'000);
        auto memory = std::make_unique<SideEffectMemory>();
        auto* probe = memory.get();
        for (std::size_t i = 0; i < test.program.size(); ++i) {
            probe->data[static_cast<uint16_t>(0x0100 + i)] = test.program[i];
        }
        for (const auto& [address, value] : test.memory) {
            probe->data[address] = value;
        }
        ASSERT_FALSE(sim.map_device(0x0000, 0xFFFF, std::move(memory)));

        sim.cpu().set_pc(0x0100);
        sim.cpu().regs().dp = test.dp;
        sim.cpu().regs().a = static_cast<uint8_t>((test.initial_q >> 24) & 0xFF);
        sim.cpu().regs().b = static_cast<uint8_t>((test.initial_q >> 16) & 0xFF);
        sim.cpu().regs().e = static_cast<uint8_t>((test.initial_q >> 8) & 0xFF);
        sim.cpu().regs().f = static_cast<uint8_t>(test.initial_q & 0xFF);

        for (std::size_t i = 0; i < test.expected_accesses.size(); ++i) {
            const auto result = sim.tick_microcycle();
            EXPECT_TRUE(result.emitted);
            EXPECT_EQ(result.instruction_started, i == 0);
            EXPECT_EQ(result.instruction_complete, i + 1 == test.expected_cycles);
            EXPECT_EQ(result.instruction_result.cycles, test.expected_cycles);
        }

        const uint8_t internal_cycles = static_cast<uint8_t>(test.expected_cycles - test.expected_accesses.size());
        if (internal_cycles > 0) {
            expect_internal_cycles(sim, internal_cycles);
        } else {
            EXPECT_FALSE(sim.has_pending_microcycles());
        }

        const uint32_t actual_q =
            (static_cast<uint32_t>(sim.cpu().regs().a) << 24) |
            (static_cast<uint32_t>(sim.cpu().regs().b) << 16) |
            (static_cast<uint32_t>(sim.cpu().regs().e) << 8) |
            sim.cpu().regs().f;
        EXPECT_EQ(sim.cpu().regs().pc, test.expected_pc);
        EXPECT_EQ(actual_q, test.expected_q);
        EXPECT_EQ((sim.cpu().regs().cc & microlind::CC_Z) != 0, test.expect_z);
        EXPECT_EQ((sim.cpu().regs().cc & microlind::CC_N) != 0, test.expect_n);
        for (const auto& [address, value] : test.expected_memory) {
            EXPECT_EQ(probe->data[address], value);
        }
        ASSERT_THAT(sim.bus().access_log(), testing::SizeIs(test.expected_accesses.size()));
        for (std::size_t i = 0; i < test.expected_accesses.size(); ++i) {
            SCOPED_TRACE(i);
            EXPECT_EQ(sim.bus().access_log()[i].type, test.expected_accesses[i].type);
            EXPECT_EQ(sim.bus().access_log()[i].address, test.expected_accesses[i].address);
            EXPECT_EQ(sim.bus().access_log()[i].value, test.expected_accesses[i].value);
            EXPECT_EQ(sim.bus().access_log()[i].cycle_kind, test.expected_accesses[i].cycle_kind);
        }
    }
}

TEST(BusPhaseTest, ResumableMicrocycleIndexedWAndQLoadStoreModes) {
    struct ExpectedAccess {
        microlind::BusAccessType type{};
        uint16_t address{};
        uint8_t value{};
        microlind::BusCycleKind cycle_kind{};
    };

    struct Case {
        const char* name;
        std::vector<uint8_t> program;
        uint16_t x{};
        uint16_t y{};
        uint16_t s{};
        uint8_t a{};
        uint8_t b{};
        uint8_t e{};
        uint8_t f{};
        std::vector<std::pair<uint16_t, uint8_t>> memory;
        std::vector<std::pair<uint16_t, uint8_t>> expected_memory;
        uint8_t expected_cycles{};
        uint16_t expected_pc{};
        uint16_t expected_x{};
        uint16_t expected_y{};
        uint16_t expected_s{};
        uint8_t expected_a{};
        uint8_t expected_b{};
        uint8_t expected_e{};
        uint8_t expected_f{};
        bool expect_z{};
        bool expect_n{};
        std::vector<ExpectedAccess> expected_accesses;
    };

    const Case cases[] = {
        {
            "LDW ,X",
            {0x10, 0xA6, 0x84},
            0x2000,
            0x2100,
            0x4000,
            0x00,
            0x00,
            0x00,
            0x00,
            {{0x2000, 0x12}, {0x2001, 0x34}},
            {},
            6,
            0x0103,
            0x2000,
            0x2100,
            0x4000,
            0x00,
            0x00,
            0x12,
            0x34,
            false,
            false,
            {
                {microlind::BusAccessType::Read, 0x0100, 0x10, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0101, 0xA6, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0102, 0x84, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x2000, 0x12, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x2001, 0x34, microlind::BusCycleKind::OperandRead},
            },
        },
        {
            "STW 8-bit offset,Y",
            {0x10, 0xA7, 0xA8, 0x02},
            0x2000,
            0x2100,
            0x4000,
            0x00,
            0x00,
            0xBE,
            0xEF,
            {},
            {{0x2102, 0xBE}, {0x2103, 0xEF}},
            7,
            0x0104,
            0x2000,
            0x2100,
            0x4000,
            0x00,
            0x00,
            0xBE,
            0xEF,
            false,
            true,
            {
                {microlind::BusAccessType::Read, 0x0100, 0x10, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0101, 0xA7, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0102, 0xA8, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x0103, 0x02, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Write, 0x2102, 0xBE, microlind::BusCycleKind::OperandWrite},
                {microlind::BusAccessType::Write, 0x2103, 0xEF, microlind::BusCycleKind::OperandWrite},
            },
        },
        {
            "LDQ [,X]",
            {0x10, 0xEC, 0x94},
            0x2000,
            0x2100,
            0x4000,
            0x00,
            0x00,
            0x00,
            0x00,
            {{0x2000, 0x30}, {0x2001, 0x00}, {0x3000, 0x80}, {0x3001, 0x00}, {0x3002, 0x00}, {0x3003, 0x01}},
            {},
            11,
            0x0103,
            0x2000,
            0x2100,
            0x4000,
            0x80,
            0x00,
            0x00,
            0x01,
            false,
            true,
            {
                {microlind::BusAccessType::Read, 0x0100, 0x10, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0101, 0xEC, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0102, 0x94, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x2000, 0x30, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x2001, 0x00, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x3000, 0x80, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x3001, 0x00, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x3002, 0x00, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x3003, 0x01, microlind::BusCycleKind::OperandRead},
            },
        },
        {
            "STQ [16-bit offset,S]",
            {0x10, 0xED, 0xF9, 0x00, 0x10},
            0x2000,
            0x2100,
            0x4000,
            0x12,
            0x34,
            0x56,
            0x78,
            {{0x4010, 0x30}, {0x4011, 0x00}},
            {{0x3000, 0x12}, {0x3001, 0x34}, {0x3002, 0x56}, {0x3003, 0x78}},
            15,
            0x0105,
            0x2000,
            0x2100,
            0x4000,
            0x12,
            0x34,
            0x56,
            0x78,
            false,
            false,
            {
                {microlind::BusAccessType::Read, 0x0100, 0x10, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0101, 0xED, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0102, 0xF9, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x0103, 0x00, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x0104, 0x10, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x4010, 0x30, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x4011, 0x00, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Write, 0x3000, 0x12, microlind::BusCycleKind::OperandWrite},
                {microlind::BusAccessType::Write, 0x3001, 0x34, microlind::BusCycleKind::OperandWrite},
                {microlind::BusAccessType::Write, 0x3002, 0x56, microlind::BusCycleKind::OperandWrite},
                {microlind::BusAccessType::Write, 0x3003, 0x78, microlind::BusCycleKind::OperandWrite},
            },
        },
    };

    for (const auto& test : cases) {
        SCOPED_TRACE(test.name);
        microlind::Simulator sim(microlind::CpuMode::HD6309, 1'000'000);
        auto memory = std::make_unique<SideEffectMemory>();
        auto* probe = memory.get();
        for (std::size_t i = 0; i < test.program.size(); ++i) {
            probe->data[static_cast<uint16_t>(0x0100 + i)] = test.program[i];
        }
        for (const auto& [address, value] : test.memory) {
            probe->data[address] = value;
        }
        ASSERT_FALSE(sim.map_device(0x0000, 0xFFFF, std::move(memory)));

        sim.cpu().set_pc(0x0100);
        sim.cpu().regs().x = test.x;
        sim.cpu().regs().y = test.y;
        sim.cpu().regs().s = test.s;
        sim.cpu().regs().a = test.a;
        sim.cpu().regs().b = test.b;
        sim.cpu().regs().e = test.e;
        sim.cpu().regs().f = test.f;

        for (std::size_t i = 0; i < test.expected_accesses.size(); ++i) {
            const auto result = sim.tick_microcycle();
            EXPECT_TRUE(result.emitted);
            EXPECT_EQ(result.instruction_started, i == 0);
            EXPECT_EQ(result.instruction_complete, i + 1 == test.expected_cycles);
            EXPECT_EQ(result.instruction_result.cycles, test.expected_cycles);
        }

        const uint8_t internal_cycles = static_cast<uint8_t>(test.expected_cycles - test.expected_accesses.size());
        if (internal_cycles > 0) {
            expect_internal_cycles(sim, internal_cycles);
        } else {
            EXPECT_FALSE(sim.has_pending_microcycles());
        }

        EXPECT_EQ(sim.cpu().regs().pc, test.expected_pc);
        EXPECT_EQ(sim.cpu().regs().x, test.expected_x);
        EXPECT_EQ(sim.cpu().regs().y, test.expected_y);
        EXPECT_EQ(sim.cpu().regs().s, test.expected_s);
        EXPECT_EQ(sim.cpu().regs().a, test.expected_a);
        EXPECT_EQ(sim.cpu().regs().b, test.expected_b);
        EXPECT_EQ(sim.cpu().regs().e, test.expected_e);
        EXPECT_EQ(sim.cpu().regs().f, test.expected_f);
        EXPECT_EQ((sim.cpu().regs().cc & microlind::CC_Z) != 0, test.expect_z);
        EXPECT_EQ((sim.cpu().regs().cc & microlind::CC_N) != 0, test.expect_n);
        for (const auto& [address, value] : test.expected_memory) {
            EXPECT_EQ(probe->data[address], value);
        }
        ASSERT_THAT(sim.bus().access_log(), testing::SizeIs(test.expected_accesses.size()));
        for (std::size_t i = 0; i < test.expected_accesses.size(); ++i) {
            SCOPED_TRACE(i);
            EXPECT_EQ(sim.bus().access_log()[i].type, test.expected_accesses[i].type);
            EXPECT_EQ(sim.bus().access_log()[i].address, test.expected_accesses[i].address);
            EXPECT_EQ(sim.bus().access_log()[i].value, test.expected_accesses[i].value);
            EXPECT_EQ(sim.bus().access_log()[i].cycle_kind, test.expected_accesses[i].cycle_kind);
        }
    }
}

TEST(BusPhaseTest, ResumableMicrocycleIndexedWArithmeticModes) {
    struct ExpectedAccess {
        microlind::BusAccessType type{};
        uint16_t address{};
        uint8_t value{};
        microlind::BusCycleKind cycle_kind{};
    };

    struct Case {
        const char* name;
        std::vector<uint8_t> program;
        uint16_t x{};
        uint16_t y{};
        uint16_t s{};
        uint16_t initial_w{};
        std::vector<std::pair<uint16_t, uint8_t>> memory;
        uint8_t expected_cycles{};
        uint16_t expected_pc{};
        uint16_t expected_w{};
        bool expect_z{};
        bool expect_n{};
        bool expect_v{};
        bool expect_c{};
        std::vector<ExpectedAccess> expected_accesses;
    };

    const Case cases[] = {
        {
            "ADDW ,X",
            {0x10, 0xAB, 0x84},
            0x2000,
            0x2100,
            0x4000,
            0x0001,
            {{0x2000, 0x00}, {0x2001, 0x01}},
            7,
            0x0103,
            0x0002,
            false,
            false,
            false,
            false,
            {
                {microlind::BusAccessType::Read, 0x0100, 0x10, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0101, 0xAB, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0102, 0x84, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x2000, 0x00, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x2001, 0x01, microlind::BusCycleKind::OperandRead},
            },
        },
        {
            "SUBW 8-bit offset,Y",
            {0x10, 0xA0, 0xA8, 0x02},
            0x2000,
            0x2100,
            0x4000,
            0x0100,
            {{0x2102, 0x00}, {0x2103, 0x01}},
            8,
            0x0104,
            0x00FF,
            false,
            false,
            false,
            false,
            {
                {microlind::BusAccessType::Read, 0x0100, 0x10, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0101, 0xA0, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0102, 0xA8, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x0103, 0x02, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x2102, 0x00, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x2103, 0x01, microlind::BusCycleKind::OperandRead},
            },
        },
        {
            "CMPW [,X]",
            {0x10, 0xA1, 0x94},
            0x2000,
            0x2100,
            0x4000,
            0x1234,
            {{0x2000, 0x30}, {0x2001, 0x00}, {0x3000, 0x12}, {0x3001, 0x34}},
            10,
            0x0103,
            0x1234,
            true,
            false,
            false,
            false,
            {
                {microlind::BusAccessType::Read, 0x0100, 0x10, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0101, 0xA1, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0102, 0x94, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x2000, 0x30, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x2001, 0x00, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x3000, 0x12, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x3001, 0x34, microlind::BusCycleKind::OperandRead},
            },
        },
        {
            "ADDW [16-bit offset,S]",
            {0x10, 0xAB, 0xF9, 0x00, 0x10},
            0x2000,
            0x2100,
            0x4000,
            0x7FFF,
            {{0x4010, 0x30}, {0x4011, 0x00}, {0x3000, 0x00}, {0x3001, 0x01}},
            14,
            0x0105,
            0x8000,
            false,
            true,
            true,
            false,
            {
                {microlind::BusAccessType::Read, 0x0100, 0x10, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0101, 0xAB, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0102, 0xF9, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x0103, 0x00, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x0104, 0x10, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x4010, 0x30, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x4011, 0x00, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x3000, 0x00, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x3001, 0x01, microlind::BusCycleKind::OperandRead},
            },
        },
    };

    for (const auto& test : cases) {
        SCOPED_TRACE(test.name);
        microlind::Simulator sim(microlind::CpuMode::HD6309, 1'000'000);
        auto memory = std::make_unique<SideEffectMemory>();
        auto* probe = memory.get();
        for (std::size_t i = 0; i < test.program.size(); ++i) {
            probe->data[static_cast<uint16_t>(0x0100 + i)] = test.program[i];
        }
        for (const auto& [address, value] : test.memory) {
            probe->data[address] = value;
        }
        ASSERT_FALSE(sim.map_device(0x0000, 0xFFFF, std::move(memory)));

        sim.cpu().set_pc(0x0100);
        sim.cpu().regs().x = test.x;
        sim.cpu().regs().y = test.y;
        sim.cpu().regs().s = test.s;
        sim.cpu().regs().e = static_cast<uint8_t>(test.initial_w >> 8);
        sim.cpu().regs().f = static_cast<uint8_t>(test.initial_w & 0xFF);

        for (std::size_t i = 0; i < test.expected_accesses.size(); ++i) {
            const auto result = sim.tick_microcycle();
            EXPECT_TRUE(result.emitted);
            EXPECT_EQ(result.instruction_started, i == 0);
            EXPECT_EQ(result.instruction_complete, i + 1 == test.expected_cycles);
            EXPECT_EQ(result.instruction_result.cycles, test.expected_cycles);
        }

        const uint8_t internal_cycles = static_cast<uint8_t>(test.expected_cycles - test.expected_accesses.size());
        if (internal_cycles > 0) {
            expect_internal_cycles(sim, internal_cycles);
        } else {
            EXPECT_FALSE(sim.has_pending_microcycles());
        }

        const uint16_t actual_w = static_cast<uint16_t>((static_cast<uint16_t>(sim.cpu().regs().e) << 8) | sim.cpu().regs().f);
        EXPECT_EQ(sim.cpu().regs().pc, test.expected_pc);
        EXPECT_EQ(sim.cpu().regs().x, test.x);
        EXPECT_EQ(sim.cpu().regs().y, test.y);
        EXPECT_EQ(sim.cpu().regs().s, test.s);
        EXPECT_EQ(actual_w, test.expected_w);
        EXPECT_EQ((sim.cpu().regs().cc & microlind::CC_Z) != 0, test.expect_z);
        EXPECT_EQ((sim.cpu().regs().cc & microlind::CC_N) != 0, test.expect_n);
        EXPECT_EQ((sim.cpu().regs().cc & microlind::CC_V) != 0, test.expect_v);
        EXPECT_EQ((sim.cpu().regs().cc & microlind::CC_C) != 0, test.expect_c);
        ASSERT_THAT(sim.bus().access_log(), testing::SizeIs(test.expected_accesses.size()));
        for (std::size_t i = 0; i < test.expected_accesses.size(); ++i) {
            SCOPED_TRACE(i);
            EXPECT_EQ(sim.bus().access_log()[i].type, test.expected_accesses[i].type);
            EXPECT_EQ(sim.bus().access_log()[i].address, test.expected_accesses[i].address);
            EXPECT_EQ(sim.bus().access_log()[i].value, test.expected_accesses[i].value);
            EXPECT_EQ(sim.bus().access_log()[i].cycle_kind, test.expected_accesses[i].cycle_kind);
        }
    }
}

TEST(BusPhaseTest, ResumableMicrocycleHD6309ModeAndSignExtendInstructions) {
    struct ExpectedAccess {
        microlind::BusAccessType type{};
        uint16_t address{};
        uint8_t value{};
        microlind::BusCycleKind cycle_kind{};
    };

    struct Case {
        const char* name{};
        std::vector<uint8_t> program;
        uint8_t initial_a{};
        uint8_t initial_b{};
        uint8_t initial_e{};
        uint8_t initial_f{};
        uint8_t initial_md{};
        uint8_t initial_cc{};
        uint8_t expected_cycles{};
        uint16_t expected_pc{};
        uint8_t expected_a{};
        uint8_t expected_b{};
        uint8_t expected_e{};
        uint8_t expected_f{};
        uint8_t expected_md{};
        bool expect_z{};
        bool expect_n{};
        bool expect_v{};
        bool expect_c{};
        std::vector<ExpectedAccess> expected_accesses;
    };

    const Case cases[] = {
        {
            "SEXW",
            {0x14},
            0x00,
            0x00,
            0x80,
            0x01,
            0x00,
            microlind::CC_Z,
            4,
            0x0101,
            0xFF,
            0xFF,
            0x80,
            0x01,
            0x00,
            false,
            true,
            false,
            false,
            {
                {microlind::BusAccessType::Read, 0x0100, 0x14, microlind::BusCycleKind::OpcodeFetch},
            },
        },
        {
            "BITMD",
            {0x11, 0x3C, 0x80},
            0x00,
            0x00,
            0x00,
            0x00,
            0x80,
            microlind::CC_C,
            4,
            0x0103,
            0x00,
            0x00,
            0x00,
            0x00,
            0x80,
            false,
            true,
            false,
            true,
            {
                {microlind::BusAccessType::Read, 0x0100, 0x11, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0101, 0x3C, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0102, 0x80, microlind::BusCycleKind::OperandRead},
            },
        },
        {
            "LDMD",
            {0x11, 0x3D, 0x01},
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            5,
            0x0103,
            0x00,
            0x00,
            0x00,
            0x00,
            0x01,
            false,
            false,
            false,
            false,
            {
                {microlind::BusAccessType::Read, 0x0100, 0x11, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0101, 0x3D, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0102, 0x01, microlind::BusCycleKind::OperandRead},
            },
        },
    };

    for (const auto& test : cases) {
        SCOPED_TRACE(test.name);
        microlind::Simulator sim(microlind::CpuMode::HD6309, 1'000'000);
        auto memory = std::make_unique<SideEffectMemory>();
        auto* probe = memory.get();
        for (std::size_t i = 0; i < test.program.size(); ++i) {
            probe->data[static_cast<uint16_t>(0x0100 + i)] = test.program[i];
        }
        ASSERT_FALSE(sim.map_device(0x0000, 0xFFFF, std::move(memory)));

        sim.cpu().set_pc(0x0100);
        sim.cpu().regs().a = test.initial_a;
        sim.cpu().regs().b = test.initial_b;
        sim.cpu().regs().e = test.initial_e;
        sim.cpu().regs().f = test.initial_f;
        sim.cpu().regs().md = test.initial_md;
        sim.cpu().regs().cc = test.initial_cc;

        for (std::size_t i = 0; i < test.expected_accesses.size(); ++i) {
            const auto result = sim.tick_microcycle();
            EXPECT_TRUE(result.emitted);
            EXPECT_EQ(result.instruction_started, i == 0);
            EXPECT_EQ(result.instruction_complete, i + 1 == test.expected_cycles);
            EXPECT_EQ(result.instruction_result.cycles, test.expected_cycles);
        }

        const uint8_t internal_cycles = static_cast<uint8_t>(test.expected_cycles - test.expected_accesses.size());
        if (internal_cycles > 0) {
            expect_internal_cycles(sim, internal_cycles);
        } else {
            EXPECT_FALSE(sim.has_pending_microcycles());
        }

        EXPECT_EQ(sim.cpu().regs().pc, test.expected_pc);
        EXPECT_EQ(sim.cpu().regs().a, test.expected_a);
        EXPECT_EQ(sim.cpu().regs().b, test.expected_b);
        EXPECT_EQ(sim.cpu().regs().e, test.expected_e);
        EXPECT_EQ(sim.cpu().regs().f, test.expected_f);
        EXPECT_EQ(sim.cpu().regs().md, test.expected_md);
        EXPECT_EQ((sim.cpu().regs().cc & microlind::CC_Z) != 0, test.expect_z);
        EXPECT_EQ((sim.cpu().regs().cc & microlind::CC_N) != 0, test.expect_n);
        EXPECT_EQ((sim.cpu().regs().cc & microlind::CC_V) != 0, test.expect_v);
        EXPECT_EQ((sim.cpu().regs().cc & microlind::CC_C) != 0, test.expect_c);
        ASSERT_THAT(sim.bus().access_log(), testing::SizeIs(test.expected_accesses.size()));
        for (std::size_t i = 0; i < test.expected_accesses.size(); ++i) {
            SCOPED_TRACE(i);
            EXPECT_EQ(sim.bus().access_log()[i].type, test.expected_accesses[i].type);
            EXPECT_EQ(sim.bus().access_log()[i].address, test.expected_accesses[i].address);
            EXPECT_EQ(sim.bus().access_log()[i].value, test.expected_accesses[i].value);
            EXPECT_EQ(sim.bus().access_log()[i].cycle_kind, test.expected_accesses[i].cycle_kind);
        }
    }
}

TEST(BusPhaseTest, ResumableMicrocycleHD6309BitTransferInstructions) {
    struct ExpectedAccess {
        microlind::BusAccessType type{};
        uint16_t address{};
        uint8_t value{};
        microlind::BusCycleKind cycle_kind{};
    };

    struct Case {
        const char* name{};
        std::vector<uint8_t> program;
        uint8_t dp{};
        uint8_t initial_a{};
        uint8_t initial_b{};
        uint8_t initial_cc{};
        std::vector<std::pair<uint16_t, uint8_t>> memory;
        std::vector<std::pair<uint16_t, uint8_t>> expected_memory;
        uint8_t expected_cycles{};
        uint8_t expected_a{};
        uint8_t expected_b{};
        uint8_t expected_cc{};
        uint32_t expected_writes{};
        std::vector<ExpectedAccess> expected_accesses;
    };

    const Case cases[] = {
        {
            "BOR A bit1 into memory bit0",
            {0x11, 0x32, 0x48, 0x20},
            0x12,
            0x02,
            0x00,
            microlind::CC_C,
            {{0x1220, 0x00}},
            {{0x1220, 0x01}},
            7,
            0x02,
            0x00,
            microlind::CC_C,
            1,
            {
                {microlind::BusAccessType::Read, 0x0100, 0x11, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0101, 0x32, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0102, 0x48, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x0103, 0x20, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x1220, 0x00, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Write, 0x1220, 0x01, microlind::BusCycleKind::OperandWrite},
            },
        },
        {
            "LDBT memory bit7 into B bit0",
            {0x11, 0x36, 0xB8, 0x20},
            0x12,
            0x00,
            0x00,
            microlind::CC_C,
            {{0x1220, 0x80}},
            {},
            7,
            0x00,
            0x01,
            microlind::CC_C,
            0,
            {
                {microlind::BusAccessType::Read, 0x0100, 0x11, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0101, 0x36, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0102, 0xB8, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x0103, 0x20, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x1220, 0x80, microlind::BusCycleKind::OperandRead},
            },
        },
        {
            "STBT CC carry into memory bit7",
            {0x11, 0x37, 0x07, 0x20},
            0x12,
            0x00,
            0x00,
            microlind::CC_C,
            {{0x1220, 0x00}},
            {{0x1220, 0x80}},
            8,
            0x00,
            0x00,
            microlind::CC_C,
            1,
            {
                {microlind::BusAccessType::Read, 0x0100, 0x11, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0101, 0x37, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0102, 0x07, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x0103, 0x20, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x1220, 0x00, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Write, 0x1220, 0x80, microlind::BusCycleKind::OperandWrite},
            },
        },
    };

    for (const auto& test : cases) {
        SCOPED_TRACE(test.name);
        microlind::Simulator sim(microlind::CpuMode::HD6309, 1'000'000);
        auto memory = std::make_unique<SideEffectMemory>();
        auto* probe = memory.get();
        for (std::size_t i = 0; i < test.program.size(); ++i) {
            probe->data[static_cast<uint16_t>(0x0100 + i)] = test.program[i];
        }
        for (const auto& [address, value] : test.memory) {
            probe->data[address] = value;
        }
        ASSERT_FALSE(sim.map_device(0x0000, 0xFFFF, std::move(memory)));

        sim.cpu().set_pc(0x0100);
        sim.cpu().regs().dp = test.dp;
        sim.cpu().regs().a = test.initial_a;
        sim.cpu().regs().b = test.initial_b;
        sim.cpu().regs().cc = test.initial_cc;

        for (std::size_t i = 0; i < test.expected_accesses.size(); ++i) {
            const auto result = sim.tick_microcycle();
            EXPECT_TRUE(result.emitted);
            EXPECT_EQ(result.instruction_started, i == 0);
            EXPECT_EQ(result.instruction_complete, i + 1 == test.expected_cycles);
            EXPECT_EQ(result.instruction_result.cycles, test.expected_cycles);
        }

        const uint8_t internal_cycles = static_cast<uint8_t>(test.expected_cycles - test.expected_accesses.size());
        if (internal_cycles > 0) {
            expect_internal_cycles(sim, internal_cycles);
        } else {
            EXPECT_FALSE(sim.has_pending_microcycles());
        }

        EXPECT_EQ(sim.cpu().regs().pc, 0x0104);
        EXPECT_EQ(sim.cpu().regs().a, test.expected_a);
        EXPECT_EQ(sim.cpu().regs().b, test.expected_b);
        EXPECT_EQ(sim.cpu().regs().cc, test.expected_cc);
        EXPECT_EQ(probe->write_count, test.expected_writes);
        for (const auto& [address, value] : test.expected_memory) {
            EXPECT_EQ(probe->data[address], value);
        }
        ASSERT_THAT(sim.bus().access_log(), testing::SizeIs(test.expected_accesses.size()));
        for (std::size_t i = 0; i < test.expected_accesses.size(); ++i) {
            SCOPED_TRACE(i);
            EXPECT_EQ(sim.bus().access_log()[i].type, test.expected_accesses[i].type);
            EXPECT_EQ(sim.bus().access_log()[i].address, test.expected_accesses[i].address);
            EXPECT_EQ(sim.bus().access_log()[i].value, test.expected_accesses[i].value);
            EXPECT_EQ(sim.bus().access_log()[i].cycle_kind, test.expected_accesses[i].cycle_kind);
        }
    }
}

TEST(BusPhaseTest, ResumableMicrocycleHD6309DivideInstructions) {
    struct ExpectedAccess {
        microlind::BusAccessType type{};
        uint16_t address{};
        uint8_t value{};
        microlind::BusCycleKind cycle_kind{};
    };

    struct Case {
        const char* name{};
        std::vector<uint8_t> program;
        uint8_t dp{};
        uint16_t x{};
        uint32_t initial_q{};
        uint8_t initial_md{};
        std::vector<std::pair<uint16_t, uint8_t>> memory;
        uint32_t expected_q{};
        uint8_t expected_md{};
        uint8_t expected_cycles{};
        uint16_t expected_pc{};
        bool expect_z{};
        bool expect_n{};
        bool expect_v{};
        bool expect_c{};
        std::vector<ExpectedAccess> expected_accesses;
    };

    const Case cases[] = {
        {
            "DIVD immediate",
            {0x11, 0x8D, 0x05},
            0x12,
            0x2000,
            0x00140000,
            0x00,
            {},
            0x00000004,
            0x00,
            25,
            0x0103,
            false,
            false,
            false,
            false,
            {
                {microlind::BusAccessType::Read, 0x0100, 0x11, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0101, 0x8D, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0102, 0x05, microlind::BusCycleKind::OperandRead},
            },
        },
        {
            "DIVQ direct",
            {0x11, 0x9E, 0x20},
            0x12,
            0x2000,
            0x00000014,
            0x00,
            {{0x1220, 0x00}, {0x1221, 0x05}},
            0x00000004,
            0x00,
            36,
            0x0103,
            false,
            false,
            false,
            false,
            {
                {microlind::BusAccessType::Read, 0x0100, 0x11, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0101, 0x9E, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0102, 0x20, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x1220, 0x00, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x1221, 0x05, microlind::BusCycleKind::OperandRead},
            },
        },
        {
            "DIVD indexed zero divisor trap",
            {0x11, 0xAD, 0x84},
            0x12,
            0x2000,
            0x00140000,
            0x00,
            {{0x2000, 0x00}, {0xFFF0, 0x12}, {0xFFF1, 0x34}},
            0x00140000,
            0x80,
            27,
            0x1234,
            false,
            false,
            false,
            false,
            {
                {microlind::BusAccessType::Read, 0x0100, 0x11, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0101, 0xAD, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0102, 0x84, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x2000, 0x00, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0xFFF0, 0x12, microlind::BusCycleKind::VectorRead},
                {microlind::BusAccessType::Read, 0xFFF1, 0x34, microlind::BusCycleKind::VectorRead},
            },
        },
    };

    for (const auto& test : cases) {
        SCOPED_TRACE(test.name);
        microlind::Simulator sim(microlind::CpuMode::HD6309, 1'000'000);
        auto memory = std::make_unique<SideEffectMemory>();
        auto* probe = memory.get();
        for (std::size_t i = 0; i < test.program.size(); ++i) {
            probe->data[static_cast<uint16_t>(0x0100 + i)] = test.program[i];
        }
        for (const auto& [address, value] : test.memory) {
            probe->data[address] = value;
        }
        ASSERT_FALSE(sim.map_device(0x0000, 0xFFFF, std::move(memory)));

        sim.cpu().set_pc(0x0100);
        sim.cpu().regs().dp = test.dp;
        sim.cpu().regs().x = test.x;
        sim.cpu().regs().a = static_cast<uint8_t>((test.initial_q >> 24) & 0xFF);
        sim.cpu().regs().b = static_cast<uint8_t>((test.initial_q >> 16) & 0xFF);
        sim.cpu().regs().e = static_cast<uint8_t>((test.initial_q >> 8) & 0xFF);
        sim.cpu().regs().f = static_cast<uint8_t>(test.initial_q & 0xFF);
        sim.cpu().regs().md = test.initial_md;

        for (std::size_t i = 0; i < test.expected_accesses.size(); ++i) {
            const auto result = sim.tick_microcycle();
            EXPECT_TRUE(result.emitted);
            EXPECT_EQ(result.instruction_started, i == 0);
            EXPECT_EQ(result.instruction_complete, i + 1 == test.expected_cycles);
            EXPECT_EQ(result.instruction_result.cycles, test.expected_cycles);
        }

        const auto internal_cycles = static_cast<uint32_t>(test.expected_cycles - test.expected_accesses.size());
        if (internal_cycles > 0) {
            expect_internal_cycles(sim, static_cast<uint8_t>(internal_cycles));
        } else {
            EXPECT_FALSE(sim.has_pending_microcycles());
        }

        const uint32_t actual_q =
            (static_cast<uint32_t>((static_cast<uint16_t>(sim.cpu().regs().a) << 8) | sim.cpu().regs().b) << 16) |
            static_cast<uint32_t>((static_cast<uint16_t>(sim.cpu().regs().e) << 8) | sim.cpu().regs().f);
        EXPECT_EQ(actual_q, test.expected_q);
        EXPECT_EQ(sim.cpu().regs().md, test.expected_md);
        EXPECT_EQ(sim.cpu().regs().pc, test.expected_pc);
        EXPECT_EQ((sim.cpu().regs().cc & microlind::CC_Z) != 0, test.expect_z);
        EXPECT_EQ((sim.cpu().regs().cc & microlind::CC_N) != 0, test.expect_n);
        EXPECT_EQ((sim.cpu().regs().cc & microlind::CC_V) != 0, test.expect_v);
        EXPECT_EQ((sim.cpu().regs().cc & microlind::CC_C) != 0, test.expect_c);
        ASSERT_THAT(sim.bus().access_log(), testing::SizeIs(test.expected_accesses.size()));
        for (std::size_t i = 0; i < test.expected_accesses.size(); ++i) {
            SCOPED_TRACE(i);
            EXPECT_EQ(sim.bus().access_log()[i].type, test.expected_accesses[i].type);
            EXPECT_EQ(sim.bus().access_log()[i].address, test.expected_accesses[i].address);
            EXPECT_EQ(sim.bus().access_log()[i].value, test.expected_accesses[i].value);
            EXPECT_EQ(sim.bus().access_log()[i].cycle_kind, test.expected_accesses[i].cycle_kind);
        }
    }
}

TEST(BusPhaseTest, ResumableMicrocycleHD6309TfmInstructions) {
    struct ExpectedAccess {
        microlind::BusAccessType type{};
        uint16_t address{};
        uint8_t value{};
        microlind::BusCycleKind cycle_kind{};
    };

    struct Case {
        const char* name{};
        uint8_t opcode{};
        uint16_t x{};
        uint16_t y{};
        uint16_t w{};
        std::vector<std::pair<uint16_t, uint8_t>> memory;
        std::vector<std::pair<uint16_t, uint8_t>> expected_memory;
        uint16_t expected_x{};
        uint16_t expected_y{};
        uint16_t expected_w{};
        std::vector<ExpectedAccess> expected_accesses;
    };

    const Case cases[] = {
        {
            "TFM X+,Y+",
            0x38,
            0x2000,
            0x3000,
            0x0002,
            {{0x2000, 0xAA}, {0x2001, 0xBB}},
            {{0x2000, 0x00}, {0x2001, 0x00}, {0x3000, 0xAA}, {0x3001, 0xBB}},
            0x2002,
            0x3002,
            0x0000,
            {
                {microlind::BusAccessType::Read, 0x0100, 0x11, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0101, 0x38, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0102, 0x12, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x2000, 0xAA, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Write, 0x3000, 0xAA, microlind::BusCycleKind::OperandWrite},
                {microlind::BusAccessType::Read, 0x2001, 0xBB, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Write, 0x3001, 0xBB, microlind::BusCycleKind::OperandWrite},
            },
        },
        {
            "TFM X+,Y",
            0x3A,
            0x2000,
            0x3000,
            0x0002,
            {{0x2000, 0xAA}, {0x2001, 0xBB}},
            {{0x2000, 0x00}, {0x2001, 0x00}, {0x3000, 0xBB}},
            0x2002,
            0x3000,
            0x0000,
            {
                {microlind::BusAccessType::Read, 0x0100, 0x11, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0101, 0x3A, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0102, 0x12, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x2000, 0xAA, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Write, 0x3000, 0xAA, microlind::BusCycleKind::OperandWrite},
                {microlind::BusAccessType::Read, 0x2001, 0xBB, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Write, 0x3000, 0xBB, microlind::BusCycleKind::OperandWrite},
            },
        },
    };

    for (const auto& test : cases) {
        SCOPED_TRACE(test.name);
        microlind::Simulator sim(microlind::CpuMode::HD6309, 1'000'000);
        auto memory = std::make_unique<SideEffectMemory>();
        auto* probe = memory.get();
        probe->data[0x0100] = 0x11;
        probe->data[0x0101] = test.opcode;
        probe->data[0x0102] = 0x12;
        for (const auto& [address, value] : test.memory) {
            probe->data[address] = value;
        }
        ASSERT_FALSE(sim.map_device(0x0000, 0xFFFF, std::move(memory)));

        sim.cpu().set_pc(0x0100);
        sim.cpu().regs().x = test.x;
        sim.cpu().regs().y = test.y;
        sim.cpu().regs().e = high_byte(test.w);
        sim.cpu().regs().f = low_byte(test.w);

        const uint32_t expected_cycles = static_cast<uint32_t>(6 + 3 * test.w);
        for (std::size_t i = 0; i < test.expected_accesses.size(); ++i) {
            const auto result = sim.tick_microcycle();
            EXPECT_TRUE(result.emitted);
            EXPECT_EQ(result.instruction_started, i == 0);
            EXPECT_EQ(result.instruction_complete, i + 1 == expected_cycles);
            EXPECT_EQ(result.instruction_result.cycles, expected_cycles);
        }

        const auto internal_cycles = static_cast<uint32_t>(expected_cycles - test.expected_accesses.size());
        if (internal_cycles > 0) {
            expect_internal_cycles(sim, static_cast<uint8_t>(internal_cycles));
        } else {
            EXPECT_FALSE(sim.has_pending_microcycles());
        }

        const uint16_t actual_w =
            static_cast<uint16_t>((static_cast<uint16_t>(sim.cpu().regs().e) << 8) | sim.cpu().regs().f);
        EXPECT_EQ(sim.cpu().regs().pc, 0x0103);
        EXPECT_EQ(sim.cpu().regs().x, test.expected_x);
        EXPECT_EQ(sim.cpu().regs().y, test.expected_y);
        EXPECT_EQ(actual_w, test.expected_w);
        for (const auto& [address, value] : test.expected_memory) {
            EXPECT_EQ(probe->data[address], value);
        }
        ASSERT_THAT(sim.bus().access_log(), testing::SizeIs(test.expected_accesses.size()));
        for (std::size_t i = 0; i < test.expected_accesses.size(); ++i) {
            SCOPED_TRACE(i);
            EXPECT_EQ(sim.bus().access_log()[i].type, test.expected_accesses[i].type);
            EXPECT_EQ(sim.bus().access_log()[i].address, test.expected_accesses[i].address);
            EXPECT_EQ(sim.bus().access_log()[i].value, test.expected_accesses[i].value);
            EXPECT_EQ(sim.bus().access_log()[i].cycle_kind, test.expected_accesses[i].cycle_kind);
        }
    }
}

TEST(BusPhaseTest, ResumableMicrocycleHD6309RegisterAluInstructions) {
    struct ExpectedAccess {
        microlind::BusAccessType type{};
        uint16_t address{};
        uint8_t value{};
        microlind::BusCycleKind cycle_kind{};
    };

    struct Case {
        const char* name{};
        uint8_t opcode{};
        uint16_t initial_d{};
        uint16_t initial_x{};
        uint8_t initial_cc{};
        uint16_t expected_x{};
        bool expect_z{};
        bool expect_n{};
        bool expect_v{};
        bool expect_c{};
    };

    const Case cases[] = {
        {"ADDR", 0x30, 0x0001, 0x7FFF, 0x00, 0x8000, false, true, true, false},
        {"ADCR", 0x31, 0x0001, 0x0001, microlind::CC_C, 0x0003, false, false, false, false},
        {"SUBR", 0x32, 0x0001, 0x0000, 0x00, 0xFFFF, false, true, false, true},
        {"SBCR", 0x33, 0x0001, 0x0001, microlind::CC_C, 0xFFFF, false, true, false, true},
        {"ANDR", 0x34, 0x0F0F, 0xFF00, microlind::CC_C, 0x0F00, false, false, false, true},
        {"ORR", 0x35, 0x8001, 0x0000, 0x00, 0x8001, false, true, false, false},
        {"EORR", 0x36, 0xFFFF, 0xFFFF, 0x00, 0x0000, true, false, false, false},
        {"CMPR", 0x37, 0x0001, 0x0001, 0x00, 0x0001, true, false, false, false},
    };

    for (const auto& test : cases) {
        SCOPED_TRACE(test.name);
        microlind::Simulator sim(microlind::CpuMode::HD6309, 1'000'000);
        auto memory = std::make_unique<SideEffectMemory>();
        auto* probe = memory.get();
        probe->data[0x0100] = 0x10;
        probe->data[0x0101] = test.opcode;
        probe->data[0x0102] = 0x01; // source D, destination X
        ASSERT_FALSE(sim.map_device(0x0000, 0xFFFF, std::move(memory)));

        sim.cpu().set_pc(0x0100);
        sim.cpu().regs().a = high_byte(test.initial_d);
        sim.cpu().regs().b = low_byte(test.initial_d);
        sim.cpu().regs().x = test.initial_x;
        sim.cpu().regs().cc = test.initial_cc;

        constexpr uint8_t expected_cycles = 4;
        const ExpectedAccess expected_accesses[] = {
            {microlind::BusAccessType::Read, 0x0100, 0x10, microlind::BusCycleKind::OpcodeFetch},
            {microlind::BusAccessType::Read, 0x0101, test.opcode, microlind::BusCycleKind::OpcodeFetch},
            {microlind::BusAccessType::Read, 0x0102, 0x01, microlind::BusCycleKind::OperandRead},
        };

        for (std::size_t i = 0; i < std::size(expected_accesses); ++i) {
            const auto result = sim.tick_microcycle();
            EXPECT_TRUE(result.emitted);
            EXPECT_EQ(result.instruction_started, i == 0);
            EXPECT_EQ(result.instruction_complete, i + 1 == expected_cycles);
            EXPECT_EQ(result.instruction_result.cycles, expected_cycles);
        }

        expect_internal_cycles(sim, 1);

        EXPECT_EQ(sim.cpu().regs().pc, 0x0103);
        EXPECT_EQ(sim.cpu().regs().x, test.expected_x);
        EXPECT_EQ((sim.cpu().regs().cc & microlind::CC_Z) != 0, test.expect_z);
        EXPECT_EQ((sim.cpu().regs().cc & microlind::CC_N) != 0, test.expect_n);
        EXPECT_EQ((sim.cpu().regs().cc & microlind::CC_V) != 0, test.expect_v);
        EXPECT_EQ((sim.cpu().regs().cc & microlind::CC_C) != 0, test.expect_c);
        ASSERT_THAT(sim.bus().access_log(), testing::SizeIs(std::size(expected_accesses)));
        for (std::size_t i = 0; i < std::size(expected_accesses); ++i) {
            SCOPED_TRACE(i);
            EXPECT_EQ(sim.bus().access_log()[i].type, expected_accesses[i].type);
            EXPECT_EQ(sim.bus().access_log()[i].address, expected_accesses[i].address);
            EXPECT_EQ(sim.bus().access_log()[i].value, expected_accesses[i].value);
            EXPECT_EQ(sim.bus().access_log()[i].cycle_kind, expected_accesses[i].cycle_kind);
        }
    }
}

TEST(BusPhaseTest, ResumableMicrocycleHD6309DAluInstructions) {
    struct ExpectedAccess {
        microlind::BusAccessType type{};
        uint16_t address{};
        uint8_t value{};
        microlind::BusCycleKind cycle_kind{};
    };

    struct Case {
        const char* name{};
        std::vector<uint8_t> program;
        uint8_t dp{};
        uint16_t x{};
        uint16_t initial_d{};
        uint8_t initial_cc{};
        std::vector<std::pair<uint16_t, uint8_t>> memory;
        uint8_t expected_cycles{};
        uint16_t expected_pc{};
        uint16_t expected_d{};
        bool expect_z{};
        bool expect_n{};
        bool expect_v{};
        bool expect_c{};
        std::vector<ExpectedAccess> expected_accesses;
    };

    const Case cases[] = {
        {
            "ANDD immediate",
            {0x10, 0x84, 0x0F, 0x0F},
            0x12,
            0x2000,
            0xF0F0,
            microlind::CC_C,
            {},
            5,
            0x0104,
            0x0000,
            true,
            false,
            false,
            false,
            {
                {microlind::BusAccessType::Read, 0x0100, 0x10, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0101, 0x84, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0102, 0x0F, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x0103, 0x0F, microlind::BusCycleKind::OperandRead},
            },
        },
        {
            "BITD direct",
            {0x10, 0x95, 0x20},
            0x12,
            0x2000,
            0x8001,
            microlind::CC_C,
            {{0x1220, 0x80}, {0x1221, 0x00}},
            7,
            0x0103,
            0x8001,
            false,
            true,
            false,
            true,
            {
                {microlind::BusAccessType::Read, 0x0100, 0x10, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0101, 0x95, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0102, 0x20, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x1220, 0x80, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x1221, 0x00, microlind::BusCycleKind::OperandRead},
            },
        },
        {
            "ADCD extended",
            {0x10, 0xB9, 0x20, 0x00},
            0x12,
            0x2000,
            0x7FFF,
            microlind::CC_C,
            {{0x2000, 0x00}, {0x2001, 0x00}},
            8,
            0x0104,
            0x8000,
            false,
            true,
            true,
            false,
            {
                {microlind::BusAccessType::Read, 0x0100, 0x10, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0101, 0xB9, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0102, 0x20, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x0103, 0x00, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x2000, 0x00, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x2001, 0x00, microlind::BusCycleKind::OperandRead},
            },
        },
        {
            "ORD indexed",
            {0x10, 0xAA, 0x84},
            0x12,
            0x2000,
            0x1200,
            microlind::CC_C,
            {{0x2000, 0x00}, {0x2001, 0xFF}},
            7,
            0x0103,
            0x12FF,
            false,
            false,
            false,
            false,
            {
                {microlind::BusAccessType::Read, 0x0100, 0x10, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0101, 0xAA, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0102, 0x84, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x2000, 0x00, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x2001, 0xFF, microlind::BusCycleKind::OperandRead},
            },
        },
    };

    for (const auto& test : cases) {
        SCOPED_TRACE(test.name);
        microlind::Simulator sim(microlind::CpuMode::HD6309, 1'000'000);
        auto memory = std::make_unique<SideEffectMemory>();
        auto* probe = memory.get();
        for (std::size_t i = 0; i < test.program.size(); ++i) {
            probe->data[static_cast<uint16_t>(0x0100 + i)] = test.program[i];
        }
        for (const auto& [address, value] : test.memory) {
            probe->data[address] = value;
        }
        ASSERT_FALSE(sim.map_device(0x0000, 0xFFFF, std::move(memory)));

        sim.cpu().set_pc(0x0100);
        sim.cpu().regs().dp = test.dp;
        sim.cpu().regs().x = test.x;
        sim.cpu().regs().a = high_byte(test.initial_d);
        sim.cpu().regs().b = low_byte(test.initial_d);
        sim.cpu().regs().cc = test.initial_cc;

        for (std::size_t i = 0; i < test.expected_accesses.size(); ++i) {
            const auto result = sim.tick_microcycle();
            EXPECT_TRUE(result.emitted);
            EXPECT_EQ(result.instruction_started, i == 0);
            EXPECT_EQ(result.instruction_complete, i + 1 == test.expected_cycles);
            EXPECT_EQ(result.instruction_result.cycles, test.expected_cycles);
        }

        const uint8_t internal_cycles = static_cast<uint8_t>(test.expected_cycles - test.expected_accesses.size());
        if (internal_cycles > 0) {
            expect_internal_cycles(sim, internal_cycles);
        } else {
            EXPECT_FALSE(sim.has_pending_microcycles());
        }

        const uint16_t actual_d =
            static_cast<uint16_t>((static_cast<uint16_t>(sim.cpu().regs().a) << 8) | sim.cpu().regs().b);
        EXPECT_EQ(sim.cpu().regs().pc, test.expected_pc);
        EXPECT_EQ(sim.cpu().regs().x, test.x);
        EXPECT_EQ(actual_d, test.expected_d);
        EXPECT_EQ((sim.cpu().regs().cc & microlind::CC_Z) != 0, test.expect_z);
        EXPECT_EQ((sim.cpu().regs().cc & microlind::CC_N) != 0, test.expect_n);
        EXPECT_EQ((sim.cpu().regs().cc & microlind::CC_V) != 0, test.expect_v);
        EXPECT_EQ((sim.cpu().regs().cc & microlind::CC_C) != 0, test.expect_c);
        ASSERT_THAT(sim.bus().access_log(), testing::SizeIs(test.expected_accesses.size()));
        for (std::size_t i = 0; i < test.expected_accesses.size(); ++i) {
            SCOPED_TRACE(i);
            EXPECT_EQ(sim.bus().access_log()[i].type, test.expected_accesses[i].type);
            EXPECT_EQ(sim.bus().access_log()[i].address, test.expected_accesses[i].address);
            EXPECT_EQ(sim.bus().access_log()[i].value, test.expected_accesses[i].value);
            EXPECT_EQ(sim.bus().access_log()[i].cycle_kind, test.expected_accesses[i].cycle_kind);
        }
    }
}

TEST(BusPhaseTest, ResumableMicrocycleHD6309ImmediateMemoryInstructions) {
    struct ExpectedAccess {
        microlind::BusAccessType type{};
        uint16_t address{};
        uint8_t value{};
        microlind::BusCycleKind cycle_kind{};
    };

    struct Case {
        const char* name{};
        std::vector<uint8_t> program;
        uint8_t dp{};
        uint16_t x{};
        std::vector<std::pair<uint16_t, uint8_t>> memory;
        std::vector<std::pair<uint16_t, uint8_t>> expected_memory;
        uint8_t expected_cycles{};
        uint16_t expected_pc{};
        uint32_t expected_writes{};
        bool expect_z{};
        bool expect_n{};
        bool expect_v{};
        bool expect_c{};
        std::vector<ExpectedAccess> expected_accesses;
    };

    const Case cases[] = {
        {
            "OIM direct",
            {0x01, 0x0F, 0x20},
            0x12,
            0x2000,
            {{0x1220, 0xF0}},
            {{0x1220, 0xFF}},
            6,
            0x0103,
            1,
            false,
            true,
            false,
            true,
            {
                {microlind::BusAccessType::Read, 0x0100, 0x01, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0101, 0x0F, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x0102, 0x20, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x1220, 0xF0, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Write, 0x1220, 0xFF, microlind::BusCycleKind::OperandWrite},
            },
        },
        {
            "AIM extended",
            {0x72, 0x0F, 0x20, 0x00},
            0x12,
            0x2000,
            {{0x2000, 0xF0}},
            {{0x2000, 0x00}},
            7,
            0x0104,
            1,
            true,
            false,
            false,
            true,
            {
                {microlind::BusAccessType::Read, 0x0100, 0x72, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0101, 0x0F, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x0102, 0x20, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x0103, 0x00, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x2000, 0xF0, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Write, 0x2000, 0x00, microlind::BusCycleKind::OperandWrite},
            },
        },
        {
            "EIM indexed",
            {0x65, 0xFF, 0x84},
            0x12,
            0x2000,
            {{0x2000, 0x0F}},
            {{0x2000, 0xF0}},
            7,
            0x0103,
            1,
            false,
            true,
            false,
            true,
            {
                {microlind::BusAccessType::Read, 0x0100, 0x65, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0101, 0xFF, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x0102, 0x84, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x2000, 0x0F, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Write, 0x2000, 0xF0, microlind::BusCycleKind::OperandWrite},
            },
        },
        {
            "TIM extended",
            {0x7B, 0xF0, 0x20, 0x00},
            0x12,
            0x2000,
            {{0x2000, 0x0F}},
            {},
            7,
            0x0104,
            0,
            true,
            false,
            false,
            true,
            {
                {microlind::BusAccessType::Read, 0x0100, 0x7B, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0101, 0xF0, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x0102, 0x20, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x0103, 0x00, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x2000, 0x0F, microlind::BusCycleKind::OperandRead},
            },
        },
    };

    for (const auto& test : cases) {
        SCOPED_TRACE(test.name);
        microlind::Simulator sim(microlind::CpuMode::HD6309, 1'000'000);
        auto memory = std::make_unique<SideEffectMemory>();
        auto* probe = memory.get();
        for (std::size_t i = 0; i < test.program.size(); ++i) {
            probe->data[static_cast<uint16_t>(0x0100 + i)] = test.program[i];
        }
        for (const auto& [address, value] : test.memory) {
            probe->data[address] = value;
        }
        ASSERT_FALSE(sim.map_device(0x0000, 0xFFFF, std::move(memory)));

        sim.cpu().set_pc(0x0100);
        sim.cpu().regs().dp = test.dp;
        sim.cpu().regs().x = test.x;
        sim.cpu().regs().cc = static_cast<uint8_t>(microlind::CC_C | microlind::CC_V);

        for (std::size_t i = 0; i < test.expected_accesses.size(); ++i) {
            const auto result = sim.tick_microcycle();
            EXPECT_TRUE(result.emitted);
            EXPECT_EQ(result.instruction_started, i == 0);
            EXPECT_EQ(result.instruction_complete, i + 1 == test.expected_cycles);
            EXPECT_EQ(result.instruction_result.cycles, test.expected_cycles);
        }

        const uint8_t internal_cycles = static_cast<uint8_t>(test.expected_cycles - test.expected_accesses.size());
        if (internal_cycles > 0) {
            expect_internal_cycles(sim, internal_cycles);
        } else {
            EXPECT_FALSE(sim.has_pending_microcycles());
        }

        EXPECT_EQ(sim.cpu().regs().pc, test.expected_pc);
        EXPECT_EQ(probe->write_count, test.expected_writes);
        EXPECT_EQ((sim.cpu().regs().cc & microlind::CC_Z) != 0, test.expect_z);
        EXPECT_EQ((sim.cpu().regs().cc & microlind::CC_N) != 0, test.expect_n);
        EXPECT_EQ((sim.cpu().regs().cc & microlind::CC_V) != 0, test.expect_v);
        EXPECT_EQ((sim.cpu().regs().cc & microlind::CC_C) != 0, test.expect_c);
        for (const auto& [address, value] : test.expected_memory) {
            EXPECT_EQ(probe->data[address], value);
        }
        ASSERT_THAT(sim.bus().access_log(), testing::SizeIs(test.expected_accesses.size()));
        for (std::size_t i = 0; i < test.expected_accesses.size(); ++i) {
            SCOPED_TRACE(i);
            EXPECT_EQ(sim.bus().access_log()[i].type, test.expected_accesses[i].type);
            EXPECT_EQ(sim.bus().access_log()[i].address, test.expected_accesses[i].address);
            EXPECT_EQ(sim.bus().access_log()[i].value, test.expected_accesses[i].value);
            EXPECT_EQ(sim.bus().access_log()[i].cycle_kind, test.expected_accesses[i].cycle_kind);
        }
    }
}

TEST(BusPhaseTest, ResumableMicrocycleIndexedMemoryUnaryConventionalModes) {
    struct ExpectedAccess {
        microlind::BusAccessType type{};
        uint16_t address{};
        uint8_t value{};
        microlind::BusCycleKind cycle_kind{};
    };

    struct Case {
        const char* name;
        std::vector<uint8_t> program;
        uint16_t x{};
        uint16_t y{};
        std::vector<std::pair<uint16_t, uint8_t>> memory;
        std::vector<std::pair<uint16_t, uint8_t>> expected_memory;
        uint8_t expected_cycles{};
        uint16_t expected_pc{};
        uint16_t expected_x{};
        uint16_t expected_y{};
        bool expect_z{};
        bool expect_n{};
        bool expect_v{};
        bool expect_c{};
        std::vector<ExpectedAccess> expected_accesses;
    };

    const Case cases[] = {
        {
            "NEG ,X",
            {0x60, 0x84},
            0x2000,
            0x2100,
            {{0x2000, 0x01}},
            {{0x2000, 0xFF}},
            6,
            0x0102,
            0x2000,
            0x2100,
            false,
            true,
            false,
            true,
            {
                {microlind::BusAccessType::Read, 0x0100, 0x60, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0101, 0x84, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x2000, 0x01, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Write, 0x2000, 0xFF, microlind::BusCycleKind::OperandWrite},
            },
        },
        {
            "TST 5-bit offset,X",
            {0x6D, 0x05},
            0x2000,
            0x2100,
            {{0x2005, 0x80}},
            {{0x2005, 0x00}},
            7,
            0x0102,
            0x2000,
            0x2100,
            false,
            true,
            false,
            false,
            {
                {microlind::BusAccessType::Read, 0x0100, 0x6D, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0101, 0x05, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x2005, 0x80, microlind::BusCycleKind::OperandRead},
            },
        },
        {
            "CLR ,X",
            {0x6F, 0x84},
            0x2000,
            0x2100,
            {{0x2000, 0x5A}},
            {{0x2000, 0x00}},
            6,
            0x0102,
            0x2000,
            0x2100,
            true,
            false,
            false,
            false,
            {
                {microlind::BusAccessType::Read, 0x0100, 0x6F, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0101, 0x84, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Write, 0x2000, 0x00, microlind::BusCycleKind::OperandWrite},
            },
        },
        {
            "INC [8-bit offset,Y]",
            {0x6C, 0xB8, 0x02},
            0x2000,
            0x2100,
            {{0x2102, 0x30}, {0x2103, 0x00}, {0x3000, 0x7F}},
            {{0x3000, 0x80}},
            10,
            0x0103,
            0x2000,
            0x2100,
            false,
            true,
            true,
            false,
            {
                {microlind::BusAccessType::Read, 0x0100, 0x6C, microlind::BusCycleKind::OpcodeFetch},
                {microlind::BusAccessType::Read, 0x0101, 0xB8, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x0102, 0x02, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x2102, 0x30, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x2103, 0x00, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Read, 0x3000, 0x7F, microlind::BusCycleKind::OperandRead},
                {microlind::BusAccessType::Write, 0x3000, 0x80, microlind::BusCycleKind::OperandWrite},
            },
        },
    };

    for (const auto& test : cases) {
        SCOPED_TRACE(test.name);
        microlind::Simulator sim(microlind::CpuMode::HD6309, 1'000'000);
        auto memory = std::make_unique<SideEffectMemory>();
        auto* probe = memory.get();
        for (std::size_t i = 0; i < test.program.size(); ++i) {
            probe->data[static_cast<uint16_t>(0x0100 + i)] = test.program[i];
        }
        for (const auto& [address, value] : test.memory) {
            probe->data[address] = value;
        }
        ASSERT_FALSE(sim.map_device(0x0000, 0xFFFF, std::move(memory)));

        sim.cpu().set_pc(0x0100);
        sim.cpu().regs().x = test.x;
        sim.cpu().regs().y = test.y;

        for (std::size_t i = 0; i < test.expected_accesses.size(); ++i) {
            const auto result = sim.tick_microcycle();
            EXPECT_TRUE(result.emitted);
            EXPECT_EQ(result.instruction_started, i == 0);
            EXPECT_EQ(result.instruction_complete, i + 1 == test.expected_cycles);
            EXPECT_EQ(result.instruction_result.cycles, test.expected_cycles);
        }

        const uint8_t internal_cycles = static_cast<uint8_t>(test.expected_cycles - test.expected_accesses.size());
        if (internal_cycles > 0) {
            expect_internal_cycles(sim, internal_cycles);
        } else {
            EXPECT_FALSE(sim.has_pending_microcycles());
        }

        EXPECT_EQ(sim.cpu().regs().pc, test.expected_pc);
        EXPECT_EQ(sim.cpu().regs().x, test.expected_x);
        EXPECT_EQ(sim.cpu().regs().y, test.expected_y);
        EXPECT_EQ((sim.cpu().regs().cc & microlind::CC_Z) != 0, test.expect_z);
        EXPECT_EQ((sim.cpu().regs().cc & microlind::CC_N) != 0, test.expect_n);
        EXPECT_EQ((sim.cpu().regs().cc & microlind::CC_V) != 0, test.expect_v);
        EXPECT_EQ((sim.cpu().regs().cc & microlind::CC_C) != 0, test.expect_c);
        for (const auto& [address, value] : test.expected_memory) {
            EXPECT_EQ(probe->data[address], value);
        }
        ASSERT_THAT(sim.bus().access_log(), testing::SizeIs(test.expected_accesses.size()));
        for (std::size_t i = 0; i < test.expected_accesses.size(); ++i) {
            SCOPED_TRACE(i);
            EXPECT_EQ(sim.bus().access_log()[i].type, test.expected_accesses[i].type);
            EXPECT_EQ(sim.bus().access_log()[i].address, test.expected_accesses[i].address);
            EXPECT_EQ(sim.bus().access_log()[i].value, test.expected_accesses[i].value);
            EXPECT_EQ(sim.bus().access_log()[i].cycle_kind, test.expected_accesses[i].cycle_kind);
        }
    }
}

TEST(BusPhaseTest, MicrocycleFallsBackForHD6309OnlyOpcodesInMC6809) {
    struct Case {
        const char* name{};
        std::vector<uint8_t> program;
        uint16_t expected_pc{};
        uint16_t expected_x{};
        uint8_t expected_a{};
        uint8_t expected_b{};
        uint8_t expected_e{};
        uint8_t expected_f{};
        std::size_t expected_emitted_cycles{};
    };

    const Case cases[] = {
        {
            "single-byte LDQ immediate",
            {0xCD, 0x12, 0x34, 0x56, 0x78},
            0x0101,
            0x2000,
            0x00,
            0x00,
            0x00,
            0x00,
            1,
        },
        {
            "prefixed register ALU",
            {0x10, 0x30, 0x01},
            0x0102,
            0x2000,
            0x00,
            0x02,
            0x00,
            0x00,
            2,
        },
        {
            "prefixed DIVD immediate",
            {0x11, 0x8D, 0x05},
            0x0102,
            0x2000,
            0x00,
            0x14,
            0x00,
            0x00,
            2,
        },
    };

    for (const auto& test : cases) {
        SCOPED_TRACE(test.name);
        microlind::Simulator sim(microlind::CpuMode::MC6809, 1'000'000);
        auto memory = std::make_unique<SideEffectMemory>();
        auto* probe = memory.get();
        for (std::size_t i = 0; i < test.program.size(); ++i) {
            probe->data[static_cast<uint16_t>(0x0100 + i)] = test.program[i];
        }
        ASSERT_FALSE(sim.map_device(0x0000, 0xFFFF, std::move(memory)));

        sim.cpu().set_pc(0x0100);
        sim.cpu().regs().x = 0x2000;
        sim.cpu().regs().b = test.expected_b;

        for (std::size_t i = 0; i < test.expected_emitted_cycles; ++i) {
            const auto result = sim.tick_microcycle();
            EXPECT_TRUE(result.emitted);
            EXPECT_EQ(result.instruction_started, i == 0);
            EXPECT_EQ(result.instruction_complete, i + 1 == test.expected_emitted_cycles);
            EXPECT_EQ(result.instruction_result.cycles, 1u);
        }

        EXPECT_FALSE(sim.has_pending_microcycles());
        EXPECT_EQ(sim.cpu().regs().pc, test.expected_pc);
        EXPECT_EQ(sim.cpu().regs().x, test.expected_x);
        EXPECT_EQ(sim.cpu().regs().a, test.expected_a);
        EXPECT_EQ(sim.cpu().regs().b, test.expected_b);
        EXPECT_EQ(sim.cpu().regs().e, test.expected_e);
        EXPECT_EQ(sim.cpu().regs().f, test.expected_f);
        EXPECT_EQ(sim.cpu().regs().md & 0x40, 0);
        EXPECT_EQ(probe->write_count, 0u);
    }
}

TEST(BusPhaseTest, ResumableMicrocycleServicesExternalIrq) {
    struct ExpectedAccess {
        microlind::BusAccessType type{};
        uint16_t address{};
        uint8_t value{};
        microlind::BusCycleKind cycle_kind{};
    };

    microlind::Simulator sim(microlind::CpuMode::HD6309, 1'000'000);
    auto memory = std::make_unique<SideEffectMemory>();
    auto* probe = memory.get();
    probe->data[0xFFF8] = 0x45;
    probe->data[0xFFF9] = 0x67;
    ASSERT_FALSE(sim.map_device(0x0000, 0xFFFF, std::move(memory)));

    sim.cpu().set_pc(0x0100);
    sim.cpu().regs().s = 0x9000;
    sim.cpu().regs().u = 0xCAFE;
    sim.cpu().regs().y = 0x5678;
    sim.cpu().regs().x = 0x1234;
    sim.cpu().regs().dp = 0xAB;
    sim.cpu().regs().b = 0x22;
    sim.cpu().regs().a = 0x11;
    sim.cpu().regs().cc = microlind::CC_C;
    sim.cpu().set_irq_line(true);

    const std::vector<ExpectedAccess> expected_accesses{
        {microlind::BusAccessType::Write, 0x8FFF, 0x00, microlind::BusCycleKind::StackWrite},
        {microlind::BusAccessType::Write, 0x8FFE, 0x01, microlind::BusCycleKind::StackWrite},
        {microlind::BusAccessType::Write, 0x8FFD, 0xFE, microlind::BusCycleKind::StackWrite},
        {microlind::BusAccessType::Write, 0x8FFC, 0xCA, microlind::BusCycleKind::StackWrite},
        {microlind::BusAccessType::Write, 0x8FFB, 0x78, microlind::BusCycleKind::StackWrite},
        {microlind::BusAccessType::Write, 0x8FFA, 0x56, microlind::BusCycleKind::StackWrite},
        {microlind::BusAccessType::Write, 0x8FF9, 0x34, microlind::BusCycleKind::StackWrite},
        {microlind::BusAccessType::Write, 0x8FF8, 0x12, microlind::BusCycleKind::StackWrite},
        {microlind::BusAccessType::Write, 0x8FF7, 0xAB, microlind::BusCycleKind::StackWrite},
        {microlind::BusAccessType::Write, 0x8FF6, 0x22, microlind::BusCycleKind::StackWrite},
        {microlind::BusAccessType::Write, 0x8FF5, 0x11, microlind::BusCycleKind::StackWrite},
        {microlind::BusAccessType::Write, 0x8FF4, static_cast<uint8_t>(microlind::CC_E | microlind::CC_C), microlind::BusCycleKind::StackWrite},
        {microlind::BusAccessType::Read, 0xFFF8, 0x45, microlind::BusCycleKind::VectorRead},
        {microlind::BusAccessType::Read, 0xFFF9, 0x67, microlind::BusCycleKind::VectorRead},
    };

    constexpr uint8_t expected_cycles = 19;
    for (std::size_t i = 0; i < expected_accesses.size(); ++i) {
        const auto result = sim.tick_microcycle();
        EXPECT_TRUE(result.emitted);
        EXPECT_EQ(result.instruction_started, i == 0);
        EXPECT_FALSE(result.instruction_complete);
        EXPECT_EQ(result.instruction_result.cycles, expected_cycles);
    }
    expect_internal_cycles(sim, static_cast<uint8_t>(expected_cycles - expected_accesses.size()));

    EXPECT_EQ(sim.cpu().regs().pc, 0x4567);
    EXPECT_EQ(sim.cpu().regs().s, 0x8FF4);
    EXPECT_EQ(sim.cpu().regs().cc, microlind::CC_E | microlind::CC_I | microlind::CC_C);
    ASSERT_THAT(sim.bus().access_log(), testing::SizeIs(expected_accesses.size()));
    for (std::size_t i = 0; i < expected_accesses.size(); ++i) {
        SCOPED_TRACE(i);
        EXPECT_EQ(sim.bus().access_log()[i].type, expected_accesses[i].type);
        EXPECT_EQ(sim.bus().access_log()[i].address, expected_accesses[i].address);
        EXPECT_EQ(sim.bus().access_log()[i].value, expected_accesses[i].value);
        EXPECT_EQ(sim.bus().access_log()[i].cycle_kind, expected_accesses[i].cycle_kind);
    }
}

TEST(BusPhaseTest, ResumableMicrocycleSoftwareInterruptsPushStateAndVector) {
    struct Case {
        const char* name{};
        std::vector<uint8_t> program;
        uint16_t vector{};
        uint16_t target{};
        uint16_t return_pc{};
        uint8_t expected_cycles{};
        uint8_t expected_cc{};
        std::vector<std::pair<uint16_t, uint8_t>> expected_fetches;
    };

    const Case cases[] = {
        {
            "SWI",
            {0x3F},
            0xFFFA,
            0x2222,
            0x0101,
            19,
            static_cast<uint8_t>(microlind::CC_E | microlind::CC_I | microlind::CC_F | microlind::CC_C),
            {{0x0100, 0x3F}},
        },
        {
            "SWI2",
            {0x10, 0x3F},
            0xFFF4,
            0x3333,
            0x0102,
            20,
            static_cast<uint8_t>(microlind::CC_E | microlind::CC_I | microlind::CC_C),
            {{0x0100, 0x10}, {0x0101, 0x3F}},
        },
        {
            "SWI3",
            {0x11, 0x3F},
            0xFFF2,
            0x4444,
            0x0102,
            20,
            static_cast<uint8_t>(microlind::CC_E | microlind::CC_I | microlind::CC_C),
            {{0x0100, 0x11}, {0x0101, 0x3F}},
        },
    };

    for (const auto& test : cases) {
        SCOPED_TRACE(test.name);
        microlind::Simulator sim(microlind::CpuMode::HD6309, 1'000'000);
        auto memory = std::make_unique<SideEffectMemory>();
        auto* probe = memory.get();
        for (std::size_t i = 0; i < test.program.size(); ++i) {
            probe->data[static_cast<uint16_t>(0x0100 + i)] = test.program[i];
        }
        probe->data[test.vector] = static_cast<uint8_t>(test.target >> 8);
        probe->data[static_cast<uint16_t>(test.vector + 1)] = static_cast<uint8_t>(test.target & 0xFF);
        ASSERT_FALSE(sim.map_device(0x0000, 0xFFFF, std::move(memory)));

        sim.cpu().set_pc(0x0100);
        sim.cpu().regs().s = 0x9000;
        sim.cpu().regs().u = 0xCAFE;
        sim.cpu().regs().y = 0x5678;
        sim.cpu().regs().x = 0x1234;
        sim.cpu().regs().dp = 0xAB;
        sim.cpu().regs().b = 0x22;
        sim.cpu().regs().a = 0x11;
        sim.cpu().regs().cc = microlind::CC_C;

        const std::size_t bus_cycles = test.expected_fetches.size() + 14;
        for (std::size_t i = 0; i < bus_cycles; ++i) {
            const auto result = sim.tick_microcycle();
            EXPECT_TRUE(result.emitted);
            EXPECT_EQ(result.instruction_started, i == 0);
            EXPECT_FALSE(result.instruction_complete);
            EXPECT_EQ(result.instruction_result.cycles, test.expected_cycles);
        }
        expect_internal_cycles(sim, static_cast<uint8_t>(test.expected_cycles - bus_cycles));

        EXPECT_EQ(sim.cpu().regs().pc, test.target);
        EXPECT_EQ(sim.cpu().regs().s, 0x8FF4);
        EXPECT_EQ(sim.cpu().regs().cc, test.expected_cc);
        EXPECT_EQ(probe->data[0x8FFF], static_cast<uint8_t>(test.return_pc & 0xFF));
        EXPECT_EQ(probe->data[0x8FFE], static_cast<uint8_t>(test.return_pc >> 8));
        EXPECT_EQ(probe->data[0x8FF4], static_cast<uint8_t>(microlind::CC_E | microlind::CC_C));
        ASSERT_THAT(sim.bus().access_log(), testing::SizeIs(bus_cycles));
        for (std::size_t i = 0; i < test.expected_fetches.size(); ++i) {
            EXPECT_EQ(sim.bus().access_log()[i].type, microlind::BusAccessType::Read);
            EXPECT_EQ(sim.bus().access_log()[i].address, test.expected_fetches[i].first);
            EXPECT_EQ(sim.bus().access_log()[i].value, test.expected_fetches[i].second);
            EXPECT_EQ(sim.bus().access_log()[i].cycle_kind, microlind::BusCycleKind::OpcodeFetch);
        }
        EXPECT_EQ(sim.bus().access_log()[bus_cycles - 2].address, test.vector);
        EXPECT_EQ(sim.bus().access_log()[bus_cycles - 2].cycle_kind, microlind::BusCycleKind::VectorRead);
        EXPECT_EQ(sim.bus().access_log()[bus_cycles - 1].address, static_cast<uint16_t>(test.vector + 1));
        EXPECT_EQ(sim.bus().access_log()[bus_cycles - 1].cycle_kind, microlind::BusCycleKind::VectorRead);
    }
}

TEST(BusPhaseTest, ResumableMicrocycleRtiPullsShortAndFullFrames) {
    struct Case {
        const char* name{};
        std::vector<std::pair<uint16_t, uint8_t>> stack;
        uint8_t expected_cycles{};
        uint16_t expected_s{};
        uint16_t expected_pc{};
        uint8_t expected_cc{};
        uint8_t expected_a{};
        uint8_t expected_b{};
        uint8_t expected_dp{};
        uint16_t expected_x{};
        uint16_t expected_y{};
        uint16_t expected_u{};
    };

    const Case cases[] = {
        {
            "short RTI",
            {{0x9000, microlind::CC_C}, {0x9001, 0x12}, {0x9002, 0x34}},
            6,
            0x9003,
            0x1234,
            microlind::CC_C,
            0xAA,
            0xBB,
            0xCC,
            0x1111,
            0x2222,
            0x3333,
        },
        {
            "full RTI",
            {
                {0x9000, static_cast<uint8_t>(microlind::CC_E | microlind::CC_C)},
                {0x9001, 0x11},
                {0x9002, 0x22},
                {0x9003, 0x33},
                {0x9004, 0x44},
                {0x9005, 0x55},
                {0x9006, 0x66},
                {0x9007, 0x77},
                {0x9008, 0x88},
                {0x9009, 0x99},
                {0x900A, 0xAA},
                {0x900B, 0xBB},
            },
            15,
            0x900C,
            0xAABB,
            static_cast<uint8_t>(microlind::CC_E | microlind::CC_C),
            0x11,
            0x22,
            0x33,
            0x4455,
            0x6677,
            0x8899,
        },
    };

    for (const auto& test : cases) {
        SCOPED_TRACE(test.name);
        microlind::Simulator sim(microlind::CpuMode::HD6309, 1'000'000);
        auto memory = std::make_unique<SideEffectMemory>();
        auto* probe = memory.get();
        probe->data[0x0100] = 0x3B;
        for (const auto& [address, value] : test.stack) {
            probe->data[address] = value;
        }
        ASSERT_FALSE(sim.map_device(0x0000, 0xFFFF, std::move(memory)));

        sim.cpu().set_pc(0x0100);
        sim.cpu().regs().s = 0x9000;
        sim.cpu().regs().a = 0xAA;
        sim.cpu().regs().b = 0xBB;
        sim.cpu().regs().dp = 0xCC;
        sim.cpu().regs().x = 0x1111;
        sim.cpu().regs().y = 0x2222;
        sim.cpu().regs().u = 0x3333;

        const std::size_t bus_cycles = 1 + test.stack.size();
        for (std::size_t i = 0; i < bus_cycles; ++i) {
            const auto result = sim.tick_microcycle();
            EXPECT_TRUE(result.emitted);
            EXPECT_EQ(result.instruction_started, i == 0);
            EXPECT_FALSE(result.instruction_complete);
            EXPECT_EQ(result.instruction_result.cycles, test.expected_cycles);
        }
        expect_internal_cycles(sim, static_cast<uint8_t>(test.expected_cycles - bus_cycles));

        EXPECT_EQ(sim.cpu().regs().s, test.expected_s);
        EXPECT_EQ(sim.cpu().regs().pc, test.expected_pc);
        EXPECT_EQ(sim.cpu().regs().cc, test.expected_cc);
        EXPECT_EQ(sim.cpu().regs().a, test.expected_a);
        EXPECT_EQ(sim.cpu().regs().b, test.expected_b);
        EXPECT_EQ(sim.cpu().regs().dp, test.expected_dp);
        EXPECT_EQ(sim.cpu().regs().x, test.expected_x);
        EXPECT_EQ(sim.cpu().regs().y, test.expected_y);
        EXPECT_EQ(sim.cpu().regs().u, test.expected_u);
        ASSERT_THAT(sim.bus().access_log(), testing::SizeIs(bus_cycles));
        EXPECT_EQ(sim.bus().access_log()[0].cycle_kind, microlind::BusCycleKind::OpcodeFetch);
        for (std::size_t i = 0; i < test.stack.size(); ++i) {
            EXPECT_EQ(sim.bus().access_log()[i + 1].address, test.stack[i].first);
            EXPECT_EQ(sim.bus().access_log()[i + 1].value, test.stack[i].second);
            EXPECT_EQ(sim.bus().access_log()[i + 1].cycle_kind, microlind::BusCycleKind::StackRead);
        }
    }
}

TEST(BusPhaseTest, ResumableMicrocycleCwaiPushesMaskedStateAndSyncConsumesOpcode) {
    {
        microlind::Simulator sim(microlind::CpuMode::HD6309, 1'000'000);
        auto memory = std::make_unique<SideEffectMemory>();
        auto* probe = memory.get();
        probe->data[0x0100] = 0x3C;
        probe->data[0x0101] = 0xFE;
        ASSERT_FALSE(sim.map_device(0x0000, 0xFFFF, std::move(memory)));

        sim.cpu().set_pc(0x0100);
        sim.cpu().regs().s = 0x9000;
        sim.cpu().regs().u = 0xCAFE;
        sim.cpu().regs().y = 0x5678;
        sim.cpu().regs().x = 0x1234;
        sim.cpu().regs().dp = 0xAB;
        sim.cpu().regs().b = 0x22;
        sim.cpu().regs().a = 0x11;
        sim.cpu().regs().cc = static_cast<uint8_t>(microlind::CC_C | microlind::CC_V);

        constexpr uint8_t expected_cycles = 19;
        constexpr std::size_t bus_cycles = 14;
        for (std::size_t i = 0; i < bus_cycles; ++i) {
            const auto result = sim.tick_microcycle();
            EXPECT_TRUE(result.emitted);
            EXPECT_EQ(result.instruction_started, i == 0);
            EXPECT_FALSE(result.instruction_complete);
            EXPECT_EQ(result.instruction_result.cycles, expected_cycles);
        }
        expect_internal_cycles(sim, static_cast<uint8_t>(expected_cycles - bus_cycles));

        EXPECT_EQ(sim.cpu().regs().pc, 0x0102);
        EXPECT_EQ(sim.cpu().regs().s, 0x8FF4);
        EXPECT_EQ(sim.cpu().regs().cc, microlind::CC_E | microlind::CC_I | microlind::CC_V);
        EXPECT_EQ(probe->data[0x8FFF], 0x02);
        EXPECT_EQ(probe->data[0x8FFE], 0x01);
        EXPECT_EQ(probe->data[0x8FF4], microlind::CC_E | microlind::CC_V);
    }

    {
        microlind::Simulator sim(microlind::CpuMode::HD6309, 1'000'000);
        auto memory = std::make_unique<SideEffectMemory>();
        memory->data[0x0200] = 0x13;
        ASSERT_FALSE(sim.map_device(0x0000, 0xFFFF, std::move(memory)));

        sim.cpu().set_pc(0x0200);
        const auto opcode = sim.tick_microcycle();
        EXPECT_TRUE(opcode.emitted);
        EXPECT_TRUE(opcode.instruction_started);
        EXPECT_FALSE(opcode.instruction_complete);
        EXPECT_EQ(opcode.signals.cycle_kind, microlind::BusCycleKind::OpcodeFetch);
        expect_internal_cycles(sim, 1);
        EXPECT_EQ(sim.cpu().regs().pc, 0x0201);
    }
}


} // namespace
