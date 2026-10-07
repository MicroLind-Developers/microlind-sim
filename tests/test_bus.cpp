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

TEST(BusPhaseTest, FastBusModeSkipsPhaseFanoutButStillExecutesCpuCycles) {
    microlind::Simulator sim(microlind::CpuMode::HD6309, 1'000'000);
    auto memory = std::make_unique<PhaseProbeMemory>();
    auto* probe = memory.get();
    probe->data[0x0100] = 0x86; // LDA immediate
    probe->data[0x0101] = 0x42;
    ASSERT_FALSE(sim.map_device(0x0000, 0xFFFF, std::move(memory)));

    sim.bus().set_detailed_bus_phases(false);
    sim.bus().set_access_logging(false);
    sim.cpu().set_pc(0x0100);
    const auto result = sim.tick();

    EXPECT_EQ(result.cycles, 2u);
    EXPECT_EQ(sim.cpu().regs().a, 0x42);
    EXPECT_EQ(sim.bus().bus_cycle_count(), 2u);
    EXPECT_TRUE(sim.bus().phase_log().empty());
    EXPECT_TRUE(sim.bus().access_log().empty());
    EXPECT_TRUE(probe->phases.empty());
    EXPECT_EQ(probe->ticked_cycles, 2u);
    EXPECT_EQ(sim.bus().last_signals().phase, microlind::BusPhase::QLowEHigh);
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
