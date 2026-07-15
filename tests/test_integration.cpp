#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <vector>

#include "microlind/bus.hpp"
#include "microlind/simulator.hpp"

namespace {

class IntegrationMemory : public microlind::BusDevice {
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

    std::vector<uint8_t> data = std::vector<uint8_t>(0x10000, 0x00);
};

struct IntegrationRun {
    microlind::Simulator sim;
    IntegrationMemory* memory{};

    explicit IntegrationRun(microlind::CpuMode mode)
        : sim(mode, 1'000'000) {
        auto ram = std::make_unique<IntegrationMemory>();
        memory = ram.get();
        if (sim.map_device(0x0000, 0xFFFF, std::move(ram))) {
            throw std::runtime_error("failed to map integration RAM");
        }
    }
};

void load_program(IntegrationMemory& memory, uint16_t address, const std::vector<uint8_t>& bytes) {
    for (std::size_t i = 0; i < bytes.size(); ++i) {
        memory.data[static_cast<uint16_t>(address + i)] = bytes[i];
    }
}

void run_regular(IntegrationRun& run, uint16_t pc, std::size_t instruction_count) {
    run.sim.cpu().set_pc(pc);
    for (std::size_t i = 0; i < instruction_count; ++i) {
        run.sim.tick();
    }
}

void run_microstepped(IntegrationRun& run, uint16_t pc, std::size_t instruction_count) {
    run.sim.cpu().set_pc(pc);
    for (std::size_t i = 0; i < instruction_count; ++i) {
        bool complete = false;
        while (!complete) {
            const auto result = run.sim.tick_microcycle();
            ASSERT_TRUE(result.emitted);
            complete = result.instruction_complete;
        }
    }
    EXPECT_FALSE(run.sim.has_pending_microcycles());
}

void expect_same_machine_state(const IntegrationRun& regular, const IntegrationRun& micro) {
    const auto& regular_regs = regular.sim.cpu().regs();
    const auto& micro_regs = micro.sim.cpu().regs();

    EXPECT_EQ(micro_regs.pc, regular_regs.pc);
    EXPECT_EQ(micro_regs.a, regular_regs.a);
    EXPECT_EQ(micro_regs.b, regular_regs.b);
    EXPECT_EQ(micro_regs.e, regular_regs.e);
    EXPECT_EQ(micro_regs.f, regular_regs.f);
    EXPECT_EQ(micro_regs.dp, regular_regs.dp);
    EXPECT_EQ(micro_regs.cc, regular_regs.cc);
    EXPECT_EQ(micro_regs.md, regular_regs.md);
    EXPECT_EQ(micro_regs.x, regular_regs.x);
    EXPECT_EQ(micro_regs.y, regular_regs.y);
    EXPECT_EQ(micro_regs.u, regular_regs.u);
    EXPECT_EQ(micro_regs.s, regular_regs.s);
    EXPECT_EQ(micro.sim.clock().total_cycles(), regular.sim.clock().total_cycles());
}

TEST(IntegrationExecutionTest, RegularAndMicroSteppedExecutionMatchForMemoryAndBranches) {
    const std::vector<uint8_t> program = {
        0x86, 0x02,       // LDA #$02
        0x97, 0x20,       // STA <$20
        0x96, 0x20,       // LDA <$20
        0x8B, 0x03,       // ADDA #$03
        0xB7, 0x30, 0x00, // STA $3000
        0x7C, 0x30, 0x00, // INC $3000
        0xB6, 0x30, 0x00, // LDA $3000
        0x81, 0x06,       // CMPA #$06
        0x27, 0x02,       // BEQ +2
        0x86, 0xFF,       // LDA #$FF
        0xB7, 0x30, 0x01, // STA $3001
        0x12,             // NOP
    };

    IntegrationRun regular(microlind::CpuMode::HD6309);
    IntegrationRun micro(microlind::CpuMode::HD6309);
    load_program(*regular.memory, 0x0100, program);
    load_program(*micro.memory, 0x0100, program);

    run_regular(regular, 0x0100, 10);
    run_microstepped(micro, 0x0100, 10);

    expect_same_machine_state(regular, micro);
    EXPECT_EQ(regular.sim.cpu().regs().pc, 0x011A);
    EXPECT_EQ(regular.sim.cpu().regs().a, 0x06);
    EXPECT_EQ(regular.memory->data[0x0020], 0x02);
    EXPECT_EQ(regular.memory->data[0x3000], 0x06);
    EXPECT_EQ(regular.memory->data[0x3001], 0x06);
    EXPECT_EQ(micro.memory->data[0x0020], 0x02);
    EXPECT_EQ(micro.memory->data[0x3000], 0x06);
    EXPECT_EQ(micro.memory->data[0x3001], 0x06);
}

TEST(IntegrationExecutionTest, RegularAndMicroSteppedExecutionMatchForStackAndIndirectIndexedCode) {
    const std::vector<uint8_t> program = {
        0x8E, 0x30, 0x00, // LDX #$3000
        0x86, 0x42,       // LDA #$42
        0xA7, 0x94,       // STA [,X]
        0xBD, 0x01, 0x10, // JSR $0110
        0xC6, 0xAA,       // LDB #$AA
        0xF7, 0x40, 0x01, // STB $4001
        0x12,             // NOP
        0xF6, 0x40, 0x00, // LDB $4000
        0xCB, 0x01,       // ADDB #$01
        0xF7, 0x40, 0x00, // STB $4000
        0x39,             // RTS
    };

    IntegrationRun regular(microlind::CpuMode::HD6309);
    IntegrationRun micro(microlind::CpuMode::HD6309);
    load_program(*regular.memory, 0x0100, program);
    load_program(*micro.memory, 0x0100, program);
    regular.memory->data[0x3000] = 0x40;
    regular.memory->data[0x3001] = 0x00;
    micro.memory->data[0x3000] = 0x40;
    micro.memory->data[0x3001] = 0x00;
    regular.sim.cpu().regs().s = 0x9000;
    micro.sim.cpu().regs().s = 0x9000;

    run_regular(regular, 0x0100, 10);
    run_microstepped(micro, 0x0100, 10);

    expect_same_machine_state(regular, micro);
    EXPECT_EQ(regular.sim.cpu().regs().pc, 0x010F);
    EXPECT_EQ(regular.sim.cpu().regs().a, 0x42);
    EXPECT_EQ(regular.sim.cpu().regs().b, 0xAA);
    EXPECT_EQ(regular.sim.cpu().regs().s, 0x9000);
    EXPECT_EQ(regular.memory->data[0x4000], 0x43);
    EXPECT_EQ(regular.memory->data[0x4001], 0xAA);
    EXPECT_EQ(micro.memory->data[0x4000], 0x43);
    EXPECT_EQ(micro.memory->data[0x4001], 0xAA);
}

} // namespace
