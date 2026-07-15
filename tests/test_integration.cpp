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

TEST(IntegrationExecutionTest, RegularAndMicroSteppedExecutionMatchAcrossCpuModes) {
    const std::vector<uint8_t> program = {
        0x12,             // NOP
        0x86, 0x7F,       // LDA #$7F
        0x97, 0x20,       // STA <$20
        0xC6, 0x81,       // LDB #$81
        0xF7, 0x30, 0x00, // STB $3000
        0xCC, 0x12, 0x34, // LDD #$1234
        0xDD, 0x22,       // STD <$22
    };

    struct Case {
        const char* name{};
        microlind::CpuMode mode{};
        uint8_t md{};
    };

    const Case cases[] = {
        {"MC6809", microlind::CpuMode::MC6809, 0x00},
        {"HD6309 emulation", microlind::CpuMode::HD6309, 0x00},
        {"HD6309 native", microlind::CpuMode::HD6309, 0x01},
    };

    for (const auto& test : cases) {
        SCOPED_TRACE(test.name);
        IntegrationRun regular(test.mode);
        IntegrationRun micro(test.mode);
        load_program(*regular.memory, 0x0100, program);
        load_program(*micro.memory, 0x0100, program);
        regular.sim.cpu().regs().md = test.md;
        micro.sim.cpu().regs().md = test.md;

        run_regular(regular, 0x0100, 7);
        run_microstepped(micro, 0x0100, 7);

        expect_same_machine_state(regular, micro);
        EXPECT_EQ(regular.memory->data[0x0020], 0x7F);
        EXPECT_EQ(regular.memory->data[0x0022], 0x12);
        EXPECT_EQ(regular.memory->data[0x0023], 0x34);
        EXPECT_EQ(regular.memory->data[0x3000], 0x81);
        EXPECT_EQ(micro.memory->data[0x0020], 0x7F);
        EXPECT_EQ(micro.memory->data[0x0022], 0x12);
        EXPECT_EQ(micro.memory->data[0x0023], 0x34);
        EXPECT_EQ(micro.memory->data[0x3000], 0x81);
    }
}

TEST(IntegrationExecutionTest, RegularAndMicroSteppedExecutionMatchForHD6309WAndQModes) {
    const std::vector<uint8_t> program = {
        0x10, 0x86, 0x12, 0x34,       // LDW #$1234
        0x10, 0x97, 0x20,             // STW <$20
        0xCD, 0x01, 0x02, 0x03, 0x04, // LDQ #$01020304
        0x10, 0xDD, 0x30,             // STQ <$30
        0x10, 0x96, 0x20,             // LDW <$20
        0x10, 0xDC, 0x30,             // LDQ <$30
    };

    for (const uint8_t md : {uint8_t{0x00}, uint8_t{0x01}}) {
        SCOPED_TRACE(md == 0 ? "HD6309 emulation" : "HD6309 native");
        IntegrationRun regular(microlind::CpuMode::HD6309);
        IntegrationRun micro(microlind::CpuMode::HD6309);
        load_program(*regular.memory, 0x0100, program);
        load_program(*micro.memory, 0x0100, program);
        regular.sim.cpu().regs().md = md;
        micro.sim.cpu().regs().md = md;

        run_regular(regular, 0x0100, 6);
        run_microstepped(micro, 0x0100, 6);

        expect_same_machine_state(regular, micro);
        EXPECT_EQ(regular.memory->data[0x0020], 0x12);
        EXPECT_EQ(regular.memory->data[0x0021], 0x34);
        EXPECT_EQ(regular.memory->data[0x0030], 0x01);
        EXPECT_EQ(regular.memory->data[0x0031], 0x02);
        EXPECT_EQ(regular.memory->data[0x0032], 0x03);
        EXPECT_EQ(regular.memory->data[0x0033], 0x04);
        EXPECT_EQ(micro.memory->data[0x0020], 0x12);
        EXPECT_EQ(micro.memory->data[0x0021], 0x34);
        EXPECT_EQ(micro.memory->data[0x0030], 0x01);
        EXPECT_EQ(micro.memory->data[0x0031], 0x02);
        EXPECT_EQ(micro.memory->data[0x0032], 0x03);
        EXPECT_EQ(micro.memory->data[0x0033], 0x04);
    }
}

TEST(IntegrationExecutionTest, RegularAndMicroSteppedExecutionMatchForTfmTimingAndState) {
    const std::vector<uint8_t> program = {
        0x11, 0x38, 0x12, // TFM X+,Y+
    };

    IntegrationRun regular(microlind::CpuMode::HD6309);
    IntegrationRun micro(microlind::CpuMode::HD6309);
    load_program(*regular.memory, 0x0100, program);
    load_program(*micro.memory, 0x0100, program);
    regular.memory->data[0x2000] = 0xAA;
    regular.memory->data[0x2001] = 0xBB;
    micro.memory->data[0x2000] = 0xAA;
    micro.memory->data[0x2001] = 0xBB;
    regular.sim.cpu().regs().x = 0x2000;
    regular.sim.cpu().regs().y = 0x3000;
    regular.sim.cpu().regs().e = 0x00;
    regular.sim.cpu().regs().f = 0x02;
    micro.sim.cpu().regs().x = 0x2000;
    micro.sim.cpu().regs().y = 0x3000;
    micro.sim.cpu().regs().e = 0x00;
    micro.sim.cpu().regs().f = 0x02;

    run_regular(regular, 0x0100, 1);
    run_microstepped(micro, 0x0100, 1);

    expect_same_machine_state(regular, micro);
    EXPECT_EQ(regular.sim.clock().total_cycles(), 12u);
    EXPECT_EQ(regular.memory->data[0x3000], 0xAA);
    EXPECT_EQ(regular.memory->data[0x3001], 0xBB);
    EXPECT_EQ(micro.memory->data[0x3000], 0xAA);
    EXPECT_EQ(micro.memory->data[0x3001], 0xBB);
}

} // namespace
