#include <gtest/gtest.h>
#include "microlind/bus.hpp"
#include "microlind/simulator.hpp"
#include "microlind/devices/memory.hpp"
#include <array>

namespace {
using namespace microlind;
class Probe : public BusDevice {
public:
    uint8_t read8(uint16_t offset) override { ++reads; return bytes[offset]; }
    uint8_t peek8(uint16_t offset) override { return bytes[offset]; }
    void write8(uint16_t offset, uint8_t value) override { ++writes; bytes[offset] = value; }
    std::array<uint8_t, 1024> bytes{};
    unsigned reads{}, writes{};
};

TEST(PrivilegeGate, OnlyQualifiedSwiNmiResetVectorsUnlock) {
    Bus bus;
    bus.enable_privilege_gate(true);
    for (uint16_t address : {0xFFF0,0xFFF1,0xFFF2,0xFFF3,0xFFF4,0xFFF5,0xFFF6,0xFFF7,0xFFF8,0xFFF9}) {
        bus.read8(address, BusCycleKind::VectorRead);
        EXPECT_FALSE(bus.privileged());
    }
    for (uint16_t address : {0xFFFA,0xFFFB,0xFFFC,0xFFFD,0xFFFE,0xFFFF}) {
        bus.read8(address); EXPECT_FALSE(bus.privileged());
        bus.peek8(address); EXPECT_FALSE(bus.privileged());
        BusSignals unqualified;
        unqualified.memory_enable = true; unqualified.rw = true;
        unqualified.address = address; unqualified.cycle_kind = BusCycleKind::VectorRead;
        bus.tick_bus_cycle(unqualified); EXPECT_FALSE(bus.privileged()); // BA=1
        bus.read8(address, BusCycleKind::VectorRead); EXPECT_TRUE(bus.privileged());
        EXPECT_EQ(bus.read8(0xF405) & 0x80, 0x80);
        bus.write8(0xF405, 0xA5); EXPECT_FALSE(bus.privileged());
        EXPECT_EQ(bus.read8(0xF405) & 0x80, 0);
    }
    bus.read8(0x00FA, BusCycleKind::VectorRead); EXPECT_FALSE(bus.privileged());
}

TEST(PrivilegeGate, EveryIoWriteIsDeniedButReadsAndRamKeepTheirEffects) {
    Bus bus;
    auto probe = std::make_unique<Probe>(); auto* io = probe.get();
    ASSERT_FALSE(bus.map_device(0xF400, 0xF7FF, std::move(probe)));
    ASSERT_FALSE(bus.map_device(0, 0xDFFF, std::make_unique<devices::Memory>(0xE000, true)));
    bus.enable_privilege_gate(true);
    for (unsigned address = 0xF400; address <= 0xF7FF; ++address) {
        bus.write8(address, 0x5A); bus.read8(address);
    }
    EXPECT_EQ(io->writes, 0u); EXPECT_EQ(io->reads, 1023u); // Status is the latch.
    bus.write8(0x1234, 0xA7); EXPECT_EQ(bus.read8(0x1234), 0xA7);
    bus.write8(0xC100, 0xB8); EXPECT_EQ(bus.read8(0xC100), 0xB8);
    bus.read8(0xFFFE, BusCycleKind::VectorRead);
    for (unsigned address = 0xF400; address <= 0xF7FF; ++address) {
        if (address != 0xF405) bus.write8(address, 0x5A);
    }
    EXPECT_EQ(io->writes, 1023u);
    bus.write8(0xF405, 0); EXPECT_FALSE(bus.privileged());
}

TEST(PrivilegeGate, DeferredWritesCommitInOrderInBothPhaseModes) {
    for (bool detailed : {false, true}) {
        Bus bus; auto probe = std::make_unique<Probe>(); auto* io = probe.get();
        ASSERT_FALSE(bus.map_device(0xF400, 0xF7FF, std::move(probe)));
        bus.enable_privilege_gate(true); bus.set_detailed_bus_phases(detailed);
        bus.begin_deferred_bus_cycles();
        bus.read8(0xFFFA, BusCycleKind::VectorRead);
        bus.write8(0xF400, 7); bus.write8(0xF405, 0); bus.write8(0xF400, 9);
        EXPECT_FALSE(bus.privileged()); EXPECT_EQ(io->writes, 0u);
        for (auto cycle : bus.take_deferred_bus_cycles()) bus.tick_bus_cycle(cycle);
        EXPECT_FALSE(bus.privileged()); EXPECT_EQ(io->bytes[0], 7); EXPECT_EQ(io->writes, 1u);
        BusSignals vector; vector.memory_enable = true; vector.rw = true;
        vector.ba = false; vector.bs = true; vector.address = 0xFFFC;
        vector.cycle_kind = BusCycleKind::VectorRead;
        bus.tick_bus_cycle(vector); EXPECT_TRUE(bus.privileged());
        BusSignals status; status.memory_enable = true; status.rw = true;
        status.address = 0xF405; status.apply_read = true;
        bus.tick_bus_cycle(status); EXPECT_EQ(bus.last_signals().data & 0x80, 0x80);
    }
}

TEST(PrivilegeGate, NativeSwiAndRtiAgreeForInstructionAndMicrocycleExecution) {
    for (bool micro : {false,true}) {
        Simulator sim(CpuMode::HD6309, 3'000'000);
        auto ram = std::make_unique<devices::Memory>(65536, true);
        // LDA #1; LDMD #1; LDS #$3000; SWI; NOP; ROM: STA $F405; RTI.
        ram->load(0x100, {0x86,1,0x11,0x3D,1,0x10,0xCE,0x30,0,0x3F,0x12});
        ram->load(0xE000, {0xB7,0xF4,5,0x3B});
        ram->load(0xFFFA, {0xE0,0}); ram->load(0xFFFE, {1,0});
        ASSERT_FALSE(sim.map_device(0,0xFFFF,std::move(ram)));
        sim.bus().enable_privilege_gate(true); sim.reset_from_vector();
        EXPECT_TRUE(sim.bus().privileged());
        sim.bus().write8(0xF405,0);
        for (unsigned steps=0; sim.cpu().regs().pc != 0x10A && steps<300; ++steps) {
            if (micro) sim.tick_microcycle(); else sim.tick();
        }
        EXPECT_EQ(sim.cpu().regs().pc,0x10A); EXPECT_EQ(sim.cpu().regs().s,0x3000);
        EXPECT_EQ(sim.cpu().regs().a,1); EXPECT_FALSE(sim.bus().privileged());
    }
}
} // namespace

#include "microlind/devices/serial.hpp"
TEST(MlosTimer, Mode7PeriodReadAcknowledgeAndSharedRxMask) {
    microlind::devices::XR88C92 uart(nullptr, nullptr, 3'686'400);
    uart.write8(4, 0xF0); uart.write8(6,0); uart.write8(7,115);
    uart.write8(5,0x0A); // RXA and timer both enabled.
    uart.read8(14);
    uart.tick(3679); EXPECT_EQ(uart.interrupt_status() & 8,0);
    uart.tick(1); EXPECT_EQ(uart.interrupt_status() & 8,8);
    EXPECT_TRUE(uart.irq_asserted());
    uart.peek8(15); EXPECT_EQ(uart.interrupt_status() & 8,8);
    uart.inject_rx(0x61);
    uart.read8(15); EXPECT_EQ(uart.interrupt_status() & 8,0);
    EXPECT_TRUE(uart.irq_asserted()); // RXA mask was retained.
    EXPECT_EQ(uart.read8(3),0x61); EXPECT_FALSE(uart.irq_asserted());
    uart.tick(3680); EXPECT_TRUE(uart.irq_asserted()); // Ack did not stop timer.
}
TEST(MlosTimer, RationalAccumulatorIsIndependentOfTickChunkSize) {
    for (uint32_t chunk : {1u, 10u, 100u, 1000u}) {
        microlind::devices::XR88C92 uart(nullptr, nullptr, 3'000'000);
        uart.write8(4,0x70); uart.write8(6,0); uart.write8(7,115); uart.read8(14);
        unsigned edges=0;
        for (unsigned cycles=0;cycles<3'000'000;cycles+=chunk) {
            uart.tick(chunk);
            if (uart.interrupt_status() & 8) { ++edges; uart.read8(15); }
        }
        EXPECT_EQ(edges,1001u);
    }
}

TEST(MlosSerial, InteractiveAndLogSinksAreIndependent) {
    std::string interactive,logs;
    microlind::devices::XR88C92 uart([&](uint8_t b){interactive+=char(b);});
    uart.set_tx_b_callback([&](uint8_t b){logs+=char(b);});
    uart.write8(3,'A');uart.write8(11,'B');
    EXPECT_EQ(interactive,"A");EXPECT_EQ(logs,"B");
}
#include "microlind/app/hardware_config.hpp"
#include "microlind/app/sim_builder.hpp"
#include "microlind/devices/compact_flash.hpp"
#include <filesystem>
#include <fstream>
TEST(MlosConfig, ClockAndSecondControllerAreExplicitAndValidated) {
    const auto path=std::filesystem::temp_directory_path()/"mlos-hardware-config-test.cfg";
    {std::ofstream f(path);f<<"[CPU]\nCLOCK_HZ=3000000\n[CF]\nIO_START_ADDRESS=0xF418\nIO_END_ADDRESS=0xF41F\nSECTORS=128\n[CF2]\nIO_START_ADDRESS=0xF640\nIO_END_ADDRESS=0xF647\nSECTORS=256\n";}
    std::string error;auto cfg=microlind::cli::load_hardware_config(path,error);
    ASSERT_TRUE(cfg.has_value())<<error;EXPECT_EQ(cfg->cpu_hz,3000000u);
    EXPECT_EQ(cfg->cf.sectors,128u);EXPECT_EQ(cfg->cf2.sectors,256u);
    auto image1=path;image1.replace_extension("first.img");
    auto image2=path;image2.replace_extension("second.img");
    {std::ofstream f(image1,std::ios::binary);f.seekp(128*512-1);f.put(0);}
    {std::ofstream f(image2,std::ios::binary);f.seekp(256*512-1);f.put(0);}
    cfg->cf.image_path=image1;cfg->cf2.image_path=image2;
    microlind::devices::CompactFlash *first=nullptr,*second=nullptr;
    auto sim=microlind::cli::build_sim(microlind::CpuMode::HD6309,nullptr,&*cfg,nullptr,nullptr,nullptr,&first,nullptr,nullptr,nullptr,&second);
    EXPECT_EQ(sim.clock().frequency_hz(),3000000u);ASSERT_NE(first,nullptr);ASSERT_NE(second,nullptr);
    EXPECT_NE(first,second);EXPECT_EQ(first->sector_count(),128u);EXPECT_EQ(second->sector_count(),256u);
    {std::ofstream f(path);f<<"[CPU]\nCLOCK_HZ=0\n";}
    EXPECT_FALSE(microlind::cli::load_hardware_config(path,error));
    std::filesystem::remove(path);
    std::filesystem::remove(image1);std::filesystem::remove(image2);
}

TEST(MlosStorage, DelayedControllerDoesNotDelayAnIndependentInstance) {
    const auto path=std::filesystem::temp_directory_path()/"mlos-cf-delay-test.img";
    {std::ofstream f(path,std::ios::binary);f.seekp(512*4-1);f.put(0);}
    microlind::devices::CompactFlash first({path,4,true}), second({path,4,true});
    first.set_command_delay_cycles(100);first.write8(3,0);first.write8(7,0x20);
    EXPECT_EQ(first.read8(7)&0x80,0x80);first.tick(99);EXPECT_EQ(first.read8(7)&8,0);
    second.write8(3,0);second.write8(7,0x20);EXPECT_EQ(second.read8(7)&8,8);
    first.tick(1);EXPECT_EQ(first.read8(7)&0x80,0);EXPECT_EQ(first.read8(7)&8,8);
    first.set_command_delay_cycles(100);first.write8(7,0x20);first.unload_disk_image();first.tick(100);
    EXPECT_EQ(first.read8(7),0xFF);std::filesystem::remove(path);
}

TEST(MlosRuntimeCpu, TransferAndExchangePcUseThePostInstructionAddress) {
    struct Case {uint8_t opcode,post;uint16_t pc,x;};
    for(auto mode:{microlind::CpuMode::MC6809,microlind::CpuMode::HD6309})
    for(bool micro:{false,true})
    for(auto c:{Case{0x1F,0x15,0x2345,0x2345},Case{0x1F,0x51,0x0102,0x0102},
                Case{0x1F,0x55,0x0102,0x2345},Case{0x1E,0x15,0x2345,0x0102},
                Case{0x1E,0x51,0x2345,0x0102},Case{0x1E,0x55,0x0102,0x2345}}) {
        microlind::Simulator sim(mode,3000000);
        auto memory=std::make_unique<microlind::devices::Memory>(65536,true);
        memory->load(0x100,{c.opcode,c.post});
        ASSERT_FALSE(sim.map_device(0,0xFFFF,std::move(memory)));
        auto&r=sim.cpu().regs();r.pc=0x100;r.x=0x2345;r.cc=0xF0;
        if(mode==microlind::CpuMode::HD6309)r.md=1;
        if(micro){do{sim.tick_microcycle();}while(sim.has_pending_microcycles());}else sim.tick();
        EXPECT_EQ(r.pc,c.pc)<<"opcode="<<int(c.opcode)<<" post="<<int(c.post)<<" micro="<<micro;
        EXPECT_EQ(r.x,c.x);EXPECT_EQ(r.cc,0xF0);
    }
}
