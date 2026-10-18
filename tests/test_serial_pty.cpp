#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <string>
#include <thread>
#include <vector>

#include "gui_runtime.hpp"
#include "microlind/app/serial_pty.hpp"
#include "microlind/app/sim_session.hpp"
#include "microlind/devices/serial.hpp"
#include "test_harness.hpp"

#if defined(__unix__) || defined(__APPLE__)
#include <fcntl.h>
#include <unistd.h>
#endif

namespace {

TEST(SerialDeviceTest, BothChannelsHaveIndependentReceiveFifosAndInterrupts) {
    microlind::devices::XR88C92 uart;
    uart.write8(5, 0x22); // RX interrupts on A and B.
    uart.inject_rx('A');
    uart.inject_rx_b('B');
    uart.inject_rx_b('C');
    EXPECT_EQ(uart.interrupt_status() & 0x22, 0x22);
    EXPECT_EQ(uart.status_a() & 1, 1);
    EXPECT_EQ(uart.status_b() & 1, 1);
    EXPECT_EQ(uart.peek8(11), 'B');
    EXPECT_EQ(uart.peek8(11), 'B');
    EXPECT_EQ(uart.read8(3), 'A');
    EXPECT_EQ(uart.interrupt_status() & 0x22, 0x20);
    EXPECT_TRUE(uart.irq_asserted());
    EXPECT_EQ(uart.read8(11), 'B');
    EXPECT_EQ(uart.read8(11), 'C');
    EXPECT_EQ(uart.status_b() & 1, 0);
    EXPECT_FALSE(uart.irq_asserted());
    uart.write8(5, 0x10); // B TX-ready interrupt.
    EXPECT_TRUE(uart.irq_asserted());
}

TEST(SerialDeviceTest, BothChannelsTransmitToTheirOwnCallbacks) {
    std::string a, b;
    microlind::devices::XR88C92 uart([&](uint8_t value) { a += static_cast<char>(value); });
    uart.set_tx_b_callback([&](uint8_t value) { b += static_cast<char>(value); });
    uart.write8(3, 'A');
    uart.write8(11, 'B');
    EXPECT_EQ(a, "A");
    EXPECT_EQ(b, "B");
}

#if defined(__unix__) || defined(__APPLE__)

struct Terminal {
    int fd{-1};
    explicit Terminal(const std::string& path) {
        fd = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    }
    ~Terminal() { if (fd >= 0) ::close(fd); }
    Terminal(const Terminal&) = delete;
    Terminal& operator=(const Terminal&) = delete;

    std::vector<uint8_t> read_available() const {
        std::vector<uint8_t> result;
        std::array<uint8_t, 4096> bytes{};
        ssize_t count;
        while ((count = ::read(fd, bytes.data(), bytes.size())) > 0) {
            result.insert(result.end(), bytes.begin(), bytes.begin() + count);
        }
        return result;
    }
};

TEST(SerialPtyTest, TransfersBinaryBytesWithoutEchoOrLineTranslation) {
    microlind::app::SerialPty port;
    ASSERT_TRUE(port.open()) << port.error();
    Terminal terminal(port.path());
    ASSERT_GE(terminal.fd, 0);
    const std::vector<uint8_t> bytes{0x00, '\r', '\n', 0x03, 0x11, 0x13, 0x7F, 0x80, 0xFF};
    ASSERT_EQ(::write(terminal.fd, bytes.data(), bytes.size()), static_cast<ssize_t>(bytes.size()));
    std::vector<uint8_t> received;
    port.poll([&](uint8_t value) { received.push_back(value); });
    EXPECT_EQ(received, bytes);
    EXPECT_TRUE(terminal.read_available().empty()); // No local echo.
    for (uint8_t value : bytes) port.transmit(value);
    port.poll([](uint8_t) { FAIL() << "Output echoed back into RX"; });
    EXPECT_EQ(terminal.read_available(), bytes);
}

TEST(SerialPtyTest, BuffersPartialWritesAndRetainsPortAcrossClientReconnects) {
    microlind::app::SerialPty port;
    ASSERT_TRUE(port.open()) << port.error();
    const auto path = port.path();
    {
        Terminal terminal(path);
        ASSERT_GE(terminal.fd, 0);
    }
    std::vector<uint8_t> expected(256 * 1024);
    for (std::size_t i = 0; i < expected.size(); ++i) {
        expected[i] = static_cast<uint8_t>(i);
        port.transmit(expected[i]);
    }
    port.poll([](uint8_t) { FAIL(); });
    ASSERT_TRUE(port.is_open());
    Terminal terminal(path);
    ASSERT_GE(terminal.fd, 0);
    std::vector<uint8_t> actual;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (actual.size() < expected.size() && std::chrono::steady_clock::now() < deadline) {
        port.poll([](uint8_t) { FAIL(); });
        auto chunk = terminal.read_available();
        actual.insert(actual.end(), chunk.begin(), chunk.end());
    }
    EXPECT_EQ(actual, expected);
    EXPECT_EQ(port.path(), path);
    port.close();
    EXPECT_FALSE(port.is_open());
    EXPECT_TRUE(port.path().empty());
    EXPECT_EQ(::access(path.c_str(), F_OK), -1);
}

TEST(SerialPtyTest, ReportsOverflowWhenTerminalDoesNotRead) {
    microlind::app::SerialPty port;
    ASSERT_TRUE(port.open()) << port.error();
    for (std::size_t i = 0; i <= 1024 * 1024; ++i) port.transmit('x');
    EXPECT_FALSE(port.is_open());
    EXPECT_NE(port.error().find("buffer full"), std::string::npos);
}

TEST(SerialPtyTest, BothSessionPortsSurviveResetAndHardwareRebuild) {
    microlind::app::SimSession session;
    ASSERT_TRUE(session.load_hardware_config("tests/data/hw_test.cfg"));
    ASSERT_TRUE(session.set_serial_pty_enabled(false, true));
    ASSERT_TRUE(session.set_serial_pty_enabled(true, true));
    const auto paths = session.serial_snapshot().pty_paths;
    ASSERT_NE(paths[0], paths[1]);
    Terminal a(paths[0]), b(paths[1]);
    ASSERT_GE(a.fd, 0);
    ASSERT_GE(b.fd, 0);
    session.reset();
    ASSERT_TRUE(session.load_hardware_config("tests/data/hw_test.cfg"));
    EXPECT_EQ(session.serial_snapshot().pty_paths, paths);
    ASSERT_EQ(::write(a.fd, "A", 1), 1);
    ASSERT_EQ(::write(b.fd, "B", 1), 1);
    session.poll_serial_ports();
    EXPECT_EQ(session.peek_memory(0xF433), 'A');
    EXPECT_EQ(session.peek_memory(0xF43B), 'B');
    session.write_memory(0xF433, 'a');
    session.write_memory(0xF43B, 'b');
    session.poll_serial_ports();
    EXPECT_EQ(a.read_available(), std::vector<uint8_t>{'a'});
    EXPECT_EQ(b.read_available(), std::vector<uint8_t>{'b'});
    ASSERT_TRUE(session.set_serial_pty_enabled(false, false));
    EXPECT_TRUE(session.serial_snapshot().pty_paths[0].empty());
    EXPECT_EQ(session.serial_snapshot().pty_paths[1], paths[1]);
}

TEST(SerialPtyTest, TrueRunBootsAndEchoesOnBothPortsWithoutGuiPolling) {
    microlind::gui::GuiRuntime runtime;
    ASSERT_TRUE(runtime.load_hardware_config("tests/data/hw_test.cfg"));
    const std::vector<uint8_t> program{
        0x86, 'a', 0xB7, 0xF4, 0x33,
        0x86, 'b', 0xB7, 0xF4, 0x3B,
        0xB6, 0xF4, 0x31, 0x85, 0x01, 0x27, 0x06,
        0xB6, 0xF4, 0x33, 0xB7, 0xF4, 0x33,
        0xB6, 0xF4, 0x39, 0x85, 0x01, 0x27, 0x06,
        0xB6, 0xF4, 0x3B, 0xB7, 0xF4, 0x3B,
        0x20, 0xE4,
    }; // Print per-channel boot banners, then poll and echo both RX FIFOs.
    std::vector<uint8_t> rom(0x2000, 0x12);
    std::copy(program.begin(), program.end(), rom.begin());
    rom[0x1FFE] = 0xE0; // Reset vector $E000.
    rom[0x1FFF] = 0x00;
    const auto rom_path = microlind::test::test_output_path("serial-console-boot.rom");
    {
        std::ofstream file(rom_path, std::ios::binary);
        file.write(reinterpret_cast<const char*>(rom.data()), static_cast<std::streamsize>(rom.size()));
        ASSERT_TRUE(file.good());
    }
    ASSERT_TRUE(runtime.load_rom(rom_path, microlind::cli::RomFormat::Raw, 0xE000));
    ASSERT_TRUE(runtime.set_serial_pty_enabled(false, true));
    ASSERT_TRUE(runtime.set_serial_pty_enabled(true, true));
    const auto paths = runtime.debugger_snapshot().serial.pty_paths;
    Terminal a(paths[0]), b(paths[1]);
    ASSERT_GE(a.fd, 0);
    ASSERT_GE(b.fd, 0);
    ASSERT_EQ(::write(a.fd, "A\r", 2), 2);
    ASSERT_EQ(::write(b.fd, "B\r", 2), 2);
    runtime.start_true_run(1'000'000);
    std::vector<uint8_t> received_a, received_b;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while ((received_a.size() < 3 || received_b.size() < 3) && std::chrono::steady_clock::now() < deadline) {
        auto chunk_a = a.read_available();
        auto chunk_b = b.read_available();
        received_a.insert(received_a.end(), chunk_a.begin(), chunk_a.end());
        received_b.insert(received_b.end(), chunk_b.begin(), chunk_b.end());
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    runtime.stop_true_run();
    EXPECT_EQ(received_a, (std::vector<uint8_t>{'a', 'A', '\r'}));
    EXPECT_EQ(received_b, (std::vector<uint8_t>{'b', 'B', '\r'}));
}

#endif

} // namespace
