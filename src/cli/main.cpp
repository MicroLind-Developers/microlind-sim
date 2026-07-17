#include <cctype>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "microlind/app/disassembler.hpp"
#include "microlind/app/hardware_config.hpp"
#include "microlind/app/image_loader.hpp"
#include "microlind/app/logic_validation.hpp"
#include "microlind/app/sim_builder.hpp"
#include "microlind/app/util.hpp"

#include "microlind/devices/serial.hpp"
#include "microlind/simulator.hpp"

using namespace microlind;
using namespace microlind::cli;

static void print_help() {
    std::cout << "Commands:\n"
              << "  help                     - show this help\n"
              << "  regs                     - show registers\n"
              << "  step [n]                 - execute n instructions (default 1)\n"
              << "  mstep [n]                - execute n bus microcycles (default 1)\n"
              << "  tick [n]                 - advance bus/clock by n cycles (default 1)\n"
              << "  run [n]                  - execute n instructions (default 1000)\n"
              << "  peek <addr> [count]      - read memory\n"
              << "  poke <addr> <value>      - write memory\n"
              << "  dump <addr> <len>        - hex dump\n"
              << "  loadbin <path> [base]    - load raw binary (default base 0x8000)\n"
              << "  loadihex <path>          - load Intel HEX (absolute addresses)\n"
              << "  loadsrec <path>          - load S-record (absolute addresses)\n"
              << "  loadcfg <path>           - load hardware config\n"
              << "  loadcf <path> [sectors]  - load raw CF disk image\n"
              << "  pldcfg <signal> <mem> <addr> - print partial hw.cfg from PLD logic\n"
              << "  pldcheck <cfg> <signal> <mem> <addr> - validate hw.cfg against PLD logic\n"
              << "  serin <text>             - push ASCII text into serial RX\n"
              << "  map                      - show mapped address ranges\n"
              << "  reset                    - reset PC from reset vector\n"
              << "  exit                     - quit\n";
}

static const char* bus_phase_label(BusPhase phase) {
    switch (phase) {
    case BusPhase::QHighELow: return "Q+ E-";
    case BusPhase::QHighEHigh: return "Q+ E+";
    case BusPhase::QLowEHigh: return "Q- E+";
    case BusPhase::QLowELow: return "Q- E-";
    }
    return "?";
}

static const char* bus_cycle_kind_label(BusCycleKind kind) {
    switch (kind) {
    case BusCycleKind::Idle: return "idle";
    case BusCycleKind::OpcodeFetch: return "opcode";
    case BusCycleKind::OperandRead: return "operand-r";
    case BusCycleKind::OperandWrite: return "operand-w";
    case BusCycleKind::StackRead: return "stack-r";
    case BusCycleKind::StackWrite: return "stack-w";
    case BusCycleKind::VectorRead: return "vector-r";
    case BusCycleKind::Internal: return "internal";
    }
    return "?";
}

static void print_microcycle_result(const SimulatorMicrocycleResult& result, const Simulator& sim) {
    if (!result.emitted) {
        std::cout << "No bus cycle emitted.\n";
        return;
    }

    const auto& s = result.signals;
    std::cout << std::hex << std::setfill('0')
              << "bus=" << std::dec << sim.bus().bus_cycle_count()
              << " phase=" << bus_phase_label(s.phase)
              << " kind=" << bus_cycle_kind_label(s.cycle_kind)
              << " addr=" << std::hex << std::setw(4) << s.address
              << " data=" << std::setw(2) << static_cast<int>(s.data)
              << " " << (s.rw ? "RD" : "WR")
              << " mem=" << (s.memory_enable ? 1 : 0)
              << " e=" << (s.e ? 1 : 0)
              << " q=" << (s.q ? 1 : 0)
              << std::dec
              << " pending=" << result.pending_bus_cycles;
    if (result.instruction_started) {
        std::cout << " started";
    }
    if (result.instruction_complete) {
        std::cout << " complete";
    }
    std::cout << "\n";
}

static void print_build_diagnostics(const std::vector<std::string>& diagnostics) {
    for (const auto& diagnostic : diagnostics) {
        std::cout << diagnostic << "\n";
    }
}

int main(int argc, char** argv) {
    CpuMode mode = CpuMode::HD6309;
    RomFormat fmt = RomFormat::None;
    std::filesystem::path rom_path;
    std::filesystem::path config_path;
    bool verbose = false;

    for (int i = 1; i < argc; ++i) {
        std::string arg{argv[i]};
        if (arg == "--6809") {
            mode = CpuMode::MC6809;
        } else if (arg == "--6309") {
            mode = CpuMode::HD6309;
        } else if (arg == "--rom" && i + 1 < argc) {
            rom_path = argv[++i];
        } else if (arg == "--ihex") {
            fmt = RomFormat::Ihex;
        } else if (arg == "--srec") {
            fmt = RomFormat::Srec;
        } else if (arg == "--config" && i + 1 < argc) {
            config_path = argv[++i];
        } else if (arg == "--verbose") {
            verbose = true;
        }
    }

    std::optional<LoadedImage> current_image;
    microlind::devices::XR88C92* serial_dev = nullptr;
    if (!rom_path.empty()) {
        current_image = load_image(rom_path, fmt == RomFormat::None ? RomFormat::Raw : fmt);
        if (!current_image) {
            std::cerr << "Failed to load image\n";
            return 1;
        }
    }

    std::optional<HardwareConfig> hw_cfg;
    if (!config_path.empty()) {
        std::string err;
        hw_cfg = load_hardware_config(config_path, err);
        if (!hw_cfg) {
            std::cerr << "Config error: " << err << "\n";
            return 1;
        }
        std::cout << "Loaded hardware config from " << config_path << "\n";
    }

    std::vector<std::string> build_diagnostics;
    Simulator sim = build_sim(
        mode,
        current_image ? &*current_image : nullptr,
        hw_cfg ? &*hw_cfg : nullptr,
        &serial_dev,
        nullptr,
        nullptr,
        nullptr,
        nullptr,
        &build_diagnostics);
    print_build_diagnostics(build_diagnostics);

    std::cout << "Microlind-sim interactive CLI. Type 'help' for commands.\n";

    std::string line;
    while (std::cout << "> " && std::getline(std::cin, line)) {
        std::istringstream iss(line);
        std::string cmd;
        if (!(iss >> cmd)) continue;
        for (auto& ch : cmd) ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));

        if (cmd == "help") {
            print_help();
        } else if (cmd == "exit" || cmd == "quit") {
            break;
        } else if (cmd == "regs") {
            const auto& r = sim.cpu().regs();
            std::cout << std::hex << std::setfill('0');
            std::cout << "PC=" << std::setw(4) << sim.cpu().regs().pc
                      << " A=" << std::setw(2) << static_cast<int>(r.a)
                      << " B=" << std::setw(2) << static_cast<int>(r.b)
                      << " E=" << std::setw(2) << static_cast<int>(r.e)
                      << " F=" << std::setw(2) << static_cast<int>(r.f)
                      << " DP=" << std::setw(2) << static_cast<int>(r.dp)
                      << " CC=" << std::setw(2) << static_cast<int>(r.cc)
                      << " X=" << std::setw(4) << r.x
                      << " Y=" << std::setw(4) << r.y
                      << " U=" << std::setw(4) << r.u
                      << " S=" << std::setw(4) << r.s
                      << " V=" << std::setw(4) << r.v
                      << " cycles=" << std::dec << sim.clock().total_cycles() << "\n";
        } else if (cmd == "step" || cmd == "run") {
            int n = (cmd == "run") ? 1000 : 1;
            if (iss) {
                std::string nstr;
                if (iss >> nstr) {
                    if (auto v = parse_number(nstr)) n = static_cast<int>(*v);
                }
            }
            for (int i = 0; i < n; ++i) {
                if (verbose) {
                    uint16_t pc = sim.cpu().regs().pc;
                    auto dasm = disassemble(sim.bus(), sim.cpu(), pc);
                    auto before_cycles = sim.clock().total_cycles();
                    auto res = sim.tick();
                    const auto& r = sim.cpu().regs();
                    std::cout << std::hex << std::setfill('0')
                              << "PC=" << std::setw(4) << pc
                              << " " << dasm.text
                              << " A=" << std::setw(2) << static_cast<int>(r.a)
                              << " B=" << std::setw(2) << static_cast<int>(r.b)
                              << " E=" << std::setw(2) << static_cast<int>(r.e)
                              << " F=" << std::setw(2) << static_cast<int>(r.f)
                              << " DP=" << std::setw(2) << static_cast<int>(r.dp)
                              << " CC=" << std::setw(2) << static_cast<int>(r.cc)
                              << " X=" << std::setw(4) << r.x
                              << " Y=" << std::setw(4) << r.y
                              << " U=" << std::setw(4) << r.u
                              << " S=" << std::setw(4) << r.s
                              << " V=" << std::setw(4) << r.v
                              << std::dec << " cycles=" << res.cycles
                              << " total=" << (before_cycles + res.cycles) << "\n";
                } else {
                    sim.tick();
                }
            }
            if (!verbose) {
                std::cout << "PC=" << std::hex << std::setw(4) << sim.cpu().regs().pc
                          << " cycles=" << std::dec << sim.clock().total_cycles() << "\n";
            }
        } else if (cmd == "mstep") {
            uint64_t n = 1;
            std::string nstr;
            if (iss >> nstr) {
                if (auto v = parse_number(nstr)) n = *v;
            }
            for (uint64_t i = 0; i < n; ++i) {
                const auto result = sim.tick_microcycle();
                print_microcycle_result(result, sim);
            }
        } else if (cmd == "tick") {
            uint64_t n = 1;
            std::string nstr;
            if (iss >> nstr) {
                if (auto v = parse_number(nstr)) n = *v;
            }
            sim.tick_clock(n);
            std::cout << "cycles=" << std::dec << sim.clock().total_cycles() << " (advanced " << n << ")\n";
        } else if (cmd == "peek") {
            std::string a1, a2;
            if (!(iss >> a1)) { std::cout << "Usage: peek <addr> [count]\n"; continue; }
            auto addr_opt = parse_number(a1);
            if (!addr_opt) { std::cout << "Bad address\n"; continue; }
            uint32_t count = 1;
            if (iss >> a2) { if (auto c = parse_number(a2)) count = *c; }
            uint32_t addr = *addr_opt;
            for (uint32_t i = 0; i < count; ++i) {
                if (i % 16 == 0) {
                    std::cout << std::hex << std::setfill('0') << std::setw(4) << (addr + i) << ": ";
                }
                uint16_t a = static_cast<uint16_t>(addr + i);
                uint8_t v = sim.bus().read8(a);
                std::cout << std::hex << std::setfill('0') << std::setw(2) << static_cast<int>(v) << " ";
                if (i % 16 == 15) std::cout << "\n";
            }
            if (count % 16 != 0) std::cout << "\n";
        } else if (cmd == "poke") {
            std::string a1, v1;
            if (!(iss >> a1 >> v1)) { std::cout << "Usage: poke <addr> <value>\n"; continue; }
            auto addr_opt = parse_number(a1);
            auto val_opt = parse_number(v1);
            if (!addr_opt || !val_opt) { std::cout << "Bad address/value\n"; continue; }
            sim.bus().write8(static_cast<uint16_t>(*addr_opt), static_cast<uint8_t>(*val_opt & 0xFF));
        } else if (cmd == "dump") {
            std::string a1, l1;
            if (!(iss >> a1 >> l1)) { std::cout << "Usage: dump <addr> <len>\n"; continue; }
            auto addr_opt = parse_number(a1);
            auto len_opt = parse_number(l1);
            if (!addr_opt || !len_opt) { std::cout << "Bad address/len\n"; continue; }
            uint16_t addr = static_cast<uint16_t>(*addr_opt);
            uint32_t len = *len_opt;
            for (uint32_t i = 0; i < len; ++i) {
                if (i % 16 == 0) {
                    std::cout << std::hex << std::setfill('0') << std::setw(4) << (addr + i) << ": ";
                }
                uint8_t v = sim.bus().read8(static_cast<uint16_t>(addr + i));
                std::cout << std::setw(2) << static_cast<int>(v) << " ";
                if (i % 16 == 15) std::cout << "\n";
            }
            if (len % 16 != 0) std::cout << "\n";
        } else if (cmd == "loadbin" || cmd == "loadihex" || cmd == "loadsrec") {
            std::string path;
            if (!(iss >> path)) { std::cout << "Usage: " << cmd << " <path> [base]\n"; continue; }
            RomFormat f = cmd == "loadbin" ? RomFormat::Raw : (cmd == "loadihex" ? RomFormat::Ihex : RomFormat::Srec);
            uint16_t base = 0x8000;
            std::string base_str;
            if (cmd == "loadbin" && (iss >> base_str)) {
                if (auto v = parse_number(base_str)) base = static_cast<uint16_t>(*v);
            }
            current_image = load_image(path, f, base);
            if (!current_image) { std::cout << "Failed to load image\n"; continue; }
            build_diagnostics.clear();
            sim = build_sim(mode, &*current_image, hw_cfg ? &*hw_cfg : nullptr, &serial_dev, nullptr, nullptr, nullptr, nullptr, &build_diagnostics);
            print_build_diagnostics(build_diagnostics);
            std::cout << "Image loaded and CPU reset.\n";
        } else if (cmd == "loadcfg") {
            std::string path;
            if (!(iss >> path)) { std::cout << "Usage: loadcfg <path>\n"; continue; }
            std::string err;
            auto cfg = load_hardware_config(path, err);
            if (!cfg) {
                std::cout << "Config error: " << err << "\n";
                continue;
            }
            hw_cfg = std::move(cfg);
            build_diagnostics.clear();
            sim = build_sim(mode, current_image ? &*current_image : nullptr, hw_cfg ? &*hw_cfg : nullptr, &serial_dev, nullptr, nullptr, nullptr, nullptr, &build_diagnostics);
            print_build_diagnostics(build_diagnostics);
            std::cout << "Hardware config loaded and CPU reset.\n";
        } else if (cmd == "loadcf") {
            std::string path;
            if (!(iss >> path)) { std::cout << "Usage: loadcf <path> [sectors]\n"; continue; }
            if (!hw_cfg || !hw_cfg->cf.present) {
                std::cout << "No CF device configured.\n";
                continue;
            }
            std::ifstream image_file(path, std::ios::binary);
            if (!image_file) {
                std::cout << "Cannot open CF disk image\n";
                continue;
            }
            hw_cfg->cf.image_path = path;
            hw_cfg->cf.sectors = 0;
            std::string sectors_str;
            if (iss >> sectors_str) {
                if (auto sectors = parse_number(sectors_str)) {
                    hw_cfg->cf.sectors = *sectors;
                } else {
                    std::cout << "Bad sector count\n";
                    continue;
                }
            }
            build_diagnostics.clear();
            sim = build_sim(mode, current_image ? &*current_image : nullptr, hw_cfg ? &*hw_cfg : nullptr, &serial_dev, nullptr, nullptr, nullptr, nullptr, &build_diagnostics);
            print_build_diagnostics(build_diagnostics);
            std::cout << "CF disk image loaded and CPU reset.\n";
        } else if (cmd == "pldcfg") {
            std::string signal_path;
            std::string memory_path;
            std::string address_path;
            if (!(iss >> signal_path >> memory_path >> address_path)) {
                std::cout << "Usage: pldcfg <signal.pld> <memory.pld> <address.pld>\n";
                continue;
            }
            std::string err;
            const auto devices = load_board_logic_devices(signal_path, memory_path, address_path, err);
            if (!devices) {
                std::cout << err;
                continue;
            }
            std::cout << generate_partial_hardware_config_from_logic(*devices);
        } else if (cmd == "pldcheck") {
            std::string cfg_path;
            std::string signal_path;
            std::string memory_path;
            std::string address_path;
            if (!(iss >> cfg_path >> signal_path >> memory_path >> address_path)) {
                std::cout << "Usage: pldcheck <hw.cfg> <signal.pld> <memory.pld> <address.pld>\n";
                continue;
            }

            std::string err;
            const auto cfg = load_hardware_config(cfg_path, err);
            if (!cfg) {
                std::cout << "Config error: " << err << "\n";
                continue;
            }

            const auto devices = load_board_logic_devices(signal_path, memory_path, address_path, err);
            if (!devices) {
                std::cout << err;
                continue;
            }

            const auto issues = validate_hardware_config_against_logic(*cfg, *devices);
            if (issues.empty()) {
                std::cout << "PLD validation OK.\n";
            } else {
                std::cout << "PLD validation found " << issues.size() << " issue(s):\n";
                for (const auto& issue : issues) {
                    std::cout << "  " << format_logic_validation_issue(issue) << "\n";
                }
            }
        } else if (cmd == "map") {
            auto summary = sim.bus().map_summary();
            for (const auto& s : summary) {
                std::cout << s << "\n";
            }
        } else if (cmd == "serin") {
            if (!serial_dev) { std::cout << "No serial device mapped.\n"; continue; }
            std::string text;
            if (!std::getline(iss >> std::ws, text)) {
                std::cout << "Usage: serin <text>\n";
                continue;
            }
            for (unsigned char ch : text) {
                serial_dev->inject_rx(static_cast<uint8_t>(ch));
            }
        } else if (cmd == "reset") {
            sim.reset_from_vector();
            std::cout << "CPU reset from vector.\n";
        } else {
            std::cout << "Unknown command. Type 'help'.\n";
        }
    }

    return 0;
}
