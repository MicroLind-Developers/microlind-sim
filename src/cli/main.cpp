#include <algorithm>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

#include "microlind/devices/memory.hpp"
#include "microlind/devices/memory_mapper.hpp"
#include "microlind/devices/serial.hpp"
#include "microlind/logic.hpp"
#include "microlind/simulator.hpp"

using namespace microlind;

std::vector<uint8_t> load_file(const std::filesystem::path& path) {
    std::ifstream file(path, std::ios::binary);
    return std::vector<uint8_t>(std::istreambuf_iterator<char>(file), {});
}

// Very small Intel HEX loader: only supports data records (00) and EOF (01).
// Address range is clipped to 64K.
std::vector<uint8_t> load_ihex(const std::filesystem::path& path) {
    std::vector<uint8_t> image(65536, 0xFF);
    std::ifstream file(path);
    std::string line;
    while (std::getline(file, line)) {
        if (line.empty() || line[0] != ':') continue;
        std::string_view sv(line);
        auto hex = [](std::string_view s) -> uint8_t {
            return static_cast<uint8_t>(std::stoul(std::string{s}, nullptr, 16));
        };
        const uint8_t count = hex(sv.substr(1, 2));
        const uint16_t addr = static_cast<uint16_t>(std::stoul(std::string{sv.substr(3, 4)}, nullptr, 16));
        const uint8_t type = hex(sv.substr(7, 2));
        if (type == 0x00) {
            for (uint8_t i = 0; i < count; ++i) {
                const size_t idx = 9 + i * 2;
                if (idx + 2 > sv.size()) break;
                uint8_t b = hex(sv.substr(idx, 2));
                const uint16_t a = static_cast<uint16_t>(addr + i);
                image[a] = b;
            }
        } else if (type == 0x01) {
            break; // EOF
        }
    }
    return image;
}

// Minimal S-Record (S1/S2/S3) loader. Clips addresses to 64K.
std::vector<uint8_t> load_srec(const std::filesystem::path& path) {
    std::vector<uint8_t> image(65536, 0xFF);
    std::ifstream file(path);
    std::string line;
    auto hex = [](std::string_view s) -> uint8_t {
        return static_cast<uint8_t>(std::stoul(std::string{s}, nullptr, 16));
    };
    while (std::getline(file, line)) {
        if (line.size() < 4 || line[0] != 'S') continue;
        const char t = line[1];
        if (t == '1' || t == '2' || t == '3') {
            std::string_view sv(line);
            const uint8_t count = hex(sv.substr(2, 2));
            size_t idx = 4;
            uint32_t addr = 0;
            if (t == '1') { addr = std::stoul(std::string{sv.substr(idx, 4)}, nullptr, 16); idx += 4; }
            else if (t == '2') { addr = std::stoul(std::string{sv.substr(idx, 6)}, nullptr, 16); idx += 6; }
            else { addr = std::stoul(std::string{sv.substr(idx, 8)}, nullptr, 16); idx += 8; }
            const uint8_t data_len = static_cast<uint8_t>(count - (idx/2 - 2) - 1);
            for (uint8_t i = 0; i < data_len; ++i) {
                if (idx + 2 > sv.size()) break;
                uint8_t b = hex(sv.substr(idx, 2));
                idx += 2;
                image[static_cast<uint16_t>(addr + i)] = b;
            }
        }
    }
    return image;
}

enum class RomFormat { None, Raw, Ihex, Srec };

struct LoadedImage {
    std::vector<uint8_t> data;
    uint16_t base{0};
};

struct RomRegion {
    uint16_t start{};
    uint16_t end{};
};

struct RamConfig {
    uint16_t start{};
    uint16_t end{};
    uint32_t bank_size{0};
    uint32_t available{0};
    bool present{false};
};

struct SerialConfig {
    uint16_t start{};
    uint16_t end{};
    bool present{false};
};

struct MapperConfig {
    uint16_t bank_reg[4]{};
    bool present{false};
};

struct HardwareConfig {
    std::vector<RomRegion> roms;
    RamConfig ram;
    SerialConfig serial;
    MapperConfig mapper;
};

std::optional<LoadedImage> load_image(const std::filesystem::path& path, RomFormat fmt, uint16_t base_override = 0x8000) {
    LoadedImage img;
    switch (fmt) {
    case RomFormat::Raw:
        img.data = load_file(path);
        img.base = base_override;
        break;
    case RomFormat::Ihex:
        img.data = load_ihex(path);
        img.base = 0x0000;
        break;
    case RomFormat::Srec:
        img.data = load_srec(path);
        img.base = 0x0000;
        break;
    case RomFormat::None:
        return std::nullopt;
    }
    return img;
}

class SerialStub : public microlind::BusDevice {
public:
    uint8_t read8(uint16_t) override { return 0xFF; }
    void write8(uint16_t, uint8_t) override {}
};

class RegisterStub : public microlind::BusDevice {
public:
    uint8_t read8(uint16_t) override { return value_; }
    void write8(uint16_t, uint8_t v) override { value_ = v; }
private:
    uint8_t value_{0};
};

Simulator build_sim(CpuMode mode, const LoadedImage* image, const HardwareConfig* cfg, microlind::devices::XR88C92** serial_out = nullptr) {
    Simulator sim(mode, 1000000);
    using microlind::devices::BankedMemory;
    using microlind::devices::Memory;
    using microlind::devices::MapperState;
    using microlind::devices::MemoryMapper;
    using microlind::devices::XR88C92;

    std::shared_ptr<MapperState> mapper_state;
    XR88C92* serial_dev_raw = nullptr;

    if (cfg && cfg->ram.present) {
        const size_t ram_size = static_cast<size_t>(cfg->ram.end - cfg->ram.start + 1);
        if (cfg->mapper.present && cfg->ram.bank_size > 0 && cfg->ram.available > 0) {
            mapper_state = std::make_shared<MapperState>();
            const std::size_t window_count = (ram_size + cfg->ram.bank_size - 1) / cfg->ram.bank_size;
            auto ram_dev = std::make_unique<BankedMemory>(mapper_state, cfg->ram.bank_size, cfg->ram.available, window_count);
            sim.map_device(cfg->ram.start, cfg->ram.end, std::move(ram_dev));
        } else {
            auto ram_dev = std::make_unique<Memory>(ram_size, true);
            sim.map_device(cfg->ram.start, cfg->ram.end, std::move(ram_dev));
        }
    }

    if (cfg && !cfg->roms.empty()) {
        for (const auto& r : cfg->roms) {
            const size_t region_size = static_cast<size_t>(r.end - r.start + 1);
            auto rom_dev = std::make_unique<Memory>(region_size, false);
            std::vector<uint8_t> slice(region_size, 0xFF);
            if (image) {
                for (size_t i = 0; i < region_size; ++i) {
                    const uint32_t abs = static_cast<uint32_t>(r.start) + static_cast<uint32_t>(i);
                    if (abs >= image->base && abs < image->base + image->data.size()) {
                        slice[i] = image->data[abs - image->base];
                    }
                }
            }
            rom_dev->load(0, slice);
            sim.map_device(r.start, r.end, std::move(rom_dev));
        }
    } else if (image && !image->data.empty()) {
        auto rom_dev = std::make_unique<Memory>(image->data.size(), false);
        rom_dev->load(0, image->data);
        sim.map_device(image->base, static_cast<uint16_t>(image->base + image->data.size() - 1), std::move(rom_dev));
    } else {
        default_memory_map(sim, 64 * 1024, 0x8000, nullptr);
    }

    if (cfg && cfg->serial.present) {
        auto on_tx = [](uint8_t ch) {
            if (std::isprint(static_cast<unsigned char>(ch))) {
                std::cout << "\n[SERIAL TX] '" << static_cast<char>(ch) << "' (0x"
                          << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(ch) << ")\n";
            } else {
                std::cout << "\n[SERIAL TX] 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(ch) << "\n";
            }
            std::cout << "> " << std::flush;
        };
        auto serial_up = std::make_unique<XR88C92>(on_tx);
        serial_dev_raw = serial_up.get();
        sim.map_device(cfg->serial.start, cfg->serial.end, std::move(serial_up));
    }

    if (cfg && cfg->mapper.present && mapper_state) {
        std::vector<uint16_t> addrs;
        for (int i = 0; i < 4; ++i) {
            if (cfg->mapper.bank_reg[i] != 0) addrs.push_back(cfg->mapper.bank_reg[i]);
        }
        if (!addrs.empty()) {
            const auto [min_it, max_it] = std::minmax_element(addrs.begin(), addrs.end());
            const uint16_t start = *min_it;
            const uint16_t end = *max_it;
            std::vector<int8_t> offset_map(static_cast<std::size_t>(end - start + 1), -1);
            for (int i = 0; i < 4; ++i) {
                if (cfg->mapper.bank_reg[i] != 0) {
                    const uint16_t off = static_cast<uint16_t>(cfg->mapper.bank_reg[i] - start);
                    if (off < offset_map.size()) offset_map[off] = static_cast<int8_t>(i);
                }
            }
            sim.map_device(start, end, std::make_unique<MemoryMapper>(mapper_state, std::move(offset_map)));
        }
    }

    sim.reset_from_vector();

    if (serial_out) {
        *serial_out = serial_dev_raw;
    }
    return sim;
}

static std::optional<uint32_t> parse_number(const std::string& s) {
    try {
        size_t idx = 0;
        int base = 10;
        if (s.size() > 2 && s[0] == '0' && (s[1] == 'x' || s[1] == 'X')) base = 16;
        else if (s.size() > 1 && s[0] == '$') base = 16, idx = 1;
        return static_cast<uint32_t>(std::stoul(s.substr(idx), nullptr, base));
    } catch (...) {
        return std::nullopt;
    }
}

static std::string trim(const std::string& s) {
    size_t b = 0;
    while (b < s.size() && std::isspace(static_cast<unsigned char>(s[b]))) ++b;
    size_t e = s.size();
    while (e > b && std::isspace(static_cast<unsigned char>(s[e - 1]))) --e;
    return s.substr(b, e - b);
}

static bool iequals(const std::string& a, const std::string& b) {
    if (a.size() != b.size()) return false;
    for (size_t i = 0; i < a.size(); ++i) {
        if (std::toupper(static_cast<unsigned char>(a[i])) != std::toupper(static_cast<unsigned char>(b[i]))) return false;
    }
    return true;
}

static std::optional<HardwareConfig> load_hardware_config(const std::filesystem::path& path, std::string& error) {
    std::ifstream file(path);
    if (!file) {
        error = "Cannot open config file";
        return std::nullopt;
    }

    HardwareConfig cfg;
    enum class Section { None, Rom, Ram, Serial, Mapper };
    Section section = Section::None;

    RomRegion pending_rom{};
    bool rom_has_start = false;
    bool rom_has_end = false;

    auto flush_rom = [&]() {
        if (rom_has_start && rom_has_end) {
            cfg.roms.push_back(pending_rom);
        }
        rom_has_start = rom_has_end = false;
        pending_rom = {};
    };

    std::string line;
    size_t lineno = 0;
    while (std::getline(file, line)) {
        ++lineno;
        line = trim(line);
        if (line.empty() || line[0] == '#' || line[0] == ';') continue;
        if (line.front() == '[' && line.back() == ']') {
            flush_rom();
            const std::string sect = trim(line.substr(1, line.size() - 2));
            if (iequals(sect, "ROM")) section = Section::Rom;
            else if (iequals(sect, "RAM")) section = Section::Ram;
            else if (iequals(sect, "SERIAL")) section = Section::Serial;
            else if (iequals(sect, "MEMORY_MAPPER")) section = Section::Mapper;
            else section = Section::None;
            continue;
        }

        const auto eq = line.find('=');
        if (eq == std::string::npos) continue;
        std::string key = trim(line.substr(0, eq));
        std::string value = trim(line.substr(eq + 1));
        if (key.empty()) continue;

        if (section == Section::Rom) {
            if (iequals(key, "START")) {
                if (auto v = parse_number(value)) { pending_rom.start = static_cast<uint16_t>(*v); rom_has_start = true; }
                else { error = "Bad ROM START at line " + std::to_string(lineno); return std::nullopt; }
            } else if (iequals(key, "END")) {
                if (auto v = parse_number(value)) { pending_rom.end = static_cast<uint16_t>(*v); rom_has_end = true; }
                else { error = "Bad ROM END at line " + std::to_string(lineno); return std::nullopt; }
            }
            if (rom_has_start && rom_has_end) {
                flush_rom();
            }
        } else if (section == Section::Ram) {
            cfg.ram.present = true;
            if (iequals(key, "START")) {
                if (auto v = parse_number(value)) cfg.ram.start = static_cast<uint16_t>(*v); else { error = "Bad RAM START at line " + std::to_string(lineno); return std::nullopt; }
            } else if (iequals(key, "END")) {
                if (auto v = parse_number(value)) cfg.ram.end = static_cast<uint16_t>(*v); else { error = "Bad RAM END at line " + std::to_string(lineno); return std::nullopt; }
            } else if (iequals(key, "BANK_SIZE")) {
                if (auto v = parse_number(value)) cfg.ram.bank_size = *v; else { error = "Bad BANK_SIZE at line " + std::to_string(lineno); return std::nullopt; }
            } else if (iequals(key, "AVAILABLE")) {
                if (auto v = parse_number(value)) cfg.ram.available = *v; else { error = "Bad AVAILABLE at line " + std::to_string(lineno); return std::nullopt; }
            }
        } else if (section == Section::Serial) {
            cfg.serial.present = true;
            if (iequals(key, "IO_START_ADDRESS")) {
                if (auto v = parse_number(value)) cfg.serial.start = static_cast<uint16_t>(*v); else { error = "Bad IO_START_ADDRESS at line " + std::to_string(lineno); return std::nullopt; }
            } else if (iequals(key, "IO_END_ADDRESS")) {
                if (auto v = parse_number(value)) cfg.serial.end = static_cast<uint16_t>(*v); else { error = "Bad IO_END_ADDRESS at line " + std::to_string(lineno); return std::nullopt; }
            }
        } else if (section == Section::Mapper) {
            cfg.mapper.present = true;
            if (iequals(key, "BANK_0_REGISTER")) {
                if (auto v = parse_number(value)) cfg.mapper.bank_reg[0] = static_cast<uint16_t>(*v); else { error = "Bad BANK_0_REGISTER at line " + std::to_string(lineno); return std::nullopt; }
            } else if (iequals(key, "BANK_1_REGISTER")) {
                if (auto v = parse_number(value)) cfg.mapper.bank_reg[1] = static_cast<uint16_t>(*v); else { error = "Bad BANK_1_REGISTER at line " + std::to_string(lineno); return std::nullopt; }
            } else if (iequals(key, "BANK_2_REGISTER")) {
                if (auto v = parse_number(value)) cfg.mapper.bank_reg[2] = static_cast<uint16_t>(*v); else { error = "Bad BANK_2_REGISTER at line " + std::to_string(lineno); return std::nullopt; }
            } else if (iequals(key, "BANK_3_REGISTER")) {
                if (auto v = parse_number(value)) cfg.mapper.bank_reg[3] = static_cast<uint16_t>(*v); else { error = "Bad BANK_3_REGISTER at line " + std::to_string(lineno); return std::nullopt; }
            }
        }
    }

    flush_rom();
    return cfg;
}

struct Disasm {
    std::string text;
    uint8_t length{1};
};

enum class OperandKind { None, Immediate, Direct, Extended, Indexed };

static OperandKind operand_kind(uint8_t prefix, uint8_t opcode, const std::string& name, uint8_t& size_override) {
    // Explicit overrides for instructions without addressing encoded in their name.
    const uint16_t key = static_cast<uint16_t>((prefix << 8) | opcode);
    switch (key) {
    case 0x113D: // LDMD #imm
    case 0x1031: // ADCR postbyte
    case 0x1032: // SUBR postbyte
    case 0x1033: // SBCR postbyte
    case 0x1035: // ORR postbyte
    case 0x1037: // CMPR postbyte
    case 0x1138: // TFM r0+,r1+
    case 0x1139: // TFM r0-,r1-
    case 0x113A: // TFM r0+,r1
    case 0x113B: // TFM r0,r1+
        size_override = 1;
        return OperandKind::Immediate;
    default:
        break;
    }

    if (name.find(" imm") != std::string::npos) return OperandKind::Immediate;
    if (name.find(" dir") != std::string::npos) return OperandKind::Direct;
    if (name.find(" ext") != std::string::npos) return OperandKind::Extended;
    if (name.find(" idx") != std::string::npos) return OperandKind::Indexed;
    return OperandKind::None;
}

static uint8_t operand_size_from_mnemonic(const std::string& name) {
    // Use the mnemonic to guess operand size (for immediate).
    // Default 1 byte.
    std::string token = name;
    const auto space = name.find(' ');
    if (space != std::string::npos) token = name.substr(0, space);
    for (auto& c : token) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));

    if (token.find("ldq") == 0 || token.find("stq") == 0 || token.find("divq") == 0 || token.find("addr") == 0 || token.find("subr") == 0) {
        // Default to 2 for inter-register (size determined by dest); 4 for Q immediates.
        if (token.find("ldq") == 0 || token.find("stq") == 0 || token.find("divq") == 0) return 4;
        return 2;
    }

    if (token.find("ldw") == 0 || token.find("stw") == 0 || token.find("addw") == 0 || token.find("subw") == 0 ||
        token.find("cmpw") == 0 || token.find("sexw") == 0) {
        return 2;
    }

    if (token.find("ldx") == 0 || token.find("ldy") == 0 || token.find("ldu") == 0 || token.find("lds") == 0 ||
        token.find("stx") == 0 || token.find("sty") == 0 || token.find("stu") == 0 || token.find("sts") == 0 ||
        token.find("cmpx") == 0 || token.find("cmpy") == 0 || token.find("cmpu") == 0 || token.find("cmps") == 0 ||
        token.find("leax") == 0 || token.find("leay") == 0 || token.find("leau") == 0 || token.find("leas") == 0) {
        return 2;
    }

    if (token.find("ldd") == 0 || token.find("std") == 0 || token.find("addd") == 0 || token.find("subd") == 0 ||
        token.find("cmpd") == 0 || token.find("adcd") == 0 || token.find("sbcd") == 0 || token.find("muld") == 0 || token.find("divd") == 0) {
        return 2;
    }

    if (token.find("ld") == 0 && token.size() == 3 && token[2] == 'q') return 4;
    return 1;
}

static std::string hex4(uint16_t v) {
    std::ostringstream oss;
    oss << std::hex << std::setw(4) << std::setfill('0') << v;
    return oss.str();
}

static Disasm disassemble(Bus& bus, Cpu& cpu, uint16_t pc) {
    const uint8_t op0 = bus.read8(pc);
    auto fallback = [&](const std::string& name, uint8_t len) {
        std::ostringstream oss;
        oss << name;
        uint8_t size_override = 0;
        const OperandKind kind = operand_kind(0x00, op0, name, size_override);
        const uint8_t op_size = size_override ? size_override : operand_size_from_mnemonic(name);
        switch (kind) {
        case OperandKind::Immediate:
            if (op_size == 1) {
                uint8_t imm = bus.read8(static_cast<uint16_t>(pc + 1));
                oss << " #$" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(imm);
            } else if (op_size == 2) {
                uint16_t imm = static_cast<uint16_t>((bus.read8(static_cast<uint16_t>(pc + 1)) << 8) |
                                                     bus.read8(static_cast<uint16_t>(pc + 2)));
                oss << " #$" << hex4(imm);
            } else if (op_size == 4) {
                uint32_t imm = (static_cast<uint32_t>(bus.read8(static_cast<uint16_t>(pc + 1))) << 24) |
                               (static_cast<uint32_t>(bus.read8(static_cast<uint16_t>(pc + 2))) << 16) |
                               (static_cast<uint32_t>(bus.read8(static_cast<uint16_t>(pc + 3))) << 8) |
                               static_cast<uint32_t>(bus.read8(static_cast<uint16_t>(pc + 4)));
                std::ostringstream tmp;
                tmp << std::hex << std::setw(8) << std::setfill('0') << imm;
                oss << " #$" << tmp.str();
            }
            break;
        case OperandKind::Direct: {
            uint8_t addr = bus.read8(static_cast<uint16_t>(pc + 1));
            oss << " <$" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(addr);
            break;
        }
        case OperandKind::Extended: {
            uint16_t addr = static_cast<uint16_t>((bus.read8(static_cast<uint16_t>(pc + 1)) << 8) |
                                                  bus.read8(static_cast<uint16_t>(pc + 2)));
            oss << " $" << hex4(addr);
            break;
        }
        case OperandKind::Indexed: {
            uint8_t pb = bus.read8(static_cast<uint16_t>(pc + 1));
            oss << " [pb $" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(pb) << "]";
            break;
        }
        case OperandKind::None:
            break;
        }
        return Disasm{oss.str(), len};
    };
    if (op0 == 0x10) {
        const uint8_t op1 = bus.read8(static_cast<uint16_t>(pc + 1));
        const std::string name = cpu.opcode_name(0x10, op1);
        // Long conditional branches 0x21-0x2F with 16-bit offset.
        if (op1 >= 0x21 && op1 <= 0x2F) {
            static const char* names[] = {
                "lbrn","lbhi","lbls","lbcc","lbcs","lbne","lbeq","lbvc",
                "lbvs","lbpl","lbmi","lbge","lblt","lbgt","lble"
            };
            const size_t idx = op1 - 0x21;
            const uint16_t off = static_cast<uint16_t>((bus.read8(pc + 2) << 8) | bus.read8(pc + 3));
            const int16_t soff = static_cast<int16_t>(off);
            const uint16_t target = static_cast<uint16_t>(pc + 4 + soff);
            return {std::string(names[idx]) + " $" + hex4(target), 4};
        }
        // Fall back to raw opcode display.
        // Generic: prefix + opcode; determine length from operand kind.
        uint8_t size_override = 0;
        const OperandKind kind = operand_kind(0x10, op1, name, size_override);
        const uint8_t op_size = size_override ? size_override : operand_size_from_mnemonic(name);
        uint8_t len = static_cast<uint8_t>(2); // prefix + opcode
        if (kind == OperandKind::Immediate) len = static_cast<uint8_t>(len + op_size);
        else if (kind == OperandKind::Direct || kind == OperandKind::Indexed) len = static_cast<uint8_t>(len + 1);
        else if (kind == OperandKind::Extended) len = static_cast<uint8_t>(len + 2);
        return fallback(name.empty() ? "op10" : name, len);
    }

    if (op0 == 0x11) {
        const uint8_t op1 = bus.read8(static_cast<uint16_t>(pc + 1));
        const std::string name = cpu.opcode_name(0x11, op1);
        uint8_t size_override = 0;
        const OperandKind kind = operand_kind(0x11, op1, name, size_override);
        const uint8_t op_size = size_override ? size_override : operand_size_from_mnemonic(name);
        uint8_t len = static_cast<uint8_t>(2); // prefix + opcode
        if (kind == OperandKind::Immediate) len = static_cast<uint8_t>(len + op_size);
        else if (kind == OperandKind::Direct || kind == OperandKind::Indexed) len = static_cast<uint8_t>(len + 1);
        else if (kind == OperandKind::Extended) len = static_cast<uint8_t>(len + 2);
        return fallback(name.empty() ? "op11" : name, len);
    }

    // LBRA / LBSR (16-bit relative)
    if (op0 == 0x16 || op0 == 0x17) {
        const uint16_t off = static_cast<uint16_t>((bus.read8(pc + 1) << 8) | bus.read8(pc + 2));
        const int16_t soff = static_cast<int16_t>(off);
        const uint16_t target = static_cast<uint16_t>(pc + 3 + soff);
        const char* name = (op0 == 0x16) ? "lbra" : "lbsr";
        return {std::string(name) + " $" + hex4(target), 3};
    }

    // BSR (8-bit), short branches 0x20-0x2F.
    if (op0 == 0x8D) {
        const int8_t off = static_cast<int8_t>(bus.read8(static_cast<uint16_t>(pc + 1)));
        const uint16_t target = static_cast<uint16_t>(pc + 2 + off);
        return {"bsr $" + hex4(target), 2};
    }

    if (op0 >= 0x20 && op0 <= 0x2F) {
        static const char* names[] = {
            "bra","brn","bhi","bls","bcc","bcs","bne","beq",
            "bvc","bvs","bpl","bmi","bge","blt","bgt","ble"
        };
        const size_t idx = op0 - 0x20;
        const int8_t off = static_cast<int8_t>(bus.read8(static_cast<uint16_t>(pc + 1)));
        const uint16_t target = static_cast<uint16_t>(pc + 2 + off);
        return {std::string(names[idx]) + " $" + hex4(target), 2};
    }

    // Generic formatting using opcode name and operand kind.
    std::string name = cpu.opcode_name(0x00, op0);
    if (name.empty()) name = "op" + hex4(op0).substr(2);
    uint8_t size_override = 0;
    const OperandKind kind = operand_kind(0x00, op0, name, size_override);
    const uint8_t op_size = size_override ? size_override : operand_size_from_mnemonic(name);
    uint8_t len = 1;
    if (kind == OperandKind::Immediate) len = static_cast<uint8_t>(len + op_size);
    else if (kind == OperandKind::Direct || kind == OperandKind::Indexed) len = static_cast<uint8_t>(len + 1);
    else if (kind == OperandKind::Extended) len = static_cast<uint8_t>(len + 2);
    return fallback(name, len);
}

static void print_help() {
    std::cout << "Commands:\n"
              << "  help                     - show this help\n"
              << "  regs                     - show registers\n"
              << "  step [n]                 - execute n instructions (default 1)\n"
              << "  tick [n]                 - advance bus/clock by n cycles (default 1)\n"
              << "  run [n]                  - execute n instructions (default 1000)\n"
              << "  peek <addr> [count]      - read memory\n"
              << "  poke <addr> <value>      - write memory\n"
              << "  dump <addr> <len>        - hex dump\n"
              << "  loadbin <path> [base]    - load raw binary (default base 0x8000)\n"
              << "  loadihex <path>          - load Intel HEX (absolute addresses)\n"
              << "  loadsrec <path>          - load S-record (absolute addresses)\n"
              << "  loadcfg <path>           - load hardware config\n"
              << "  serin <text>             - push ASCII text into serial RX\n"
              << "  map                      - show mapped address ranges\n"
              << "  reset                    - reset PC from reset vector\n"
              << "  exit                     - quit\n";
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

    Simulator sim = build_sim(mode, current_image ? &*current_image : nullptr, hw_cfg ? &*hw_cfg : nullptr, &serial_dev);

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
            sim = build_sim(mode, &*current_image, hw_cfg ? &*hw_cfg : nullptr, &serial_dev);
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
            sim = build_sim(mode, current_image ? &*current_image : nullptr, hw_cfg ? &*hw_cfg : nullptr, &serial_dev);
            std::cout << "Hardware config loaded and CPU reset.\n";
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
            std::cout << "Reset PC from vector.\n";
        } else {
            std::cout << "Unknown command. Type 'help'.\n";
        }
    }

    return 0;
}
