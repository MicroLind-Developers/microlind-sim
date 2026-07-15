#include "microlind/app/logic_validation.hpp"

#include <array>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <utility>

namespace microlind::cli {
namespace {

struct AddressRange {
    uint16_t start{};
    uint16_t end{};
};

struct RangeBuilder {
    std::vector<AddressRange> ranges;
    bool active{};
    uint16_t start{};

    void sample(uint16_t address, bool selected) {
        if (selected && !active) {
            active = true;
            start = address;
        } else if (!selected && active) {
            ranges.push_back(AddressRange{start, static_cast<uint16_t>(address - 1)});
            active = false;
        }
    }

    void finish() {
        if (active) {
            ranges.push_back(AddressRange{start, 0xFFFF});
            active = false;
        }
    }
};

std::string hex_address(uint16_t address) {
    constexpr char digits[] = "0123456789ABCDEF";
    std::string out = "$0000";
    out[1] = digits[(address >> 12) & 0x0F];
    out[2] = digits[(address >> 8) & 0x0F];
    out[3] = digits[(address >> 4) & 0x0F];
    out[4] = digits[address & 0x0F];
    return out;
}

std::string config_hex(uint16_t address) {
    std::string out = hex_address(address);
    out[0] = 'x';
    out.insert(out.begin(), '0');
    return out;
}

void add_issue(std::vector<LogicValidationIssue>& issues, uint16_t address, std::string message) {
    issues.push_back(LogicValidationIssue{LogicValidationSeverity::Error, address, std::move(message)});
}

microlind::logic::BoardDecodeResult decode(
    const microlind::logic::BoardLogicDevices& devices,
    uint16_t address,
    uint8_t mapper_bits = 0) {
    return microlind::logic::decode_board_logic(devices, microlind::logic::BoardSignals{
        .address = address,
        .mapper_bits = mapper_bits,
    });
}

void add_decode_errors(
    std::vector<LogicValidationIssue>& issues,
    uint16_t address,
    const microlind::logic::BoardDecodeResult& decoded) {
    for (const auto& error : decoded.errors) {
        add_issue(issues, address, "PLD decode failed at " + hex_address(address) + ": " + error);
    }
}

std::optional<microlind::logic::LogicDeviceDescription> load_pld_device(
    const std::filesystem::path& path,
    std::string& error) {
    std::ifstream file(path);
    if (!file) {
        error = "Cannot open PLD file: " + path.string();
        return std::nullopt;
    }

    std::ostringstream source;
    source << file.rdbuf();
    auto parsed = microlind::logic::parse_pld(source.str());
    if (!parsed.ok()) {
        std::ostringstream out;
        out << "Cannot parse PLD file " << path << ":\n";
        for (const auto& parse_error : parsed.errors) {
            out << "  " << parse_error << "\n";
        }
        error = out.str();
        return std::nullopt;
    }
    return parsed.device;
}

void write_ranges_as_comments(std::ostringstream& out, const char* label, const std::vector<AddressRange>& ranges) {
    if (ranges.empty()) return;
    out << "# " << label << ":";
    for (const auto& range : ranges) {
        out << " " << config_hex(range.start) << "-" << config_hex(range.end);
    }
    out << "\n";
}

} // namespace

std::optional<microlind::logic::BoardLogicDevices> load_board_logic_devices(
    const std::filesystem::path& signal_path,
    const std::filesystem::path& memory_path,
    const std::filesystem::path& address_path,
    std::string& error) {
    auto signal = load_pld_device(signal_path, error);
    if (!signal) return std::nullopt;
    auto memory = load_pld_device(memory_path, error);
    if (!memory) return std::nullopt;
    auto address = load_pld_device(address_path, error);
    if (!address) return std::nullopt;
    return microlind::logic::BoardLogicDevices{std::move(*signal), std::move(*memory), std::move(*address)};
}

std::optional<microlind::logic::BoardLogicDevices> load_board_logic_devices(
    const LogicConfig& cfg,
    std::string& error) {
    if (!cfg.present) {
        error = "No PLD logic configured.";
        return std::nullopt;
    }
    if (cfg.signal_logic_path.empty() || cfg.memory_logic_path.empty() || cfg.address_logic_path.empty()) {
        error = "PLD logic config requires SIGNAL_LOGIC, MEMORY_LOGIC, and ADDRESS_LOGIC.";
        return std::nullopt;
    }
    return load_board_logic_devices(
        cfg.signal_logic_path,
        cfg.memory_logic_path,
        cfg.address_logic_path,
        error);
}

std::vector<LogicValidationIssue> validate_hardware_config_against_logic(
    const HardwareConfig& cfg,
    const microlind::logic::BoardLogicDevices& devices) {
    std::vector<LogicValidationIssue> issues;

    for (const auto& rom : cfg.roms) {
        for (const uint16_t address : {rom.start, rom.end}) {
            const auto decoded = decode(devices, address);
            add_decode_errors(issues, address, decoded);
            if (!decoded.ok()) continue;
            if (!decoded.rom_en) {
                add_issue(issues, address, "ROM region address " + hex_address(address) + " does not assert ROM_EN");
            }
            if (decoded.io_en) {
                add_issue(issues, address, "ROM region address " + hex_address(address) + " unexpectedly asserts IO_EN");
            }
        }
    }

    if (cfg.ram.present) {
        const auto low_ram = decode(devices, cfg.ram.start, 0x00);
        add_decode_errors(issues, cfg.ram.start, low_ram);
        if (low_ram.ok() && (!low_ram.raml_en || low_ram.rom_en || low_ram.io_en)) {
            add_issue(issues, cfg.ram.start, "RAM start " + hex_address(cfg.ram.start) + " does not decode as low RAM");
        }

        const auto ram_end = decode(devices, cfg.ram.end, 0x00);
        add_decode_errors(issues, cfg.ram.end, ram_end);
        if (ram_end.ok() && (!ram_end.raml_en || ram_end.rom_en || ram_end.io_en)) {
            add_issue(issues, cfg.ram.end, "RAM end " + hex_address(cfg.ram.end) + " does not decode as low RAM");
        }

        if (cfg.mapper.present && cfg.ram.bank_size > 0) {
            const auto high_ram = decode(devices, cfg.ram.start, 0x01);
            add_decode_errors(issues, cfg.ram.start, high_ram);
            if (high_ram.ok() && !high_ram.ramh_en) {
                add_issue(issues, cfg.ram.start, "Mapper AM19 path does not decode as high RAM at " + hex_address(cfg.ram.start));
            }

            const auto expansion_ram = decode(devices, cfg.ram.start, 0x02);
            add_decode_errors(issues, cfg.ram.start, expansion_ram);
            if (expansion_ram.ok() && !expansion_ram.ramx_en) {
                add_issue(issues, cfg.ram.start, "Mapper AM20 path does not decode as expansion RAM at " + hex_address(cfg.ram.start));
            }
        }
    }

    if (cfg.mapper.present) {
        for (int index = 0; index < 4; ++index) {
            const uint16_t address = cfg.mapper.bank_reg[index];
            const auto decoded = decode(devices, address);
            add_decode_errors(issues, address, decoded);
            if (!decoded.ok()) continue;
            if (!decoded.io_en || !decoded.mapper_register_en) {
                add_issue(issues, address, "Mapper register " + std::to_string(index) + " at " + hex_address(address) + " is not decoded as a mapper IO register");
            }
            if (decoded.bank_select != static_cast<uint8_t>(index)) {
                add_issue(issues, address, "Mapper register " + std::to_string(index) + " selects bank " + std::to_string(decoded.bank_select));
            }
        }
    }

    if (cfg.cf.present) {
        for (const uint16_t address : {cfg.cf.start, cfg.cf.end}) {
            const auto decoded = decode(devices, address);
            add_decode_errors(issues, address, decoded);
            if (decoded.ok() && (!decoded.io_en || !decoded.cf_en)) {
                add_issue(issues, address, "CompactFlash address " + hex_address(address) + " is not decoded as CF IO");
            }
        }
    }

    if (cfg.serial.present) {
        for (const uint16_t address : {cfg.serial.start, cfg.serial.end}) {
            const auto decoded = decode(devices, address);
            add_decode_errors(issues, address, decoded);
            if (decoded.ok() && (!decoded.io_en || !decoded.ser_en)) {
                add_issue(issues, address, "Serial address " + hex_address(address) + " is not decoded as serial IO");
            }
        }
    }

    return issues;
}

std::string format_logic_validation_issue(const LogicValidationIssue& issue) {
    std::ostringstream out;
    out << (issue.severity == LogicValidationSeverity::Error ? "error" : "warning")
        << " at 0x" << std::uppercase << std::hex << std::setw(4) << std::setfill('0')
        << issue.address << std::dec << ": " << issue.message;
    return out.str();
}

std::string generate_partial_hardware_config_from_logic(const microlind::logic::BoardLogicDevices& devices) {
    RangeBuilder rom;
    RangeBuilder ram;
    RangeBuilder cf;
    RangeBuilder serial;
    RangeBuilder mapper;
    RangeBuilder ps2;
    RangeBuilder par;
    RangeBuilder vdc;
    RangeBuilder snd;
    RangeBuilder exp;
    std::array<RangeBuilder, 4> mapper_registers;
    std::array<RangeBuilder, 4> mapper_windows;

    for (uint32_t address = 0; address <= 0xFFFF; ++address) {
        const auto decoded = microlind::logic::decode_board_logic(devices, microlind::logic::BoardSignals{
            .address = static_cast<uint16_t>(address),
        });
        const bool ok = decoded.ok();
        rom.sample(static_cast<uint16_t>(address), ok && decoded.rom_en && !decoded.io_en);
        ram.sample(static_cast<uint16_t>(address), ok && decoded.raml_en && !decoded.rom_en && !decoded.io_en);
        cf.sample(static_cast<uint16_t>(address), ok && decoded.cf_en);
        serial.sample(static_cast<uint16_t>(address), ok && decoded.ser_en);
        mapper.sample(static_cast<uint16_t>(address), ok && decoded.mapper_register_en);
        ps2.sample(static_cast<uint16_t>(address), ok && decoded.ps2_en);
        par.sample(static_cast<uint16_t>(address), ok && decoded.par_en);
        vdc.sample(static_cast<uint16_t>(address), ok && decoded.vdc_en);
        snd.sample(static_cast<uint16_t>(address), ok && decoded.snd_en);
        exp.sample(static_cast<uint16_t>(address), ok && decoded.exp_en);
        for (int index = 0; index < 4; ++index) {
            mapper_registers[static_cast<std::size_t>(index)].sample(
                static_cast<uint16_t>(address),
                ok && decoded.mapper_register_en && decoded.bank_select == static_cast<uint8_t>(index));
        }

        const auto window_decoded = microlind::logic::decode_board_logic(devices, microlind::logic::BoardSignals{
            .address = static_cast<uint16_t>(address),
            .rw = false,
        });
        const bool window_ok = window_decoded.ok();
        for (int index = 0; index < 4; ++index) {
            mapper_windows[static_cast<std::size_t>(index)].sample(
                static_cast<uint16_t>(address),
                window_ok && (window_decoded.raml_en || window_decoded.ramh_en || window_decoded.ramx_en) &&
                    window_decoded.bank_select == static_cast<uint8_t>(index));
        }
    }

    rom.finish();
    ram.finish();
    cf.finish();
    serial.finish();
    mapper.finish();
    ps2.finish();
    par.finish();
    vdc.finish();
    snd.finish();
    exp.finish();
    for (auto& builder : mapper_registers) builder.finish();
    for (auto& builder : mapper_windows) builder.finish();

    std::ostringstream out;
    out << "# Partial hw.cfg generated from PLD decode logic.\n";
    out << "# Review before use: physical RAM size, ROM images, CF image path, and some device details are not encoded in the PLDs.\n\n";

    for (const auto& range : rom.ranges) {
        out << "[ROM]\n";
        out << "START=" << config_hex(range.start) << "\n";
        out << "END=" << config_hex(range.end) << "\n\n";
    }

    if (!ram.ranges.empty()) {
        out << "[RAM]\n";
        out << "# Derived from RAML_EN with AM19..AM21 clear.\n";
        out << "START=" << config_hex(ram.ranges.front().start) << "\n";
        out << "END=" << config_hex(ram.ranges.back().end) << "\n";
        out << "# BANK_SIZE and AVAILABLE must be supplied from board population, not PLD logic.\n\n";
    }

    if (!cf.ranges.empty()) {
        out << "[CF]\n";
        out << "IO_START_ADDRESS=" << config_hex(cf.ranges.front().start) << "\n";
        out << "IO_END_ADDRESS=" << config_hex(cf.ranges.back().end) << "\n";
        out << "# IMAGE, SECTORS, and READ_ONLY are runtime configuration.\n\n";
    }

    if (!serial.ranges.empty()) {
        out << "[SERIAL]\n";
        out << "IO_START_ADDRESS=" << config_hex(serial.ranges.front().start) << "\n";
        out << "IO_END_ADDRESS=" << config_hex(serial.ranges.back().end) << "\n";
        out << "# Simulator default; adjust if board wiring differs.\n";
        out << "IRQ_LEVEL=1\n\n";
    }

    if (!mapper.ranges.empty()) {
        out << "[MEMORY_MAPPER]\n";
        for (int index = 0; index < 4; ++index) {
            const auto& bank_ranges = mapper_registers[static_cast<std::size_t>(index)].ranges;
            if (!bank_ranges.empty()) {
                out << "BANK_" << index << "_REGISTER=" << config_hex(bank_ranges.front().start) << "\n";
            }
        }

        for (int index = 0; index < 4; ++index) {
            const auto& window_ranges = mapper_windows[static_cast<std::size_t>(index)].ranges;
            if (!window_ranges.empty()) {
                out << "WINDOW_" << index << "=" << config_hex(window_ranges.front().start)
                    << "-" << config_hex(window_ranges.back().end) << "\n";
            }
        }
        out << "# BUS_SIGNALS_PROVIDER and physical mapper backing RAM are board/runtime details.\n\n";
    }

    out << "# Additional decoded IO ranges not currently represented as first-class hw.cfg devices:\n";
    write_ranges_as_comments(out, "PS2_EN", ps2.ranges);
    write_ranges_as_comments(out, "PAR_EN", par.ranges);
    write_ranges_as_comments(out, "VDC_EN", vdc.ranges);
    write_ranges_as_comments(out, "SND_EN", snd.ranges);
    write_ranges_as_comments(out, "EXP_EN", exp.ranges);

    return out.str();
}

} // namespace microlind::cli
