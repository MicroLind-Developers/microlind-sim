#include "microlind/app/hardware_config.hpp"

#include "microlind/app/util.hpp"

#include <cstddef>
#include <cctype>
#include <fstream>
#include <string_view>

namespace microlind::cli {

namespace {

std::optional<int> mapper_window_index(const std::string& key, std::string* suffix = nullptr) {
    constexpr std::string_view prefix = "WINDOW_";
    if (key.size() < prefix.size() + 1) return std::nullopt;
    for (std::size_t i = 0; i < prefix.size(); ++i) {
        if (std::toupper(static_cast<unsigned char>(key[i])) != prefix[i]) return std::nullopt;
    }
    if (!std::isdigit(static_cast<unsigned char>(key[prefix.size()]))) return std::nullopt;
    const int index = key[prefix.size()] - '0';
    if (index < 0 || index >= 4) return std::nullopt;
    if (suffix) {
        *suffix = key.substr(prefix.size() + 1);
    }
    return index;
}

bool parse_range(const std::string& value, uint16_t& start, uint16_t& end) {
    const auto dash = value.find('-');
    if (dash == std::string::npos) return false;
    const auto parsed_start = parse_number(trim(value.substr(0, dash)));
    const auto parsed_end = parse_number(trim(value.substr(dash + 1)));
    if (!parsed_start || !parsed_end || *parsed_start > 0xFFFF || *parsed_end > 0xFFFF || *parsed_start > *parsed_end) {
        return false;
    }
    start = static_cast<uint16_t>(*parsed_start);
    end = static_cast<uint16_t>(*parsed_end);
    return true;
}

std::optional<BusDecodeMode> parse_bus_decode_mode(const std::string& value) {
    if (iequals(value, "range") || iequals(value, "rangemap") || iequals(value, "off") || iequals(value, "disabled")) {
        return BusDecodeMode::RangeMap;
    }
    if (iequals(value, "validate") || iequals(value, "validation") || iequals(value, "warn")) {
        return BusDecodeMode::Validate;
    }
    if (iequals(value, "route") || iequals(value, "routing") || iequals(value, "pld")) {
        return BusDecodeMode::Route;
    }
    return std::nullopt;
}

} // namespace

std::optional<HardwareConfig> load_hardware_config(const std::filesystem::path& path, std::string& error) {
    std::ifstream file(path);
    if (!file) {
        error = "Cannot open config file";
        return std::nullopt;
    }

    HardwareConfig cfg;
    enum class Section { None, Rom, Ram, Serial, Parallel, Cf, Mapper, Logic };
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
            else if (iequals(sect, "PARALLEL") || iequals(sect, "PAR")) section = Section::Parallel;
            else if (iequals(sect, "CF") || iequals(sect, "COMPACT_FLASH")) section = Section::Cf;
            else if (iequals(sect, "MEMORY_MAPPER")) section = Section::Mapper;
            else if (iequals(sect, "PLD_LOGIC") || iequals(sect, "LOGIC")) section = Section::Logic;
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
            } else if (iequals(key, "IRQ_LEVEL")) {
                if (auto v = parse_number(value); v && *v <= 0x0F) cfg.serial.irq_level = static_cast<uint8_t>(*v); else { error = "Bad SERIAL IRQ_LEVEL at line " + std::to_string(lineno); return std::nullopt; }
            }
        } else if (section == Section::Parallel) {
            cfg.parallel.present = true;
            if (iequals(key, "IO_START_ADDRESS")) {
                if (auto v = parse_number(value)) cfg.parallel.start = static_cast<uint16_t>(*v); else { error = "Bad PARALLEL IO_START_ADDRESS at line " + std::to_string(lineno); return std::nullopt; }
            } else if (iequals(key, "IO_END_ADDRESS")) {
                if (auto v = parse_number(value)) cfg.parallel.end = static_cast<uint16_t>(*v); else { error = "Bad PARALLEL IO_END_ADDRESS at line " + std::to_string(lineno); return std::nullopt; }
            } else if (iequals(key, "IRQ_LEVEL")) {
                if (auto v = parse_number(value); v && *v <= 0x0F) cfg.parallel.irq_level = static_cast<uint8_t>(*v); else { error = "Bad PARALLEL IRQ_LEVEL at line " + std::to_string(lineno); return std::nullopt; }
            }
        } else if (section == Section::Cf) {
            cfg.cf.present = true;
            if (iequals(key, "IO_START_ADDRESS")) {
                if (auto v = parse_number(value)) cfg.cf.start = static_cast<uint16_t>(*v); else { error = "Bad CF IO_START_ADDRESS at line " + std::to_string(lineno); return std::nullopt; }
            } else if (iequals(key, "IO_END_ADDRESS")) {
                if (auto v = parse_number(value)) cfg.cf.end = static_cast<uint16_t>(*v); else { error = "Bad CF IO_END_ADDRESS at line " + std::to_string(lineno); return std::nullopt; }
            } else if (iequals(key, "IMAGE") || iequals(key, "IMAGE_PATH")) {
                cfg.cf.image_path = value;
            } else if (iequals(key, "SECTORS")) {
                if (auto v = parse_number(value)) cfg.cf.sectors = *v; else { error = "Bad CF SECTORS at line " + std::to_string(lineno); return std::nullopt; }
            } else if (iequals(key, "READ_ONLY")) {
                cfg.cf.read_only = iequals(value, "true") || value == "1" || iequals(value, "yes");
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
            } else {
                std::string suffix;
                if (const auto window = mapper_window_index(key, &suffix)) {
                    auto& cfg_window = cfg.mapper.windows[static_cast<std::size_t>(*window)];
                    if (suffix.empty()) {
                        if (!parse_range(value, cfg_window.start, cfg_window.end)) {
                            error = "Bad WINDOW_" + std::to_string(*window) + " range at line " + std::to_string(lineno);
                            return std::nullopt;
                        }
                        cfg_window.present = true;
                    } else if (iequals(suffix, "_START")) {
                        if (auto v = parse_number(value)) {
                            cfg_window.start = static_cast<uint16_t>(*v);
                            cfg_window.present = true;
                        } else {
                            error = "Bad WINDOW_" + std::to_string(*window) + "_START at line " + std::to_string(lineno);
                            return std::nullopt;
                        }
                    } else if (iequals(suffix, "_END")) {
                        if (auto v = parse_number(value)) {
                            cfg_window.end = static_cast<uint16_t>(*v);
                            cfg_window.present = true;
                        } else {
                            error = "Bad WINDOW_" + std::to_string(*window) + "_END at line " + std::to_string(lineno);
                            return std::nullopt;
                        }
                    }
                }
            }
        } else if (section == Section::Logic) {
            cfg.logic.present = true;
            if (iequals(key, "SIGNAL_LOGIC") || iequals(key, "SIGNAL_LOGIC_PATH")) {
                cfg.logic.signal_logic_path = value;
            } else if (iequals(key, "MEMORY_LOGIC") || iequals(key, "MEMORY_LOGIC_PATH")) {
                cfg.logic.memory_logic_path = value;
            } else if (iequals(key, "ADDRESS_LOGIC") || iequals(key, "ADDRESS_LOGIC_PATH")) {
                cfg.logic.address_logic_path = value;
            } else if (iequals(key, "BUS_MODE") || iequals(key, "ROUTING")) {
                if (auto mode = parse_bus_decode_mode(value)) {
                    cfg.logic.bus_mode = *mode;
                } else {
                    error = "Bad PLD BUS_MODE at line " + std::to_string(lineno);
                    return std::nullopt;
                }
            }
        }
    }

    flush_rom();
    if (cfg.cf.present && !cfg.cf.image_path.empty() && cfg.cf.image_path.is_relative()) {
        cfg.cf.image_path = path.parent_path() / cfg.cf.image_path;
    }
    if (cfg.logic.present) {
        if (!cfg.logic.signal_logic_path.empty() && cfg.logic.signal_logic_path.is_relative()) {
            cfg.logic.signal_logic_path = path.parent_path() / cfg.logic.signal_logic_path;
        }
        if (!cfg.logic.memory_logic_path.empty() && cfg.logic.memory_logic_path.is_relative()) {
            cfg.logic.memory_logic_path = path.parent_path() / cfg.logic.memory_logic_path;
        }
        if (!cfg.logic.address_logic_path.empty() && cfg.logic.address_logic_path.is_relative()) {
            cfg.logic.address_logic_path = path.parent_path() / cfg.logic.address_logic_path;
        }
    }
    return cfg;
}

} // namespace microlind::cli
