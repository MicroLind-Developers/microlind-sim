#include "microlind/app/session_file.hpp"

#include <algorithm>
#include <fstream>
#include <sstream>
#include <utility>

#include "microlind/app/util.hpp"

namespace microlind::app {
namespace {

std::filesystem::path resolve_session_path(
    const std::filesystem::path& session_path,
    const std::filesystem::path& value) {
    if (value.empty() || value.is_absolute()) return value;
    return session_path.parent_path() / value;
}

char hex_digit(uint8_t value) {
    return static_cast<char>(value < 10 ? ('0' + value) : ('A' + value - 10));
}

std::optional<uint8_t> hex_nibble(char ch) {
    if (ch >= '0' && ch <= '9') return static_cast<uint8_t>(ch - '0');
    if (ch >= 'a' && ch <= 'f') return static_cast<uint8_t>(ch - 'a' + 10);
    if (ch >= 'A' && ch <= 'F') return static_cast<uint8_t>(ch - 'A' + 10);
    return std::nullopt;
}

std::optional<bool> parse_bool(std::string value) {
    value = cli::trim(value);
    if (cli::iequals(value, "1") || cli::iequals(value, "true") || cli::iequals(value, "yes") ||
        cli::iequals(value, "on")) {
        return true;
    }
    if (cli::iequals(value, "0") || cli::iequals(value, "false") || cli::iequals(value, "no") ||
        cli::iequals(value, "off")) {
        return false;
    }
    return std::nullopt;
}

std::vector<std::string> split_fields(std::string_view value) {
    std::vector<std::string> fields;
    std::size_t start = 0;
    while (start <= value.size()) {
        const std::size_t end = value.find(';', start);
        const std::string_view field = end == std::string_view::npos
                                           ? value.substr(start)
                                           : value.substr(start, end - start);
        fields.push_back(cli::trim(std::string(field)));
        if (end == std::string_view::npos) break;
        start = end + 1;
    }
    return fields;
}

std::optional<WatchpointType> parse_watchpoint_type(std::string value) {
    value = cli::trim(value);
    if (cli::iequals(value, "R") || cli::iequals(value, "READ")) return WatchpointType::Read;
    if (cli::iequals(value, "W") || cli::iequals(value, "WRITE")) return WatchpointType::Write;
    if (cli::iequals(value, "RW") || cli::iequals(value, "READWRITE") || cli::iequals(value, "READ/WRITE")) {
        return WatchpointType::ReadWrite;
    }
    return std::nullopt;
}

std::optional<GuiTheme> parse_gui_theme(std::string value) {
    value = cli::trim(value);
    if (cli::iequals(value, "dark")) return GuiTheme::Dark;
    if (cli::iequals(value, "light")) return GuiTheme::Light;
    return std::nullopt;
}

const char* gui_theme_name(GuiTheme theme) {
    switch (theme) {
    case GuiTheme::Dark: return "dark";
    case GuiTheme::Light: return "light";
    }
    return "dark";
}

const char* watchpoint_type_name(WatchpointType type) {
    switch (type) {
    case WatchpointType::Read: return "R";
    case WatchpointType::Write: return "W";
    case WatchpointType::ReadWrite: return "RW";
    }
    return "R";
}

std::optional<Breakpoint> parse_breakpoint(std::string_view value, std::string& error, std::size_t lineno) {
    Breakpoint breakpoint;
    bool has_address = false;

    for (const std::string& field : split_fields(value)) {
        if (field.empty()) continue;
        const auto eq = field.find('=');
        if (eq == std::string::npos) {
            const auto parsed = cli::parse_number(field);
            if (!parsed || *parsed > 0xFFFF) {
                error = "Bad BREAKPOINT address at line " + std::to_string(lineno);
                return std::nullopt;
            }
            breakpoint.address = static_cast<uint16_t>(*parsed);
            has_address = true;
            continue;
        }

        const std::string key = cli::trim(field.substr(0, eq));
        const std::string field_value = cli::trim(field.substr(eq + 1));
        if (cli::iequals(key, "ADDRESS") || cli::iequals(key, "ADDR")) {
            const auto parsed = cli::parse_number(field_value);
            if (!parsed || *parsed > 0xFFFF) {
                error = "Bad BREAKPOINT address at line " + std::to_string(lineno);
                return std::nullopt;
            }
            breakpoint.address = static_cast<uint16_t>(*parsed);
            has_address = true;
        } else if (cli::iequals(key, "ENABLED")) {
            const auto parsed = parse_bool(field_value);
            if (!parsed) {
                error = "Bad BREAKPOINT ENABLED at line " + std::to_string(lineno);
                return std::nullopt;
            }
            breakpoint.enabled = *parsed;
        } else if (cli::iequals(key, "HITS")) {
            const auto parsed = cli::parse_number(field_value);
            if (!parsed) {
                error = "Bad BREAKPOINT HITS at line " + std::to_string(lineno);
                return std::nullopt;
            }
            breakpoint.hits = *parsed;
        } else if (cli::iequals(key, "LABEL_HEX")) {
            const auto decoded = hex_decode(field_value);
            if (!decoded) {
                error = "Bad BREAKPOINT LABEL_HEX at line " + std::to_string(lineno);
                return std::nullopt;
            }
            breakpoint.label = *decoded;
        }
    }

    if (!has_address) {
        error = "BREAKPOINT is missing address at line " + std::to_string(lineno);
        return std::nullopt;
    }
    return breakpoint;
}

std::optional<Watchpoint> parse_watchpoint(std::string_view value, std::string& error, std::size_t lineno) {
    Watchpoint watchpoint;
    bool has_address = false;

    for (const std::string& field : split_fields(value)) {
        if (field.empty()) continue;
        const auto eq = field.find('=');
        if (eq == std::string::npos) {
            const auto parsed = cli::parse_number(field);
            if (!parsed || *parsed > 0xFFFF) {
                error = "Bad WATCHPOINT address at line " + std::to_string(lineno);
                return std::nullopt;
            }
            watchpoint.address = static_cast<uint16_t>(*parsed);
            has_address = true;
            continue;
        }

        const std::string key = cli::trim(field.substr(0, eq));
        const std::string field_value = cli::trim(field.substr(eq + 1));
        if (cli::iequals(key, "ADDRESS") || cli::iequals(key, "ADDR")) {
            const auto parsed = cli::parse_number(field_value);
            if (!parsed || *parsed > 0xFFFF) {
                error = "Bad WATCHPOINT address at line " + std::to_string(lineno);
                return std::nullopt;
            }
            watchpoint.address = static_cast<uint16_t>(*parsed);
            has_address = true;
        } else if (cli::iequals(key, "TYPE")) {
            const auto parsed = parse_watchpoint_type(field_value);
            if (!parsed) {
                error = "Bad WATCHPOINT TYPE at line " + std::to_string(lineno);
                return std::nullopt;
            }
            watchpoint.type = *parsed;
        } else if (cli::iequals(key, "ENABLED")) {
            const auto parsed = parse_bool(field_value);
            if (!parsed) {
                error = "Bad WATCHPOINT ENABLED at line " + std::to_string(lineno);
                return std::nullopt;
            }
            watchpoint.enabled = *parsed;
        } else if (cli::iequals(key, "HITS")) {
            const auto parsed = cli::parse_number(field_value);
            if (!parsed) {
                error = "Bad WATCHPOINT HITS at line " + std::to_string(lineno);
                return std::nullopt;
            }
            watchpoint.hits = *parsed;
        } else if (cli::iequals(key, "LABEL_HEX")) {
            const auto decoded = hex_decode(field_value);
            if (!decoded) {
                error = "Bad WATCHPOINT LABEL_HEX at line " + std::to_string(lineno);
                return std::nullopt;
            }
            watchpoint.label = *decoded;
        }
    }

    if (!has_address) {
        error = "WATCHPOINT is missing address at line " + std::to_string(lineno);
        return std::nullopt;
    }
    return watchpoint;
}

bool write_path(std::ostream& out, const char* key, const std::filesystem::path& path) {
    if (path.empty()) return true;
    out << key << "=" << path.generic_string() << '\n';
    return static_cast<bool>(out);
}

bool parse_named_bool(
    const std::string& key,
    const std::string& value,
    bool& out,
    std::string& error,
    std::size_t lineno) {
    const auto parsed = parse_bool(value);
    if (!parsed) {
        error = "Bad " + key + " at line " + std::to_string(lineno);
        return false;
    }
    out = *parsed;
    return true;
}

void write_bool(std::ostream& out, const char* key, bool value) {
    out << key << "=" << (value ? "true" : "false") << '\n';
}

} // namespace

std::optional<cli::RomFormat> parse_rom_format(std::string value) {
    value = cli::trim(value);
    if (cli::iequals(value, "raw") || cli::iequals(value, "bin") || cli::iequals(value, "binary")) {
        return cli::RomFormat::Raw;
    }
    if (cli::iequals(value, "ihex") || cli::iequals(value, "intelhex") || cli::iequals(value, "hex")) {
        return cli::RomFormat::Ihex;
    }
    if (cli::iequals(value, "srec") || cli::iequals(value, "srecord") || cli::iequals(value, "s19")) {
        return cli::RomFormat::Srec;
    }
    return std::nullopt;
}

const char* rom_format_name(cli::RomFormat format) {
    switch (format) {
    case cli::RomFormat::Raw: return "raw";
    case cli::RomFormat::Ihex: return "ihex";
    case cli::RomFormat::Srec: return "srec";
    case cli::RomFormat::None: return "ihex";
    }
    return "ihex";
}

const char* cpu_mode_name(CpuMode mode) {
    return mode == CpuMode::HD6309 ? "HD6309" : "MC6809";
}

std::string hex_encode(std::string_view value) {
    std::string out;
    out.reserve(value.size() * 2);
    for (unsigned char ch : value) {
        out.push_back(hex_digit(static_cast<uint8_t>(ch >> 4)));
        out.push_back(hex_digit(static_cast<uint8_t>(ch & 0x0F)));
    }
    return out;
}

std::optional<std::string> hex_decode(std::string_view value) {
    if ((value.size() % 2) != 0) return std::nullopt;

    std::string out;
    out.reserve(value.size() / 2);
    for (std::size_t i = 0; i < value.size(); i += 2) {
        const auto high = hex_nibble(value[i]);
        const auto low = hex_nibble(value[i + 1]);
        if (!high || !low) return std::nullopt;
        out.push_back(static_cast<char>((*high << 4) | *low));
    }
    return out;
}

std::filesystem::path session_relative_path(
    const std::filesystem::path& session_path,
    const std::filesystem::path& value) {
    if (value.empty()) return {};

    std::error_code ec;
    const auto base = std::filesystem::absolute(session_path.parent_path().empty() ? "." : session_path.parent_path(), ec);
    if (ec) return value;

    const auto absolute_value = std::filesystem::absolute(value, ec);
    if (ec) return value;

    const auto relative = std::filesystem::relative(absolute_value, base, ec);
    if (ec || relative.empty()) return value;
    return relative;
}

std::optional<SessionDefinition> load_session_definition(const std::filesystem::path& path, std::string& error) {
    std::ifstream file(path);
    if (!file) {
        error = "Cannot open session: " + path.string();
        return std::nullopt;
    }

    SessionDefinition session;
    std::string line;
    std::size_t lineno = 0;
    while (std::getline(file, line)) {
        ++lineno;
        line = cli::trim(line);
        if (line.empty() || line[0] == '#' || line[0] == ';') continue;
        if (line.front() == '[' && line.back() == ']') continue;

        const auto eq = line.find('=');
        if (eq == std::string::npos) {
            error = "Bad session line " + std::to_string(lineno);
            return std::nullopt;
        }
        const std::string key = cli::trim(line.substr(0, eq));
        const std::string value = cli::trim(line.substr(eq + 1));

        if (cli::iequals(key, "CONFIG") || cli::iequals(key, "HW_CFG") || cli::iequals(key, "HARDWARE_CONFIG")) {
            session.config_path = resolve_session_path(path, value);
        } else if (cli::iequals(key, "ROM")) {
            session.rom_path = resolve_session_path(path, value);
        } else if (cli::iequals(key, "ROM_FORMAT")) {
            const auto format = parse_rom_format(value);
            if (!format) {
                error = "Bad ROM_FORMAT at line " + std::to_string(lineno);
                return std::nullopt;
            }
            session.rom_format = *format;
        } else if (cli::iequals(key, "RAW_BASE")) {
            const auto parsed = cli::parse_number(value);
            if (!parsed || *parsed > 0xFFFF) {
                error = "Bad RAW_BASE at line " + std::to_string(lineno);
                return std::nullopt;
            }
            session.raw_base = static_cast<uint16_t>(*parsed);
        } else if (cli::iequals(key, "CF") || cli::iequals(key, "CF_IMAGE")) {
            session.cf_path = resolve_session_path(path, value);
        } else if (cli::iequals(key, "CF_SECTORS")) {
            const auto parsed = cli::parse_number(value);
            if (!parsed) {
                error = "Bad CF_SECTORS at line " + std::to_string(lineno);
                return std::nullopt;
            }
            session.cf_sectors = *parsed;
        } else if (cli::iequals(key, "CPU") || cli::iequals(key, "CPU_MODE")) {
            if (cli::iequals(value, "HD6309")) {
                session.mode = CpuMode::HD6309;
            } else if (cli::iequals(value, "MC6809") || cli::iequals(value, "6809")) {
                session.mode = CpuMode::MC6809;
            } else {
                error = "Bad CPU mode at line " + std::to_string(lineno);
                return std::nullopt;
            }
        } else if (cli::iequals(key, "LAYOUT_HEX")) {
            const auto decoded = hex_decode(value);
            if (!decoded) {
                error = "Bad LAYOUT_HEX at line " + std::to_string(lineno);
                return std::nullopt;
            }
            session.layout_ini = *decoded;
        } else if (cli::iequals(key, "MEMORY_START")) {
            const auto parsed = cli::parse_number(value);
            if (!parsed || *parsed > 0xFFFF) {
                error = "Bad MEMORY_START at line " + std::to_string(lineno);
                return std::nullopt;
            }
            session.gui.memory_start = static_cast<uint16_t>(*parsed);
        } else if (cli::iequals(key, "MEMORY_ROWS")) {
            const auto parsed = cli::parse_number(value);
            if (!parsed) {
                error = "Bad MEMORY_ROWS at line " + std::to_string(lineno);
                return std::nullopt;
            }
            session.gui.memory_rows = static_cast<int>(*parsed);
        } else if (cli::iequals(key, "MEMORY_FOLLOW_PC")) {
            const auto parsed = parse_bool(value);
            if (!parsed) {
                error = "Bad MEMORY_FOLLOW_PC at line " + std::to_string(lineno);
                return std::nullopt;
            }
            session.gui.memory_follow_pc = *parsed;
        } else if (cli::iequals(key, "STACK_REGISTER")) {
            if (cli::iequals(value, "S") || value == "0") {
                session.gui.stack_register_index = 0;
            } else if (cli::iequals(value, "U") || value == "1") {
                session.gui.stack_register_index = 1;
            } else {
                error = "Bad STACK_REGISTER at line " + std::to_string(lineno);
                return std::nullopt;
            }
        } else if (cli::iequals(key, "STACK_START")) {
            const auto parsed = cli::parse_number(value);
            if (!parsed || *parsed > 0xFFFF) {
                error = "Bad STACK_START at line " + std::to_string(lineno);
                return std::nullopt;
            }
            session.gui.stack_start = static_cast<uint16_t>(*parsed);
        } else if (cli::iequals(key, "STACK_ROWS")) {
            const auto parsed = cli::parse_number(value);
            if (!parsed) {
                error = "Bad STACK_ROWS at line " + std::to_string(lineno);
                return std::nullopt;
            }
            session.gui.stack_rows = static_cast<int>(*parsed);
        } else if (cli::iequals(key, "STACK_FOLLOW")) {
            const auto parsed = parse_bool(value);
            if (!parsed) {
                error = "Bad STACK_FOLLOW at line " + std::to_string(lineno);
                return std::nullopt;
            }
            session.gui.stack_follow_pointer = *parsed;
        } else if (cli::iequals(key, "SERIAL_HEX_VIEW")) {
            const auto parsed = parse_bool(value);
            if (!parsed) {
                error = "Bad SERIAL_HEX_VIEW at line " + std::to_string(lineno);
                return std::nullopt;
            }
            session.gui.serial_hex_view = *parsed;
        } else if (cli::iequals(key, "SERIAL_RX_HEX")) {
            const auto parsed = parse_bool(value);
            if (!parsed) {
                error = "Bad SERIAL_RX_HEX at line " + std::to_string(lineno);
                return std::nullopt;
            }
            session.gui.serial_rx_hex = *parsed;
        } else if (cli::iequals(key, "OPERATIONS_PER_MINUTE") || cli::iequals(key, "STEPS_PER_FRAME")) {
            const auto parsed = cli::parse_number(value);
            if (!parsed) {
                error = "Bad " + key + " at line " + std::to_string(lineno);
                return std::nullopt;
            }
            session.gui.operations_per_minute = static_cast<int>(*parsed);
        } else if (cli::iequals(key, "RUN_MICRO_STEPS")) {
            const auto parsed = parse_bool(value);
            if (!parsed) {
                error = "Bad RUN_MICRO_STEPS at line " + std::to_string(lineno);
                return std::nullopt;
            }
            session.gui.run_micro_steps = *parsed;
        } else if (cli::iequals(key, "TRUE_CLOCK_HZ")) {
            const auto parsed = cli::parse_number(value);
            if (!parsed || *parsed == 0) {
                error = "Bad TRUE_CLOCK_HZ at line " + std::to_string(lineno);
                return std::nullopt;
            }
            session.gui.true_clock_hz = *parsed;
        } else if (cli::iequals(key, "GUI_THEME") || cli::iequals(key, "THEME")) {
            const auto parsed = parse_gui_theme(value);
            if (!parsed) {
                error = "Bad " + key + " at line " + std::to_string(lineno);
                return std::nullopt;
            }
            session.gui.theme = *parsed;
        } else if (cli::iequals(key, "SHOW_FILES")) {
            if (!parse_named_bool(key, value, session.gui.show_file_panel, error, lineno)) return std::nullopt;
        } else if (cli::iequals(key, "SHOW_CONTROL")) {
            if (!parse_named_bool(key, value, session.gui.show_control_panel, error, lineno)) return std::nullopt;
        } else if (cli::iequals(key, "SHOW_REGISTERS")) {
            if (!parse_named_bool(key, value, session.gui.show_registers, error, lineno)) return std::nullopt;
        } else if (cli::iequals(key, "SHOW_DISASSEMBLY")) {
            if (!parse_named_bool(key, value, session.gui.show_disassembly, error, lineno)) return std::nullopt;
        } else if (cli::iequals(key, "SHOW_MEMORY")) {
            if (!parse_named_bool(key, value, session.gui.show_memory_viewer, error, lineno)) return std::nullopt;
        } else if (cli::iequals(key, "SHOW_STACK")) {
            if (!parse_named_bool(key, value, session.gui.show_stack, error, lineno)) return std::nullopt;
        } else if (cli::iequals(key, "SHOW_MEMORY_MAP")) {
            if (!parse_named_bool(key, value, session.gui.show_memory_map, error, lineno)) return std::nullopt;
        } else if (cli::iequals(key, "SHOW_MEMORY_MAPPER")) {
            if (!parse_named_bool(key, value, session.gui.show_mapper, error, lineno)) return std::nullopt;
        } else if (cli::iequals(key, "SHOW_PLD_LOGIC")) {
            if (!parse_named_bool(key, value, session.gui.show_pld_logic, error, lineno)) return std::nullopt;
        } else if (cli::iequals(key, "SHOW_COMPACT_FLASH")) {
            if (!parse_named_bool(key, value, session.gui.show_compact_flash, error, lineno)) return std::nullopt;
        } else if (cli::iequals(key, "SHOW_PARALLEL")) {
            if (!parse_named_bool(key, value, session.gui.show_parallel, error, lineno)) return std::nullopt;
        } else if (cli::iequals(key, "SHOW_BREAKPOINTS")) {
            if (!parse_named_bool(key, value, session.gui.show_breakpoints, error, lineno)) return std::nullopt;
        } else if (cli::iequals(key, "SHOW_WATCHPOINTS")) {
            if (!parse_named_bool(key, value, session.gui.show_watchpoints, error, lineno)) return std::nullopt;
        } else if (cli::iequals(key, "SHOW_TRACE")) {
            if (!parse_named_bool(key, value, session.gui.show_trace, error, lineno)) return std::nullopt;
        } else if (cli::iequals(key, "SHOW_SERIAL")) {
            if (!parse_named_bool(key, value, session.gui.show_serial, error, lineno)) return std::nullopt;
        } else if (cli::iequals(key, "SHOW_LOG")) {
            if (!parse_named_bool(key, value, session.gui.show_log, error, lineno)) return std::nullopt;
        } else if (cli::iequals(key, "BREAKPOINT")) {
            auto breakpoint = parse_breakpoint(value, error, lineno);
            if (!breakpoint) return std::nullopt;
            session.breakpoints.push_back(std::move(*breakpoint));
        } else if (cli::iequals(key, "WATCHPOINT")) {
            auto watchpoint = parse_watchpoint(value, error, lineno);
            if (!watchpoint) return std::nullopt;
            session.watchpoints.push_back(std::move(*watchpoint));
        }
    }

    if (session.config_path.empty()) {
        error = "Session is missing CONFIG.";
        return std::nullopt;
    }
    if (session.rom_path.empty()) {
        error = "Session is missing ROM.";
        return std::nullopt;
    }
    return session;
}

bool save_session_definition(const std::filesystem::path& path, const SessionDefinition& session, std::string& error) {
    if (path.empty()) {
        error = "Session path is empty.";
        return false;
    }

    std::ofstream file(path);
    if (!file) {
        error = "Cannot save session: " + path.string();
        return false;
    }

    write_path(file, "CONFIG", session_relative_path(path, session.config_path));
    write_path(file, "ROM", session_relative_path(path, session.rom_path));
    file << "ROM_FORMAT=" << rom_format_name(session.rom_format) << '\n';
    file << "RAW_BASE=0x" << cli::hex4(session.raw_base) << '\n';
    if (!session.cf_path.empty()) {
        write_path(file, "CF", session_relative_path(path, session.cf_path));
        file << "CF_SECTORS=" << session.cf_sectors << '\n';
    }
    if (session.mode) {
        file << "CPU=" << cpu_mode_name(*session.mode) << '\n';
    }
    if (!session.layout_ini.empty()) {
        file << "LAYOUT_HEX=" << hex_encode(session.layout_ini) << '\n';
    }

    file << "MEMORY_START=0x" << cli::hex4(session.gui.memory_start) << '\n';
    file << "MEMORY_ROWS=" << session.gui.memory_rows << '\n';
    file << "MEMORY_FOLLOW_PC=" << (session.gui.memory_follow_pc ? "true" : "false") << '\n';
    file << "STACK_REGISTER=" << (session.gui.stack_register_index == 1 ? "U" : "S") << '\n';
    file << "STACK_START=0x" << cli::hex4(session.gui.stack_start) << '\n';
    file << "STACK_ROWS=" << session.gui.stack_rows << '\n';
    file << "STACK_FOLLOW=" << (session.gui.stack_follow_pointer ? "true" : "false") << '\n';
    file << "SERIAL_HEX_VIEW=" << (session.gui.serial_hex_view ? "true" : "false") << '\n';
    file << "SERIAL_RX_HEX=" << (session.gui.serial_rx_hex ? "true" : "false") << '\n';
    file << "OPERATIONS_PER_MINUTE=" << session.gui.operations_per_minute << '\n';
    file << "RUN_MICRO_STEPS=" << (session.gui.run_micro_steps ? "true" : "false") << '\n';
    file << "TRUE_CLOCK_HZ=" << session.gui.true_clock_hz << '\n';
    file << "GUI_THEME=" << gui_theme_name(session.gui.theme) << '\n';
    write_bool(file, "SHOW_FILES", session.gui.show_file_panel);
    write_bool(file, "SHOW_CONTROL", session.gui.show_control_panel);
    write_bool(file, "SHOW_REGISTERS", session.gui.show_registers);
    write_bool(file, "SHOW_DISASSEMBLY", session.gui.show_disassembly);
    write_bool(file, "SHOW_MEMORY", session.gui.show_memory_viewer);
    write_bool(file, "SHOW_STACK", session.gui.show_stack);
    write_bool(file, "SHOW_MEMORY_MAP", session.gui.show_memory_map);
    write_bool(file, "SHOW_MEMORY_MAPPER", session.gui.show_mapper);
    write_bool(file, "SHOW_PLD_LOGIC", session.gui.show_pld_logic);
    write_bool(file, "SHOW_COMPACT_FLASH", session.gui.show_compact_flash);
    write_bool(file, "SHOW_PARALLEL", session.gui.show_parallel);
    write_bool(file, "SHOW_BREAKPOINTS", session.gui.show_breakpoints);
    write_bool(file, "SHOW_WATCHPOINTS", session.gui.show_watchpoints);
    write_bool(file, "SHOW_TRACE", session.gui.show_trace);
    write_bool(file, "SHOW_SERIAL", session.gui.show_serial);
    write_bool(file, "SHOW_LOG", session.gui.show_log);

    for (const auto& breakpoint : session.breakpoints) {
        file << "BREAKPOINT=0x" << cli::hex4(breakpoint.address)
             << ";ENABLED=" << (breakpoint.enabled ? "true" : "false")
             << ";HITS=" << breakpoint.hits;
        if (!breakpoint.label.empty()) file << ";LABEL_HEX=" << hex_encode(breakpoint.label);
        file << '\n';
    }

    for (const auto& watchpoint : session.watchpoints) {
        file << "WATCHPOINT=0x" << cli::hex4(watchpoint.address)
             << ";TYPE=" << watchpoint_type_name(watchpoint.type)
             << ";ENABLED=" << (watchpoint.enabled ? "true" : "false")
             << ";HITS=" << watchpoint.hits;
        if (!watchpoint.label.empty()) file << ";LABEL_HEX=" << hex_encode(watchpoint.label);
        file << '\n';
    }

    if (!file) {
        error = "Failed while saving session: " + path.string();
        return false;
    }
    return true;
}

} // namespace microlind::app
