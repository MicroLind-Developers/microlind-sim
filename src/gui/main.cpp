#include <algorithm>
#include <array>
#include <cctype>
#include <cstdint>
#include <cstdio>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

#include <SDL.h>
#include "imgui.h"
#include "imgui_impl_sdl2.h"
#include "imgui_impl_sdlrenderer2.h"
#ifdef MICROLIND_HAS_PORTABLE_FILE_DIALOGS
#include "portable-file-dialogs.h"
#endif

#include "microlind/app/disassembler.hpp"
#include "microlind/app/image_loader.hpp"
#include "microlind/app/sim_session.hpp"
#include "microlind/app/util.hpp"
#include "microlind/cpu.hpp"

namespace {

constexpr uint8_t kFlagBits[] = {
    microlind::CC_E,
    microlind::CC_F,
    microlind::CC_H,
    microlind::CC_I,
    microlind::CC_N,
    microlind::CC_Z,
    microlind::CC_V,
    microlind::CC_C,
};

constexpr const char* kFlagNames[] = {"E", "F", "H", "I", "N", "Z", "V", "C"};

template <std::size_t N>
void set_buffer(std::array<char, N>& buffer, std::string_view value) {
    buffer.fill('\0');
    const std::size_t count = std::min(value.size(), N - 1);
    std::copy_n(value.data(), count, buffer.data());
}

template <std::size_t N>
std::string buffer_string(const std::array<char, N>& buffer) {
    return std::string(buffer.data());
}

std::string hex_value(uint32_t value, int width) {
    std::ostringstream out;
    out << "0x" << std::uppercase << std::hex << std::setfill('0') << std::setw(width) << value;
    return out.str();
}

std::string instruction_bytes(microlind::Bus& bus, uint16_t pc, uint8_t length) {
    std::ostringstream out;
    out << std::uppercase << std::hex << std::setfill('0');
    for (uint8_t i = 0; i < length; ++i) {
        if (i > 0) out << ' ';
        out << std::setw(2) << static_cast<int>(bus.read8(static_cast<uint16_t>(pc + i)));
    }
    return out.str();
}

std::string serial_terminal_text(const std::vector<uint8_t>& bytes) {
    std::string out;
    out.reserve(bytes.size());
    for (uint8_t value : bytes) {
        if (value == '\r') continue;
        if (value == '\n' || value == '\t' || std::isprint(static_cast<unsigned char>(value))) {
            out.push_back(static_cast<char>(value));
        } else {
            out.push_back('.');
        }
    }
    return out;
}

std::vector<uint8_t> parse_hex_bytes(std::string_view input, bool& ok) {
    std::vector<uint8_t> bytes;
    std::string token;
    ok = true;

    auto flush_token = [&]() {
        if (token.empty()) return;
        if (token.size() > 2) {
            ok = false;
            token.clear();
            return;
        }
        uint32_t value = 0;
        std::istringstream in(token);
        in >> std::hex >> value;
        if (!in || value > 0xFF) {
            ok = false;
        } else {
            bytes.push_back(static_cast<uint8_t>(value));
        }
        token.clear();
    };

    for (char ch : input) {
        if (std::isxdigit(static_cast<unsigned char>(ch))) {
            token.push_back(ch);
        } else if (std::isspace(static_cast<unsigned char>(ch)) || ch == ',' || ch == ';') {
            flush_token();
        } else {
            ok = false;
        }
    }
    flush_token();
    if (!ok) bytes.clear();
    return bytes;
}

const char* watchpoint_type_label(microlind::app::WatchpointType type) {
    switch (type) {
    case microlind::app::WatchpointType::Read: return "R";
    case microlind::app::WatchpointType::Write: return "W";
    case microlind::app::WatchpointType::ReadWrite: return "RW";
    }
    return "?";
}

const char* cf_transfer_label(microlind::app::CfTransferMode mode) {
    switch (mode) {
    case microlind::app::CfTransferMode::None: return "Idle";
    case microlind::app::CfTransferMode::Read: return "Read";
    case microlind::app::CfTransferMode::Write: return "Write";
    }
    return "?";
}

std::string cf_status_flags(uint8_t status) {
    std::vector<std::string_view> flags;
    if ((status & 0x01) != 0) flags.push_back("ERR");
    if ((status & 0x08) != 0) flags.push_back("DRQ");
    if ((status & 0x10) != 0) flags.push_back("DSC");
    if ((status & 0x40) != 0) flags.push_back("DRDY");
    if (flags.empty()) return "-";

    std::ostringstream out;
    for (std::size_t i = 0; i < flags.size(); ++i) {
        if (i > 0) out << ' ';
        out << flags[i];
    }
    return out.str();
}

std::string cf_command_name(uint8_t command) {
    switch (command) {
    case 0x00: return "-";
    case 0x10: return "Recalibrate";
    case 0x20:
    case 0x21: return "Read Sectors";
    case 0x30:
    case 0x31: return "Write Sectors";
    case 0x40:
    case 0x41: return "Read Verify";
    case 0x50: return "Format Track";
    case 0x90: return "Execute Diagnostic";
    case 0x91: return "Initialize Drive Parameters";
    case 0x95:
    case 0xE1: return "Idle Immediate";
    case 0x96:
    case 0xE2: return "Standby";
    case 0x97:
    case 0xE3: return "Idle";
    case 0x98:
    case 0xE5: return "Check Power Mode";
    case 0x99:
    case 0xE6: return "Sleep";
    case 0xC0: return "Erase Sectors";
    case 0xC4: return "Read Multiple";
    case 0xC5: return "Write Multiple";
    case 0xC6: return "Set Multiple Mode";
    case 0xEC: return "Identify";
    case 0xEF: return "Set Features";
    default: return "Unknown";
    }
}

#ifdef MICROLIND_HAS_PORTABLE_FILE_DIALOGS
std::string pick_file(const std::string& title, const std::vector<std::string>& filters) {
    const auto result = pfd::open_file(title, "", filters).result();
    if (result.empty()) return {};
    return result.front();
}

std::string pick_save_file(const std::string& title, const std::string& default_path, const std::vector<std::string>& filters) {
    return pfd::save_file(title, default_path, filters).result();
}
#endif

microlind::cli::RomFormat selected_rom_format(int index) {
    switch (index) {
    case 0: return microlind::cli::RomFormat::Raw;
    case 1: return microlind::cli::RomFormat::Ihex;
    case 2: return microlind::cli::RomFormat::Srec;
    default: return microlind::cli::RomFormat::Ihex;
    }
}

int rom_format_combo_index(microlind::cli::RomFormat format) {
    switch (format) {
    case microlind::cli::RomFormat::Raw: return 0;
    case microlind::cli::RomFormat::Ihex: return 1;
    case microlind::cli::RomFormat::Srec: return 2;
    case microlind::cli::RomFormat::None: return 1;
    }
    return 1;
}

std::optional<microlind::cli::RomFormat> parse_rom_format(std::string value) {
    value = microlind::cli::trim(value);
    if (microlind::cli::iequals(value, "raw") || microlind::cli::iequals(value, "bin") ||
        microlind::cli::iequals(value, "binary")) {
        return microlind::cli::RomFormat::Raw;
    }
    if (microlind::cli::iequals(value, "ihex") || microlind::cli::iequals(value, "intelhex") ||
        microlind::cli::iequals(value, "hex")) {
        return microlind::cli::RomFormat::Ihex;
    }
    if (microlind::cli::iequals(value, "srec") || microlind::cli::iequals(value, "srecord") ||
        microlind::cli::iequals(value, "s19")) {
        return microlind::cli::RomFormat::Srec;
    }
    return std::nullopt;
}

std::optional<std::string> hex_decode(std::string_view value);

struct SessionDefinition {
    std::filesystem::path config_path;
    std::filesystem::path rom_path;
    std::filesystem::path cf_path;
    std::string layout_ini;
    microlind::cli::RomFormat rom_format{microlind::cli::RomFormat::Ihex};
    uint16_t raw_base{0x8000};
    uint32_t cf_sectors{};
    std::optional<microlind::CpuMode> mode;
};

std::filesystem::path resolve_session_path(
    const std::filesystem::path& session_path,
    const std::filesystem::path& value) {
    if (value.empty() || value.is_absolute()) return value;
    return session_path.parent_path() / value;
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
        line = microlind::cli::trim(line);
        if (line.empty() || line[0] == '#' || line[0] == ';') continue;
        if (line.front() == '[' && line.back() == ']') continue;

        const auto eq = line.find('=');
        if (eq == std::string::npos) {
            error = "Bad session line " + std::to_string(lineno);
            return std::nullopt;
        }
        const std::string key = microlind::cli::trim(line.substr(0, eq));
        const std::string value = microlind::cli::trim(line.substr(eq + 1));

        if (microlind::cli::iequals(key, "CONFIG") || microlind::cli::iequals(key, "HW_CFG") ||
            microlind::cli::iequals(key, "HARDWARE_CONFIG")) {
            session.config_path = resolve_session_path(path, value);
        } else if (microlind::cli::iequals(key, "ROM")) {
            session.rom_path = resolve_session_path(path, value);
        } else if (microlind::cli::iequals(key, "ROM_FORMAT")) {
            const auto format = parse_rom_format(value);
            if (!format) {
                error = "Bad ROM_FORMAT at line " + std::to_string(lineno);
                return std::nullopt;
            }
            session.rom_format = *format;
        } else if (microlind::cli::iequals(key, "RAW_BASE")) {
            const auto parsed = microlind::cli::parse_number(value);
            if (!parsed || *parsed > 0xFFFF) {
                error = "Bad RAW_BASE at line " + std::to_string(lineno);
                return std::nullopt;
            }
            session.raw_base = static_cast<uint16_t>(*parsed);
        } else if (microlind::cli::iequals(key, "CF") || microlind::cli::iequals(key, "CF_IMAGE")) {
            session.cf_path = resolve_session_path(path, value);
        } else if (microlind::cli::iequals(key, "CF_SECTORS")) {
            const auto parsed = microlind::cli::parse_number(value);
            if (!parsed) {
                error = "Bad CF_SECTORS at line " + std::to_string(lineno);
                return std::nullopt;
            }
            session.cf_sectors = *parsed;
        } else if (microlind::cli::iequals(key, "CPU") || microlind::cli::iequals(key, "CPU_MODE")) {
            if (microlind::cli::iequals(value, "HD6309")) {
                session.mode = microlind::CpuMode::HD6309;
            } else if (microlind::cli::iequals(value, "MC6809") || microlind::cli::iequals(value, "6809")) {
                session.mode = microlind::CpuMode::MC6809;
            } else {
                error = "Bad CPU mode at line " + std::to_string(lineno);
                return std::nullopt;
            }
        } else if (microlind::cli::iequals(key, "LAYOUT_HEX")) {
            const auto decoded = hex_decode(value);
            if (!decoded) {
                error = "Bad LAYOUT_HEX at line " + std::to_string(lineno);
                return std::nullopt;
            }
            session.layout_ini = *decoded;
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

const char* rom_format_name(microlind::cli::RomFormat format) {
    switch (format) {
    case microlind::cli::RomFormat::Raw: return "raw";
    case microlind::cli::RomFormat::Ihex: return "ihex";
    case microlind::cli::RomFormat::Srec: return "srec";
    case microlind::cli::RomFormat::None: return "ihex";
    }
    return "ihex";
}

const char* cpu_mode_name(microlind::CpuMode mode) {
    return mode == microlind::CpuMode::HD6309 ? "HD6309" : "MC6809";
}

char hex_digit(uint8_t value) {
    return static_cast<char>(value < 10 ? ('0' + value) : ('A' + value - 10));
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

std::optional<uint8_t> hex_nibble(char ch) {
    if (ch >= '0' && ch <= '9') return static_cast<uint8_t>(ch - '0');
    if (ch >= 'a' && ch <= 'f') return static_cast<uint8_t>(ch - 'a' + 10);
    if (ch >= 'A' && ch <= 'F') return static_cast<uint8_t>(ch - 'A' + 10);
    return std::nullopt;
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

struct GuiState {
    microlind::app::SimSession session{microlind::CpuMode::HD6309};

    std::array<char, 512> rom_path{};
    std::array<char, 512> config_path{};
    std::array<char, 512> cf_path{};
    std::array<char, 512> session_path{};
    std::array<char, 256> serial_input{};
    std::string pending_layout_ini;
    int rom_format_index{1};
    int raw_base{0x8000};
    int cf_min_sectors{0};
    int steps_per_frame{100};
    int memory_start{0x0000};
    int memory_rows{16};
    bool memory_follow_pc{false};
    int breakpoint_address{0x0000};
    int run_until_address{0x0000};
    int watchpoint_address{0x0000};
    int stack_register_index{0};
    int stack_start{0x0000};
    int stack_rows{32};
    bool stack_follow_pointer{true};
    bool serial_hex_view{false};
    bool serial_rx_hex{false};
    bool running{false};
    bool run_until_active{false};
    bool quit_requested{false};

    GuiState() {
        set_buffer(session_path, "examples/bios.session");
        set_buffer(rom_path, "examples/bios.ihex");
        set_buffer(config_path, "examples/hw.cfg");
        set_buffer(cf_path, "examples/sim.img");
        session.add_log("GUI ready.");
    }

    void stop_execution() {
        running = false;
        run_until_active = false;
    }

    void load_rom() {
        const std::string path = buffer_string(rom_path);
        session.load_rom(path, selected_rom_format(rom_format_index), static_cast<uint16_t>(raw_base));
    }

    void load_config() {
        const std::string path = buffer_string(config_path);
        session.load_hardware_config(path);
    }

    void attach_cf() {
        const std::string path = buffer_string(cf_path);
        session.attach_cf_image(path, static_cast<uint32_t>(std::max(cf_min_sectors, 0)));
    }

    void load_session_file(const std::filesystem::path& path) {
        std::string error;
        const auto loaded = load_session_definition(path, error);
        if (!loaded) {
            session.add_log("Session error: " + error);
            return;
        }

        stop_execution();
        if (loaded->mode) {
            session.set_mode(*loaded->mode);
        }
        set_buffer(session_path, path.string());
        set_buffer(config_path, loaded->config_path.string());
        set_buffer(rom_path, loaded->rom_path.string());
        set_buffer(cf_path, loaded->cf_path.string());
        rom_format_index = rom_format_combo_index(loaded->rom_format);
        raw_base = loaded->raw_base;
        cf_min_sectors = static_cast<int>(loaded->cf_sectors);

        const bool config_ok = session.load_hardware_config(loaded->config_path);
        const bool rom_ok = config_ok && session.load_rom(loaded->rom_path, loaded->rom_format, loaded->raw_base);
        if (rom_ok && !loaded->cf_path.empty()) {
            session.attach_cf_image(loaded->cf_path, loaded->cf_sectors);
        }
        if (config_ok && rom_ok) {
            pending_layout_ini = loaded->layout_ini;
            session.add_log("Loaded session: " + path.string());
        }
    }

    void load_session_from_field() {
        load_session_file(buffer_string(session_path));
    }

    bool save_session_file(const std::filesystem::path& path) {
        if (path.empty()) {
            session.add_log("Session path is empty.");
            return false;
        }

        std::ofstream file(path);
        if (!file) {
            session.add_log("Cannot save session: " + path.string());
            return false;
        }

        const auto format = selected_rom_format(rom_format_index);
        const auto config = session_relative_path(path, buffer_string(config_path));
        const auto rom = session_relative_path(path, buffer_string(rom_path));
        const auto cf = session_relative_path(path, buffer_string(cf_path));

        file << "CONFIG=" << config.generic_string() << '\n';
        file << "ROM=" << rom.generic_string() << '\n';
        file << "ROM_FORMAT=" << rom_format_name(format) << '\n';
        file << "RAW_BASE=" << hex_value(static_cast<uint16_t>(raw_base), 4) << '\n';
        if (!cf.empty()) {
            file << "CF=" << cf.generic_string() << '\n';
            file << "CF_SECTORS=" << std::max(cf_min_sectors, 0) << '\n';
        }
        file << "CPU=" << cpu_mode_name(session.mode()) << '\n';

        std::size_t layout_size = 0;
        const char* layout = ImGui::SaveIniSettingsToMemory(&layout_size);
        if (layout != nullptr && layout_size > 0) {
            file << "LAYOUT_HEX=" << hex_encode(std::string_view(layout, layout_size)) << '\n';
        }

        if (!file) {
            session.add_log("Failed while saving session: " + path.string());
            return false;
        }

        set_buffer(session_path, path.string());
        session.add_log("Saved session: " + path.string());
        return true;
    }

    void save_session_from_field() {
        save_session_file(buffer_string(session_path));
    }

    void save_session_as() {
#ifdef MICROLIND_HAS_PORTABLE_FILE_DIALOGS
        const auto path = pick_save_file(
            "Save session",
            buffer_string(session_path),
            {"Session files", "*.session", "All files", "*"});
        if (!path.empty()) {
            save_session_file(path);
        }
#else
        save_session_from_field();
#endif
    }

    void send_serial_text() {
        const std::string text = buffer_string(serial_input);
        bool ok = true;
        const std::vector<uint8_t> bytes = serial_rx_hex ? parse_hex_bytes(text, ok) : std::vector<uint8_t>(text.begin(), text.end());
        if (!ok) {
            session.add_log("Serial RX hex parse error.");
            return;
        }
        if (session.inject_serial_bytes(bytes)) {
            set_buffer(serial_input, "");
        }
    }

    void step_once() {
        const auto result = session.run_instructions(1);
        if (result.hit_breakpoint || result.hit_watchpoint) {
            stop_execution();
        }
    }

    void toggle_run() {
        running = !running;
        if (running) {
            run_until_active = false;
        }
    }

    void step_over() {
        const auto target = session.step_over_target();
        if (!target) {
            step_once();
            return;
        }
        run_until_address = *target;
        running = false;
        run_until_active = true;
        session.add_log("Stepping over until " + hex_value(static_cast<uint16_t>(run_until_address), 4) + ".");
    }

    void run_until_return() {
        const auto target = session.return_address_from_stack();
        if (!target) {
            session.add_log("No return address is available on S.");
            return;
        }
        run_until_address = *target;
        running = false;
        run_until_active = true;
        session.add_log("Running until return " + hex_value(static_cast<uint16_t>(run_until_address), 4) + ".");
    }
};

void set_next_window_defaults(float x, float y, float w, float h) {
    ImGui::SetNextWindowPos(ImVec2(x, y), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(w, h), ImGuiCond_FirstUseEver);
}

void draw_register_row(const char* name, uint32_t value, int width) {
    ImGui::TableNextRow();
    ImGui::TableNextColumn();
    ImGui::TextUnformatted(name);
    ImGui::TableNextColumn();
    const std::string text = hex_value(value, width);
    ImGui::TextUnformatted(text.c_str());
}

void draw_file_panel(GuiState& state) {
    set_next_window_defaults(8.0f, 28.0f, 360.0f, 340.0f);
    ImGui::Begin("Files");

    ImGui::InputText("Session", state.session_path.data(), state.session_path.size());
#ifdef MICROLIND_HAS_PORTABLE_FILE_DIALOGS
    ImGui::SameLine();
    if (ImGui::Button("Browse##session")) {
        const auto path = pick_file("Load session", {"Session files", "*.session *.ini *.cfg", "All files", "*"});
        if (!path.empty()) set_buffer(state.session_path, path);
    }
#endif
    if (ImGui::Button("Load Session")) {
        state.load_session_from_field();
    }

    ImGui::Separator();
    const char* formats[] = {"Raw", "Intel HEX", "S-record"};
    ImGui::Combo("ROM format", &state.rom_format_index, formats, static_cast<int>(std::size(formats)));
    if (state.rom_format_index == 0) {
        ImGui::InputInt("Raw base", &state.raw_base, 0x100, 0x1000, ImGuiInputTextFlags_CharsHexadecimal);
        state.raw_base = std::clamp(state.raw_base, 0, 0xFFFF);
    }
    ImGui::InputText("ROM", state.rom_path.data(), state.rom_path.size());
#ifdef MICROLIND_HAS_PORTABLE_FILE_DIALOGS
    ImGui::SameLine();
    if (ImGui::Button("Browse##rom")) {
        const auto path = pick_file("Load ROM", {"ROM files", "*.rom *.bin *.hex *.ihex *.s19 *.srec", "All files", "*"});
        if (!path.empty()) set_buffer(state.rom_path, path);
    }
#endif
    if (ImGui::Button("Load ROM")) {
        state.load_rom();
    }

    ImGui::Separator();
    ImGui::InputText("Config", state.config_path.data(), state.config_path.size());
#ifdef MICROLIND_HAS_PORTABLE_FILE_DIALOGS
    ImGui::SameLine();
    if (ImGui::Button("Browse##config")) {
        const auto path = pick_file("Load hardware config", {"Config files", "*.cfg *.ini", "All files", "*"});
        if (!path.empty()) set_buffer(state.config_path, path);
    }
#endif
    if (ImGui::Button("Load Config")) {
        state.load_config();
    }

    ImGui::Separator();
    ImGui::InputText("CF image", state.cf_path.data(), state.cf_path.size());
#ifdef MICROLIND_HAS_PORTABLE_FILE_DIALOGS
    ImGui::SameLine();
    if (ImGui::Button("Browse##cf")) {
        const auto path = pick_file("Attach CF image", {"Disk images", "*.img *.bin", "All files", "*"});
        if (!path.empty()) set_buffer(state.cf_path, path);
    }
#endif
    ImGui::InputInt("Min sectors", &state.cf_min_sectors);
    state.cf_min_sectors = std::max(state.cf_min_sectors, 0);
    if (ImGui::Button("Attach CF")) {
        state.attach_cf();
    }

    ImGui::Separator();
    const char* modes[] = {"MC6809", "HD6309"};
    int mode_index = state.session.mode() == microlind::CpuMode::HD6309 ? 1 : 0;
    if (ImGui::Combo("CPU mode", &mode_index, modes, static_cast<int>(std::size(modes)))) {
        state.session.set_mode(mode_index == 1 ? microlind::CpuMode::HD6309 : microlind::CpuMode::MC6809);
    }

    ImGui::End();
}

void draw_control_panel(GuiState& state) {
    set_next_window_defaults(376.0f, 28.0f, 500.0f, 150.0f);
    ImGui::Begin("Control");
    if (ImGui::Button("Reset")) {
        state.stop_execution();
        state.session.reset();
    }
    ImGui::SameLine();
    if (ImGui::Button("Step")) {
        state.step_once();
    }
    ImGui::SameLine();
    if (ImGui::Button("Step Over")) {
        state.step_over();
    }
    ImGui::SameLine();
    if (ImGui::Button(state.running ? "Pause" : "Run")) {
        state.toggle_run();
    }
    ImGui::SliderInt("Steps/frame", &state.steps_per_frame, 1, 5000);
    ImGui::InputInt("Run until", &state.run_until_address, 1, 256, ImGuiInputTextFlags_CharsHexadecimal);
    state.run_until_address = std::clamp(state.run_until_address, 0, 0xFFFF);
    if (ImGui::Button(state.run_until_active ? "Stop Until" : "Run Until")) {
        state.run_until_active = !state.run_until_active;
        if (state.run_until_active) {
            state.running = false;
            state.session.add_log("Running until " + hex_value(static_cast<uint16_t>(state.run_until_address), 4) + ".");
        }
    }
    ImGui::SameLine();
    if (ImGui::Button("Until Return")) {
        state.run_until_return();
    }
    ImGui::Text("Serial mapped: %s", state.session.serial_mapped() ? "yes" : "no");
    ImGui::End();
}

void draw_registers(const GuiState& state) {
    const auto& sim = state.session.simulator();
    const auto& regs = sim.cpu().regs();

    set_next_window_defaults(8.0f, 376.0f, 240.0f, 360.0f);
    ImGui::Begin("Registers");
    if (ImGui::BeginTable("registers", 2, ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_RowBg)) {
        draw_register_row("PC", regs.pc, 4);
        draw_register_row("A", regs.a, 2);
        draw_register_row("B", regs.b, 2);
        draw_register_row("E", regs.e, 2);
        draw_register_row("F", regs.f, 2);
        draw_register_row("DP", regs.dp, 2);
        draw_register_row("CC", regs.cc, 2);
        draw_register_row("MD", regs.md, 2);
        draw_register_row("X", regs.x, 4);
        draw_register_row("Y", regs.y, 4);
        draw_register_row("U", regs.u, 4);
        draw_register_row("S", regs.s, 4);
        draw_register_row("V", regs.v, 4);
        ImGui::EndTable();
    }

    ImGui::Separator();
    ImGui::TextUnformatted("Flags");
    for (std::size_t i = 0; i < std::size(kFlagBits); ++i) {
        bool active = (regs.cc & kFlagBits[i]) != 0;
        ImGui::BeginDisabled();
        ImGui::Checkbox(kFlagNames[i], &active);
        ImGui::EndDisabled();
        if (i + 1 < std::size(kFlagBits)) {
            ImGui::SameLine();
        }
    }

    ImGui::Separator();
    ImGui::TextUnformatted("Clock");
    ImGui::Text("Cycles: %llu", static_cast<unsigned long long>(sim.clock().total_cycles()));
    ImGui::Text("Clock: %llu Hz", static_cast<unsigned long long>(sim.clock().frequency_hz()));
    ImGui::End();
}

void draw_disassembly(GuiState& state) {
    set_next_window_defaults(376.0f, 186.0f, 560.0f, 340.0f);
    ImGui::Begin("Disassembly");
    auto& sim = state.session.simulator();
    uint16_t pc = sim.cpu().regs().pc;
    const uint16_t current_pc = pc;

    if (ImGui::BeginTable("disassembly", 3, ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_RowBg | ImGuiTableFlags_Resizable)) {
        ImGui::TableSetupColumn("Address", ImGuiTableColumnFlags_WidthFixed, 72.0f);
        ImGui::TableSetupColumn("Bytes", ImGuiTableColumnFlags_WidthFixed, 160.0f);
        ImGui::TableSetupColumn("Disassembly");
        ImGui::TableHeadersRow();

        for (int i = 0; i < 18; ++i) {
            const uint16_t line_pc = pc;
            const auto disasm = microlind::cli::disassemble(sim.bus(), sim.cpu(), line_pc);
            const uint8_t length = std::max<uint8_t>(disasm.length, 1);
            const std::string bytes = instruction_bytes(sim.bus(), line_pc, length);
            const bool is_current = line_pc == current_pc;

            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            if (is_current) {
                ImGui::TextColored(ImVec4(0.3f, 0.9f, 0.6f, 1.0f), "%04X", line_pc);
            } else {
                ImGui::Text("%04X", line_pc);
            }

            ImGui::TableNextColumn();
            if (is_current) {
                ImGui::TextColored(ImVec4(0.3f, 0.9f, 0.6f, 1.0f), "%s", bytes.c_str());
            } else {
                ImGui::TextUnformatted(bytes.c_str());
            }

            ImGui::TableNextColumn();
            if (is_current) {
                ImGui::TextColored(ImVec4(0.3f, 0.9f, 0.6f, 1.0f), "%s", disasm.text.c_str());
            } else {
                ImGui::TextUnformatted(disasm.text.c_str());
            }

            pc = static_cast<uint16_t>(pc + length);
        }
        ImGui::EndTable();
    }
    ImGui::End();
}

void draw_memory_map(const GuiState& state) {
    set_next_window_defaults(944.0f, 28.0f, 240.0f, 190.0f);
    ImGui::Begin("Memory Map");
    const std::vector<std::string> summary = state.session.memory_map();
    if (summary.empty()) {
        ImGui::TextDisabled("No mapped devices.");
    } else {
        for (const auto& line : summary) {
            ImGui::TextUnformatted(line.c_str());
        }
    }
    ImGui::End();
}

void draw_memory_viewer(GuiState& state) {
    set_next_window_defaults(944.0f, 528.0f, 560.0f, 260.0f);
    ImGui::Begin("Memory");

    auto& sim = state.session.simulator();
    const auto& regs = sim.cpu().regs();

    if (state.memory_follow_pc) {
        state.memory_start = regs.pc & 0xFFF0;
    }

    ImGui::SetNextItemWidth(96.0f);
    ImGui::InputInt("Start", &state.memory_start, 16, 256, ImGuiInputTextFlags_CharsHexadecimal);
    state.memory_start = std::clamp(state.memory_start, 0, 0xFFFF);
    state.memory_start &= 0xFFF0;

    ImGui::SameLine();
    ImGui::SetNextItemWidth(86.0f);
    ImGui::SliderInt("Rows", &state.memory_rows, 4, 64);

    ImGui::SameLine();
    ImGui::Checkbox("Follow PC", &state.memory_follow_pc);

    ImGui::SameLine();
    if (ImGui::Button("PC")) {
        state.memory_follow_pc = false;
        state.memory_start = regs.pc & 0xFFF0;
    }
    ImGui::SameLine();
    if (ImGui::Button("S")) {
        state.memory_follow_pc = false;
        state.memory_start = regs.s & 0xFFF0;
    }
    ImGui::SameLine();
    if (ImGui::Button("U")) {
        state.memory_follow_pc = false;
        state.memory_start = regs.u & 0xFFF0;
    }

    ImGui::SameLine();
    if (ImGui::ArrowButton("mem_prev", ImGuiDir_Left)) {
        state.memory_follow_pc = false;
        state.memory_start = std::clamp(state.memory_start - 0x100, 0, 0xFFFF) & 0xFFF0;
    }
    ImGui::SameLine();
    if (ImGui::ArrowButton("mem_next", ImGuiDir_Right)) {
        state.memory_follow_pc = false;
        state.memory_start = std::clamp(state.memory_start + 0x100, 0, 0xFFFF) & 0xFFF0;
    }

    constexpr int kCols = 16;
    const ImGuiTableFlags flags = ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_RowBg |
                                  ImGuiTableFlags_Resizable | ImGuiTableFlags_ScrollY |
                                  ImGuiTableFlags_SizingFixedFit;
    const float line_height = ImGui::GetTextLineHeightWithSpacing();
    const float table_height = std::max(line_height * 6.0f, ImGui::GetContentRegionAvail().y);
    if (ImGui::BeginTable("memory_view", kCols + 2, flags, ImVec2(0.0f, table_height))) {
        ImGui::TableSetupScrollFreeze(1, 1);
        ImGui::TableSetupColumn("Address", ImGuiTableColumnFlags_WidthFixed, 64.0f);
        for (int col = 0; col < kCols; ++col) {
            char label[4]{};
            std::snprintf(label, sizeof(label), "%X", col);
            ImGui::TableSetupColumn(
                label,
                ImGuiTableColumnFlags_WidthFixed | ImGuiTableColumnFlags_NoResize,
                32.0f);
        }
        ImGui::TableSetupColumn("ASCII", ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableHeadersRow();

        for (int row = 0; row < state.memory_rows; ++row) {
            ImGui::TableNextRow();
            const uint16_t row_address = static_cast<uint16_t>(state.memory_start + row * kCols);
            ImGui::TableNextColumn();
            ImGui::Text("%04X", row_address);

            std::array<char, kCols + 1> ascii{};
            for (int col = 0; col < kCols; ++col) {
                ImGui::TableNextColumn();
                const uint16_t address = static_cast<uint16_t>(row_address + col);
                uint8_t value = state.session.read_memory(address);
                ascii[static_cast<std::size_t>(col)] =
                    std::isprint(static_cast<unsigned char>(value)) ? static_cast<char>(value) : '.';

                const bool at_pc = address == regs.pc;
                const bool at_s = address == regs.s;
                const bool at_u = address == regs.u;
                const bool watched = state.session.is_watchpoint(address, microlind::app::WatchpointType::Read) ||
                                     state.session.is_watchpoint(address, microlind::app::WatchpointType::Write);
                if (at_pc) {
                    ImGui::TableSetBgColor(ImGuiTableBgTarget_CellBg, IM_COL32(38, 96, 56, 180));
                } else if (at_s || at_u) {
                    ImGui::TableSetBgColor(ImGuiTableBgTarget_CellBg, IM_COL32(72, 72, 116, 180));
                } else if (watched) {
                    ImGui::TableSetBgColor(ImGuiTableBgTarget_CellBg, IM_COL32(120, 92, 38, 180));
                }

                ImGui::PushID(static_cast<int>(address));
                ImGui::SetNextItemWidth(30.0f);
                if (ImGui::InputScalar(
                        "##byte",
                        ImGuiDataType_U8,
                        &value,
                        nullptr,
                        nullptr,
                        "%02X",
                        ImGuiInputTextFlags_CharsHexadecimal | ImGuiInputTextFlags_AutoSelectAll)) {
                    state.session.write_memory(address, value);
                    state.session.add_log("Wrote " + hex_value(value, 2) + " to " + hex_value(address, 4) + ".");
                }
                ImGui::PopID();
            }

            ImGui::TableNextColumn();
            ImGui::TextUnformatted(ascii.data());
        }
        ImGui::EndTable();
    }

    ImGui::End();
}

void draw_stack(GuiState& state) {
    const auto& regs = state.session.simulator().cpu().regs();
    const uint16_t stack_pointer = state.stack_register_index == 0 ? regs.s : regs.u;

    set_next_window_defaults(1188.0f, 28.0f, 236.0f, 490.0f);
    ImGui::Begin("Stack");

    const char* stack_names[] = {"S", "U"};
    ImGui::Combo("Register", &state.stack_register_index, stack_names, static_cast<int>(std::size(stack_names)));
    ImGui::Checkbox("Follow SP", &state.stack_follow_pointer);
    ImGui::SliderInt("Rows", &state.stack_rows, 8, 128);

    if (state.stack_follow_pointer) {
        state.stack_start = stack_pointer;
    } else {
        ImGui::InputInt("Start", &state.stack_start, 16, 256, ImGuiInputTextFlags_CharsHexadecimal);
        state.stack_start = std::clamp(state.stack_start, 0, 0xFFFF);
    }

    ImGui::Text("%s = %04X", stack_names[state.stack_register_index], stack_pointer);

    if (ImGui::BeginTable("stack", 6, ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_RowBg | ImGuiTableFlags_Resizable)) {
        ImGui::TableSetupColumn("");
        ImGui::TableSetupColumn("Address");
        ImGui::TableSetupColumn("+0");
        ImGui::TableSetupColumn("+1");
        ImGui::TableSetupColumn("Word");
        ImGui::TableSetupColumn("ASCII");
        ImGui::TableHeadersRow();

        for (int row = 0; row < state.stack_rows; ++row) {
            const uint16_t address = static_cast<uint16_t>(state.stack_start + row * 2);
            const uint8_t high = state.session.read_memory(address);
            const uint8_t low = state.session.read_memory(static_cast<uint16_t>(address + 1));
            const uint16_t word = static_cast<uint16_t>((high << 8) | low);
            const bool at_pointer = address == stack_pointer;

            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            if (at_pointer) {
                ImGui::TextColored(ImVec4(0.3f, 0.9f, 0.6f, 1.0f), "SP");
            } else {
                ImGui::TextUnformatted("");
            }

            ImGui::TableNextColumn();
            if (at_pointer) {
                ImGui::TextColored(ImVec4(0.3f, 0.9f, 0.6f, 1.0f), "%04X", address);
            } else {
                ImGui::Text("%04X", address);
            }

            ImGui::TableNextColumn();
            ImGui::Text("%02X", high);
            ImGui::TableNextColumn();
            ImGui::Text("%02X", low);
            ImGui::TableNextColumn();
            ImGui::Text("%04X", word);
            ImGui::TableNextColumn();
            const char c0 = std::isprint(static_cast<unsigned char>(high)) ? static_cast<char>(high) : '.';
            const char c1 = std::isprint(static_cast<unsigned char>(low)) ? static_cast<char>(low) : '.';
            ImGui::Text("%c%c", c0, c1);
        }

        ImGui::EndTable();
    }

    ImGui::End();
}

void draw_breakpoints(GuiState& state) {
    set_next_window_defaults(252.0f, 376.0f, 220.0f, 200.0f);
    ImGui::Begin("Breakpoints");
    ImGui::InputInt("Address", &state.breakpoint_address, 1, 256, ImGuiInputTextFlags_CharsHexadecimal);
    state.breakpoint_address = std::clamp(state.breakpoint_address, 0, 0xFFFF);

    if (ImGui::Button("Add")) {
        state.session.add_breakpoint(static_cast<uint16_t>(state.breakpoint_address));
    }
    ImGui::SameLine();
    if (ImGui::Button("Remove")) {
        state.session.remove_breakpoint(static_cast<uint16_t>(state.breakpoint_address));
    }
    ImGui::SameLine();
    if (ImGui::Button("Clear")) {
        state.session.clear_breakpoints();
        state.session.add_log("Cleared breakpoints.");
    }

    ImGui::Separator();
    for (uint16_t address : state.session.breakpoints()) {
        ImGui::Text("%04X", address);
        ImGui::SameLine();
        const std::string button_id = "Remove##bp" + std::to_string(address);
        if (ImGui::SmallButton(button_id.c_str())) {
            state.session.remove_breakpoint(address);
            break;
        }
    }
    ImGui::End();
}

void draw_watchpoints(GuiState& state) {
    set_next_window_defaults(252.0f, 582.0f, 220.0f, 210.0f);
    ImGui::Begin("Watchpoints");
    ImGui::InputInt("Address", &state.watchpoint_address, 1, 256, ImGuiInputTextFlags_CharsHexadecimal);
    state.watchpoint_address = std::clamp(state.watchpoint_address, 0, 0xFFFF);

    if (ImGui::Button("Add Read")) {
        state.session.add_watchpoint(static_cast<uint16_t>(state.watchpoint_address), microlind::app::WatchpointType::Read);
    }
    ImGui::SameLine();
    if (ImGui::Button("Add Write")) {
        state.session.add_watchpoint(static_cast<uint16_t>(state.watchpoint_address), microlind::app::WatchpointType::Write);
    }
    ImGui::SameLine();
    if (ImGui::Button("Clear")) {
        state.session.clear_watchpoints();
        state.session.add_log("Cleared watchpoints.");
    }

    ImGui::Separator();
    if (ImGui::BeginTable("watchpoints", 3, ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_RowBg)) {
        ImGui::TableSetupColumn("Address");
        ImGui::TableSetupColumn("Type");
        ImGui::TableSetupColumn("");
        ImGui::TableHeadersRow();

        for (const auto& watchpoint : state.session.watchpoints()) {
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::Text("%04X", watchpoint.address);
            ImGui::TableNextColumn();
            ImGui::TextUnformatted(watchpoint_type_label(watchpoint.type));
            ImGui::TableNextColumn();
            const std::string button_id = "Remove##wp" + std::to_string(watchpoint.address);
            if (ImGui::SmallButton(button_id.c_str())) {
                state.session.remove_watchpoint(watchpoint.address);
                break;
            }
        }
        ImGui::EndTable();
    }
    ImGui::End();
}

void draw_mapper(GuiState& state) {
    set_next_window_defaults(944.0f, 224.0f, 240.0f, 294.0f);
    ImGui::Begin("Memory Mapper");
    const auto mapper = state.session.mapper_snapshot();
    if (!mapper.present) {
        ImGui::TextDisabled("No memory mapper configured.");
        ImGui::End();
        return;
    }

    ImGui::Text("Bank size: %u", mapper.bank_size);
    ImGui::Text("Backing RAM: %u", mapper.available);

    if (ImGui::BeginTable("mapper_regs", 3, ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_RowBg)) {
        ImGui::TableSetupColumn("Window");
        ImGui::TableSetupColumn("Register");
        ImGui::TableSetupColumn("Selected bank");
        ImGui::TableHeadersRow();
        for (int i = 0; i < 4; ++i) {
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::Text("%d", i);
            ImGui::TableNextColumn();
            if (mapper.bank_registers[i] != 0) {
                ImGui::Text("%04X", mapper.bank_registers[i]);
            } else {
                ImGui::TextDisabled("-");
            }
            ImGui::TableNextColumn();
            ImGui::Text("%02X", mapper.selected_banks[i]);
        }
        ImGui::EndTable();
    }

    ImGui::Separator();
    if (ImGui::BeginTable("mapper_windows", 4, ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_RowBg)) {
        ImGui::TableSetupColumn("Range");
        ImGui::TableSetupColumn("Window");
        ImGui::TableSetupColumn("Bank");
        ImGui::TableSetupColumn("Physical");
        ImGui::TableHeadersRow();
        for (const auto& window : mapper.windows) {
            const uint32_t physical = static_cast<uint32_t>(window.selected_bank) * mapper.bank_size;
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::Text("%04X-%04X", window.start, window.end);
            ImGui::TableNextColumn();
            ImGui::Text("%u", window.window);
            ImGui::TableNextColumn();
            ImGui::Text("%02X", window.selected_bank);
            ImGui::TableNextColumn();
            ImGui::Text("%05X", physical);
        }
        ImGui::EndTable();
    }

    ImGui::End();
}

void draw_compact_flash(GuiState& state) {
    set_next_window_defaults(480.0f, 520.0f, 456.0f, 180.0f);
    ImGui::Begin("CompactFlash");
    const auto cf = state.session.cf_snapshot();
    if (!cf.present) {
        ImGui::TextDisabled("No CompactFlash device configured.");
        ImGui::End();
        return;
    }

    const std::string path = cf.image_path.empty() ? std::string("-") : cf.image_path.string();
    ImGui::Text("I/O: %04X-%04X", cf.start, cf.end);
    ImGui::Text("Image: %s", path.c_str());
    ImGui::Text("Sectors: %u", cf.sector_count);
    ImGui::Text("Mode: %s", cf.read_only ? "read-only" : "read/write");
    ImGui::Text("Transfer: %s", cf_transfer_label(cf.transfer_mode));
    if (cf.transfer_size > 0) {
        ImGui::SameLine();
        ImGui::Text("(%llu/%llu bytes)",
                    static_cast<unsigned long long>(cf.transfer_index),
                    static_cast<unsigned long long>(cf.transfer_size));
    }

    ImGui::Separator();
    if (ImGui::BeginTable("cf_registers", 3, ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_RowBg)) {
        ImGui::TableSetupColumn("Register");
        ImGui::TableSetupColumn("Value");
        ImGui::TableSetupColumn("Decoded");
        ImGui::TableHeadersRow();

        auto row = [](const char* name, uint32_t value, int width, const std::string& decoded = {}) {
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::TextUnformatted(name);
            ImGui::TableNextColumn();
            const std::string hex = hex_value(value, width);
            ImGui::TextUnformatted(hex.c_str());
            ImGui::TableNextColumn();
            ImGui::TextUnformatted(decoded.c_str());
        };

        row("Error", cf.error, 2);
        row("Features", cf.features, 2);
        row("Sector Count", cf.sector_count_reg, 2, std::to_string(cf.requested_sector_count));
        row("Sector Number", cf.sector_number, 2);
        row("Cylinder Low", cf.cylinder_low, 2);
        row("Cylinder High", cf.cylinder_high, 2);
        row("Drive/Head", cf.drive_head, 2, (cf.drive_head & 0x40) != 0 ? "LBA" : "CHS");
        row("Status", cf.status, 2, cf_status_flags(cf.status));
        row("Command", cf.command, 2, cf_command_name(cf.command));
        row("Selected LBA", cf.selected_lba, 8);

        ImGui::EndTable();
    }

    ImGui::End();
}

void draw_trace(GuiState& state) {
    set_next_window_defaults(480.0f, 706.0f, 456.0f, 160.0f);
    ImGui::Begin("Trace");
    const auto& trace = state.session.trace();
    if (ImGui::Button("Clear")) {
        state.session.clear_trace();
    }
    ImGui::SameLine();
    ImGui::Text("Entries: %llu", static_cast<unsigned long long>(state.session.trace().size()));
    ImGui::Separator();

    ImGui::BeginChild("trace_scroll", ImVec2(0, 0), false, ImGuiWindowFlags_HorizontalScrollbar);
    if (ImGui::BeginTable("trace", 4, ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_RowBg | ImGuiTableFlags_Resizable)) {
        ImGui::TableSetupColumn("PC");
        ImGui::TableSetupColumn("Instruction");
        ImGui::TableSetupColumn("Cycles");
        ImGui::TableSetupColumn("Total");
        ImGui::TableHeadersRow();

        for (auto it = trace.rbegin(); it != trace.rend(); ++it) {
            const auto& entry = *it;
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::Text("%04X", entry.pc);
            ImGui::TableNextColumn();
            ImGui::TextUnformatted(entry.instruction.c_str());
            ImGui::TableNextColumn();
            ImGui::Text("%u", entry.cycles);
            ImGui::TableNextColumn();
            ImGui::Text("%llu", static_cast<unsigned long long>(entry.total_cycles));
        }
        ImGui::EndTable();
    }
    ImGui::EndChild();
    ImGui::End();
}

void draw_serial(GuiState& state) {
    set_next_window_defaults(944.0f, 694.0f, 480.0f, 80.0f);
    ImGui::Begin("Serial");
    ImGui::BeginDisabled(!state.session.serial_mapped());
    ImGui::Checkbox("Hex RX", &state.serial_rx_hex);
    ImGui::SameLine();
    ImGui::InputText("RX text", state.serial_input.data(), state.serial_input.size());
    ImGui::SameLine();
    if (ImGui::Button("Send")) {
        state.send_serial_text();
    }
    ImGui::EndDisabled();

    if (ImGui::Button("Clear TX")) {
        state.session.clear_serial_tx();
    }
    ImGui::SameLine();
    ImGui::Checkbox("Hex view", &state.serial_hex_view);

    ImGui::Separator();
    ImGui::TextUnformatted("TX");
    ImGui::BeginChild("serial_tx", ImVec2(0, 0), true, ImGuiWindowFlags_HorizontalScrollbar);
    const auto& tx = state.session.serial_tx();
    if (state.serial_hex_view) {
        if (ImGui::BeginTable("serial_hex", 17, ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_RowBg)) {
            ImGui::TableSetupColumn("Offset");
            for (int i = 0; i < 16; ++i) {
                char label[4]{};
                std::snprintf(label, sizeof(label), "%X", i);
                ImGui::TableSetupColumn(label);
            }
            ImGui::TableHeadersRow();

            for (std::size_t row = 0; row < tx.size(); row += 16) {
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::Text("%04X", static_cast<unsigned>(row));
                for (std::size_t col = 0; col < 16; ++col) {
                    ImGui::TableNextColumn();
                    const std::size_t index = row + col;
                    if (index < tx.size()) {
                        ImGui::Text("%02X", tx[index]);
                    }
                }
            }
            ImGui::EndTable();
        }
    } else {
        const std::string text = serial_terminal_text(tx);
        ImGui::TextUnformatted(text.c_str());
    }
    ImGui::EndChild();
    ImGui::End();
}

void draw_log(GuiState& state) {
    set_next_window_defaults(944.0f, 780.0f, 480.0f, 86.0f);
    ImGui::Begin("Log");
    if (ImGui::Button("Clear")) {
        state.session.clear_log();
    }
    ImGui::Separator();
    for (const auto& line : state.session.log()) {
        ImGui::TextUnformatted(line.c_str());
    }
    if (ImGui::GetScrollY() >= ImGui::GetScrollMaxY()) {
        ImGui::SetScrollHereY(1.0f);
    }
    ImGui::End();
}

void draw_main_menu(GuiState& state) {
    if (!ImGui::BeginMainMenuBar()) return;

    if (ImGui::BeginMenu("File")) {
#ifdef MICROLIND_HAS_PORTABLE_FILE_DIALOGS
        if (ImGui::MenuItem("Load Session...", "Ctrl+O")) {
            const auto path = pick_file("Load session", {"Session files", "*.session *.ini *.cfg", "All files", "*"});
            if (!path.empty()) {
                state.load_session_file(path);
            }
        }
#else
        if (ImGui::MenuItem("Load Session", "Ctrl+O")) {
            state.load_session_from_field();
        }
#endif
        if (ImGui::MenuItem("Load Example Session")) {
            state.load_session_file("examples/bios.session");
        }
        ImGui::Separator();
        if (ImGui::MenuItem("Save Session", "Ctrl+S")) {
            state.save_session_from_field();
        }
        if (ImGui::MenuItem("Save Session As...", "Ctrl+Shift+S")) {
            state.save_session_as();
        }
        ImGui::Separator();
        if (ImGui::MenuItem("Exit", "Ctrl+Q")) {
            state.quit_requested = true;
        }
        ImGui::EndMenu();
    }

    if (ImGui::BeginMenu("Simulator")) {
        if (ImGui::MenuItem("Reset", "Ctrl+R")) {
            state.stop_execution();
            state.session.reset();
        }
        if (ImGui::MenuItem(state.running ? "Pause" : "Run", "F5")) {
            state.toggle_run();
        }
        if (ImGui::MenuItem("Step", "F10")) {
            state.step_once();
        }
        if (ImGui::MenuItem("Step Over", "F11")) {
            state.step_over();
        }
        if (ImGui::MenuItem("Run Until Return", "Shift+F11")) {
            state.run_until_return();
        }
        ImGui::EndMenu();
    }

    ImGui::EndMainMenuBar();
}

void draw_status_bar(GuiState& state) {
    const ImGuiViewport* viewport = ImGui::GetMainViewport();
    const float height = ImGui::GetFrameHeight() + ImGui::GetStyle().WindowPadding.y * 2.0f;
    ImGui::SetNextWindowPos(ImVec2(viewport->WorkPos.x, viewport->WorkPos.y + viewport->WorkSize.y - height));
    ImGui::SetNextWindowSize(ImVec2(viewport->WorkSize.x, height));

    constexpr ImGuiWindowFlags flags =
        ImGuiWindowFlags_NoDecoration |
        ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoSavedSettings |
        ImGuiWindowFlags_NoNav |
        ImGuiWindowFlags_NoBringToFrontOnFocus;

    if (ImGui::Begin("Status Bar", nullptr, flags)) {
        const char* run_state = state.run_until_active ? "until" : (state.running ? "running" : "paused");
        const auto& sim = state.session.simulator();
        ImGui::Text("State: %s", run_state);
        ImGui::SameLine();
        ImGui::TextUnformatted("|");
        ImGui::SameLine();
        ImGui::Text("PC: %04X", sim.cpu().regs().pc);
        ImGui::SameLine();
        ImGui::TextUnformatted("|");
        ImGui::SameLine();
        ImGui::Text("Cycles: %llu", static_cast<unsigned long long>(sim.clock().total_cycles()));
        ImGui::SameLine();
        ImGui::TextUnformatted("|");
        ImGui::SameLine();
        ImGui::Text("Session: %s", buffer_string(state.session_path).c_str());
    }
    ImGui::End();
}

void draw_workbench(GuiState& state) {
    if (!state.pending_layout_ini.empty()) {
        ImGui::LoadIniSettingsFromMemory(state.pending_layout_ini.data(), state.pending_layout_ini.size());
        state.pending_layout_ini.clear();
        state.session.add_log("Restored session window layout.");
    }

#ifdef IMGUI_HAS_DOCK
    ImGui::DockSpaceOverViewport(0, ImGui::GetMainViewport());
#endif

    draw_main_menu(state);

    draw_file_panel(state);
    draw_control_panel(state);
    draw_registers(state);
    draw_disassembly(state);
    draw_memory_viewer(state);
    draw_stack(state);
    draw_memory_map(state);
    draw_mapper(state);
    draw_compact_flash(state);
    draw_breakpoints(state);
    draw_watchpoints(state);
    draw_trace(state);
    draw_serial(state);
    draw_log(state);
    draw_status_bar(state);
}

void handle_shortcut(GuiState& state, SDL_Keycode key, SDL_Keymod mods) {
    const bool ctrl = (mods & KMOD_CTRL) != 0;
    const bool shift = (mods & KMOD_SHIFT) != 0;

    if (ctrl && key == SDLK_o) {
        state.load_session_from_field();
    } else if (ctrl && shift && key == SDLK_s) {
        state.save_session_as();
    } else if (ctrl && key == SDLK_s) {
        state.save_session_from_field();
    } else if (ctrl && key == SDLK_q) {
        state.quit_requested = true;
    } else if (ctrl && key == SDLK_r) {
        state.stop_execution();
        state.session.reset();
    } else if (key == SDLK_F5) {
        state.toggle_run();
    } else if (key == SDLK_F10) {
        state.step_once();
    } else if (shift && key == SDLK_F11) {
        state.run_until_return();
    } else if (key == SDLK_F11) {
        state.step_over();
    }
}

int run_gui() {
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) != 0) {
        SDL_Log("SDL_Init failed: %s", SDL_GetError());
        return 1;
    }

    SDL_Window* window = SDL_CreateWindow(
        "microlind-sim GUI",
        SDL_WINDOWPOS_CENTERED,
        SDL_WINDOWPOS_CENTERED,
        1440,
        900,
        SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI);
    if (!window) {
        SDL_Log("SDL_CreateWindow failed: %s", SDL_GetError());
        SDL_Quit();
        return 1;
    }

    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_PRESENTVSYNC | SDL_RENDERER_ACCELERATED);
    if (!renderer) {
        SDL_Log("SDL_CreateRenderer failed: %s", SDL_GetError());
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
#ifdef IMGUI_HAS_DOCK
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
#endif
    (void)io;
    ImGui::StyleColorsDark();

    ImGui_ImplSDL2_InitForSDLRenderer(window, renderer);
    ImGui_ImplSDLRenderer2_Init(renderer);

    GuiState state;
    bool done = false;

    while (!done) {
        SDL_Event event;
        while (SDL_PollEvent(&event) != 0) {
            ImGui_ImplSDL2_ProcessEvent(&event);
            if (event.type == SDL_QUIT) {
                done = true;
            }
            if (event.type == SDL_WINDOWEVENT && event.window.event == SDL_WINDOWEVENT_CLOSE &&
                event.window.windowID == SDL_GetWindowID(window)) {
                done = true;
            }
            if (event.type == SDL_KEYDOWN && event.key.repeat == 0 && !io.WantCaptureKeyboard) {
                handle_shortcut(state, event.key.keysym.sym, static_cast<SDL_Keymod>(event.key.keysym.mod));
            }
        }

        if (state.run_until_active) {
            const auto result = state.session.run_until_address(
                static_cast<uint16_t>(state.run_until_address),
                static_cast<uint32_t>(state.steps_per_frame));
            if (result.hit_target || result.hit_breakpoint || result.hit_watchpoint) {
                state.run_until_active = false;
            }
        } else if (state.running) {
            const auto result = state.session.run_instructions(static_cast<uint32_t>(state.steps_per_frame));
            if (result.hit_breakpoint || result.hit_watchpoint) {
                state.running = false;
            }
        }

        ImGui_ImplSDLRenderer2_NewFrame();
        ImGui_ImplSDL2_NewFrame();
        ImGui::NewFrame();

        draw_workbench(state);
        if (state.quit_requested) {
            done = true;
        }

        ImGui::Render();
        SDL_SetRenderDrawColor(renderer, 20, 22, 24, 255);
        SDL_RenderClear(renderer);
        ImGui_ImplSDLRenderer2_RenderDrawData(ImGui::GetDrawData(), renderer);
        SDL_RenderPresent(renderer);
    }

    ImGui_ImplSDLRenderer2_Shutdown();
    ImGui_ImplSDL2_Shutdown();
    ImGui::DestroyContext();

    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}

} // namespace

int main() {
    try {
        return run_gui();
    } catch (const std::exception& ex) {
        SDL_Log("Unhandled exception: %s", ex.what());
        return 1;
    }
}
