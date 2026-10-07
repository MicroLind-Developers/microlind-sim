#include "microlind/app/config_editor.hpp"

#include <algorithm>
#include <chrono>
#include <cctype>
#include <fstream>
#include <iomanip>
#include <map>
#include <set>
#include <sstream>

#include "microlind/app/util.hpp"

namespace microlind::app {
namespace {

std::filesystem::path absolute_path(const std::filesystem::path& path) {
    return std::filesystem::absolute(path).lexically_normal();
}

std::filesystem::path resolve(const std::filesystem::path& value, const std::filesystem::path& directory) {
    return value.empty() ? value : absolute_path(value.is_absolute() ? value : directory / value);
}

void relative_paths(cli::HardwareConfig& config, const std::filesystem::path& directory) {
    for (auto* value : {&config.cf.image_path, &config.logic.signal_logic_path,
                       &config.logic.memory_logic_path, &config.logic.address_logic_path}) {
        if (!value->empty()) {
            const auto relative = absolute_path(*value).lexically_relative(directory);
            if (!relative.empty()) *value = relative;
        }
    }
}

std::string hex(uint16_t value) {
    std::ostringstream out;
    out << "0x" << std::hex << std::uppercase << std::setw(4) << std::setfill('0') << value;
    return out.str();
}

std::string section_name(std::string name) {
    for (auto& ch : name) ch = static_cast<char>(std::toupper(static_cast<unsigned char>(ch)));
    if (name == "VDC") return "VIDEO";
    if (name == "PAR") return "PARALLEL";
    if (name == "COMPACT_FLASH") return "CF";
    if (name == "LOGIC") return "PLD_LOGIC";
    return name;
}

const std::map<std::string, std::set<std::string>> known_keys{
    {"ROM", {"START", "END"}},
    {"RAM", {"START", "END", "BANK_SIZE", "AVAILABLE"}},
    {"SERIAL", {"IO_START_ADDRESS", "IO_END_ADDRESS", "IRQ_LEVEL"}},
    {"PARALLEL", {"IO_START_ADDRESS", "IO_END_ADDRESS", "IRQ_LEVEL"}},
    {"VIDEO", {"IO_START_ADDRESS", "IO_END_ADDRESS", "VRAM_SIZE"}},
    {"CF", {"IO_START_ADDRESS", "IO_END_ADDRESS", "SECTORS", "IMAGE", "IMAGE_PATH", "READ_ONLY"}},
    {"MEMORY_MAPPER", {"BANK_0_REGISTER", "BANK_1_REGISTER", "BANK_2_REGISTER", "BANK_3_REGISTER",
        "WINDOW_0", "WINDOW_1", "WINDOW_2", "WINDOW_3", "WINDOW_0_START", "WINDOW_0_END",
        "WINDOW_1_START", "WINDOW_1_END", "WINDOW_2_START", "WINDOW_2_END", "WINDOW_3_START", "WINDOW_3_END"}},
    {"PLD_LOGIC", {"SIGNAL_LOGIC", "SIGNAL_LOGIC_PATH", "MEMORY_LOGIC", "MEMORY_LOGIC_PATH",
        "ADDRESS_LOGIC", "ADDRESS_LOGIC_PATH", "BUS_MODE", "ROUTING"}}
};

} // namespace

std::vector<ConfigAddressRange> config_address_map(const cli::HardwareConfig& config) {
    std::vector<ConfigAddressRange> result;
    std::vector<cli::MapperWindowConfig> windows;
    const bool banked = config.ram.present && config.mapper.present && config.ram.bank_size && config.ram.available;
    if (banked) {
        for (const auto& window : config.mapper.windows) if (window.present) windows.push_back(window);
    }
    if (config.ram.present) {
        if (windows.empty()) {
            result.push_back({"RAM", "RAM", config.ram.start, config.ram.end});
            // The builder supplies a fourth stack window for the legacy 16 KiB layout.
            if (banked && config.mapper.bank_reg[3] && config.ram.bank_size == 0x4000 &&
                (config.ram.start > 0xC000 || config.ram.end < 0xDFFF)) {
                windows.push_back({0xC000, 0xDFFF, true});
                result.push_back({"RAM", "RAM stack window", 0xC000, 0xDFFF});
            }
        } else {
            for (const auto& window : windows) result.push_back({"RAM", "RAM window", window.start, window.end});
        }
    }
    for (std::size_t i = 0; i < config.roms.size(); ++i) {
        // Explicit banked windows overlay ROM in the simulator builder.
        std::vector<std::pair<uint32_t, uint32_t>> segments{{config.roms[i].start, config.roms[i].end}};
        for (const auto& window : windows) {
            std::vector<std::pair<uint32_t, uint32_t>> next;
            for (auto [start, end] : segments) {
                if (start > window.end || window.start > end) next.emplace_back(start, end);
                else {
                    if (start < window.start) next.emplace_back(start, window.start - 1);
                    if (end > window.end) next.emplace_back(window.end + 1, end);
                }
            }
            segments = std::move(next);
        }
        for (auto [start, end] : segments) result.push_back({"ROM", "ROM " + std::to_string(i + 1),
            static_cast<uint16_t>(start), static_cast<uint16_t>(end)});
    }
    if (config.serial.present) result.push_back({"SERIAL", "Serial", config.serial.start, config.serial.end});
    if (config.parallel.present) result.push_back({"PARALLEL", "Parallel I/O", config.parallel.start, config.parallel.end});
    if (config.video.present) result.push_back({"VIDEO", "VDC", config.video.start, config.video.end});
    if (config.cf.present) result.push_back({"CF", "CompactFlash", config.cf.start, config.cf.end});
    if (banked) {
        uint16_t first = 0xFFFF, last = 0;
        for (auto address : config.mapper.bank_reg) if (address) {
            first = std::min(first, address);
            last = std::max(last, address);
        }
        if (last) result.push_back({"MEMORY_MAPPER", "Mapper registers", first, last});
    }
    result.push_back({"IRQ", "IRQ controller (fixed)", 0xF404, 0xF404});
    std::sort(result.begin(), result.end(), [](const auto& a, const auto& b) { return a.start < b.start; });
    return result;
}

std::vector<ConfigIssue> validate_config(const cli::HardwareConfig& config) {
    std::vector<ConfigIssue> issues;
    const auto add = [&](const std::string& section, std::string message, bool warning = false) {
        issues.push_back({section, std::move(message), warning});
    };
    const auto range = [&](const char* section, uint16_t start, uint16_t end, int size = 0) {
        if (start > end) add(section, "Start address must not exceed end address.");
        else if (size && static_cast<int>(end) - start + 1 != size)
            add(section, "The register window must contain " + std::to_string(size) + " bytes.");
    };
    for (const auto& rom : config.roms) range("ROM", rom.start, rom.end);
    if (config.ram.present) range("RAM", config.ram.start, config.ram.end);
    if (config.serial.present) {
        range("SERIAL", config.serial.start, config.serial.end, 16);
        if (config.serial.irq_level > 15) add("SERIAL", "IRQ level must be 0-15 (0 disables interrupts).");
    }
    if (config.parallel.present) {
        range("PARALLEL", config.parallel.start, config.parallel.end, 16);
        if (config.parallel.irq_level > 15) add("PARALLEL", "IRQ level must be 0-15 (0 disables interrupts).");
    }
    if (config.video.present) {
        range("VIDEO", config.video.start, config.video.end, 2);
        if (config.video.vram_size != 65536) add("VIDEO", "The VDC currently requires 65536 bytes of VRAM.");
    }
    if (config.cf.present) range("CF", config.cf.start, config.cf.end, 8);
    if (config.mapper.present) {
        if (!config.ram.present || !config.ram.bank_size || !config.ram.available)
            add("MEMORY_MAPPER", "Enable RAM and set bank size and backing size to use the mapper.");
        else {
            const auto banks = config.ram.available / config.ram.bank_size;
            if (!banks || config.ram.available % config.ram.bank_size || (banks & (banks - 1)))
                add("RAM", "Backing size must contain a power-of-two number of complete banks.");
            if (config.ram.bank_size > 65536) add("RAM", "Bank size must not exceed the 64 KiB CPU address space.");
        }
        std::set<uint16_t> addresses;
        for (auto address : config.mapper.bank_reg) if (address && !addresses.insert(address).second)
            add("MEMORY_MAPPER", "Enabled bank registers must have distinct addresses.");
        if (addresses.empty()) add("MEMORY_MAPPER", "Enable at least one bank register.");
        for (const auto& window : config.mapper.windows) if (window.present) {
            range("MEMORY_MAPPER", window.start, window.end);
            if (window.start <= window.end && static_cast<uint32_t>(window.end) - window.start + 1 > config.ram.bank_size)
                add("MEMORY_MAPPER", "A mapper window must not exceed the bank size.");
        }
    }
    if (config.roms.empty()) add("ROM", "No ROM region: provide executable code in RAM before running.", true);
    const auto mapped = config_address_map(config);
    for (std::size_t i = 0; i < mapped.size(); ++i) {
        for (std::size_t j = i + 1; j < mapped.size(); ++j) {
            const auto& a = mapped[i];
            const auto& b = mapped[j];
            if (a.start <= a.end && b.start <= b.end && a.start <= b.end && b.start <= a.end)
                add(a.section, a.label + " overlaps " + b.label + " at " + hex(std::max(a.start, b.start)) + ".");
        }
    }
    for (const auto& [section, path] : std::vector<std::pair<std::string, std::filesystem::path>>{
            {"CF", config.cf.present ? config.cf.image_path : std::filesystem::path{}},
            {"PLD_LOGIC", config.logic.present ? config.logic.signal_logic_path : std::filesystem::path{}},
            {"PLD_LOGIC", config.logic.present ? config.logic.memory_logic_path : std::filesystem::path{}},
            {"PLD_LOGIC", config.logic.present ? config.logic.address_logic_path : std::filesystem::path{}}}) {
        if (path.string().find_first_of("\r\n") != std::string::npos) add(section, "File paths must not contain line breaks.");
    }
    if (config.logic.present && (config.logic.signal_logic_path.empty() || config.logic.memory_logic_path.empty() ||
                                config.logic.address_logic_path.empty()))
        add("PLD_LOGIC", "Select all three PLD source files.");
    return issues;
}

std::string serialize_config(const cli::HardwareConfig& config, const std::filesystem::path& destination,
                             const std::string& original_source) {
    std::map<std::string, std::string> extras;
    std::string prefix, unknown, section, identity;
    int rom_index = 0;
    std::istringstream original(original_source);
    for (std::string line; std::getline(original, line);) {
        const auto trimmed = cli::trim(line);
        if (trimmed.size() >= 2 && trimmed.front() == '[' && trimmed.back() == ']') {
            section = section_name(cli::trim(trimmed.substr(1, trimmed.size() - 2)));
            identity = section == "ROM" ? "ROM" + std::to_string(rom_index++) : section;
            if (!known_keys.contains(section)) unknown += line + '\n';
        } else if (section.empty()) prefix += line + '\n';
        else if (!known_keys.contains(section)) unknown += line + '\n';
        else {
            const auto equals = trimmed.find('=');
            const auto key = section_name(cli::trim(trimmed.substr(0, equals)));
            if (trimmed.empty() || trimmed.front() == '#' || trimmed.front() == ';' ||
                equals == std::string::npos || !known_keys.at(section).contains(key)) extras[identity] += line + '\n';
            else if (key != "IMAGE" && key != "IMAGE_PATH" && key.find("LOGIC") == std::string::npos) {
                const auto comment = line.find_first_of("#;", line.find('=') + 1);
                if (comment != std::string::npos) extras[identity] += line.substr(comment) + '\n';
            }
        }
    }
    std::ostringstream out;
    out << prefix;
    const auto emit = [&](const std::string& name, const std::string& content, const std::string& id = "") {
        auto extra = extras[id.empty() ? name : id];
        while (!extra.empty() && extra.back() == '\n') extra.pop_back();
        out << '[' << name << "]\n" << content;
        if (!extra.empty()) out << extra << '\n';
        out << '\n';
    };
    const auto io = [](auto device) {
        return "IO_START_ADDRESS=" + hex(device.start) + "\nIO_END_ADDRESS=" + hex(device.end) + '\n';
    };
    const auto file_path = [&](const std::filesystem::path& value) {
        if (value.empty()) return std::string{};
        const auto relative = absolute_path(value).lexically_relative(absolute_path(destination).parent_path());
        return (relative.empty() ? value : relative).generic_string();
    };
    for (std::size_t i = 0; i < config.roms.size(); ++i) emit("ROM",
        "START=" + hex(config.roms[i].start) + "\nEND=" + hex(config.roms[i].end) + '\n', "ROM" + std::to_string(i));
    if (config.ram.present) emit("RAM", "START=" + hex(config.ram.start) + "\nEND=" + hex(config.ram.end) +
        "\nBANK_SIZE=" + std::to_string(config.ram.bank_size) + "\nAVAILABLE=" + std::to_string(config.ram.available) + '\n');
    if (config.serial.present) emit("SERIAL", io(config.serial) + "IRQ_LEVEL=" + std::to_string(config.serial.irq_level) + '\n');
    if (config.parallel.present) emit("PARALLEL", io(config.parallel) + "IRQ_LEVEL=" + std::to_string(config.parallel.irq_level) + '\n');
    if (config.video.present) emit("VIDEO", io(config.video) + "VRAM_SIZE=" + std::to_string(config.video.vram_size) + '\n');
    if (config.cf.present) emit("CF", io(config.cf) + "SECTORS=" + std::to_string(config.cf.sectors) +
        "\nIMAGE=" + file_path(config.cf.image_path) + "\nREAD_ONLY=" + (config.cf.read_only ? "true\n" : "false\n"));
    if (config.mapper.present) {
        std::string content;
        for (int i = 0; i < 4; ++i) {
            content += "BANK_" + std::to_string(i) + "_REGISTER=" + hex(config.mapper.bank_reg[i]) + '\n';
            const auto& window = config.mapper.windows[i];
            if (window.present) content += "WINDOW_" + std::to_string(i) + '=' + hex(window.start) + '-' + hex(window.end) + '\n';
        }
        emit("MEMORY_MAPPER", content);
    }
    if (config.logic.present) emit("PLD_LOGIC", "SIGNAL_LOGIC=" + file_path(config.logic.signal_logic_path) +
        "\nMEMORY_LOGIC=" + file_path(config.logic.memory_logic_path) + "\nADDRESS_LOGIC=" + file_path(config.logic.address_logic_path) +
        "\nBUS_MODE=" + (config.logic.bus_mode == BusDecodeMode::Route ? "route\n" :
                          config.logic.bus_mode == BusDecodeMode::Validate ? "validate\n" : "range\n"));
    out << unknown;
    return out.str();
}

ConfigEditor::ConfigEditor() { new_config(); }

void ConfigEditor::new_config() {
    config = {};
    config.roms = {{0xE000, 0xF3FF}, {0xF800, 0xFFFF}};
    config.ram = {0x0000, 0xDFFF, 16384, 524288, true};
    config.serial = {0xF430, 0xF43F, 1, true};
    config.parallel = {0xF420, 0xF42F, 2, true};
    config.video = {0xF440, 0xF441, 65536, true};
    config.cf = {0xF418, 0xF41F, {}, 512, false, true};
    config.mapper.present = true;
    for (int i = 0; i < 4; ++i) {
        config.mapper.bank_reg[i] = static_cast<uint16_t>(0xF400 + i);
        config.mapper.windows[i] = {static_cast<uint16_t>(i * 0x4000),
            static_cast<uint16_t>(i == 3 ? 0xDFFF : (i + 1) * 0x4000 - 1), true};
    }
    rom_enabled = true;
    path.clear();
    source_.clear();
    saved_config_ = config;
    saved_rom_enabled_ = rom_enabled;
    modified = true;
}

bool ConfigEditor::open(const std::filesystem::path& source, std::string& error) {
    error.clear();
    if (source.empty()) { error = "Choose a config file path."; return false; }
    const auto filename = absolute_path(source);
    auto loaded = cli::load_hardware_config(filename, error);
    if (!loaded) return false;
    std::ifstream input(filename);
    std::ostringstream content;
    content << input.rdbuf();
    if (input.bad()) { error = "Cannot read config file."; return false; }
    relative_paths(*loaded, filename.parent_path());
    config = std::move(*loaded);
    path = filename;
    source_ = content.str();
    rom_enabled = !config.roms.empty();
    saved_config_ = config;
    saved_rom_enabled_ = rom_enabled;
    modified = false;
    return true;
}

cli::HardwareConfig ConfigEditor::effective_config() const {
    auto result = config;
    if (!rom_enabled) result.roms.clear();
    const auto directory = path.empty() ? std::filesystem::current_path() : path.parent_path();
    result.cf.image_path = resolve(result.cf.image_path, directory);
    result.logic.signal_logic_path = resolve(result.logic.signal_logic_path, directory);
    result.logic.memory_logic_path = resolve(result.logic.memory_logic_path, directory);
    result.logic.address_logic_path = resolve(result.logic.address_logic_path, directory);
    return result;
}

bool ConfigEditor::save(const std::filesystem::path& destination, std::string& error) {
    error.clear();
    if (destination.empty()) { error = "Choose a config file path."; return false; }
    const auto effective = effective_config();
    for (const auto& issue : validate_config(effective)) if (!issue.warning) {
        error = issue.section + ": " + issue.message;
        return false;
    }
    const auto filename = absolute_path(destination);
    if (filename == path && !path.empty()) {
        std::ifstream current(filename);
        std::ostringstream content;
        content << current.rdbuf();
        if (!current || content.str() != source_) {
            error = "This file changed on disk. Open it again or save to another file.";
            return false;
        }
    }
    const auto content = serialize_config(effective, filename, source_);
    auto temporary = filename;
    temporary += ".tmp." + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
    std::ofstream output(temporary, std::ios::binary);
    if (!output) { error = "Cannot create config file in " + filename.parent_path().string(); return false; }
    output << content;
    output.close();
    std::error_code ec;
    if (!output) {
        error = "Could not finish writing config file.";
        std::filesystem::remove(temporary, ec);
        return false;
    }
    std::filesystem::rename(temporary, filename, ec);
    if (ec) {
        error = "Could not replace config file: " + ec.message();
        std::filesystem::remove(temporary, ec);
        return false;
    }
    // Rebase draft paths as well, including values of temporarily disabled devices.
    const auto old_directory = path.empty() ? std::filesystem::current_path() : path.parent_path();
    for (auto* value : {&config.cf.image_path, &config.logic.signal_logic_path,
                       &config.logic.memory_logic_path, &config.logic.address_logic_path}) *value = resolve(*value, old_directory);
    relative_paths(config, filename.parent_path());
    path = filename;
    source_ = content;
    saved_config_ = config;
    saved_rom_enabled_ = rom_enabled;
    modified = false;
    return true;
}

void ConfigEditor::discard() {
    config = saved_config_;
    rom_enabled = saved_rom_enabled_;
    modified = false;
}

} // namespace microlind::app
