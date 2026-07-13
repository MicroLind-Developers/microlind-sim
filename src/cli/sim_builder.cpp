#include "microlind/app/sim_builder.hpp"

#include <algorithm>
#include <cctype>
#include <iomanip>
#include <iostream>
#include <memory>
#include <utility>
#include <vector>

#include "microlind/devices/compact_flash.hpp"
#include "microlind/devices/memory.hpp"
#include "microlind/devices/memory_mapper.hpp"
#include "microlind/devices/serial.hpp"

namespace microlind::cli {
namespace {

struct AddressRange {
    uint16_t start{};
    uint16_t end{};
};

bool overlaps(AddressRange a, AddressRange b) {
    return a.start <= b.end && b.start <= a.end;
}

std::vector<AddressRange> subtract_ranges(AddressRange source, const std::vector<AddressRange>& cuts) {
    std::vector<AddressRange> ranges{source};
    for (const auto& cut : cuts) {
        std::vector<AddressRange> next;
        for (const auto& range : ranges) {
            if (!overlaps(range, cut)) {
                next.push_back(range);
                continue;
            }
            if (cut.start > range.start) {
                next.push_back({range.start, static_cast<uint16_t>(cut.start - 1)});
            }
            if (cut.end < range.end) {
                next.push_back({static_cast<uint16_t>(cut.end + 1), range.end});
            }
        }
        ranges = std::move(next);
    }
    return ranges;
}

std::vector<std::pair<std::size_t, AddressRange>> explicit_mapper_windows(const HardwareConfig& cfg) {
    std::vector<std::pair<std::size_t, AddressRange>> windows;
    for (std::size_t i = 0; i < cfg.mapper.windows.size(); ++i) {
        const auto& window = cfg.mapper.windows[i];
        if (!window.present || window.start > window.end) continue;
        windows.push_back({i, AddressRange{window.start, window.end}});
    }
    return windows;
}

bool image_byte_at(const LoadedImage* image, uint32_t address, uint8_t& out) {
    if (!image) return false;
    if (address < image->base) return false;
    const uint32_t offset = address - image->base;
    if (offset >= image->data.size()) return false;
    out = image->data[offset];
    return true;
}

} // namespace

Simulator build_sim(
    CpuMode mode,
    const LoadedImage* image,
    const HardwareConfig* cfg,
    microlind::devices::XR88C92** serial_out,
    std::function<void(uint8_t)> serial_tx,
    std::shared_ptr<microlind::devices::MapperState>* mapper_state_out,
    microlind::devices::CompactFlash** cf_out) {
    Simulator sim(mode, 1000000);
    using microlind::devices::BankedMemory;
    using microlind::devices::CompactFlash;
    using microlind::devices::MapperState;
    using microlind::devices::Memory;
    using microlind::devices::MemoryMapper;
    using microlind::devices::XR88C92;

    std::shared_ptr<MapperState> mapper_state;
    BankedMemory::BackingStore banked_ram_store;
    std::vector<AddressRange> ram_overlays;
    XR88C92* serial_dev_raw = nullptr;
    CompactFlash* cf_dev_raw = nullptr;

    if (cfg && cfg->ram.present) {
        const size_t ram_size = static_cast<size_t>(cfg->ram.end - cfg->ram.start + 1);
        if (cfg->mapper.present && cfg->ram.bank_size > 0 && cfg->ram.available > 0) {
            mapper_state = std::make_shared<MapperState>();
            const auto windows = explicit_mapper_windows(*cfg);
            const std::size_t configured_window_count = windows.empty()
                ? (ram_size + cfg->ram.bank_size - 1) / cfg->ram.bank_size
                : 4;
            const std::size_t window_count = std::max<std::size_t>(configured_window_count, 4);
            banked_ram_store = std::make_shared<std::vector<uint8_t>>(cfg->ram.available, 0x00);

            if (image && banked_ram_store) {
                if (windows.empty()) {
                    for (std::size_t i = 0; i < ram_size; ++i) {
                        uint8_t value = 0;
                        const uint32_t abs = static_cast<uint32_t>(cfg->ram.start) + static_cast<uint32_t>(i);
                        if (!image_byte_at(image, abs, value)) continue;
                        const std::size_t window = i / cfg->ram.bank_size;
                        const std::size_t phys = window * cfg->ram.bank_size + (i % cfg->ram.bank_size);
                        if (phys < banked_ram_store->size()) {
                            (*banked_ram_store)[phys] = value;
                        }
                    }
                } else {
                    for (const auto& [window_index, range] : windows) {
                        for (uint32_t abs = range.start; abs <= range.end; ++abs) {
                            uint8_t value = 0;
                            if (!image_byte_at(image, abs, value)) continue;
                            const std::size_t phys =
                                window_index * cfg->ram.bank_size + (static_cast<std::size_t>(abs - range.start) % cfg->ram.bank_size);
                            if (phys < banked_ram_store->size()) {
                                (*banked_ram_store)[phys] = value;
                            }
                            if (abs == 0xFFFF) break;
                        }
                    }
                }
            }

            if (windows.empty()) {
                auto ram_dev = std::make_unique<BankedMemory>(
                    mapper_state,
                    cfg->ram.bank_size,
                    cfg->ram.available,
                    window_count,
                    0,
                    banked_ram_store);
                sim.map_device(cfg->ram.start, cfg->ram.end, std::move(ram_dev));
            } else {
                for (const auto& [window_index, range] : windows) {
                    auto window_ram = std::make_unique<BankedMemory>(
                        mapper_state,
                        cfg->ram.bank_size,
                        cfg->ram.available,
                        window_count,
                        window_index,
                        banked_ram_store);
                    if (!sim.map_device(range.start, range.end, std::move(window_ram))) {
                        ram_overlays.push_back(range);
                    }
                }
            }

            if (windows.empty() && cfg->mapper.bank_reg[3] != 0 && cfg->ram.bank_size == 0x4000) {
                constexpr AddressRange stack_window{0xC000, 0xDFFF};
                if (stack_window.start < cfg->ram.start || stack_window.end > cfg->ram.end) {
                    auto stack_ram = std::make_unique<BankedMemory>(
                        mapper_state,
                        cfg->ram.bank_size,
                        cfg->ram.available,
                        window_count,
                        3,
                        banked_ram_store);
                    if (!sim.map_device(stack_window.start, stack_window.end, std::move(stack_ram))) {
                        ram_overlays.push_back(stack_window);
                    }
                }
            }
        } else {
            auto ram_dev = std::make_unique<Memory>(ram_size, true);
            if (image) {
                std::vector<uint8_t> slice(ram_size, 0x00);
                for (std::size_t i = 0; i < ram_size; ++i) {
                    uint8_t value = 0;
                    const uint32_t abs = static_cast<uint32_t>(cfg->ram.start) + static_cast<uint32_t>(i);
                    if (image_byte_at(image, abs, value)) {
                        slice[i] = value;
                    }
                }
                ram_dev->load(0, slice);
            }
            sim.map_device(cfg->ram.start, cfg->ram.end, std::move(ram_dev));
        }
    }

    if (cfg && !cfg->roms.empty()) {
        for (const auto& r : cfg->roms) {
            for (const auto& segment : subtract_ranges({r.start, r.end}, ram_overlays)) {
                const size_t region_size = static_cast<size_t>(segment.end - segment.start + 1);
                auto rom_dev = std::make_unique<Memory>(region_size, false);
                std::vector<uint8_t> slice(region_size, 0xFF);
                if (image) {
                    for (size_t i = 0; i < region_size; ++i) {
                        const uint32_t abs = static_cast<uint32_t>(segment.start) + static_cast<uint32_t>(i);
                        uint8_t value = 0;
                        if (image_byte_at(image, abs, value)) {
                            slice[i] = value;
                        }
                    }
                }
                rom_dev->load(0, slice);
                sim.map_device(segment.start, segment.end, std::move(rom_dev));
            }
        }
    } else if (image && !image->data.empty()) {
        auto rom_dev = std::make_unique<Memory>(image->data.size(), false);
        rom_dev->load(0, image->data);
        sim.map_device(image->base, static_cast<uint16_t>(image->base + image->data.size() - 1), std::move(rom_dev));
    } else {
        default_memory_map(sim, 64 * 1024, 0x8000, nullptr);
    }

    if (cfg && cfg->serial.present) {
        auto on_tx = std::move(serial_tx);
        if (!on_tx) {
            on_tx = [](uint8_t ch) {
                if (std::isprint(static_cast<unsigned char>(ch))) {
                    std::cout << "\n[SERIAL TX] '" << static_cast<char>(ch) << "' (0x"
                              << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(ch) << ")\n";
                } else {
                    std::cout << "\n[SERIAL TX] 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(ch) << "\n";
                }
                std::cout << "> " << std::flush;
            };
        }
        auto serial_up = std::make_unique<XR88C92>(on_tx);
        serial_dev_raw = serial_up.get();
        sim.map_device(cfg->serial.start, cfg->serial.end, std::move(serial_up));
    }

    if (cfg && cfg->cf.present) {
        CompactFlash::Options options;
        options.image_path = cfg->cf.image_path;
        if (cfg->cf.sectors > 0) {
            options.sectors = cfg->cf.sectors;
        } else if (!cfg->cf.image_path.empty()) {
            options.sectors = 0;
        }
        options.read_only = cfg->cf.read_only;
        auto cf_up = std::make_unique<CompactFlash>(std::move(options));
        cf_dev_raw = cf_up.get();
        sim.map_device(cfg->cf.start, cfg->cf.end, std::move(cf_up));
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
    if (mapper_state_out) {
        *mapper_state_out = mapper_state;
    }
    if (cf_out) {
        *cf_out = cf_dev_raw;
    }
    return sim;
}

} // namespace microlind::cli
