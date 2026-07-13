#include "sim_builder.hpp"

#include <algorithm>
#include <cctype>
#include <iomanip>
#include <iostream>
#include <memory>
#include <vector>

#include "microlind/devices/compact_flash.hpp"
#include "microlind/devices/memory.hpp"
#include "microlind/devices/memory_mapper.hpp"
#include "microlind/devices/serial.hpp"

namespace microlind::cli {

Simulator build_sim(
    CpuMode mode,
    const LoadedImage* image,
    const HardwareConfig* cfg,
    microlind::devices::XR88C92** serial_out) {
    Simulator sim(mode, 1000000);
    using microlind::devices::BankedMemory;
    using microlind::devices::CompactFlash;
    using microlind::devices::MapperState;
    using microlind::devices::Memory;
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

    if (cfg && cfg->cf.present) {
        CompactFlash::Options options;
        options.image_path = cfg->cf.image_path;
        if (cfg->cf.sectors > 0) {
            options.sectors = cfg->cf.sectors;
        } else if (!cfg->cf.image_path.empty()) {
            options.sectors = 0;
        }
        options.read_only = cfg->cf.read_only;
        sim.map_device(cfg->cf.start, cfg->cf.end, std::make_unique<CompactFlash>(std::move(options)));
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

} // namespace microlind::cli
