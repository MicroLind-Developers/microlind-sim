#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "microlind/app/hardware_config.hpp"
#include "microlind/app/image_loader.hpp"

#include "microlind/cpu.hpp"
#include "microlind/simulator.hpp"

namespace microlind::devices {
class CompactFlash;
struct MapperState;
class Vdc8568;
class W65C22;
class XR88C92;
}

namespace microlind::cli {

Simulator build_sim(
    CpuMode mode,
    const LoadedImage* image,
    const HardwareConfig* cfg,
    microlind::devices::XR88C92** serial_out = nullptr,
    std::function<void(uint8_t)> serial_tx = nullptr,
    std::shared_ptr<microlind::devices::MapperState>* mapper_state_out = nullptr,
    microlind::devices::CompactFlash** cf_out = nullptr,
    microlind::devices::W65C22** parallel_out = nullptr,
    microlind::devices::Vdc8568** vdc_out = nullptr,
    std::vector<std::string>* diagnostics_out = nullptr);

} // namespace microlind::cli
