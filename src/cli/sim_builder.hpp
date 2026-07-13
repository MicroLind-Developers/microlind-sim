#pragma once

#include "hardware_config.hpp"
#include "image_loader.hpp"

#include "microlind/cpu.hpp"
#include "microlind/simulator.hpp"

namespace microlind::devices {
class XR88C92;
}

namespace microlind::cli {

Simulator build_sim(
    CpuMode mode,
    const LoadedImage* image,
    const HardwareConfig* cfg,
    microlind::devices::XR88C92** serial_out = nullptr);

} // namespace microlind::cli
