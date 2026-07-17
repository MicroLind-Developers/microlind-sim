#include "microlind/bus.hpp"

#include <algorithm>
#include <cstdio>
#include <utility>

namespace microlind {

namespace {
bool ranges_overlap(uint16_t a_start, uint16_t a_end, uint16_t b_start, uint16_t b_end) {
    return a_start <= b_end && b_start <= a_end;
}

const char* select_label(BusDeviceSelect select) {
    switch (select) {
    case BusDeviceSelect::None: return "none";
    case BusDeviceSelect::Ram: return "RAM";
    case BusDeviceSelect::Rom: return "ROM";
    case BusDeviceSelect::MemoryMapper: return "memory mapper";
    case BusDeviceSelect::InterruptController: return "interrupt controller";
    case BusDeviceSelect::CompactFlash: return "CompactFlash";
    case BusDeviceSelect::Serial: return "serial";
    case BusDeviceSelect::Ps2: return "PS/2";
    case BusDeviceSelect::Parallel: return "parallel";
    case BusDeviceSelect::Video: return "video";
    case BusDeviceSelect::Sound: return "sound";
    case BusDeviceSelect::Expansion: return "expansion";
    case BusDeviceSelect::Unknown: return "unknown";
    }
    return "unknown";
}

void set_phase(BusSignals& signals, BusPhase phase) {
    signals.phase = phase;
    switch (phase) {
    case BusPhase::QHighELow:
        signals.q = true;
        signals.e = false;
        break;
    case BusPhase::QHighEHigh:
        signals.q = true;
        signals.e = true;
        break;
    case BusPhase::QLowEHigh:
        signals.q = false;
        signals.e = true;
        break;
    case BusPhase::QLowELow:
        signals.q = false;
        signals.e = false;
        break;
    }
}

BusSignals signals_from_access(const BusAccess& access) {
    BusSignals signals;
    signals.rw = access.type == BusAccessType::Read;
    signals.memory_enable = true;
    signals.address = access.address;
    signals.data = access.value;
    signals.decoded_select = access.decoded_select;
    signals.mapped_select = access.mapped_select;
    signals.cycle_kind = access.cycle_kind;
    signals.apply_read = access.apply_read;
    signals.apply_write = access.apply_write;
    return signals;
}

BusSignals idle_signals(uint16_t address, BusCycleKind cycle_kind) {
    BusSignals signals;
    signals.memory_enable = false;
    signals.address = address;
    signals.data = 0xFF;
    signals.cycle_kind = cycle_kind;
    return signals;
}

}

std::optional<BusError> Bus::map_device(uint16_t start, uint16_t end, std::unique_ptr<BusDevice> device) {
    return map_device(start, end, BusDeviceSelect::Unknown, std::move(device));
}

std::optional<BusError> Bus::map_device(
    uint16_t start,
    uint16_t end,
    BusDeviceSelect select,
    std::unique_ptr<BusDevice> device) {
    if (start > end) {
        return BusError{BusErrorType::EmptyRange, start, end};
    }

    const bool overlaps = std::any_of(devices_.begin(), devices_.end(), [&](const MappedDevice& m) {
        return ranges_overlap(start, end, m.start, m.end);
    });

    if (overlaps) {
        return BusError{BusErrorType::Overlap, start, end};
    }

    devices_.push_back(MappedDevice{start, end, select, std::move(device)});
    std::sort(devices_.begin(), devices_.end(), [](const MappedDevice& a, const MappedDevice& b) {
        if (a.start == b.start) {
            return a.end < b.end;
        }
        return a.start < b.start;
    });

    return std::nullopt;
}

void Bus::set_decoder(BusDecodeMode mode, std::function<BusDecodeResult(const BusSignals&)> decoder) {
    decode_mode_ = mode;
    decoder_ = std::move(decoder);
}

namespace {

BusSignals access_signals(BusAccessType type, uint16_t address, uint8_t value, BusCycleKind cycle_kind) {
    BusSignals signals;
    signals.rw = type == BusAccessType::Read;
    signals.e = true;
    signals.q = false;
    signals.memory_enable = true;
    signals.address = address;
    signals.data = value;
    signals.cycle_kind = cycle_kind;
    return signals;
}

} // namespace

Bus::MappedDevice* Bus::find_range_device(uint16_t address) {
    const auto it = std::find_if(devices_.begin(), devices_.end(), [&](const MappedDevice& m) {
        return m.contains(address);
    });
    return it == devices_.end() ? nullptr : &*it;
}

Bus::MappedDevice* Bus::find_selected_device(uint16_t address, BusDeviceSelect select) {
    const auto it = std::find_if(devices_.begin(), devices_.end(), [&](const MappedDevice& m) {
        return m.select == select && m.contains(address);
    });
    return it == devices_.end() ? nullptr : &*it;
}

Bus::MappedDevice* Bus::find_bus_cycle_device(const BusSignals& signals) {
    MappedDevice* device = nullptr;
    if (signals.mapped_select != BusDeviceSelect::None) {
        device = find_selected_device(signals.address, signals.mapped_select);
    }
    if (!device) {
        device = find_range_device(signals.address);
    }
    return device;
}

void Bus::log_decode_issue(std::string message) {
    decode_log_.push_back(std::move(message));
    if (decode_log_.size() > 256) {
        decode_log_.erase(decode_log_.begin());
    }
}

BusDecodeResult Bus::decode_signals(const BusSignals& signals, bool log_diagnostics) {
    if (decode_mode_ == BusDecodeMode::RangeMap || !decoder_ || !signals.memory_enable) {
        return {};
    }

    BusDecodeResult decoded = decoder_(signals);
    if (log_diagnostics && !decoded.diagnostic.empty()) {
        log_decode_issue(decoded.diagnostic);
    }
    return decoded;
}

Bus::ResolvedAccess Bus::resolve_access(
    BusAccessType type,
    uint16_t address,
    uint8_t value,
    BusCycleKind cycle_kind,
    bool log_diagnostics) {
    BusSignals signals = access_signals(type, address, value, cycle_kind);
    BusDecodeResult decoded = decode_signals(signals, log_diagnostics);

    MappedDevice* range_device = find_range_device(address);
    MappedDevice* selected_device = range_device;
    if (decode_mode_ == BusDecodeMode::Route && decoded.selected) {
        selected_device = find_selected_device(address, decoded.select);
        if (!selected_device) {
            char buffer[160];
            std::snprintf(
                buffer,
                sizeof(buffer),
                "PLD route selected %s at 0x%04X, but no mapped device matches.",
                select_label(decoded.select),
                address);
            if (log_diagnostics) {
                log_decode_issue(buffer);
            }
        }
    } else if (decode_mode_ == BusDecodeMode::Route && !decoded.selected) {
        selected_device = nullptr;
    }

    const BusDeviceSelect mapped_select = selected_device ? selected_device->select : BusDeviceSelect::None;
    if (log_diagnostics && decode_mode_ == BusDecodeMode::Validate && decoded.selected) {
        if (!range_device) {
            char buffer[160];
            std::snprintf(
                buffer,
                sizeof(buffer),
                "PLD validation selected %s at 0x%04X, but the range map is unmapped.",
                select_label(decoded.select),
                address);
            log_decode_issue(buffer);
        } else if (range_device->select != BusDeviceSelect::Unknown && range_device->select != decoded.select) {
            char buffer[192];
            std::snprintf(
                buffer,
                sizeof(buffer),
                "PLD validation selected %s at 0x%04X, but range map selected %s.",
                select_label(decoded.select),
                address,
                select_label(range_device->select));
            log_decode_issue(buffer);
        }
    }

    return ResolvedAccess{
        selected_device,
        decoded.selected ? decoded.select : BusDeviceSelect::None,
        mapped_select,
    };
}

void Bus::record_access(BusAccess access) {
    if (defer_bus_cycles_) {
        BusSignals signals = signals_from_access(access);
        signals.log_access = access_logging_;
        deferred_bus_cycles_.push_back(signals);
        return;
    }
    if (access_logging_) {
        access_log_.push_back(access);
    }
    tick_bus_cycle(signals_from_access(access));
}

uint8_t Bus::read8(uint16_t address, BusCycleKind cycle_kind) {
    ResolvedAccess resolved = resolve_access(BusAccessType::Read, address, 0xFF, cycle_kind, true);
    if (resolved.device) {
        const uint8_t value = defer_bus_cycles_
            ? resolved.device->device->peek8(resolved.device->offset(address))
            : resolved.device->device->read8(resolved.device->offset(address));
        record_access(BusAccess{
            BusAccessType::Read,
            address,
            value,
            resolved.decoded_select,
            resolved.mapped_select,
            cycle_kind,
            defer_bus_cycles_,
            false});
        return value;
    }
    record_access(BusAccess{BusAccessType::Read, address, 0xFF, resolved.decoded_select, resolved.mapped_select, cycle_kind});
    return 0xFF;
}

uint8_t Bus::peek8(uint16_t address) {
    ResolvedAccess resolved = resolve_access(BusAccessType::Read, address, 0xFF, BusCycleKind::Internal, false);
    if (resolved.device) {
        return resolved.device->device->peek8(resolved.device->offset(address));
    }
    return 0xFF;
}

void Bus::write8(uint16_t address, uint8_t value, BusCycleKind cycle_kind) {
    ResolvedAccess resolved = resolve_access(BusAccessType::Write, address, value, cycle_kind, true);
    BusAccess access{
        BusAccessType::Write,
        address,
        value,
        resolved.decoded_select,
        resolved.mapped_select,
        cycle_kind,
        false,
        defer_bus_cycles_ && resolved.device != nullptr};
    if (resolved.device && !defer_bus_cycles_) {
        resolved.device->device->write8(resolved.device->offset(address), value);
    }
    record_access(access);
}

void Bus::tick_devices(uint32_t cycles) {
    for (auto& m : devices_) {
        m.device->tick(cycles);
    }
}

void Bus::tick_idle_cycles(uint32_t cycles, uint16_t idle_address, BusCycleKind cycle_kind) {
    for (uint32_t i = 0; i < cycles; ++i) {
        BusSignals signals = idle_signals(idle_address, cycle_kind);
        if (defer_bus_cycles_) {
            deferred_bus_cycles_.push_back(signals);
        } else {
            tick_bus_cycle(signals);
        }
    }
}

void Bus::tick_bus_cycles(uint32_t cycles, const std::vector<BusAccess>& accesses, uint16_t idle_address) {
    for (uint32_t i = 0; i < cycles; ++i) {
        const BusSignals signals = i < accesses.size() ? signals_from_access(accesses[i]) : idle_signals(idle_address, BusCycleKind::Idle);
        if (defer_bus_cycles_) {
            deferred_bus_cycles_.push_back(signals);
        } else {
            tick_bus_cycle(signals);
        }
    }
}

void Bus::begin_deferred_bus_cycles() {
    deferred_bus_cycles_.clear();
    defer_bus_cycles_ = true;
}

std::vector<BusSignals> Bus::take_deferred_bus_cycles() {
    defer_bus_cycles_ = false;
    return std::move(deferred_bus_cycles_);
}

void Bus::tick_bus_cycle(BusSignals signals) {
    ++bus_cycle_count_;
    if (!detailed_bus_phases_) {
        set_phase(signals, BusPhase::QLowEHigh);
        const BusDecodeResult decoded = decode_signals(signals, false);
        signals.decoded_select = decoded.selected ? decoded.select : BusDeviceSelect::None;
        if (signals.mapped_select == BusDeviceSelect::None && signals.memory_enable) {
            if (MappedDevice* mapped = find_range_device(signals.address)) {
                signals.mapped_select = mapped->select;
            }
        }
        if (MappedDevice* device = find_bus_cycle_device(signals)) {
            if (signals.apply_read) {
                signals.data = device->device->read8(device->offset(signals.address));
            }
            if (signals.apply_write) {
                device->device->write8(device->offset(signals.address), signals.data);
            }
        }
        last_signals_ = signals;
        if (signals.log_access && access_logging_) {
            access_log_.push_back(BusAccess{
                signals.rw ? BusAccessType::Read : BusAccessType::Write,
                signals.address,
                signals.data,
                signals.decoded_select,
                signals.mapped_select,
                signals.cycle_kind,
                signals.apply_read,
                signals.apply_write});
        }
        return;
    }

    bool applied_deferred_access = false;
    constexpr BusPhase phases[] = {
        BusPhase::QHighELow,
        BusPhase::QHighEHigh,
        BusPhase::QLowEHigh,
        BusPhase::QLowELow,
    };

    for (const BusPhase phase : phases) {
        set_phase(signals, phase);
        const BusDecodeResult decoded = decode_signals(signals, false);
        signals.decoded_select = decoded.selected ? decoded.select : BusDeviceSelect::None;
        if (signals.mapped_select == BusDeviceSelect::None && signals.memory_enable) {
            if (MappedDevice* mapped = find_range_device(signals.address)) {
                signals.mapped_select = mapped->select;
            }
        }
        if (!applied_deferred_access && signals.phase == BusPhase::QLowEHigh) {
            MappedDevice* device = find_bus_cycle_device(signals);
            if (device && signals.apply_read) {
                signals.data = device->device->read8(device->offset(signals.address));
            }
            if (device && signals.apply_write) {
                device->device->write8(device->offset(signals.address), signals.data);
            }
            applied_deferred_access = true;
        }
        last_signals_ = signals;
        phase_log_.push_back(signals);
        if (phase_log_.size() > 1024) {
            phase_log_.erase(phase_log_.begin());
        }
        for (auto& m : devices_) {
            m.device->tick_phase(signals);
        }
    }
    if (signals.log_access && access_logging_) {
        access_log_.push_back(BusAccess{
            signals.rw ? BusAccessType::Read : BusAccessType::Write,
            signals.address,
            signals.data,
            signals.decoded_select,
            signals.mapped_select,
            signals.cycle_kind,
            signals.apply_read,
            signals.apply_write});
    }
}

std::vector<std::string> Bus::map_summary() const {
    std::vector<std::string> out;
    out.reserve(devices_.size());
    for (const auto& m : devices_) {
        char buffer[64];
        std::snprintf(buffer, sizeof(buffer), "0x%04X-0x%04X %s", m.start, m.end, select_label(m.select));
        out.emplace_back(buffer);
    }
    return out;
}

} // namespace microlind
