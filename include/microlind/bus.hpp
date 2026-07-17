#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace microlind {

enum class BusPhase {
    QHighELow,
    QHighEHigh,
    QLowEHigh,
    QLowELow,
};

enum class BusDeviceSelect {
    None,
    Ram,
    Rom,
    MemoryMapper,
    InterruptController,
    CompactFlash,
    Serial,
    Ps2,
    Parallel,
    Video,
    Sound,
    Expansion,
    Unknown,
};

enum class BusDecodeMode {
    RangeMap,
    Validate,
    Route,
};

enum class BusCycleKind {
    Idle,
    OpcodeFetch,
    OperandRead,
    OperandWrite,
    StackRead,
    StackWrite,
    VectorRead,
    Internal,
};

struct BusSignals {
    BusPhase phase{BusPhase::QLowELow};
    bool e{};
    bool q{};
    bool ba{true};
    bool bs{true};
    bool breq{true};
    bool rw{true};
    bool memory_enable{};
    bool mapper_enable{true};
    uint16_t address{};
    uint8_t data{0xFF};
    BusDeviceSelect decoded_select{BusDeviceSelect::None};
    BusDeviceSelect mapped_select{BusDeviceSelect::None};
    BusCycleKind cycle_kind{BusCycleKind::Idle};
    bool apply_read{};
    bool apply_write{};
    bool log_access{};
};

struct BusDecodeResult {
    BusDeviceSelect select{BusDeviceSelect::None};
    bool selected{};
    std::string diagnostic;
};

class BusDevice {
public:
    virtual ~BusDevice() = default;
    virtual uint8_t read8(uint16_t offset) = 0;
    virtual uint8_t peek8(uint16_t offset) { return read8(offset); }
    virtual void write8(uint16_t offset, uint8_t value) = 0;
    virtual void tick(uint32_t /*cycles*/) {}
    virtual void tick_phase(const BusSignals& /*signals*/) {}
};

enum class BusErrorType {
    EmptyRange,
    Overlap,
};

struct BusError {
    BusErrorType type;
    uint16_t start;
    uint16_t end;
};

enum class BusAccessType {
    Read,
    Write,
};

struct BusAccess {
    BusAccessType type;
    uint16_t address{};
    uint8_t value{};
    BusDeviceSelect decoded_select{BusDeviceSelect::None};
    BusDeviceSelect mapped_select{BusDeviceSelect::None};
    BusCycleKind cycle_kind{BusCycleKind::OperandRead};
    bool apply_read{};
    bool apply_write{};
};

class Bus {
public:
    Bus() = default;

    std::optional<BusError> map_device(uint16_t start, uint16_t end, std::unique_ptr<BusDevice> device);
    std::optional<BusError> map_device(
        uint16_t start,
        uint16_t end,
        BusDeviceSelect select,
        std::unique_ptr<BusDevice> device);

    void set_decoder(BusDecodeMode mode, std::function<BusDecodeResult(const BusSignals&)> decoder);
    [[nodiscard]] BusDecodeMode decode_mode() const { return decode_mode_; }

    uint8_t read8(uint16_t address, BusCycleKind cycle_kind = BusCycleKind::OperandRead);
    uint8_t peek8(uint16_t address);
    void write8(uint16_t address, uint8_t value, BusCycleKind cycle_kind = BusCycleKind::OperandWrite);
    void tick_devices(uint32_t cycles);
    void tick_idle_cycles(
        uint32_t cycles,
        uint16_t idle_address,
        BusCycleKind cycle_kind = BusCycleKind::Idle);
    void tick_bus_cycles(uint32_t cycles, const std::vector<BusAccess>& accesses, uint16_t idle_address);
    void tick_bus_cycle(BusSignals signals);
    [[nodiscard]] uint64_t bus_cycle_count() const { return bus_cycle_count_; }
    void set_detailed_bus_phases(bool enabled) { detailed_bus_phases_ = enabled; }
    [[nodiscard]] bool detailed_bus_phases() const { return detailed_bus_phases_; }
    void set_access_logging(bool enabled) { access_logging_ = enabled; }
    [[nodiscard]] bool access_logging() const { return access_logging_; }
    void begin_deferred_bus_cycles();
    [[nodiscard]] std::size_t deferred_bus_cycle_count() const { return deferred_bus_cycles_.size(); }
    [[nodiscard]] bool deferring_bus_cycles() const { return defer_bus_cycles_; }
    std::vector<BusSignals> take_deferred_bus_cycles();

    std::vector<std::string> map_summary() const;
    void clear_access_log() { access_log_.clear(); }
    [[nodiscard]] const std::vector<BusAccess>& access_log() const { return access_log_; }
    void clear_decode_log() { decode_log_.clear(); }
    [[nodiscard]] const std::vector<std::string>& decode_log() const { return decode_log_; }
    void clear_phase_log() { phase_log_.clear(); }
    [[nodiscard]] const std::vector<BusSignals>& phase_log() const { return phase_log_; }
    [[nodiscard]] const BusSignals& last_signals() const { return last_signals_; }

private:
    struct MappedDevice {
        uint16_t start{};
        uint16_t end{};
        BusDeviceSelect select{BusDeviceSelect::Unknown};
        std::unique_ptr<BusDevice> device;

        [[nodiscard]] bool contains(uint16_t address) const {
            return address >= start && address <= end;
        }

        [[nodiscard]] uint16_t offset(uint16_t address) const {
            return static_cast<uint16_t>(address - start);
        }
    };

    struct ResolvedAccess {
        MappedDevice* device{};
        BusDeviceSelect decoded_select{BusDeviceSelect::None};
        BusDeviceSelect mapped_select{BusDeviceSelect::None};
    };

    MappedDevice* find_range_device(uint16_t address);
    MappedDevice* find_selected_device(uint16_t address, BusDeviceSelect select);
    MappedDevice* find_bus_cycle_device(const BusSignals& signals);
    BusDecodeResult decode_signals(const BusSignals& signals, bool log_diagnostics);
    ResolvedAccess resolve_access(
        BusAccessType type,
        uint16_t address,
        uint8_t value,
        BusCycleKind cycle_kind,
        bool log_diagnostics);
    void record_access(BusAccess access);
    void log_decode_issue(std::string message);

    std::vector<MappedDevice> devices_{};
    std::vector<BusAccess> access_log_{};
    std::vector<std::string> decode_log_{};
    std::vector<BusSignals> phase_log_{};
    std::vector<BusSignals> deferred_bus_cycles_{};
    BusSignals last_signals_{};
    uint64_t bus_cycle_count_{};
    bool defer_bus_cycles_{};
    bool detailed_bus_phases_{true};
    bool access_logging_{true};
    BusDecodeMode decode_mode_{BusDecodeMode::RangeMap};
    std::function<BusDecodeResult(const BusSignals&)> decoder_;
};

} // namespace microlind
