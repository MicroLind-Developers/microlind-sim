#pragma once

#include <cstdint>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

namespace microlind::logic {

enum class LogicDialect {
    Auto,
    WinCUPL,
    GALasm,
    PALasm,
};

enum class NodeKind {
    Signal,
    Not,
    And,
    Or,
};

struct ExprNode {
    NodeKind kind{NodeKind::Signal};
    std::string signal;
    std::vector<ExprNode> children;
};

struct Pin {
    int number{};
    std::string signal;
    bool active_low{};
};

struct Equation {
    std::string output;
    bool output_active_low{};
    std::string expression_text;
    ExprNode expression;
};

struct LogicDeviceDescription {
    std::string name;
    std::string device;
    LogicDialect dialect{LogicDialect::Auto};
    std::vector<Pin> inputs;
    std::vector<Pin> outputs;
    std::vector<Equation> equations;
};

struct ParseResult {
    LogicDeviceDescription device;
    std::vector<std::string> errors;

    [[nodiscard]] bool ok() const { return errors.empty(); }
};

struct EvalContext {
    std::unordered_map<std::string, bool> signals;
};

struct EvalResult {
    std::unordered_map<std::string, bool> outputs;
    std::vector<std::string> errors;

    [[nodiscard]] bool ok() const { return errors.empty(); }
};

struct BoardLogicDevices {
    LogicDeviceDescription signal_logic;
    LogicDeviceDescription memory_logic;
    LogicDeviceDescription address_logic;
};

struct BoardSignals {
    uint16_t address{};
    uint8_t mapper_bits{};
    bool rw{true};
    bool e{true};
    bool q{false};
    bool ba{true};
    bool bs{true};
    bool breq{true};
    bool memory_enable{true};
    bool mapper_enable{true};
};

struct BoardDecodeResult {
    std::vector<std::string> errors;

    bool rw1{};
    bool mem_wr{};
    bool mem_rd{};
    bool bavail{};
    bool rd{};
    bool wr{};

    bool rom_en{};
    bool raml_en{};
    bool ramh_en{};
    bool ramx_en{};
    bool io_en{};
    bool map_rd{};
    bool bank_sel0{};
    bool bank_sel1{};
    uint8_t bank_select{};

    bool mapper_register_en{};
    bool irq_en{};
    bool ps2_en{};
    bool cf_en{};
    bool par_en{};
    bool ser_en{};
    bool vdc_en{};
    bool snd_en{};
    bool exp_en{};

    [[nodiscard]] bool ok() const { return errors.empty(); }
};

ParseResult parse_pld(std::string_view source, LogicDialect dialect = LogicDialect::Auto);
LogicDeviceDescription parse_simple_logic(std::string_view source);
EvalResult evaluate(const LogicDeviceDescription& device, const EvalContext& context);
BoardDecodeResult decode_board_logic(const BoardLogicDevices& devices, const BoardSignals& signals);

} // namespace microlind::logic
