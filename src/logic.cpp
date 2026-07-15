#include "microlind/logic.hpp"

#include <algorithm>
#include <cctype>
#include <sstream>
#include <string>
#include <utility>

namespace microlind::logic {
namespace {

enum class PinSection {
    Unknown,
    Inputs,
    Outputs,
};

struct Statement {
    std::string text;
    PinSection section{PinSection::Unknown};
};

struct Token {
    enum class Kind {
        Identifier,
        Not,
        And,
        Or,
        LParen,
        RParen,
        End,
    };

    Kind kind{Kind::End};
    std::string text;
};

std::string trim(std::string_view value) {
    const char* ws = " \t\r\n";
    const auto start = value.find_first_not_of(ws);
    if (start == std::string_view::npos) return {};
    const auto end = value.find_last_not_of(ws);
    return std::string(value.substr(start, end - start + 1));
}

std::string uppercase(std::string_view value) {
    std::string out(value);
    std::transform(out.begin(), out.end(), out.begin(), [](unsigned char ch) {
        return static_cast<char>(std::toupper(ch));
    });
    return out;
}

std::pair<std::string, bool> parse_signal_name(std::string_view text) {
    std::string signal = trim(text);
    bool active_low = false;
    if (!signal.empty() && signal.front() == '!') {
        active_low = true;
        signal = trim(std::string_view(signal).substr(1));
    }
    return {signal, active_low};
}

std::vector<Statement> split_statements(std::string_view source) {
    std::vector<Statement> statements;
    std::string current;
    PinSection section = PinSection::Unknown;

    for (std::size_t i = 0; i < source.size();) {
        if (i + 1 < source.size() && source[i] == '/' && source[i + 1] == '*') {
            const std::size_t end = source.find("*/", i + 2);
            const std::size_t comment_end = end == std::string_view::npos ? source.size() : end + 2;
            const std::string comment = uppercase(source.substr(i, comment_end - i));
            if (comment.find("INPUT PINS") != std::string::npos) {
                section = PinSection::Inputs;
            } else if (comment.find("OUTPUT PINS") != std::string::npos) {
                section = PinSection::Outputs;
            }
            i = comment_end;
            continue;
        }

        const char ch = source[i++];
        if (ch == ';') {
            const std::string text = trim(current);
            if (!text.empty()) statements.push_back({text, section});
            current.clear();
            continue;
        }
        current.push_back(ch);
    }

    const std::string text = trim(current);
    if (!text.empty()) statements.push_back({text, section});
    return statements;
}

std::vector<Token> tokenize_wincupl_expr(std::string_view expression, std::vector<std::string>& errors) {
    std::vector<Token> tokens;
    for (std::size_t i = 0; i < expression.size();) {
        const char ch = expression[i];
        if (std::isspace(static_cast<unsigned char>(ch))) {
            ++i;
            continue;
        }
        switch (ch) {
        case '!': tokens.push_back({Token::Kind::Not, "!"}); ++i; continue;
        case '&': tokens.push_back({Token::Kind::And, "&"}); ++i; continue;
        case '#': tokens.push_back({Token::Kind::Or, "#"}); ++i; continue;
        case '(': tokens.push_back({Token::Kind::LParen, "("}); ++i; continue;
        case ')': tokens.push_back({Token::Kind::RParen, ")"}); ++i; continue;
        default:
            break;
        }

        if (std::isalnum(static_cast<unsigned char>(ch)) || ch == '_') {
            const std::size_t start = i;
            while (i < expression.size()) {
                const char ident_ch = expression[i];
                if (!std::isalnum(static_cast<unsigned char>(ident_ch)) && ident_ch != '_') break;
                ++i;
            }
            tokens.push_back({Token::Kind::Identifier, std::string(expression.substr(start, i - start))});
            continue;
        }

        errors.push_back("Unexpected character in expression: '" + std::string(1, ch) + "'");
        ++i;
    }
    tokens.push_back({Token::Kind::End, {}});
    return tokens;
}

class WinCuplExprParser {
public:
    explicit WinCuplExprParser(std::vector<Token> tokens) : tokens_(std::move(tokens)) {}

    ExprNode parse(std::vector<std::string>& errors) {
        errors_ = &errors;
        ExprNode expr = parse_or();
        if (peek().kind != Token::Kind::End) {
            add_error("Unexpected token after expression: " + peek().text);
        }
        return expr;
    }

private:
    const Token& peek() const { return tokens_[pos_]; }

    const Token& consume() {
        const Token& token = tokens_[pos_];
        if (pos_ + 1 < tokens_.size()) ++pos_;
        return token;
    }

    void add_error(std::string message) {
        if (errors_) errors_->push_back(std::move(message));
    }

    ExprNode parse_or() {
        ExprNode left = parse_and();
        while (peek().kind == Token::Kind::Or) {
            consume();
            ExprNode right = parse_and();
            left = ExprNode{NodeKind::Or, {}, {std::move(left), std::move(right)}};
        }
        return left;
    }

    ExprNode parse_and() {
        ExprNode left = parse_unary();
        while (peek().kind == Token::Kind::And) {
            consume();
            ExprNode right = parse_unary();
            left = ExprNode{NodeKind::And, {}, {std::move(left), std::move(right)}};
        }
        return left;
    }

    ExprNode parse_unary() {
        if (peek().kind == Token::Kind::Not) {
            consume();
            return ExprNode{NodeKind::Not, {}, {parse_unary()}};
        }
        return parse_primary();
    }

    ExprNode parse_primary() {
        if (peek().kind == Token::Kind::Identifier) {
            return ExprNode{NodeKind::Signal, consume().text, {}};
        }
        if (peek().kind == Token::Kind::LParen) {
            consume();
            ExprNode expr = parse_or();
            if (peek().kind != Token::Kind::RParen) {
                add_error("Expected ')' in expression");
                return expr;
            }
            consume();
            return expr;
        }
        add_error("Expected signal or '(' in expression");
        if (peek().kind != Token::Kind::End) consume();
        return ExprNode{NodeKind::Signal, {}, {}};
    }

    std::vector<Token> tokens_;
    std::size_t pos_{0};
    std::vector<std::string>* errors_{nullptr};
};

LogicDialect detect_dialect(std::string_view source) {
    const std::string upper = uppercase(source);
    if (upper.find("PIN ") != std::string::npos && upper.find("DEVICE") != std::string::npos) {
        return LogicDialect::WinCUPL;
    }
    return LogicDialect::WinCUPL;
}

std::vector<std::string> split_words(std::string_view text) {
    std::istringstream stream{std::string(text)};
    std::vector<std::string> words;
    std::string word;
    while (stream >> word) words.push_back(word);
    return words;
}

bool eval_node(const ExprNode& node, const EvalContext& context, bool& value, std::vector<std::string>& errors) {
    switch (node.kind) {
    case NodeKind::Signal: {
        const auto it = context.signals.find(node.signal);
        if (it == context.signals.end()) {
            errors.push_back("Missing input signal: " + node.signal);
            value = false;
            return false;
        }
        value = it->second;
        return true;
    }
    case NodeKind::Not: {
        bool child = false;
        const bool ok = !node.children.empty() && eval_node(node.children.front(), context, child, errors);
        value = !child;
        return ok;
    }
    case NodeKind::And: {
        bool all_ok = true;
        value = true;
        for (const auto& child_node : node.children) {
            bool child = false;
            all_ok = eval_node(child_node, context, child, errors) && all_ok;
            value = value && child;
        }
        return all_ok;
    }
    case NodeKind::Or: {
        bool all_ok = true;
        value = false;
        for (const auto& child_node : node.children) {
            bool child = false;
            all_ok = eval_node(child_node, context, child, errors) && all_ok;
            value = value || child;
        }
        return all_ok;
    }
    }
    errors.push_back("Unknown expression node");
    value = false;
    return false;
}

} // namespace

ParseResult parse_pld(std::string_view source, LogicDialect dialect) {
    ParseResult result;
    if (dialect == LogicDialect::Auto) {
        dialect = detect_dialect(source);
    }
    result.device.dialect = dialect;

    if (dialect != LogicDialect::WinCUPL) {
        result.errors.push_back("Only WinCUPL PLD parsing is currently implemented");
        return result;
    }

    result.device.name = "anonymous";
    const auto statements = split_statements(source);
    for (const auto& statement : statements) {
        const auto words = split_words(statement.text);
        if (words.empty()) continue;

        const std::string keyword = uppercase(words.front());
        if (keyword == "NAME" && words.size() >= 2) {
            result.device.name = words[1];
            continue;
        }
        if (keyword == "DEVICE" && words.size() >= 2) {
            result.device.device = words[1];
            continue;
        }
        if (keyword == "PIN") {
            if (words.size() < 4 || words[2] != "=") {
                result.errors.push_back("Malformed PIN declaration: " + statement.text);
                continue;
            }
            int pin_number = 0;
            try {
                pin_number = std::stoi(words[1]);
            } catch (...) {
                result.errors.push_back("Bad PIN number: " + words[1]);
                continue;
            }
            const auto [signal, active_low] = parse_signal_name(words[3]);
            if (signal.empty()) {
                result.errors.push_back("Empty PIN signal in declaration: " + statement.text);
                continue;
            }
            Pin pin{pin_number, signal, active_low};
            if (statement.section == PinSection::Outputs) {
                result.device.outputs.push_back(std::move(pin));
            } else {
                result.device.inputs.push_back(std::move(pin));
            }
            continue;
        }

        const std::size_t eq_pos = statement.text.find('=');
        if (eq_pos == std::string::npos) continue;

        const auto [output, output_active_low] = parse_signal_name(statement.text.substr(0, eq_pos));
        const std::string expr_text = trim(std::string_view(statement.text).substr(eq_pos + 1));
        if (output.empty() || expr_text.empty()) {
            result.errors.push_back("Malformed equation: " + statement.text);
            continue;
        }

        std::vector<std::string> expr_errors;
        auto tokens = tokenize_wincupl_expr(expr_text, expr_errors);
        WinCuplExprParser parser(std::move(tokens));
        ExprNode expr = parser.parse(expr_errors);
        if (!expr_errors.empty()) {
            for (const auto& error : expr_errors) {
                result.errors.push_back(output + ": " + error);
            }
            continue;
        }
        result.device.equations.push_back({output, output_active_low, expr_text, std::move(expr)});
    }

    return result;
}

LogicDeviceDescription parse_simple_logic(std::string_view source) {
    return parse_pld(source, LogicDialect::Auto).device;
}

EvalResult evaluate(const LogicDeviceDescription& device, const EvalContext& context) {
    EvalResult result;
    for (const auto& equation : device.equations) {
        bool value = false;
        eval_node(equation.expression, context, value, result.errors);
        result.outputs[equation.output] = equation.output_active_low ? !value : value;
    }
    return result;
}

namespace {

void append_errors(std::vector<std::string>& dst, std::string_view prefix, const std::vector<std::string>& src) {
    for (const auto& error : src) {
        dst.push_back(std::string(prefix) + ": " + error);
    }
}

bool output_value(const EvalResult& result, std::string_view name, std::vector<std::string>& errors, std::string_view stage) {
    const std::string key(name);
    const auto it = result.outputs.find(key);
    if (it == result.outputs.end()) {
        errors.push_back(std::string(stage) + ": Missing output signal: " + key);
        return false;
    }
    return it->second;
}

void set_address_bits(EvalContext& context, uint16_t address, int high_bit) {
    for (int bit = 0; bit <= high_bit; ++bit) {
        context.signals["A" + std::to_string(bit)] = (address & (1u << bit)) != 0;
    }
}

} // namespace

BoardDecodeResult decode_board_logic(const BoardLogicDevices& devices, const BoardSignals& signals) {
    BoardDecodeResult decoded;

    EvalContext signal_context;
    signal_context.signals["E"] = signals.e;
    signal_context.signals["Q"] = signals.q;
    signal_context.signals["RW"] = signals.rw;
    signal_context.signals["BA"] = signals.ba;
    signal_context.signals["BS"] = signals.bs;
    signal_context.signals["MEM_EN"] = signals.memory_enable;
    signal_context.signals["BREQ"] = signals.breq;

    const EvalResult signal_eval = evaluate(devices.signal_logic, signal_context);
    append_errors(decoded.errors, "SIGNAL-LOGIC", signal_eval.errors);
    decoded.rw1 = output_value(signal_eval, "RW1", decoded.errors, "SIGNAL-LOGIC");
    decoded.mem_wr = output_value(signal_eval, "MEM_WR", decoded.errors, "SIGNAL-LOGIC");
    decoded.mem_rd = output_value(signal_eval, "MEM_RD", decoded.errors, "SIGNAL-LOGIC");
    decoded.bavail = output_value(signal_eval, "BAVAIL", decoded.errors, "SIGNAL-LOGIC");
    decoded.rd = output_value(signal_eval, "RD", decoded.errors, "SIGNAL-LOGIC");
    decoded.wr = output_value(signal_eval, "WR", decoded.errors, "SIGNAL-LOGIC");

    EvalContext memory_context;
    set_address_bits(memory_context, signals.address, 15);
    memory_context.signals["AM19"] = (signals.mapper_bits & 0x01) != 0;
    memory_context.signals["AM20"] = (signals.mapper_bits & 0x02) != 0;
    memory_context.signals["AM21"] = (signals.mapper_bits & 0x04) != 0;
    memory_context.signals["MEM_RD"] = decoded.mem_rd;
    memory_context.signals["MAP_EN"] = signals.mapper_enable;

    const EvalResult memory_eval = evaluate(devices.memory_logic, memory_context);
    append_errors(decoded.errors, "MEM-LOGIC", memory_eval.errors);
    decoded.rom_en = output_value(memory_eval, "ROM_EN", decoded.errors, "MEM-LOGIC");
    decoded.raml_en = output_value(memory_eval, "RAML_EN", decoded.errors, "MEM-LOGIC");
    decoded.ramh_en = output_value(memory_eval, "RAMH_EN", decoded.errors, "MEM-LOGIC");
    decoded.ramx_en = output_value(memory_eval, "RAMX_EN", decoded.errors, "MEM-LOGIC");
    decoded.io_en = output_value(memory_eval, "IO_EN", decoded.errors, "MEM-LOGIC");
    decoded.map_rd = output_value(memory_eval, "MAP_RD", decoded.errors, "MEM-LOGIC");
    decoded.bank_sel0 = output_value(memory_eval, "BANK_SEL0", decoded.errors, "MEM-LOGIC");
    decoded.bank_sel1 = output_value(memory_eval, "BANK_SEL1", decoded.errors, "MEM-LOGIC");
    decoded.bank_select = static_cast<uint8_t>((decoded.bank_sel0 ? 0x01 : 0x00) | (decoded.bank_sel1 ? 0x02 : 0x00));

    EvalContext address_context;
    set_address_bits(address_context, signals.address, 9);
    address_context.signals["IOEN"] = decoded.io_en;
    address_context.signals["WR"] = decoded.wr;

    const EvalResult address_eval = evaluate(devices.address_logic, address_context);
    append_errors(decoded.errors, "ADDRESS-LOGIC", address_eval.errors);
    decoded.mapper_register_en = output_value(address_eval, "MEM_EN", decoded.errors, "ADDRESS-LOGIC");
    decoded.irq_en = output_value(address_eval, "IRQ_EN", decoded.errors, "ADDRESS-LOGIC");
    decoded.ps2_en = output_value(address_eval, "PS2_EN", decoded.errors, "ADDRESS-LOGIC");
    decoded.cf_en = output_value(address_eval, "CF_EN", decoded.errors, "ADDRESS-LOGIC");
    decoded.par_en = output_value(address_eval, "PAR_EN", decoded.errors, "ADDRESS-LOGIC");
    decoded.ser_en = output_value(address_eval, "SER_EN", decoded.errors, "ADDRESS-LOGIC");
    decoded.vdc_en = output_value(address_eval, "VDC_EN", decoded.errors, "ADDRESS-LOGIC");
    decoded.snd_en = output_value(address_eval, "SND_EN", decoded.errors, "ADDRESS-LOGIC");
    decoded.exp_en = output_value(address_eval, "EXP_EN", decoded.errors, "ADDRESS-LOGIC");

    return decoded;
}

} // namespace microlind::logic
