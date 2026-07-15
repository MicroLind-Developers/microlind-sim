#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "microlind/app/hardware_config.hpp"
#include "microlind/app/logic_validation.hpp"
#include "microlind/logic.hpp"

namespace {

using ::testing::IsEmpty;
using ::testing::HasSubstr;
using ::testing::SizeIs;
using microlind::logic::BoardLogicDevices;
using microlind::logic::BoardSignals;
using microlind::logic::EvalContext;
using microlind::logic::LogicDialect;

std::string read_text_file(const char* path) {
    std::ifstream file(path);
    return {std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
}

EvalContext address_context(uint16_t address) {
    EvalContext context;
    for (int bit = 0; bit <= 9; ++bit) {
        context.signals["A" + std::to_string(bit)] = (address & (1u << bit)) != 0;
    }
    context.signals["IOEN"] = true;
    context.signals["WR"] = false;
    return context;
}

EvalContext memory_context(uint16_t address) {
    EvalContext context;
    for (int bit = 0; bit <= 15; ++bit) {
        context.signals["A" + std::to_string(bit)] = (address & (1u << bit)) != 0;
    }
    context.signals["AM19"] = false;
    context.signals["AM20"] = false;
    context.signals["AM21"] = false;
    context.signals["MEM_RD"] = true;
    context.signals["MAP_EN"] = true;
    return context;
}

BoardLogicDevices load_board_logic_devices() {
    const auto signal_logic = microlind::logic::parse_pld(read_text_file("examples/signal-logic.pld"));
    const auto memory_logic = microlind::logic::parse_pld(read_text_file("examples/mem-logic.pld"));
    const auto address_logic = microlind::logic::parse_pld(read_text_file("examples/address-logic.pld"));
    EXPECT_TRUE(signal_logic.ok());
    EXPECT_TRUE(memory_logic.ok());
    EXPECT_TRUE(address_logic.ok());
    return {signal_logic.device, memory_logic.device, address_logic.device};
}

TEST(LogicParserTest, ParsesProjectWinCuplFiles) {
    const auto signal_logic = microlind::logic::parse_pld(read_text_file("examples/signal-logic.pld"));
    const auto mem_logic = microlind::logic::parse_pld(read_text_file("examples/mem-logic.pld"));
    const auto address_logic = microlind::logic::parse_pld(read_text_file("examples/address-logic.pld"));

    ASSERT_TRUE(signal_logic.ok());
    ASSERT_TRUE(mem_logic.ok());
    ASSERT_TRUE(address_logic.ok());

    EXPECT_EQ(signal_logic.device.dialect, LogicDialect::WinCUPL);
    EXPECT_EQ(mem_logic.device.dialect, LogicDialect::WinCUPL);
    EXPECT_EQ(address_logic.device.dialect, LogicDialect::WinCUPL);

    EXPECT_EQ(signal_logic.device.name, "Signal-Logic");
    EXPECT_EQ(mem_logic.device.name, "Memory-Logic");
    EXPECT_EQ(address_logic.device.name, "Address-Logic");

    EXPECT_THAT(signal_logic.device.inputs, SizeIs(7));
    EXPECT_THAT(signal_logic.device.outputs, SizeIs(6));
    EXPECT_THAT(signal_logic.device.equations, SizeIs(6));

    EXPECT_THAT(mem_logic.device.inputs, SizeIs(13));
    EXPECT_THAT(mem_logic.device.outputs, SizeIs(8));
    EXPECT_THAT(mem_logic.device.equations, SizeIs(8));

    EXPECT_THAT(address_logic.device.inputs, SizeIs(12));
    EXPECT_THAT(address_logic.device.outputs, SizeIs(10));
    EXPECT_THAT(address_logic.device.equations, SizeIs(9));
}

TEST(LogicParserTest, ReportsMalformedWinCuplExpressions) {
    const auto parsed = microlind::logic::parse_pld("Name Bad; Device G22V10; OUT = A & (B # ;");

    EXPECT_FALSE(parsed.ok());
    EXPECT_THAT(parsed.errors, ::testing::Not(IsEmpty()));
}

TEST(LogicEvaluatorTest, EvaluatesSignalLogicOutputs) {
    const auto parsed = microlind::logic::parse_pld(read_text_file("examples/signal-logic.pld"));
    ASSERT_TRUE(parsed.ok());

    EvalContext context;
    context.signals["E"] = true;
    context.signals["Q"] = false;
    context.signals["RW"] = false;
    context.signals["BA"] = true;
    context.signals["BS"] = true;
    context.signals["MEM_EN"] = true;
    context.signals["BREQ"] = true;

    const auto result = microlind::logic::evaluate(parsed.device, context);
    ASSERT_TRUE(result.ok());

    EXPECT_FALSE(result.outputs.at("RW1"));
    EXPECT_TRUE(result.outputs.at("MEM_WR"));
    EXPECT_FALSE(result.outputs.at("MEM_RD"));
    EXPECT_TRUE(result.outputs.at("BAVAIL"));
    EXPECT_FALSE(result.outputs.at("RD"));
    EXPECT_TRUE(result.outputs.at("WR"));
}

TEST(LogicEvaluatorTest, EvaluatesAddressDecodeOutputs) {
    const auto parsed = microlind::logic::parse_pld(read_text_file("examples/address-logic.pld"));
    ASSERT_TRUE(parsed.ok());

    auto result = microlind::logic::evaluate(parsed.device, address_context(0xF400));
    ASSERT_TRUE(result.ok());
    EXPECT_TRUE(result.outputs.at("MEM_EN"));
    EXPECT_FALSE(result.outputs.at("IRQ_EN"));

    result = microlind::logic::evaluate(parsed.device, address_context(0xF404));
    ASSERT_TRUE(result.ok());
    EXPECT_FALSE(result.outputs.at("MEM_EN"));
    EXPECT_TRUE(result.outputs.at("IRQ_EN"));

    result = microlind::logic::evaluate(parsed.device, address_context(0xF418));
    ASSERT_TRUE(result.ok());
    EXPECT_TRUE(result.outputs.at("CF_EN"));

    result = microlind::logic::evaluate(parsed.device, address_context(0xF430));
    ASSERT_TRUE(result.ok());
    EXPECT_TRUE(result.outputs.at("SER_EN"));

    result = microlind::logic::evaluate(parsed.device, address_context(0xF680));
    ASSERT_TRUE(result.ok());
    EXPECT_TRUE(result.outputs.at("EXP_EN"));
}

TEST(LogicEvaluatorTest, EvaluatesMemoryLogicOutputs) {
    const auto parsed = microlind::logic::parse_pld(read_text_file("examples/mem-logic.pld"));
    ASSERT_TRUE(parsed.ok());

    auto result = microlind::logic::evaluate(parsed.device, memory_context(0xF000));
    ASSERT_TRUE(result.ok());
    EXPECT_TRUE(result.outputs.at("ROM_EN"));
    EXPECT_FALSE(result.outputs.at("IO_EN"));

    result = microlind::logic::evaluate(parsed.device, memory_context(0x0000));
    ASSERT_TRUE(result.ok());
    EXPECT_TRUE(result.outputs.at("RAML_EN"));
    EXPECT_FALSE(result.outputs.at("RAMH_EN"));

    auto high_ram = memory_context(0x0000);
    high_ram.signals["AM19"] = true;
    result = microlind::logic::evaluate(parsed.device, high_ram);
    ASSERT_TRUE(result.ok());
    EXPECT_FALSE(result.outputs.at("RAML_EN"));
    EXPECT_TRUE(result.outputs.at("RAMH_EN"));

    result = microlind::logic::evaluate(parsed.device, memory_context(0xF400));
    ASSERT_TRUE(result.ok());
    EXPECT_FALSE(result.outputs.at("ROM_EN"));
    EXPECT_TRUE(result.outputs.at("IO_EN"));
}

TEST(LogicEvaluatorTest, ReportsMissingSignals) {
    const auto parsed = microlind::logic::parse_pld("Name Missing; Device G22V10; OUT = A & B;");
    ASSERT_TRUE(parsed.ok());

    EvalContext context;
    context.signals["A"] = true;

    const auto result = microlind::logic::evaluate(parsed.device, context);
    EXPECT_FALSE(result.ok());
    EXPECT_THAT(result.errors, ::testing::Contains("Missing input signal: B"));
}

TEST(BoardLogicTest, DecodesControlSignals) {
    const auto devices = load_board_logic_devices();

    BoardSignals read_signals;
    read_signals.address = 0xF000;
    read_signals.rw = true;
    auto decoded = microlind::logic::decode_board_logic(devices, read_signals);
    ASSERT_TRUE(decoded.ok());
    EXPECT_TRUE(decoded.mem_rd);
    EXPECT_FALSE(decoded.mem_wr);
    EXPECT_TRUE(decoded.rd);
    EXPECT_FALSE(decoded.wr);

    BoardSignals write_signals;
    write_signals.address = 0xF000;
    write_signals.rw = false;
    write_signals.q = false;
    decoded = microlind::logic::decode_board_logic(devices, write_signals);
    ASSERT_TRUE(decoded.ok());
    EXPECT_FALSE(decoded.mem_rd);
    EXPECT_TRUE(decoded.mem_wr);
    EXPECT_FALSE(decoded.rd);
    EXPECT_TRUE(decoded.wr);
}

TEST(BoardLogicTest, DecodesMemoryRegions) {
    const auto devices = load_board_logic_devices();

    auto decoded = microlind::logic::decode_board_logic(devices, BoardSignals{.address = 0xF000});
    ASSERT_TRUE(decoded.ok());
    EXPECT_TRUE(decoded.rom_en);
    EXPECT_FALSE(decoded.raml_en);
    EXPECT_FALSE(decoded.ramh_en);
    EXPECT_FALSE(decoded.ramx_en);
    EXPECT_FALSE(decoded.io_en);

    decoded = microlind::logic::decode_board_logic(devices, BoardSignals{.address = 0x0000});
    ASSERT_TRUE(decoded.ok());
    EXPECT_FALSE(decoded.rom_en);
    EXPECT_TRUE(decoded.raml_en);
    EXPECT_FALSE(decoded.ramh_en);
    EXPECT_FALSE(decoded.ramx_en);

    BoardSignals high_ram{.address = 0x0000};
    high_ram.mapper_bits = 0x01;
    decoded = microlind::logic::decode_board_logic(devices, high_ram);
    ASSERT_TRUE(decoded.ok());
    EXPECT_FALSE(decoded.raml_en);
    EXPECT_TRUE(decoded.ramh_en);
    EXPECT_FALSE(decoded.ramx_en);

    BoardSignals expansion_ram{.address = 0x0000};
    expansion_ram.mapper_bits = 0x02;
    decoded = microlind::logic::decode_board_logic(devices, expansion_ram);
    ASSERT_TRUE(decoded.ok());
    EXPECT_FALSE(decoded.raml_en);
    EXPECT_FALSE(decoded.ramh_en);
    EXPECT_TRUE(decoded.ramx_en);
}

TEST(BoardLogicTest, DecodesIoRangesAndMapperRegisterSelect) {
    const auto devices = load_board_logic_devices();

    auto decoded = microlind::logic::decode_board_logic(devices, BoardSignals{.address = 0xF400});
    ASSERT_TRUE(decoded.ok());
    EXPECT_TRUE(decoded.io_en);
    EXPECT_TRUE(decoded.mapper_register_en);
    EXPECT_FALSE(decoded.cf_en);
    EXPECT_FALSE(decoded.ser_en);
    EXPECT_EQ(decoded.bank_select, 0u);

    decoded = microlind::logic::decode_board_logic(devices, BoardSignals{.address = 0xF403});
    ASSERT_TRUE(decoded.ok());
    EXPECT_TRUE(decoded.mapper_register_en);
    EXPECT_EQ(decoded.bank_select, 3u);

    decoded = microlind::logic::decode_board_logic(devices, BoardSignals{.address = 0xF418});
    ASSERT_TRUE(decoded.ok());
    EXPECT_TRUE(decoded.io_en);
    EXPECT_FALSE(decoded.mapper_register_en);
    EXPECT_TRUE(decoded.cf_en);
    EXPECT_FALSE(decoded.ser_en);

    decoded = microlind::logic::decode_board_logic(devices, BoardSignals{.address = 0xF430});
    ASSERT_TRUE(decoded.ok());
    EXPECT_TRUE(decoded.io_en);
    EXPECT_FALSE(decoded.cf_en);
    EXPECT_TRUE(decoded.ser_en);

    decoded = microlind::logic::decode_board_logic(devices, BoardSignals{.address = 0xF680});
    ASSERT_TRUE(decoded.ok());
    EXPECT_TRUE(decoded.io_en);
    EXPECT_TRUE(decoded.exp_en);
}

TEST(BoardLogicTest, MatchesConfiguredHardwareMapForRepresentativeAddresses) {
    std::string error;
    const auto cfg = microlind::cli::load_hardware_config("examples/hw.cfg", error);
    ASSERT_TRUE(cfg.has_value()) << error;
    ASSERT_TRUE(cfg->logic.present);
    EXPECT_EQ(cfg->logic.signal_logic_path, std::filesystem::path("examples/signal-logic.pld"));
    EXPECT_EQ(cfg->logic.memory_logic_path, std::filesystem::path("examples/mem-logic.pld"));
    EXPECT_EQ(cfg->logic.address_logic_path, std::filesystem::path("examples/address-logic.pld"));
    EXPECT_EQ(cfg->logic.bus_mode, microlind::BusDecodeMode::Route);
    ASSERT_TRUE(cfg->serial.present);
    EXPECT_EQ(cfg->serial.irq_level, 1);
    const auto devices = load_board_logic_devices();

    const auto issues = microlind::cli::validate_hardware_config_against_logic(*cfg, devices);
    EXPECT_THAT(issues, IsEmpty());
}

TEST(BoardLogicTest, ReportsHardwareConfigDecodeMismatches) {
    std::string error;
    auto cfg = microlind::cli::load_hardware_config("examples/hw.cfg", error);
    ASSERT_TRUE(cfg.has_value()) << error;
    ASSERT_TRUE(cfg->cf.present);
    ASSERT_TRUE(cfg->serial.present);
    const auto devices = load_board_logic_devices();

    cfg->cf.start = cfg->serial.start;
    cfg->cf.end = cfg->serial.end;
    const auto issues = microlind::cli::validate_hardware_config_against_logic(*cfg, devices);

    ASSERT_THAT(issues, ::testing::Not(IsEmpty()));
    bool found_cf_issue = false;
    for (const auto& issue : issues) {
        if (issue.message.find("CompactFlash address") != std::string::npos) {
            found_cf_issue = true;
            break;
        }
    }
    EXPECT_TRUE(found_cf_issue);
}

TEST(BoardLogicTest, GeneratesPartialHardwareConfigForVisualValidation) {
    const auto devices = load_board_logic_devices();

    const std::string generated = microlind::cli::generate_partial_hardware_config_from_logic(devices);

    EXPECT_THAT(generated, HasSubstr("# Partial hw.cfg generated from PLD decode logic."));
    EXPECT_THAT(generated, HasSubstr("[ROM]\nSTART=0xE000\nEND=0xF3FF"));
    EXPECT_THAT(generated, HasSubstr("[ROM]\nSTART=0xF800\nEND=0xFFFF"));
    EXPECT_THAT(generated, HasSubstr("[RAM]\n# Derived from RAML_EN with AM19..AM21 clear.\nSTART=0x0000\nEND=0xDFFF"));
    EXPECT_THAT(generated, HasSubstr("[CF]\nIO_START_ADDRESS=0xF418\nIO_END_ADDRESS=0xF41F"));
    EXPECT_THAT(generated, HasSubstr("[SERIAL]\nIO_START_ADDRESS=0xF430\nIO_END_ADDRESS=0xF43F"));
    EXPECT_THAT(generated, HasSubstr("BANK_0_REGISTER=0xF400"));
    EXPECT_THAT(generated, HasSubstr("BANK_3_REGISTER=0xF403"));
    EXPECT_THAT(generated, HasSubstr("WINDOW_0=0x0000-0x3FFF"));
    EXPECT_THAT(generated, HasSubstr("WINDOW_3=0xC000-0xDFFF"));
    EXPECT_THAT(generated, HasSubstr("# EXP_EN: 0xF680-0xF7FF"));
}

} // namespace
