#include <gtest/gtest.h>

#include <algorithm>
#include <fstream>
#include <sstream>

#include "microlind/app/config_editor.hpp"
#include "microlind/app/sim_builder.hpp"
#include "test_harness.hpp"

namespace {

std::string read_file(const std::filesystem::path& path) {
    std::ifstream input(path);
    std::ostringstream contents;
    contents << input.rdbuf();
    return contents.str();
}

bool errors(const microlind::cli::HardwareConfig& config) {
    const auto issues = microlind::app::validate_config(config);
    return std::any_of(issues.begin(), issues.end(), [](const auto& issue) { return !issue.warning; });
}

TEST(ConfigEditorTest, NewBoardDefaultsValidateAndRoundTripAllDeviceSettings) {
    microlind::app::ConfigEditor editor;
    EXPECT_TRUE(editor.modified);
    EXPECT_FALSE(errors(editor.effective_config()));
    editor.config.logic = {"signal.pld", "memory.pld", "address.pld", microlind::BusDecodeMode::Validate, true};
    editor.config.cf.image_path = "disk image.img";
    editor.config.cf.read_only = true;
    editor.config.parallel.irq_level = 3;
    const auto resolved = editor.effective_config();
    const auto filename = microlind::test::test_output_path("config-editor-roundtrip.cfg");
    std::string error;
    ASSERT_TRUE(editor.save(filename, error)) << error;
    EXPECT_FALSE(editor.modified);
    const auto loaded = microlind::cli::load_hardware_config(filename, error);
    ASSERT_TRUE(loaded) << error;
    EXPECT_EQ(loaded->roms.size(), 2);
    EXPECT_EQ(loaded->ram.available, 524288);
    EXPECT_EQ(loaded->serial.irq_level, 1);
    EXPECT_EQ(loaded->parallel.irq_level, 3);
    EXPECT_EQ(loaded->video.vram_size, 65536);
    EXPECT_TRUE(loaded->cf.read_only);
    EXPECT_EQ(loaded->mapper.windows[3].end, 0xDFFF);
    EXPECT_EQ(std::filesystem::absolute(loaded->cf.image_path).lexically_normal(), resolved.cf.image_path);
    EXPECT_EQ(std::filesystem::absolute(loaded->logic.signal_logic_path).lexically_normal(), resolved.logic.signal_logic_path);
    EXPECT_EQ(loaded->logic.bus_mode, microlind::BusDecodeMode::Validate);
}

TEST(ConfigEditorTest, SaveAsRebasesFilesAndRetainsCommentsAndUnknownSettings) {
    const auto first = microlind::test::test_output_path("config-editor-source/hw.cfg");
    const auto second = microlind::test::test_output_path("config-editor-destination/hw.cfg");
    std::filesystem::create_directories(first.parent_path());
    std::filesystem::create_directories(second.parent_path());
    {
        std::ofstream file(first);
        file << "# Board comment\n[ROM]\nSTART=$F800\nEND=$FFFF\nCUSTOM_ROM=yes\n"
             << "[RAM]\nSTART=0\nEND=$DFFF\nBANK_SIZE=16384\nAVAILABLE=524288\n"
             << "[CF]\nIO_START_ADDRESS=$F418\nIO_END_ADDRESS=$F41F\nIMAGE=assets/disk.img\nREAD_ONLY=true\n"
             << "[MEMORY_MAPPER]\nBANK_0_REGISTER=$F400 # bank comment\nBANK_1_REGISTER=$F401\n"
             << "BANK_2_REGISTER=$F402\nBANK_3_REGISTER=$F403\n"
             << "BUS_SIGNALS_PROVIDER=AM14, AM15, AM16\n"
             << "[PLD_LOGIC]\nSIGNAL_LOGIC=signal.pld\nMEMORY_LOGIC=memory.pld\nADDRESS_LOGIC=address.pld\nBUS_MODE=route\n"
             << "[CUSTOM_DEVICE]\nTHING=keep me\n";
    }
    microlind::app::ConfigEditor editor;
    std::string error;
    ASSERT_TRUE(editor.open(first, error)) << error;
    const auto original = editor.effective_config();
    editor.config.cf.sectors = 1024;
    ASSERT_TRUE(editor.save(second, error)) << error;
    const auto loaded = microlind::cli::load_hardware_config(second, error);
    ASSERT_TRUE(loaded) << error;
    EXPECT_EQ(std::filesystem::absolute(loaded->cf.image_path).lexically_normal(), original.cf.image_path);
    EXPECT_EQ(std::filesystem::absolute(loaded->logic.address_logic_path).lexically_normal(), original.logic.address_logic_path);
    EXPECT_EQ(editor.effective_config().cf.image_path, original.cf.image_path);
    const auto content = read_file(second);
    for (const auto* preserved : {"# Board comment", "# bank comment", "CUSTOM_ROM=yes",
                                  "BUS_SIGNALS_PROVIDER=AM14, AM15, AM16", "[CUSTOM_DEVICE]\nTHING=keep me"}) {
        EXPECT_NE(content.find(preserved), std::string::npos);
    }
    EXPECT_NE(read_file(first).find("IMAGE=assets/disk.img"), std::string::npos);
    ASSERT_TRUE(editor.save(second, error)) << error; // no duplicate comments on repeated saves
    EXPECT_EQ(read_file(second), content);
}

TEST(ConfigEditorTest, DisabledDevicesAreOmittedAndDraftCanBeRestored) {
    microlind::app::ConfigEditor editor;
    const auto filename = microlind::test::test_output_path("config-editor-disabled.cfg");
    std::string error;
    ASSERT_TRUE(editor.save(filename, error)) << error;
    editor.config.video.present = false;
    editor.config.cf.present = false;
    editor.rom_enabled = false;
    editor.modified = true;
    editor.discard();
    EXPECT_TRUE(editor.rom_enabled);
    EXPECT_TRUE(editor.config.video.present);
    EXPECT_FALSE(editor.modified);
    editor.config.video.present = false;
    editor.rom_enabled = false;
    ASSERT_TRUE(editor.save(filename, error)) << error;
    const auto loaded = microlind::cli::load_hardware_config(filename, error);
    ASSERT_TRUE(loaded);
    EXPECT_TRUE(loaded->roms.empty());
    EXPECT_FALSE(loaded->video.present);
    EXPECT_EQ(editor.config.video.start, 0xF440); // disabled values retained in the draft
    auto sim = microlind::cli::build_sim(microlind::CpuMode::HD6309, nullptr, &*loaded);
    sim.bus().write8(0x0000, 0x5A);
    EXPECT_EQ(sim.bus().peek8(0x0000), 0x5A);
    EXPECT_EQ(sim.bus().peek8(0xF440), 0xFF);
}

TEST(ConfigEditorTest, InvalidEditsCannotOverwriteExistingFileOrResetDraftOnFailedOpen) {
    microlind::app::ConfigEditor editor;
    const auto filename = microlind::test::test_output_path("config-editor-invalid.cfg");
    std::string error;
    ASSERT_TRUE(editor.save(filename, error)) << error;
    const auto original = read_file(filename);
    editor.config.video.end = 0xF443;
    EXPECT_FALSE(editor.save(filename, error));
    EXPECT_EQ(read_file(filename), original);
    EXPECT_FALSE(editor.open(filename / "missing", error));
    EXPECT_EQ(editor.config.video.end, 0xF443);
    editor.discard();
    {
        std::ofstream file(filename, std::ios::app);
        file << "# changed externally\n";
    }
    EXPECT_FALSE(editor.save(filename, error));
    EXPECT_NE(error.find("changed on disk"), std::string::npos);
    EXPECT_NE(read_file(filename).find("# changed externally"), std::string::npos);
}

TEST(ConfigEditorTest, ValidationFindsCollisionsIrqReservationAndMapperErrors) {
    microlind::app::ConfigEditor editor;
    editor.config.cf.start = 0xF430;
    editor.config.cf.end = 0xF437;
    EXPECT_TRUE(errors(editor.effective_config()));
    editor.new_config();
    editor.config.video.start = 0xF404;
    editor.config.video.end = 0xF405;
    EXPECT_TRUE(errors(editor.effective_config()));
    editor.new_config();
    editor.config.mapper.bank_reg[1] = editor.config.mapper.bank_reg[0];
    EXPECT_TRUE(errors(editor.effective_config()));
    editor.new_config();
    editor.config.ram.available = 16384 * 3;
    EXPECT_TRUE(errors(editor.effective_config()));
    editor.new_config();
    editor.config.mapper.windows[0].end = 0x8000;
    EXPECT_TRUE(errors(editor.effective_config()));
    editor.new_config();
    editor.config.roms = {{0xC000, 0xF3FF}}; // explicit banked RAM overlays the lower ROM range
    EXPECT_FALSE(errors(editor.effective_config()));
    const auto map = microlind::app::config_address_map(editor.effective_config());
    const auto rom = std::find_if(map.begin(), map.end(), [](const auto& item) { return item.section == "ROM"; });
    ASSERT_NE(rom, map.end());
    EXPECT_EQ(rom->start, 0xE000);
}

TEST(ConfigEditorTest, ConfigWithNoMemoryDoesNotFallBackToDefaultBoard) {
    microlind::cli::HardwareConfig config;
    auto sim = microlind::cli::build_sim(microlind::CpuMode::HD6309, nullptr, &config);
    EXPECT_EQ(sim.bus().map_summary().size(), 1); // only the fixed IRQ register
    sim.bus().write8(0x0000, 0x5A);
    EXPECT_EQ(sim.bus().peek8(0x0000), 0xFF);
}

} // namespace
