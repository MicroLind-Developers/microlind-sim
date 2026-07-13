#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

#include "microlind/bus.hpp"

namespace microlind::devices {

class CompactFlash : public BusDevice {
public:
    enum class TransferMode {
        None,
        Read,
        Write,
    };

    struct Options {
        std::filesystem::path image_path{};
        uint32_t sectors{2048};
        bool read_only{false};
    };

    struct Snapshot {
        std::filesystem::path image_path{};
        uint32_t sector_count{};
        bool read_only{};
        uint8_t error{};
        uint8_t features{};
        uint8_t sector_count_reg{};
        uint8_t sector_number{};
        uint8_t cylinder_low{};
        uint8_t cylinder_high{};
        uint8_t drive_head{};
        uint8_t status{};
        uint8_t command{};
        uint32_t selected_lba{};
        uint32_t requested_sector_count{};
        TransferMode transfer_mode{TransferMode::None};
        std::size_t transfer_size{};
        std::size_t transfer_index{};
    };

    CompactFlash();
    explicit CompactFlash(Options options);

    uint8_t read8(uint16_t offset) override;
    void write8(uint16_t offset, uint8_t value) override;

    bool load_disk_image(const std::filesystem::path& path, std::string* error = nullptr);
    bool load_disk_image(const std::filesystem::path& path, uint32_t minimum_sectors, std::string* error = nullptr);

    [[nodiscard]] const std::filesystem::path& image_path() const { return options_.image_path; }
    [[nodiscard]] uint32_t sector_count() const { return sector_count_; }
    [[nodiscard]] Snapshot snapshot() const;

private:
    static constexpr std::size_t SectorSize = 512;

    bool load_image(std::string* error = nullptr);
    void flush_image();
    void reset_registers();
    void execute_command(uint8_t command);
    void set_error(uint8_t error);
    void finish_command();

    uint32_t selected_lba() const;
    uint32_t requested_sector_count() const;
    void prepare_identify();
    void prepare_read();
    void prepare_write();
    void commit_write();

    void set_identify_word(std::size_t index, uint16_t value);
    void set_identify_string(std::size_t start_word, std::size_t word_count, const std::string& value);

    Options options_;
    std::vector<uint8_t> storage_;
    uint32_t sector_count_{};

    uint8_t error_{0};
    uint8_t features_{0};
    uint8_t sector_count_reg_{1};
    uint8_t sector_number_{1};
    uint8_t cylinder_low_{0};
    uint8_t cylinder_high_{0};
    uint8_t drive_head_{0xE0};
    uint8_t status_{0x50};
    uint8_t command_{0};

    TransferMode transfer_mode_{TransferMode::None};
    std::vector<uint8_t> transfer_buffer_;
    std::size_t transfer_index_{0};
    uint32_t write_lba_{0};
    uint32_t write_sector_count_{0};
};

} // namespace microlind::devices
