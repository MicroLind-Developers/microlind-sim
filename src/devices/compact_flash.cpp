#include "microlind/devices/compact_flash.hpp"

#include <algorithm>
#include <fstream>
#include <iterator>
#include <limits>
#include <utility>

namespace microlind::devices {
namespace {

constexpr uint8_t ERR_AMNF = 0x01;
constexpr uint8_t ERR_ABRT = 0x04;
constexpr uint8_t ERR_IDNF = 0x10;

constexpr uint8_t STATUS_ERR = 0x01;
constexpr uint8_t STATUS_DRQ = 0x08;
constexpr uint8_t STATUS_DSC = 0x10;
constexpr uint8_t STATUS_DRDY = 0x40;

constexpr uint8_t CMD_RECALIBRATE_MASK = 0xF0;
constexpr uint8_t CMD_RECALIBRATE = 0x10;
constexpr uint8_t CMD_READ_SECTORS = 0x20;
constexpr uint8_t CMD_READ_SECTORS_RETRY = 0x21;
constexpr uint8_t CMD_WRITE_SECTORS = 0x30;
constexpr uint8_t CMD_WRITE_SECTORS_RETRY = 0x31;
constexpr uint8_t CMD_READ_VERIFY = 0x40;
constexpr uint8_t CMD_READ_VERIFY_RETRY = 0x41;
constexpr uint8_t CMD_FORMAT_TRACK = 0x50;
constexpr uint8_t CMD_EXECUTE_DIAGNOSTIC = 0x90;
constexpr uint8_t CMD_INITIALIZE_DRIVE_PARAMETERS = 0x91;
constexpr uint8_t CMD_IDLE_IMMEDIATE = 0x95;
constexpr uint8_t CMD_IDLE_IMMEDIATE_ALT = 0xE1;
constexpr uint8_t CMD_STANDBY = 0x96;
constexpr uint8_t CMD_STANDBY_ALT = 0xE2;
constexpr uint8_t CMD_IDLE = 0x97;
constexpr uint8_t CMD_IDLE_ALT = 0xE3;
constexpr uint8_t CMD_CHECK_POWER_MODE = 0x98;
constexpr uint8_t CMD_CHECK_POWER_MODE_ALT = 0xE5;
constexpr uint8_t CMD_SLEEP = 0x99;
constexpr uint8_t CMD_SLEEP_ALT = 0xE6;
constexpr uint8_t CMD_ERASE_SECTORS = 0xC0;
constexpr uint8_t CMD_READ_MULTIPLE = 0xC4;
constexpr uint8_t CMD_WRITE_MULTIPLE = 0xC5;
constexpr uint8_t CMD_SET_MULTIPLE_MODE = 0xC6;
constexpr uint8_t CMD_IDENTIFY = 0xEC;
constexpr uint8_t CMD_SET_FEATURES = 0xEF;

constexpr uint16_t DEFAULT_HEADS = 16;
constexpr uint16_t DEFAULT_SECTORS_PER_TRACK = 63;

} // namespace

CompactFlash::CompactFlash() : CompactFlash(Options{}) {}

CompactFlash::CompactFlash(Options options) : options_(std::move(options)) {
    std::string ignored_error;
    if (!load_image(&ignored_error)) {
        options_.image_path.clear();
        load_image();
    }
    reset_registers();
}

bool CompactFlash::load_disk_image(const std::filesystem::path& path, std::string* error) {
    return load_disk_image(path, 0, error);
}

bool CompactFlash::load_disk_image(const std::filesystem::path& path, uint32_t minimum_sectors, std::string* error) {
    options_.image_path = path;
    options_.sectors = minimum_sectors;
    if (!load_image(error)) return false;
    reset_registers();
    return true;
}

CompactFlash::Snapshot CompactFlash::snapshot() const {
    return Snapshot{
        options_.image_path,
        sector_count_,
        options_.read_only,
        error_,
        features_,
        sector_count_reg_,
        sector_number_,
        cylinder_low_,
        cylinder_high_,
        drive_head_,
        status_,
        command_,
        selected_lba(),
        requested_sector_count(),
        transfer_mode_,
        transfer_buffer_.size(),
        transfer_index_,
    };
}

bool CompactFlash::load_image(std::string* error) {
    storage_.clear();
    if (!options_.image_path.empty()) {
        std::ifstream file(options_.image_path, std::ios::binary);
        if (!file) {
            if (error) *error = "Cannot open CF disk image: " + options_.image_path.string();
            return false;
        }
        storage_.assign(std::istreambuf_iterator<char>(file), {});
    }

    if (storage_.empty()) {
        storage_.resize(static_cast<std::size_t>(options_.sectors) * SectorSize, 0);
    }

    if (storage_.size() % SectorSize != 0) {
        storage_.resize(((storage_.size() / SectorSize) + 1) * SectorSize, 0);
    }

    if (options_.sectors > 0) {
        const std::size_t requested_size = static_cast<std::size_t>(options_.sectors) * SectorSize;
        if (storage_.size() < requested_size) {
            storage_.resize(requested_size, 0);
        }
    }

    const std::size_t sectors = storage_.size() / SectorSize;
    if (sectors > std::numeric_limits<uint32_t>::max()) {
        if (error) *error = "CF disk image is too large";
        storage_.clear();
        sector_count_ = 0;
        return false;
    }

    sector_count_ = static_cast<uint32_t>(storage_.size() / SectorSize);
    return true;
}

void CompactFlash::flush_image() {
    if (options_.read_only || options_.image_path.empty()) return;

    std::ofstream file(options_.image_path, std::ios::binary | std::ios::trunc);
    if (!file) return;
    file.write(reinterpret_cast<const char*>(storage_.data()), static_cast<std::streamsize>(storage_.size()));
}

void CompactFlash::reset_registers() {
    error_ = 0;
    features_ = 0;
    sector_count_reg_ = 1;
    sector_number_ = 1;
    cylinder_low_ = 0;
    cylinder_high_ = 0;
    drive_head_ = 0xE0;
    status_ = STATUS_DRDY | STATUS_DSC;
    command_ = 0;
    transfer_mode_ = TransferMode::None;
    transfer_buffer_.clear();
    transfer_index_ = 0;
    write_lba_ = 0;
    write_sector_count_ = 0;
}

uint8_t CompactFlash::read8(uint16_t offset) {
    switch (offset & 0x07) {
    case 0x00:
        if (transfer_mode_ == TransferMode::Read && transfer_index_ < transfer_buffer_.size()) {
            const uint8_t value = transfer_buffer_[transfer_index_++];
            if (transfer_index_ >= transfer_buffer_.size()) {
                transfer_buffer_.clear();
                transfer_index_ = 0;
                finish_command();
            }
            return value;
        }
        return 0x00;
    case 0x01: return error_;
    case 0x02: return sector_count_reg_;
    case 0x03: return sector_number_;
    case 0x04: return cylinder_low_;
    case 0x05: return cylinder_high_;
    case 0x06: return drive_head_;
    case 0x07: return status_;
    default: return 0xFF;
    }
}

void CompactFlash::write8(uint16_t offset, uint8_t value) {
    switch (offset & 0x07) {
    case 0x00:
        if (transfer_mode_ == TransferMode::Write && transfer_index_ < transfer_buffer_.size()) {
            transfer_buffer_[transfer_index_++] = value;
            if (transfer_index_ >= transfer_buffer_.size()) {
                commit_write();
            }
        }
        break;
    case 0x01:
        features_ = value;
        break;
    case 0x02:
        sector_count_reg_ = value;
        break;
    case 0x03:
        sector_number_ = value;
        break;
    case 0x04:
        cylinder_low_ = value;
        break;
    case 0x05:
        cylinder_high_ = value;
        break;
    case 0x06:
        drive_head_ = value;
        break;
    case 0x07:
        execute_command(value);
        break;
    default:
        break;
    }
}

void CompactFlash::execute_command(uint8_t command) {
    command_ = command;
    error_ = 0;
    status_ = STATUS_DRDY | STATUS_DSC;
    transfer_mode_ = TransferMode::None;
    transfer_buffer_.clear();
    transfer_index_ = 0;

    if ((command & CMD_RECALIBRATE_MASK) == CMD_RECALIBRATE) {
        finish_command();
        return;
    }

    switch (command) {
    case CMD_IDENTIFY:
        prepare_identify();
        break;
    case CMD_READ_SECTORS:
    case CMD_READ_SECTORS_RETRY:
    case CMD_READ_MULTIPLE:
        prepare_read();
        break;
    case CMD_WRITE_SECTORS:
    case CMD_WRITE_SECTORS_RETRY:
    case CMD_WRITE_MULTIPLE:
        prepare_write();
        break;
    case CMD_READ_VERIFY:
    case CMD_READ_VERIFY_RETRY:
        if (selected_lba() + requested_sector_count() > sector_count_) set_error(ERR_IDNF);
        else finish_command();
        break;
    case CMD_EXECUTE_DIAGNOSTIC:
        error_ = 0x01;
        finish_command();
        break;
    case CMD_CHECK_POWER_MODE:
    case CMD_CHECK_POWER_MODE_ALT:
        sector_count_reg_ = 0xFF; // Active or idle.
        finish_command();
        break;
    case CMD_ERASE_SECTORS: {
        const uint32_t lba = selected_lba();
        const uint32_t count = requested_sector_count();
        if (options_.read_only) {
            set_error(ERR_ABRT);
        } else if (count == 0 || lba + count > sector_count_) {
            set_error(ERR_IDNF);
        } else {
            const std::size_t begin = static_cast<std::size_t>(lba) * SectorSize;
            const std::size_t end = begin + static_cast<std::size_t>(count) * SectorSize;
            std::fill(storage_.begin() + static_cast<std::ptrdiff_t>(begin),
                      storage_.begin() + static_cast<std::ptrdiff_t>(end),
                      0);
            flush_image();
            finish_command();
        }
        break;
    }
    case CMD_FORMAT_TRACK:
    case CMD_INITIALIZE_DRIVE_PARAMETERS:
    case CMD_IDLE:
    case CMD_IDLE_ALT:
    case CMD_IDLE_IMMEDIATE:
    case CMD_IDLE_IMMEDIATE_ALT:
    case CMD_SET_MULTIPLE_MODE:
    case CMD_SET_FEATURES:
    case CMD_SLEEP:
    case CMD_SLEEP_ALT:
    case CMD_STANDBY:
    case CMD_STANDBY_ALT:
        finish_command();
        break;
    default:
        set_error(ERR_ABRT);
        break;
    }
}

void CompactFlash::set_error(uint8_t error) {
    error_ = error;
    status_ = STATUS_DRDY | STATUS_DSC | STATUS_ERR;
    transfer_mode_ = TransferMode::None;
    transfer_buffer_.clear();
    transfer_index_ = 0;
}

void CompactFlash::finish_command() {
    status_ = STATUS_DRDY | STATUS_DSC;
    transfer_mode_ = TransferMode::None;
}

uint32_t CompactFlash::selected_lba() const {
    if ((drive_head_ & 0x40) != 0) {
        return (static_cast<uint32_t>(drive_head_ & 0x0F) << 24) |
               (static_cast<uint32_t>(cylinder_high_) << 16) |
               (static_cast<uint32_t>(cylinder_low_) << 8) |
               sector_number_;
    }

    const uint32_t cylinder = (static_cast<uint32_t>(cylinder_high_) << 8) | cylinder_low_;
    const uint32_t head = drive_head_ & 0x0F;
    const uint32_t sector = sector_number_ == 0 ? 0 : static_cast<uint32_t>(sector_number_ - 1);
    return ((cylinder * DEFAULT_HEADS) + head) * DEFAULT_SECTORS_PER_TRACK + sector;
}

uint32_t CompactFlash::requested_sector_count() const {
    return sector_count_reg_ == 0 ? 256u : sector_count_reg_;
}

void CompactFlash::prepare_identify() {
    transfer_buffer_.assign(SectorSize, 0);

    const uint16_t cylinders = static_cast<uint16_t>(
        std::clamp<uint32_t>(sector_count_ / (DEFAULT_HEADS * DEFAULT_SECTORS_PER_TRACK), 1, 16383));
    set_identify_word(0, 0x848A);
    set_identify_word(1, cylinders);
    set_identify_word(3, DEFAULT_HEADS);
    set_identify_word(6, DEFAULT_SECTORS_PER_TRACK);
    set_identify_string(10, 10, "MICROLIND0001");
    set_identify_string(23, 4, "0.1");
    set_identify_string(27, 20, "MICROLIND CF");
    set_identify_word(47, 0x8001);
    set_identify_word(49, 0x0200); // LBA supported.
    set_identify_word(51, 0x0200);
    set_identify_word(53, 0x0001);
    set_identify_word(54, cylinders);
    set_identify_word(55, DEFAULT_HEADS);
    set_identify_word(56, DEFAULT_SECTORS_PER_TRACK);
    set_identify_word(57, static_cast<uint16_t>(sector_count_ & 0xFFFF));
    set_identify_word(58, static_cast<uint16_t>((sector_count_ >> 16) & 0xFFFF));
    set_identify_word(60, static_cast<uint16_t>(sector_count_ & 0xFFFF));
    set_identify_word(61, static_cast<uint16_t>((sector_count_ >> 16) & 0xFFFF));

    transfer_mode_ = TransferMode::Read;
    transfer_index_ = 0;
    status_ = STATUS_DRDY | STATUS_DSC | STATUS_DRQ;
}

void CompactFlash::prepare_read() {
    const uint32_t lba = selected_lba();
    const uint32_t count = requested_sector_count();
    if (count == 0 || lba + count > sector_count_) {
        set_error(ERR_IDNF);
        return;
    }

    const std::size_t byte_offset = static_cast<std::size_t>(lba) * SectorSize;
    const std::size_t byte_count = static_cast<std::size_t>(count) * SectorSize;
    transfer_buffer_.assign(storage_.begin() + static_cast<std::ptrdiff_t>(byte_offset),
                            storage_.begin() + static_cast<std::ptrdiff_t>(byte_offset + byte_count));
    transfer_index_ = 0;
    transfer_mode_ = TransferMode::Read;
    status_ = STATUS_DRDY | STATUS_DSC | STATUS_DRQ;
}

void CompactFlash::prepare_write() {
    const uint32_t lba = selected_lba();
    const uint32_t count = requested_sector_count();
    if (options_.read_only) {
        set_error(ERR_ABRT);
        return;
    }
    if (count == 0 || lba + count > sector_count_) {
        set_error(ERR_IDNF);
        return;
    }

    write_lba_ = lba;
    write_sector_count_ = count;
    transfer_buffer_.assign(static_cast<std::size_t>(count) * SectorSize, 0);
    transfer_index_ = 0;
    transfer_mode_ = TransferMode::Write;
    status_ = STATUS_DRDY | STATUS_DSC | STATUS_DRQ;
}

void CompactFlash::commit_write() {
    if (write_sector_count_ == 0 || write_lba_ + write_sector_count_ > sector_count_) {
        set_error(ERR_AMNF);
        return;
    }

    const std::size_t byte_offset = static_cast<std::size_t>(write_lba_) * SectorSize;
    std::copy(transfer_buffer_.begin(), transfer_buffer_.end(), storage_.begin() + static_cast<std::ptrdiff_t>(byte_offset));
    flush_image();
    transfer_buffer_.clear();
    transfer_index_ = 0;
    write_lba_ = 0;
    write_sector_count_ = 0;
    finish_command();
}

void CompactFlash::set_identify_word(std::size_t index, uint16_t value) {
    const std::size_t byte_index = index * 2;
    if (byte_index + 1 >= transfer_buffer_.size()) return;
    transfer_buffer_[byte_index] = static_cast<uint8_t>(value & 0xFF);
    transfer_buffer_[byte_index + 1] = static_cast<uint8_t>((value >> 8) & 0xFF);
}

void CompactFlash::set_identify_string(std::size_t start_word, std::size_t word_count, const std::string& value) {
    std::string padded = value.substr(0, word_count * 2);
    padded.resize(word_count * 2, ' ');
    for (std::size_t i = 0; i < word_count; ++i) {
        const std::size_t src = i * 2;
        const std::size_t dst = (start_word + i) * 2;
        if (dst + 1 >= transfer_buffer_.size()) break;
        transfer_buffer_[dst] = static_cast<uint8_t>(padded[src + 1]);
        transfer_buffer_[dst + 1] = static_cast<uint8_t>(padded[src]);
    }
}

} // namespace microlind::devices
