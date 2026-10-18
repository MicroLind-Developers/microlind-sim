#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <string>

namespace microlind::app {

// Single-threaded, nonblocking binary bridge. The owner serializes all access.
class SerialPty {
public:
    SerialPty();
    ~SerialPty();
    SerialPty(SerialPty&&) noexcept;
    SerialPty& operator=(SerialPty&&) noexcept;
    SerialPty(const SerialPty&) = delete;
    SerialPty& operator=(const SerialPty&) = delete;

    [[nodiscard]] static bool supported();
    bool open();
    void close();
    [[nodiscard]] bool is_open() const;
    [[nodiscard]] std::string path() const;
    [[nodiscard]] const std::string& error() const { return error_; }
    void transmit(uint8_t value);
    void poll(const std::function<void(uint8_t)>& receive);

private:
    void fail(std::string error);
    struct State;
    std::unique_ptr<State> state_;
    std::string error_;
};

} // namespace microlind::app
