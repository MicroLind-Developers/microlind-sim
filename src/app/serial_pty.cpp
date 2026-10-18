#include "microlind/app/serial_pty.hpp"

#include <array>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <utility>
#include <vector>

#if defined(__unix__) || defined(__APPLE__)
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#endif

namespace microlind::app {

struct SerialPty::State {
    std::string path;
    std::vector<uint8_t> pending;
    std::size_t sent{};
#if defined(__unix__) || defined(__APPLE__)
    int master{-1};
    // Keep the slave alive across client disconnects, including before boot.
    int slave{-1};
    ~State() {
        if (slave >= 0) ::close(slave);
        if (master >= 0) ::close(master);
    }
#endif
};

SerialPty::SerialPty() = default;
SerialPty::~SerialPty() = default;
SerialPty::SerialPty(SerialPty&&) noexcept = default;
SerialPty& SerialPty::operator=(SerialPty&&) noexcept = default;

bool SerialPty::supported() {
#if defined(__unix__) || defined(__APPLE__)
    return true;
#else
    return false;
#endif
}

bool SerialPty::open() {
    if (is_open()) return true;
    error_.clear();
#if defined(__unix__) || defined(__APPLE__)
    state_ = std::make_unique<State>();
    auto system_error = [this] {
        fail("Cannot create serial PTY: " + std::string(std::strerror(errno)));
        return false;
    };
    state_->master = ::posix_openpt(O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (state_->master < 0) return system_error();
    if (::grantpt(state_->master) < 0 || ::unlockpt(state_->master) < 0) return system_error();
    const char* name = ::ptsname(state_->master);
    if (!name) return system_error();
    state_->path = name;
    state_->slave = ::open(name, O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (state_->slave < 0) return system_error();
    termios settings{};
    if (::tcgetattr(state_->slave, &settings) < 0) return system_error();
    ::cfmakeraw(&settings);
    settings.c_cflag |= CLOCAL | CREAD;
#ifdef CRTSCTS
    settings.c_cflag &= ~CRTSCTS;
#endif
    if (::tcsetattr(state_->slave, TCSANOW, &settings) < 0) return system_error();
    return true;
#else
    fail("Serial PTYs are supported on Unix systems only.");
    return false;
#endif
}

void SerialPty::close() {
    state_.reset();
    error_.clear();
}

void SerialPty::fail(std::string error) {
    state_.reset();
    error_ = std::move(error);
}

bool SerialPty::is_open() const { return state_ != nullptr; }
std::string SerialPty::path() const { return state_ ? state_->path : std::string{}; }

void SerialPty::transmit(uint8_t value) {
    if (!state_) return;
    // Bound memory if the terminal is disconnected or stops reading. Never
    // stall the CPU worker or silently drop output on overflow.
    constexpr std::size_t max_pending = 1024 * 1024;
    if (state_->pending.size() - state_->sent >= max_pending) {
        fail("Serial PTY closed: output buffer full (terminal is not reading).");
        return;
    }
    state_->pending.push_back(value);
}

void SerialPty::poll(const std::function<void(uint8_t)>& receive) {
#if defined(__unix__) || defined(__APPLE__)
    if (!state_) return;
    // Limit work per poll so large pastes cannot starve simulation or the GUI.
    std::array<uint8_t, 4096> bytes{};
    for (int batch = 0; batch < 16; ++batch) {
        const auto count = ::read(state_->master, bytes.data(), bytes.size());
        if (count > 0) {
            for (ssize_t i = 0; i < count; ++i) receive(bytes[static_cast<std::size_t>(i)]);
        } else if (count < 0 && errno == EINTR) {
            continue;
        } else if (count == 0 || errno == EAGAIN || errno == EWOULDBLOCK) {
            break;
        } else {
            fail("Serial PTY read failed: " + std::string(std::strerror(errno)));
            return;
        }
    }
    while (state_->sent < state_->pending.size()) {
        const auto count = ::write(state_->master, state_->pending.data() + state_->sent,
                                   state_->pending.size() - state_->sent);
        if (count > 0) {
            state_->sent += static_cast<std::size_t>(count);
        } else if (count < 0 && errno == EINTR) {
            continue;
        } else if (count == 0 || errno == EAGAIN || errno == EWOULDBLOCK) {
            break;
        } else {
            fail("Serial PTY write failed: " + std::string(std::strerror(errno)));
            return;
        }
    }
    if (state_->sent > 0) {
        state_->pending.erase(state_->pending.begin(), state_->pending.begin() +
                              static_cast<std::ptrdiff_t>(state_->sent));
        state_->sent = 0;
    }
#else
    (void)receive;
#endif
}

} // namespace microlind::app
