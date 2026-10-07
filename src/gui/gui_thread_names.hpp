#pragma once

#include <algorithm>
#include <array>
#include <string>
#include <string_view>
#include <thread>

#if defined(__linux__) || defined(__APPLE__)
#include <pthread.h>
#endif

namespace microlind::gui {

inline std::array<char, 16> linux_thread_name(std::string_view name) {
    std::array<char, 16> buffer{};
    const std::size_t count = std::min<std::size_t>(name.size(), buffer.size() - 1);
    std::copy_n(name.data(), count, buffer.data());
    return buffer;
}

inline void set_current_thread_name(std::string_view name) {
#if defined(__linux__)
    const auto buffer = linux_thread_name(name);
    pthread_setname_np(pthread_self(), buffer.data());
#elif defined(__APPLE__)
    pthread_setname_np(std::string(name).c_str());
#else
    (void)name;
#endif
}

inline void set_thread_name(std::thread& thread, std::string_view name) {
#if defined(__linux__)
    const auto buffer = linux_thread_name(name);
    pthread_setname_np(thread.native_handle(), buffer.data());
#else
    (void)thread;
    (void)name;
#endif
}

} // namespace microlind::gui
