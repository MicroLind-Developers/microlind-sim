#include "microlind/app/util.hpp"

#include <cctype>
#include <iomanip>
#include <sstream>

namespace microlind::cli {

std::optional<uint32_t> parse_number(const std::string& s) {
    try {
        size_t idx = 0;
        int base = 10;
        if (s.size() > 2 && s[0] == '0' && (s[1] == 'x' || s[1] == 'X')) {
            base = 16;
        } else if (s.size() > 1 && s[0] == '$') {
            base = 16;
            idx = 1;
        }
        return static_cast<uint32_t>(std::stoul(s.substr(idx), nullptr, base));
    } catch (...) {
        return std::nullopt;
    }
}

std::string trim(const std::string& s) {
    size_t b = 0;
    while (b < s.size() && std::isspace(static_cast<unsigned char>(s[b]))) ++b;
    size_t e = s.size();
    while (e > b && std::isspace(static_cast<unsigned char>(s[e - 1]))) --e;
    return s.substr(b, e - b);
}

bool iequals(const std::string& a, const std::string& b) {
    if (a.size() != b.size()) return false;
    for (size_t i = 0; i < a.size(); ++i) {
        if (std::toupper(static_cast<unsigned char>(a[i])) != std::toupper(static_cast<unsigned char>(b[i]))) return false;
    }
    return true;
}

std::string hex4(uint16_t v) {
    std::ostringstream oss;
    oss << std::hex << std::setw(4) << std::setfill('0') << v;
    return oss.str();
}

} // namespace microlind::cli
