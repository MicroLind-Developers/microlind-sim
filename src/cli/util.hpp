#pragma once

#include <cstdint>
#include <optional>
#include <string>

namespace microlind::cli {

std::optional<uint32_t> parse_number(const std::string& s);
std::string trim(const std::string& s);
bool iequals(const std::string& a, const std::string& b);
std::string hex4(uint16_t v);

} // namespace microlind::cli
