#include "microlind/logic.hpp"

#include <sstream>

namespace microlind::logic {

LogicDeviceDescription parse_simple_logic(std::string_view source) {
    LogicDeviceDescription desc;
    desc.name = "anonymous";

    std::istringstream ss{std::string(source)};
    std::string line;
    while (std::getline(ss, line)) {
        auto comment_pos = line.find("/*");
        if (comment_pos != std::string::npos) {
            line = line.substr(0, comment_pos);
        }
        auto semicolon = line.find(';');
        if (semicolon != std::string::npos) {
            line = line.substr(0, semicolon);
        }
        if (line.find('=') != std::string::npos) {
            auto eq_pos = line.find('=');
            std::string output = line.substr(0, eq_pos);
            std::string expr = line.substr(eq_pos + 1);

            // trim whitespace
            auto trim = [](std::string& s) {
                const char* ws = " \t\r\n";
                const auto start = s.find_first_not_of(ws);
                const auto end = s.find_last_not_of(ws);
                if (start == std::string::npos || end == std::string::npos) {
                    s.clear();
                    return;
                }
                s = s.substr(start, end - start + 1);
            };

            trim(output);
            trim(expr);
            if (!output.empty() && !expr.empty()) {
                desc.equations.push_back({output, expr});
            }
        }
    }

    return desc;
}

} // namespace microlind::logic
