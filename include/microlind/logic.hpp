#pragma once

#include <string>
#include <string_view>
#include <vector>

namespace microlind::logic {

// Placeholder for PAL/GAL logic loader. Future work will parse the provided
// equations and expose outputs for the simulator to query per cycle.
struct Equation {
    std::string output;
    std::string expression;
};

struct LogicDeviceDescription {
    std::string name;
    std::vector<std::string> inputs;
    std::vector<std::string> outputs;
    std::vector<Equation> equations;
};

LogicDeviceDescription parse_simple_logic(std::string_view source);

} // namespace microlind::logic
