cmake_minimum_required(VERSION 3.20)
project(microlind-sim LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(microlind-sim
    src/bus.cpp
    src/clock.cpp
    src/cpu.cpp
    src/simulator.cpp
    src/devices/memory.cpp
    src/devices/memory_mapper.cpp
    src/devices/serial.cpp
    src/logic.cpp
)

target_include_directories(microlind-sim PUBLIC include)

add_executable(microlind-sim-cli src/cli/main.cpp)
target_link_libraries(microlind-sim-cli PRIVATE microlind-sim)

option(MICROLIND_WARNINGS "Enable extra warnings" ON)
if(MICROLIND_WARNINGS)
    target_compile_options(microlind-sim PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(microlind-sim-cli PRIVATE -Wall -Wextra -Wpedantic)
endif()
