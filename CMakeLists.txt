cmake_minimum_required(VERSION 3.20)
project(microlind-sim LANGUAGES CXX)

include(FetchContent)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(microlind-sim
    src/bus.cpp
    src/clock.cpp
    src/cpu.cpp
    src/cpu_instructions.cpp
    src/devices/compact_flash.cpp
    src/simulator.cpp
    src/devices/memory.cpp
    src/devices/memory_mapper.cpp
    src/devices/serial.cpp
    src/logic.cpp
)

target_include_directories(microlind-sim PUBLIC include)

add_library(microlind-sim-app
    src/app/session_file.cpp
    src/app/sim_session.cpp
    src/cli/disassembler.cpp
    src/cli/hardware_config.cpp
    src/cli/image_loader.cpp
    src/cli/sim_builder.cpp
    src/cli/util.cpp
)
target_link_libraries(microlind-sim-app PUBLIC microlind-sim)

add_executable(microlind-sim-cli
    src/cli/main.cpp
)
target_link_libraries(microlind-sim-cli PRIVATE microlind-sim-app)

include(CTest)
if(BUILD_TESTING)
    find_package(GTest CONFIG QUIET)
    if(NOT GTest_FOUND)
        set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
        set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
        FetchContent_Declare(
            googletest
            GIT_REPOSITORY https://github.com/google/googletest.git
            GIT_TAG v1.15.2
            GIT_SHALLOW TRUE
            UPDATE_DISCONNECTED TRUE
        )
        FetchContent_MakeAvailable(googletest)
    endif()
    include(GoogleTest)

    add_executable(microlind-sim-tests
        tests/test_cpu.cpp
        tests/test_sim_session.cpp
    )
    target_link_libraries(microlind-sim-tests PRIVATE
        microlind-sim-app
        GTest::gmock
        GTest::gtest_main
    )
    gtest_discover_tests(microlind-sim-tests
        WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
    )
endif()

option(MICROLIND_BUILD_GUI "Build the Dear ImGui desktop GUI" OFF)
set(MICROLIND_IMGUI_DIR "" CACHE PATH "Path to a Dear ImGui source checkout")
set(MICROLIND_IMGUI_GIT_TAG "docking" CACHE STRING "Dear ImGui git tag or branch to fetch when MICROLIND_IMGUI_DIR is empty")
set(MICROLIND_PFD_GIT_TAG "main" CACHE STRING "Portable File Dialogs git tag or branch to fetch")

if(MICROLIND_BUILD_GUI)
    find_package(SDL2 REQUIRED)
    find_package(PNG REQUIRED)

    if(NOT TARGET SDL2::SDL2)
        add_library(SDL2::SDL2 INTERFACE IMPORTED)
        set_target_properties(SDL2::SDL2 PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES "${SDL2_INCLUDE_DIRS}"
            INTERFACE_LINK_LIBRARIES "${SDL2_LIBRARIES}"
        )
    endif()

    if(MICROLIND_IMGUI_DIR)
        set(MICROLIND_RESOLVED_IMGUI_DIR "${MICROLIND_IMGUI_DIR}")
    else()
        FetchContent_Declare(
            imgui
            GIT_REPOSITORY https://github.com/ocornut/imgui.git
            GIT_TAG ${MICROLIND_IMGUI_GIT_TAG}
            GIT_SHALLOW TRUE
            UPDATE_DISCONNECTED TRUE
        )
        FetchContent_GetProperties(imgui)
        if(NOT imgui_POPULATED)
            FetchContent_Populate(imgui)
        endif()
        set(MICROLIND_RESOLVED_IMGUI_DIR "${imgui_SOURCE_DIR}")
    endif()
    if(NOT EXISTS "${MICROLIND_RESOLVED_IMGUI_DIR}/imgui.cpp")
        message(FATAL_ERROR "Dear ImGui source directory does not contain imgui.cpp")
    endif()

    add_library(microlind-imgui
        ${MICROLIND_RESOLVED_IMGUI_DIR}/imgui.cpp
        ${MICROLIND_RESOLVED_IMGUI_DIR}/imgui_draw.cpp
        ${MICROLIND_RESOLVED_IMGUI_DIR}/imgui_tables.cpp
        ${MICROLIND_RESOLVED_IMGUI_DIR}/imgui_widgets.cpp
        ${MICROLIND_RESOLVED_IMGUI_DIR}/backends/imgui_impl_sdl2.cpp
        ${MICROLIND_RESOLVED_IMGUI_DIR}/backends/imgui_impl_sdlrenderer2.cpp
    )
    target_include_directories(microlind-imgui PUBLIC
        ${MICROLIND_RESOLVED_IMGUI_DIR}
        ${MICROLIND_RESOLVED_IMGUI_DIR}/backends
    )
    target_link_libraries(microlind-imgui PUBLIC SDL2::SDL2)

    FetchContent_Declare(
        portable_file_dialogs
        GIT_REPOSITORY https://github.com/samhocevar/portable-file-dialogs.git
        GIT_TAG ${MICROLIND_PFD_GIT_TAG}
        GIT_SHALLOW TRUE
        UPDATE_DISCONNECTED TRUE
    )
    FetchContent_GetProperties(portable_file_dialogs)
    if(NOT portable_file_dialogs_POPULATED)
        FetchContent_Populate(portable_file_dialogs)
    endif()

    add_executable(microlind-sim-gui
        src/gui/gui_debugger_panels.cpp
        src/gui/gui_file_panels.cpp
        src/gui/gui_memory_panels.cpp
        src/gui/gui_panels.cpp
        src/gui/gui_state.cpp
        src/gui/main.cpp
    )
    target_include_directories(microlind-sim-gui PRIVATE ${portable_file_dialogs_SOURCE_DIR})
    target_compile_definitions(microlind-sim-gui PRIVATE MICROLIND_HAS_PORTABLE_FILE_DIALOGS=1)
    target_link_libraries(microlind-sim-gui PRIVATE microlind-sim-app microlind-imgui PNG::PNG)
endif()

option(MICROLIND_WARNINGS "Enable extra warnings" ON)
if(MICROLIND_WARNINGS)
    target_compile_options(microlind-sim PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(microlind-sim-app PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(microlind-sim-cli PRIVATE -Wall -Wextra -Wpedantic)
    if(TARGET microlind-sim-tests)
        target_compile_options(microlind-sim-tests PRIVATE -Wall -Wextra -Wpedantic)
    endif()
    if(TARGET microlind-sim-gui)
        target_compile_options(microlind-sim-gui PRIVATE -Wall -Wextra -Wpedantic)
    endif()
    if(TARGET microlind-imgui)
        target_compile_options(microlind-imgui PRIVATE -Wall -Wextra -Wpedantic)
    endif()
endif()
