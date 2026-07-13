# GUI Planning Document

This document sketches a practical path for adding a GUI to `microlind-sim`
without turning the simulator core into a frontend-specific project. The first
GUI should be a debugger/workbench for bring-up: fast stepping, register and
memory inspection, serial I/O, disk/config loading, and visibility into mapped
devices.

## Goals

- Keep the simulator core reusable by both the CLI and GUI.
- Preserve the existing CLI behavior while moving shared command/session logic
  out of `src/cli/main.cpp`.
- Provide an interactive desktop GUI for stepping, running, pausing, inspecting,
  and editing simulator state.
- Make hardware configuration, ROM/image loading, serial input, and CF image
  attachment discoverable from the GUI.
- Add device views that help debug the current microLind hardware map: memory,
  bank mapper, XR88C92 serial, and CompactFlash.
- Leave room for later visualization features such as bus traces, PAL/GAL logic
  state, interrupts, and timing analysis.

## Non-Goals for the First Version

- Cycle-perfect visual timing diagrams.
- Full source-level debugging.
- Complete peripheral emulation beyond what the simulator already models.
- A web UI or remote-control protocol.
- Replacing the CLI.

## Recommended UI Stack

Use a native C++ desktop GUI based on Dear ImGui with an SDL backend. This is
the selected direction for the first GUI.

Reasons:

- Fits a debugger/workbench better than a traditional document-style UI.
- Keeps the project in C++ and avoids introducing a second runtime.
- Provides immediate-mode widgets that are simple to wire to simulator state.
- Works well for register panes, memory hex views, logs, trace tables, popups,
  and dockable panels.
- Can be added as a separate executable, leaving `microlind-sim` as the core
  library.

Executable name:

```text
microlind-sim-gui
```

Possible dependency strategy:

- Initial local build: CMake `FetchContent` for SDL and Dear ImGui, or use
  system packages when available.
- Later packaging: vendor Dear ImGui or add a reproducible dependency manager
  once release packaging matters.

## Product Shape

The first screen should be the simulator workbench, not a landing page.

Primary layout:

- Top toolbar: open ROM, open hardware config, attach CF image, reset, step,
  run, pause, run speed, CPU mode.
- Left panel: CPU registers and condition-code flags.
- Center panel: disassembly around PC and execution controls.
- Bottom panel: serial terminal and event log.
- Right panel: memory inspector, mapped-device list, and device-specific tabs.

Expected first-version panels:

- Registers: `A`, `B`, `E`, `F`, `DP`, `CC`, `X`, `Y`, `U`, `S`, `V`, `PC`,
  total cycles, last opcode, CPU mode.
- Flags: individual `E`, `F`, `H`, `I`, `N`, `Z`, `V`, `C` bits.
- Disassembly: current PC, surrounding bytes, instruction text, step button.
- Memory: address entry, page size, hex/ASCII grid, editable bytes.
- Map: output equivalent to the CLI `map` command.
- Serial: TX transcript, RX input box, send text/send byte controls.
- Files: loaded ROM path, hardware config path, CF image path, read-only state.

## Architecture

### Current State

The simulator core is already separated as the `microlind-sim` library. The CLI
currently owns several things the GUI will also need:

- Image loading.
- Hardware config parsing.
- Simulator construction via `build_sim`.
- Disassembly helpers.
- Command behavior for step, run, tick, peek, poke, load ROM, load config,
  load CF, serial input, map, and reset.
- Serial TX callback formatting.

### Proposed Library Split

Keep low-level hardware models under `include/microlind/` and `src/`.

Move shared app/session helpers out of `src/cli/` into a reusable application
layer:

```text
include/microlind/app/
  disassembler.hpp
  hardware_config.hpp
  image_loader.hpp
  sim_builder.hpp
  sim_session.hpp

src/app/
  disassembler.cpp
  hardware_config.cpp
  image_loader.cpp
  sim_builder.cpp
  sim_session.cpp
```

Then make both frontends thin:

```text
src/cli/main.cpp
src/gui/main.cpp
```

The CLI can continue to parse command lines and print text, while the GUI can
call the same session operations and render results interactively.

### Session API

Add a `SimSession` class that owns user-facing simulator state:

- CPU mode.
- Loaded ROM/image metadata.
- Loaded hardware config.
- Optional CF image metadata.
- The `Simulator` instance.
- Pointers or handles for observable devices such as serial.
- Log messages and recent execution results.

Candidate operations:

```cpp
class SimSession {
public:
    bool load_rom(const std::filesystem::path& path, RomFormat format, uint16_t base);
    bool load_hardware_config(const std::filesystem::path& path);
    bool load_cf_image(const std::filesystem::path& path, uint32_t min_sectors);

    void reset();
    CpuTickResult step_instruction();
    void step_instructions(uint32_t count);
    void tick_cycles(uint64_t cycles);

    uint8_t read_memory(uint16_t address) const;
    void write_memory(uint16_t address, uint8_t value);

    void inject_serial_text(std::string_view text);
    std::vector<std::string> memory_map() const;

    const Simulator& simulator() const;
    Simulator& simulator();
};
```

The exact return types should carry error messages without forcing UI code to
parse printed text. A small `Result<T>` type, `std::expected` if available, or
`std::optional<T>` plus error strings would all be acceptable.

### GUI Runtime Model

Run the simulator on the GUI thread at first. This keeps the first version
simple and avoids data races.

Execution modes:

- Paused: UI is fully interactive.
- Step: execute one instruction and refresh panels.
- Run fixed batch: execute a bounded number of instructions per frame.
- Run by wall-clock budget: execute until a configurable frame budget is spent.

Important constraint:

- Do not run the CPU in an unbounded loop inside a frame. The UI must remain
  responsive and pause must take effect quickly.

Later, a worker thread can be introduced for long-running simulation. That will
require snapshotting, command queues, and clear ownership rules around `Bus`,
devices, and logs.

## Core Changes Needed

### Simulator Observability

The GUI should not scrape terminal output. Add structured access where needed:

- Expose CPU register snapshots.
- Expose total cycles, last PC, last opcode, and disassembly text.
- Return memory map entries as structured data, not only formatted strings.
- Provide safe memory read/write helpers for inspection.
- Let devices emit events through callbacks or an event sink.

### Serial Device

The current XR88C92 model already supports RX injection and a TX callback.
For the GUI:

- Capture TX bytes into a serial transcript buffer.
- Preserve byte values, not just printable characters.
- Add controls for text input and raw hex input.
- Consider exposing channel status in a small device panel.

### CompactFlash Device

The current CF model can load an image through config/session rebuild. For the
GUI:

- Display current image path, sector count, read-only state, and status/error
  registers if exposed.
- Start with attach/reload image support.
- Later add sector inspection and command trace.

### Memory Mapper

The GUI should make bank state visible because the hardware config includes
mapper registers.

Initial panel:

- Show configured bank registers.
- Show current selected bank values if `MapperState` can expose them.
- Show RAM window ranges and available backing memory.

This may require adding read-only accessors to `MapperState`.

## Build Plan

1. Create a reusable app library target.

```cmake
add_library(microlind-sim-app
    src/app/disassembler.cpp
    src/app/hardware_config.cpp
    src/app/image_loader.cpp
    src/app/sim_builder.cpp
    src/app/sim_session.cpp
)
target_link_libraries(microlind-sim-app PUBLIC microlind-sim)
```

2. Update the CLI to link against `microlind-sim-app`.

3. Add the GUI target.

```cmake
add_executable(microlind-sim-gui
    src/gui/main.cpp
)
target_link_libraries(microlind-sim-gui PRIVATE microlind-sim-app)
```

4. Add GUI dependencies after choosing the dependency strategy.

## Milestones

### Milestone 1: Shared Session Layer

- Move reusable CLI helpers from `src/cli/` to `src/app/`.
- Add public headers under `include/microlind/app/`.
- Add `SimSession`.
- Keep the CLI behavior unchanged.
- Add focused tests for image loading, config loading, session rebuild, memory
  read/write, and serial injection.

Exit criteria:

- `microlind-sim-cli` still builds and supports the same commands.
- Session operations can be called without using stdin/stdout.

### Milestone 2: Minimal GUI Shell

- Add `microlind-sim-gui`.
- Create the main window, menu bar, toolbar, and dockable layout.
- Load ROM and hardware config from file dialogs.
- Show registers, flags, total cycles, and current PC.
- Implement reset, step, run, and pause.

Exit criteria:

- A user can load `examples/hw.cfg` and `examples/bios.ihex`, step code, and
  watch register/cycle changes.

### Milestone 3: Inspection Panels

- Add disassembly panel.
- Add memory hex viewer with address navigation.
- Add editable memory cells.
- Add mapped-device panel.
- Add serial terminal with RX input and TX transcript.

Exit criteria:

- The GUI covers the core CLI debugging loop: `regs`, `step`, `run`, `peek`,
  `poke`, `dump`, `serin`, `map`, and `reset`.

### Milestone 4: Device Views

- Add CompactFlash status panel and image attach/reload flow.
- Add memory mapper panel.
- Add basic event log for serial TX, CF commands, reset, file loads, and errors.

Exit criteria:

- Device state that matters during microLind bring-up is visible without adding
  temporary `std::cout` instrumentation.

### Milestone 5: Debugger Features

- Add breakpoints by address.
- Add run-until-address.
- Add watch expressions for memory addresses.
- Add instruction trace buffer.
- Add exportable log/trace files.

Exit criteria:

- The GUI can support normal firmware bring-up sessions without falling back to
  the CLI for every investigation.

## Testing Strategy

- Keep simulator and session logic testable without GUI dependencies.
- Add regression tests around `SimSession` rather than testing ImGui widgets.
- Add a smoke test that starts the GUI target if the environment supports it.
- Keep CLI tests where possible so both frontends prove they share behavior.
- Use sample files in `examples/` for integration tests, but avoid modifying
  user disk images during tests.

## Current Implementation Status

- `microlind-sim-gui` exists as a separate executable beside the CLI.
- CMake fetches Dear ImGui into the build tree when `MICROLIND_BUILD_GUI=ON`;
  `MICROLIND_IMGUI_DIR` can still point at a local checkout.
- SDL2 is discovered through CMake.
- `microlind-sim-app` shares loader, config, builder, disassembler, and session
  code between frontends.
- `SimSession` owns simulator state, ROM/config/CF loading, reset/step/run
  operations, memory access, serial RX/TX capture, and app log messages.
- The first GUI workbench supports text-path ROM/config/CF loading, run/pause,
  browse dialogs, step, step-over, reset, registers, flags, disassembly, memory
  read/write, serial I/O, stack inspection, breakpoints, watchpoints,
  run-until-address, run-until-return, instruction trace, memory mapper display,
  mapped-device display, and logs.

## Open Questions

- Should the first GUI support only Linux, or should Windows/macOS portability
  be part of the first implementation?
- Should long-running simulation stay single-threaded for longer, or should a
  worker-thread model be introduced before breakpoints and tracing?
- How much device internals should be exposed through public headers versus a
  narrower observer/event API?
- Should file loading stay as text-path entry fields for now, or should the GUI
  add a native/portable file dialog dependency?

## Suggested First Implementation Task

Continue by finishing the app-layer split and adding richer device visibility.
The next useful patch is:

- Move the remaining shared helper headers from `src/cli/` into
  `include/microlind/app/`.
- Add structured device-state accessors for CF and memory mapper panels.
- Add richer memory mapper controls and structured map entries.
- Add focused tests around `SimSession`.
