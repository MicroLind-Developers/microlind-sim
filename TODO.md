# TODO

## GUI Debugger

- Add side-effect-free memory inspection.
  - Add `peek8()` support to `BusDevice`/`Bus`.
  - Use peek reads in the Memory, Stack, and disassembly byte views so inspecting memory-mapped I/O does not change device state.

- Polish the Memory window.
  - Add copy/paste for byte ranges.
  - Add fill range.
  - Consider save/load binary slice actions.

- Improve session and layout persistence.
  - Consider moving ImGui layout data out of the main session file into a sidecar `.imgui.ini` file.
  - Keep support for inline `LAYOUT_HEX` if useful for portable single-file sessions.
  - Persist GUI state such as memory start, row counts, stack view settings, trace options, and serial display modes.

- Improve breakpoint and watchpoint UX.
  - Add enable/disable toggles.
  - Add optional labels.
  - Add hit counts.
  - Consider separate instruction-fetch breakpoints.

- Continue disassembler coverage.
  - Add more HD6309 indexed-mode tests as edge cases appear.
  - Add tests for W-register and stack-mask oddities beyond the current coverage.

- Expand tests.
  - Add session save/load parser tests.
  - Add GUI-adjacent app-layer tests for future persisted debugger state.
  - Add memory peek tests once side-effect-free reads exist.

- Update documentation.
  - Document building and running the GUI.
  - Document the session file format.
  - Add a short debugger workflow section.
