# TODO

## GUI Debugger

- Polish the Memory window.
  - Add copy/paste for byte ranges.
  - Add fill range.
  - Consider save/load binary slice actions.

- Improve session and layout persistence.
  - Consider moving ImGui layout data out of the main session file into a sidecar `.imgui.ini` file.
  - Keep support for inline `LAYOUT_HEX` if useful for portable single-file sessions.
  - Persist any future GUI state such as trace filters/options as those controls are added.

- Improve breakpoint and watchpoint UX.
  - Consider separate instruction-fetch breakpoints.

- Continue disassembler coverage.
  - Add more HD6309 indexed-mode tests as edge cases appear.

- Update documentation.
  - Document building and running the GUI.
  - Document the session file format.
  - Add a short debugger workflow section.
