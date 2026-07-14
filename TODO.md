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

## CPU Specification Compliance

- Improve cycle accuracy.
  - Continue auditing implemented 6809 and HD6309 cycle counts against `docs/hd6309ref.txt`.
  - Account for remaining MD native/emulation timing where the HD6309 differs.
  - Add more representative cycle-count tests as instruction coverage expands.

- Improve TFM execution accuracy.
  - Return `6 + 3n` cycles.
  - Consider modeling interruptible transfer behavior.
  - Be careful with memory-mapped I/O reads that may be repeated after an interrupt.
