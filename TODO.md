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

- Enforce micro-op parity with regular execution modes.
  - Add tests that converted micro-ops match regular execution timing in HD6309 emulation mode (`MD=0`) and native mode (`MD=1`).
  - Include converted prefixed/indexed HD6309 instructions such as `LDW/STW/LDQ/STQ` in the parity tests.

- Improve cycle accuracy.
  - Continue auditing implemented 6809 and HD6309 cycle counts against `docs/hd6309ref.txt`.
  - Account for remaining MD native/emulation timing where the HD6309 differs.
  - Add more representative cycle-count tests as instruction coverage expands.

- Complete interrupt execution.
  - Add FIRQ and NMI line support.
  - Audit HD6309 native-mode interrupt stack layout and RTI behavior.
  - Replace the current `CWAI`/`SYNC` entry-cycle support with real wait-state behavior.

- Improve TFM execution accuracy.
  - Consider modeling interruptible transfer behavior.
  - Be careful with memory-mapped I/O reads that may be repeated after an interrupt.

## Other TODOs

- Add support for changing values in:
  - Memory Mapper Registers, to be able to test RAM banking behaviour.
