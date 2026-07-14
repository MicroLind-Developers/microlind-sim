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

- Complete indexed addressing execution coverage.
  - Add E-register and F-register offset forms.
  - Add W-register offset forms such as `W,R` and `[W,R]`.
  - Add W-relative forms such as `,W`, `n,W`, `,W++`, `,--W`, and indirect variants.
  - Replace fallback-to-base behavior for unsupported postbytes with a trap or explicit unsupported handling.

- Improve cycle accuracy.
  - Audit implemented 6809 and HD6309 cycle counts against `docs/hd6309ref.txt`.
  - Account for MD native/emulation timing where the HD6309 differs.
  - Fix known mismatches such as `LDB indexed`, `JMP indexed`, and `DIVQ immediate`.
  - Model indexed-mode cycle additions more precisely.

- Improve TFM execution accuracy.
  - Return `6 + 3n` cycles.
  - Consider modeling interruptible transfer behavior.
  - Be careful with memory-mapped I/O reads that may be repeated after an interrupt.

- Expand CPU execution tests.
  - Add GTest coverage for divide-by-zero traps.
  - Add effective-address tests for E/F/W indexed modes.
  - Add focused flag/result tests for arithmetic, logic, shifts, stack ops, and 6309 extensions.
  - Add cycle-count tests for representative instructions and indexed-mode variants.
