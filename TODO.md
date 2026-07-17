# TODO

## GUI Debugger

- Implement unified GUI runtime handling.
  - Follow `docs/gui-runtime-threading-plan.md`.
  - Done: added the single-threaded `GuiRuntime` facade.
  - Done: converted panel display reads to runtime snapshots/range requests.
  - Done: moved True Run to a worker thread with queued runtime commands.
  - Done: unified normal debugger run modes behind `GuiRuntime` state.
  - Done: routed GUI panel mutations through runtime APIs instead of direct `SimSession` access.
  - Future option: move debug Run/Run Until to a worker if long operations need cancellation/responsiveness improvements.

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
  - Document the session file format.
  - Add a short debugger workflow section.

## CPU Specification Compliance

- Keep micro-op parity with regular execution modes.
  - Add parity tests when new CPU instructions or timing fixes are added.
  - Keep checking 6809 mode, HD6309 emulation mode, and HD6309 native mode.

- Continue CPU implementation cleanup.
  - Consider splitting the private `cpu_detail.hpp` helper set into smaller focused helper headers if it keeps growing.
  - Keep `cpu.cpp`, `cpu_instructions.cpp`, and `cpu_micro_ops.cpp` aligned around shared helper behavior.

- Improve cycle accuracy.
  - Continue auditing implemented 6809 and HD6309 cycle counts against `docs/hd6309ref.txt`.
  - Account for remaining MD native/emulation timing where the HD6309 differs.
  - Add more representative cycle-count tests as instruction coverage expands.
  - Done: corrected and covered documented `JSR` direct/indexed/extended timing.

- Complete interrupt execution.
  - Compare `CWAI`/`SYNC` wake-up timing against hardware traces when available.

- Improve TFM execution accuracy.
  - Consider modeling interruptible transfer behavior.
  - Be careful with memory-mapped I/O reads that may be repeated after an interrupt.

## Device Models

- Add deeper Serial/XR88C92 behavior as software needs it.
  - Model more channel B behavior if used.
  - Audit interrupt/status register details against the datasheet.

- Add device modules for remaining board peripherals.
  - Parallel I/O.
  - Video/sound stubs.
