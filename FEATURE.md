# Simulator GUI Features

The simulator GUI is an ImGui/SDL application that runs beside the CLI and provides an interactive debugger-oriented view of the Microlind simulator.

## Session and Files

- Load and save session files.
- Load hardware configuration, ROM images, and optional CompactFlash images.
- Support raw, Intel HEX, and S-record ROM formats.
- Save and restore GUI window/table layout with the session.
- Save and restore debugger view state, breakpoints, watchpoints, labels, and hit counts.
- Use native/portable file dialogs when available.
- About dialog with project summary, logo, and third-party library credits.

## Execution Control

- Reset the simulated machine.
- Run, pause, and single-step instructions.
- Run full instructions or individual micro-steps.
- Rate-limit Run by operations per minute with a live operations-per-second
  frequency label.
- True Run mode for 1, 2, or 3 MHz cycle-targeted execution, with debugger
  refresh paused while peripherals such as serial continue updating.
- Threaded runtime for True Run, with the GUI reading copied snapshots and
  serial/control commands queued safely to the simulator worker.
- Unified runtime mode handling for Run, Micro Run, Run Until, Step Over, True
  Run, Pause, and Reset.
- Step over subroutine calls.
- Run until return.
- Run until a chosen address.
- Keyboard shortcuts for common debugger actions.
- Bottom status bar showing run state, PC, cycle count, and active session.

## CPU and Disassembly

- Register window for CPU state and condition flags.
- Disassembly window showing address, instruction bytes, and decoded instruction text.
- Current PC highlighting.
- Improved HD6309 formatting for indexed modes, stack masks, register-pair instructions, TFM, and bit-immediate operations.

## Memory Tools

- Editable hex memory grid with ASCII view.
- Adjustable row count and scrollable table.
- Jump-to-PC, jump-to-S, and jump-to-U controls.
- Follow-PC mode.
- Highlighting for PC, stack pointers, and watchpoint addresses.
- Memory map window showing mapped address ranges.
- Stack window for following S or U stack contents.

## Breakpoints, Watchpoints, and Trace

- Add and remove breakpoints.
- Add read, write, or read/write watchpoints.
- Enable or disable breakpoints and watchpoints without deleting them.
- Add labels and inspect hit counts for breakpoints and watchpoints.
- Stop execution on watchpoint hits.
- Instruction trace window with newest entries at the top.
- Trace entries are recorded for regular stepping and for completed
  micro-stepped instructions.
- Trace cycle counts reset on reset/clear.

## Device Panels

- Memory mapper panel showing mapper presence, bank size, bank registers,
  selected banks, and visible windows.
- Editable memory mapper bank registers for testing RAM banking behavior through
  the normal bus/device path.
- CompactFlash panel showing image path, sector count, status/error registers, selected LBA, command state, and transfer progress.
- Parallel I/O panel showing W65C22 port pins, output latches, data direction
  registers, control registers, interrupt flags, and IRQ state.
- VDC Display panel composing the MOS 8563/8568 text and attribute RAM through
  its character-generator RAM into a native-pixel framebuffer at 25 Hz, with
  fit-to-panel, integer zoom, and CRT pixel-aspect display options.
- Serial panel with terminal view, raw hex view, RX text injection, RX hex-byte
  injection, TX clear, output-port byte display, and RGB power LED display.

## Diagnostics

- Log window for session actions, load/save messages, run events, and debugger notifications.
- GTest coverage for app-layer session behavior, watchpoints,
  mapper/CompactFlash/serial snapshots, micro-step trace behavior, session
  persistence, and disassembler formatting.
