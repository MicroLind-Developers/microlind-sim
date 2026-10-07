# GUI Runtime Threading Plan

This plan describes how to move the GUI from direct, single-threaded simulator
driving to a unified runtime controller that can support both responsive
high-speed True Run mode and full debugger mode.

## Goals

- Keep the GUI responsive while True Run targets 1, 2, or 3 MHz.
- Keep simulator state owned by exactly one execution context at a time.
- Preserve current debugger behavior: breakpoints, watchpoints, trace, logs,
  memory inspection, disassembly, and step controls.
- Let True Run avoid expensive live debugger work while still updating live
  peripheral-facing state such as serial output.
- End with one execution interface for Step, Micro Step, Run, Run Until, Step
  Over, True Run, Pause, Reset, and device actions.

## Non-Goals

- Do not parallelize CPU/bus/device execution internally. The simulated machine
  is sequential and should remain deterministic.
- Do not let ImGui read mutable simulator internals while another thread is
  executing the simulator.
- Do not make every debugger panel live-refresh at MHz speed. Heavy snapshots
  should be explicit and throttled.

## Original State

- The GUI owns `GuiState`, which contains a `SimSession`.
- Normal run, step, micro-step, run-until, trace recording, breakpoint checks,
  watchpoint checks, and panel reads happen on the GUI thread.
- True Run currently uses `SimSession::run_realtime_cycles()` on the GUI thread
  with a small per-frame work budget and `SDL_Delay(1)` to avoid hard lockups.
- Many panels read the simulator directly: registers, memory, stack,
  disassembly, mapper, CompactFlash, PLD decode, trace, log, and serial.

## Target Architecture

Add a `GuiRuntime` or `SimRunner` layer between the GUI and `SimSession`.

The runtime owns:

- One `SimSession`.
- Optional worker thread.
- Command queue from GUI to runtime.
- Snapshot cache from runtime to GUI.
- Runtime state such as mode, target frequency, effective frequency, pause
  state, and pending command status.

The GUI owns:

- SDL and ImGui.
- View/layout state.
- Input widgets and modal state.
- The latest immutable runtime snapshots.

The runtime is the only layer allowed to mutate `SimSession`.

## Data Model

### Runtime Mode

Use one enum for execution state:

```cpp
enum class RuntimeMode {
    Paused,
    DebugRun,
    DebugMicroRun,
    RunUntilAddress,
    RunUntilReturn,
    StepPending,
    MicroStepPending,
    StepOverPending,
    TrueRun,
    Stopping,
};
```

The exact names can change, but the key point is that execution state should
collapse into one runtime-owned state machine rather than scattered GUI booleans.

### Commands

Commands should be value types, queued by the GUI and consumed by the runtime:

- `LoadSession`
- `SaveSession`
- `LoadHardwareConfig`
- `LoadRom`
- `AttachCompactFlash`
- `RemoveCompactFlash`
- `Reset`
- `Pause`
- `StepInstruction`
- `StepMicrocycle`
- `StepOver`
- `Run`
- `RunMicro`
- `RunUntilAddress`
- `RunUntilReturn`
- `TrueRun`
- `SetTrueClockHz`
- `InjectSerialBytes`
- `WriteMemory`
- `WriteMapperRegister`
- `AddBreakpoint`
- `RemoveBreakpoint`
- `EditBreakpoint`
- `AddWatchpoint`
- `RemoveWatchpoint`
- `EditWatchpoint`
- `ClearTrace`
- `ClearLog`

Some commands can complete synchronously while paused. Commands that touch
files or mutate simulator state should still go through the runtime so the
ownership rule stays simple.

### Snapshots

Use immutable snapshot structs copied from the runtime to the GUI.

Small live snapshot, updated often:

- Runtime mode.
- PC.
- Total cycles.
- Bus cycle count.
- Effective True Run Hz.
- Serial TX buffer or appended serial bytes.
- Serial LED/IRQ state.
- Compact status summary.

Full debugger snapshot, updated when paused or at a low rate in debug mode:

- CPU registers and flags.
- Disassembly window data.
- Memory viewer data.
- Stack view data.
- Memory map.
- Mapper snapshot.
- CompactFlash snapshot.
- PLD decode snapshot.
- Breakpoints/watchpoints.
- Trace.
- Log.

Memory-heavy views should be demand-driven: the GUI requests the current
visible address range instead of copying all memory every frame.

## Phase 1: Extract Runtime Facade Without Threading

Status: implemented. `GuiRuntime` now owns `SimSession`, routes the main GUI
execution calls, and provides the first status snapshot for the bottom status
bar. The temporary direct-session bridge has since been retired from GUI panel
code.

Create `GuiRuntime` as a single-threaded facade around `SimSession`.

Work:

- Move direct execution calls out of `GuiState` into `GuiRuntime`.
- Keep everything on the GUI thread.
- Introduce command methods, but execute them immediately.
- Introduce snapshot structs, but build them synchronously from `SimSession`.
- Keep existing panels working.
- Keep session save/load behavior unchanged.

Expected result:

- No performance change yet.
- Cleaner boundary: GUI no longer calls `SimSession::run_*()` directly.
- Easier tests for GUI-adjacent runtime behavior without SDL/ImGui.

Tests:

- Runtime step updates PC/cycles and trace.
- Runtime reset clears trace/cycles.
- Runtime attach/remove CF updates snapshots and bus behavior.
- Runtime breakpoint/watchpoint commands update debugger state.

## Phase 2: Snapshot-Driven Panels

Status: implemented. Panels now read displayed simulator state through runtime
snapshots and range requests. Panel mutations, session persistence, debugger
edits, memory writes, and CPU-mode changes are routed through `GuiRuntime`
methods.

Convert panels to read snapshots instead of live `SimSession` wherever
practical.

Work:

- Add `RuntimeSnapshot` for status/control/serial.
- Add `DebuggerSnapshot` for registers, trace, mapper, CF, PLD, log, etc.
- Keep memory and disassembly as range requests.
- Update panels to use snapshot data.
- Leave writes and commands routed through `GuiRuntime`.

Expected result:

- GUI behavior remains the same.
- The GUI is no longer structurally dependent on direct mutable simulator
  access for most panels.

Tests:

- Snapshot content matches current `SimSession` state after representative
  commands.
- Memory/disassembly range requests use peek reads and have no side effects.

## Phase 3: Threaded True Run

Status: implemented as the first threaded slice. True Run now runs on a
`GuiRuntime` worker thread with locked status/debugger snapshots, runtime-owned
target/effective Hz tracking, and thread-safe serial RX/TX helpers. The GUI
thread no longer performs True Run cycle batches directly.

Move only True Run execution to a worker thread.

Work:

- Add worker lifecycle to `GuiRuntime`.
- Worker owns execution while in `RuntimeMode::TrueRun`.
- GUI sends `TrueRun`, `Pause`, `SetTrueClockHz`, `InjectSerialBytes`, and
  selected device commands through the command queue.
- Worker runs a timing loop with target Hz and effective Hz measurement.
- Worker publishes small live snapshots at a fixed rate, for example 20-60 Hz.
- On `Pause`, worker stops at an instruction boundary, publishes a full
  debugger snapshot, and returns to paused mode.

Important behavior:

- Breakpoints/watchpoints are not checked in True Run unless explicitly enabled
  later as an optional slower mode.
- Trace is not recorded in True Run.
- Bus/decode/access logs are cleared or disabled during True Run to avoid
  memory growth.
- Serial output remains live through appended bytes or periodic serial
  snapshots.

Tests:

- True Run worker advances cycles while GUI thread can poll snapshots.
- Pause returns a stable full snapshot.
- Serial injection while True Run is active reaches the device.
- Remove/attach CF while True Run is active is serialized safely.
- No direct data race under thread sanitizer when available.

## Phase 4: Thread-Safe Command Queue

Status: implemented as a True Run command queue slice. `GuiRuntime` now owns a
mutex/condition-variable command queue used while the worker is active for
target clock changes, serial RX injection, serial/log clears, and log messages.
The worker drains commands between execution batches, and tests cover command
ordering plus serial injection during True Run. File/device load operations
still pause True Run first and then execute synchronously through the runtime
path; the wider all-command state machine remains Phase 5/7 work.

Make the command model robust enough for all simulator mutations.

Work:

- Implement a mutex/condition-variable command queue.
- Define command completion results for commands that can fail.
- Add a small UI-visible command status/error log.
- Ensure file operations happen in the runtime path and return status.
- Add a clean shutdown command so the worker always joins before SDL teardown.

Expected result:

- GUI can request simulator actions without knowing whether the runtime is
  currently threaded or paused.
- All simulator mutation paths are serialized.

Tests:

- Commands execute in order.
- Pause/Reset during True Run cannot interleave with a half-applied command.
- Runtime shutdown joins the worker cleanly.

## Phase 5: Move Debug Run Onto Runtime State Machine

Status: implemented for GUI-thread debugger execution. Normal Run, Micro Run,
Run Until, Step Over, and Run Until Return now start through `GuiRuntime` mode
methods and advance through one `run_debug_batch()` path. `GuiState` no longer
stores separate `running`, `run_until_active`, or `true_running` booleans; panel
labels and the main loop derive execution state from `RuntimeMode`. Debugger
execution is still intentionally on the GUI thread; moving it to the worker
remains optional Phase 6 work.

Unify normal Run, Micro Run, Run Until, Step Over, and True Run behind the same
runtime state machine.

Recommended staging:

- Keep debugger execution on the GUI thread initially, but call it through the
  same `GuiRuntime::update()` method.
- Once snapshot handling is solid, optionally let debug run use the worker too.

Debug run behavior:

- Breakpoints and watchpoints remain active.
- Trace remains active.
- Log/decode diagnostics remain active.
- Full debugger snapshots can update each GUI frame or after each run batch.
- Run rate remains operations/minute.

Expected result:

- `GuiState` no longer has separate booleans for every run mode.
- Control panel and menu commands all use one mode/status model.
- True Run and normal Run differ by execution policy, not by completely
  separate plumbing.

Tests:

- Existing execution tests still pass.
- Regular run through runtime matches direct `SimSession` behavior.
- Micro run through runtime records trace on completed instructions.
- Run-until and step-over stop at the same addresses as today.

## Phase 6: Optional Worker For Debug Run

Status: decided/deferred. Normal debug Run, Micro Run, Run Until, Step Over,
and Run Until Return now share the runtime state machine, but still execute
small batches on the GUI thread. This keeps breakpoint/watchpoint checks, trace
updates, and full debugger refresh simple and deterministic. True Run remains
the threaded high-speed path. A debug worker can still be added later if long
run-until operations prove to be a responsiveness problem.

After Phase 5, decide whether normal debug run should also execute on the
worker.

Reasons to do it:

- GUI remains responsive during long run-until operations.
- One threading model for every run mode.
- Easier to add cancel/pause for long debugger runs.

Reasons to wait:

- More synchronization around trace/log/break/watch updates.
- More snapshot traffic.
- More complex tests.

If implemented:

- Debug worker should run small batches.
- Publish debugger snapshots after each batch.
- Honor pause/cancel quickly.
- Keep breakpoint/watchpoint checks inside the runtime thread.

## Phase 7: Cleanup

Status: implemented for the threading milestone. `GuiState` keeps visual
settings and input fields, while runtime-only state such as mode, debug-run
rate, micro-step run preference, target/effective True Run frequency,
break/watch state, memory writes, trace clears, and CPU-mode changes live
behind `GuiRuntime`. Session persistence reads runtime preferences through
runtime APIs instead of direct `SimSession` access. README, FEATURE, and this
plan document the threaded runtime behavior.

Once unified mode handling is stable:

- Continue shrinking convenience wrappers such as `running()`,
  `run_until_active()`, and `true_running()` as panel code becomes more
  snapshot-driven.
- Keep purely visual settings and text edit buffers in `GuiState`.

## Locking Rules

- `SimSession` is never accessed directly by ImGui panel rendering once the
  threaded runtime exists.
- Runtime thread mutates `SimSession`.
- GUI thread reads only copied snapshots.
- Commands are copied into a queue.
- Snapshots are swapped under a small mutex or atomically through shared
  immutable pointers.
- No locks should be held while running CPU instructions.
- No locks should be held while rendering ImGui.

## Performance Notes

- True Run should use cycle budgeting, not operation budgeting.
- If the host cannot keep the target MHz, drop effective Hz rather than
  accumulating unbounded catch-up work.
- Publish snapshots at a human/UI rate, not every instruction.
- Serial output should be incremental to avoid copying the full TX buffer every
  snapshot.
- Memory/disassembly should be range-based and refreshed on demand.

## Risk Areas

- Data races from accidental direct `SimSession` reads in panels.
- Deadlocks during shutdown or file dialogs.
- Commands that need a return value while the runtime is busy.
- Loading/removing CF media while the CPU is using the CF registers.
- True Run pause timing: pause should happen at a deterministic boundary.
- Snapshot staleness: the UI should make it clear when it is showing live,
  paused, or last-known state.

## Suggested First Implementation Slice

Start with Phase 1 only:

1. Add `src/gui/gui_runtime.hpp` and `src/gui/gui_runtime.cpp`.
2. Move `SimSession session` from `GuiState` into `GuiRuntime`.
3. Give `GuiState` a `GuiRuntime runtime`.
4. Add synchronous runtime methods matching current GUI actions.
5. Update panels to read snapshots and route commands through `GuiRuntime`.
6. Add first runtime snapshot struct and use it in the status bar.
7. Build and run the full test suite.

This gives us a safe compileable checkpoint before introducing threads.
