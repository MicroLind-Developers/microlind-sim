# Stage-3 MLOS privilege gate and timer subset

The banked motherboard builder enables a behavioral I/O-write gate. Generic
flat-memory CPU-test buses remain unchanged unless explicitly enabled with
`Bus::enable_privilege_gate(true)`. The latch starts locked; qualified reset
vector reads unlock before ROM's first map write.

Qualified VectorRead bus cycles with BA=0/BS=1 and addresses FFFA–FFFF unlock
(plain SWI, NMI, reset, either byte). Ordinary vector-address reads/peeks,
low-byte aliases, IRQ/FIRQ/trap/SWI2/SWI3 and idle cycles do not. Privileged
writes to F400–F7FF retain ordinary semantics except F405, which consumes any
payload and relocks. Locked writes throughout that I/O range are ignored.
All reads and their normal device side effects, and ordinary/common RAM
writes, remain available. F405 read/peek bit7 is 1=privileged, 0=locked; mask
$80, since lower bits are unspecified. The simulator chooses zero lower bits.

CPU accesses expose BA/BS=0/0 normally and 0/1 for vector entry. Deferred and
microcycle accesses apply the latch/write decision at E-high/Q-low in program
order. The mode is retained independently of process state; ROM entry code
must preserve pre-SWI privilege before IRQ bridges and use explicit nested
continuations. Reading F405 after SWI cannot recover the previous privilege.

The selected XR88C92 timer subset implements ACR mode7, fixed crystal
3.6864 MHz /16, preload115 and an IRQ rising period of 2*preload: approximately
1001.739 Hz. Rational accumulation uses the simulator's CPU clock. Reading the
start command starts/restarts counting. Reading stop clears timer IRQ and
keeps periodic mode7 running; peek has no command effect. Other counter modes,
current-counter readback and UART B RX remain incomplete. This subset is for
MLOS entry/timer qualification, not a complete XR88C92 emulation.

Run `microlind-sim-tests --gtest_filter='PrivilegeGate.*:MlosTimer.*'` for the
focused regressions. The sibling software repository's `mlos/README.md` explains
its assembled target tests, native register/map/stack checks and CMOC client.

This models the contract in MLOS architecture revision0.44. The local stage-3
schematics provision U26/U27/U23 privilege signals, mapping write-enable and
status circuitry, but the retained status route and all-I/O-write equations
are unfinished. No GAL fitter result, electrical timing or physical-board
qualification is implied. See the software repository's `mlos/docs/hardware.md`
for the observed wiring constraints and board acceptance procedure.

## CF-booted runtime support

The sibling MLOS S03–S09 runtime uses `[CPU] CLOCK_HZ=3000000` (validated range
100000–100000000; default 1000000) and the same rational UART crystal timer.
Optional `[CF2]` accepts the same register-range/image/sectors/read-only keys as
`[CF]`, with a distinct CompactFlash instance. The current MLOS test manually
binds it at F640–F647; this is not expansion-card discovery or a physical IRQ
route. Config editor load/save/path resolution retains CPU and second-CF state.

UART A uses the existing TX callback. UART B has `set_tx_b_callback` and does not
feed its output into the interactive callback. B output is ready, independent
and output-only in this minimal model; B receive is not implemented. Consumers
must subscribe to B explicitly to retain logs. The target console tests capture
both sinks independently.

`CompactFlash::set_command_delay_cycles` is a deterministic test hook for READ/
WRITE command BSY latency. `tick` completes the pending command; other devices
continue independently. Reset/unload cancel pending commands. IDENTIFY is not
artificially delayed. This hook exercises target request scheduling/cancellation
and two-drive isolation, rather than modeling physical card timing or caches.

CMOC execution exposed a microcycle TFR/EXG PC ordering error. The postbyte must
advance PC before reading or replacing a transferred PC register. Incrementing
a newly written target skipped its first byte. The fix is covered for both CPU
modes and execution engines, including PC as source, destination and both.

Focused runtime tests are `MlosSerial.*`, `MlosConfig.*`, `MlosStorage.*` and
`MlosRuntimeCpu.*`, alongside `PrivilegeGate.*` and `MlosTimer.*`. The software
repository's `make -C mlos acceptance` boots real ROM/core/services/CMOC programs
and exercises CF corruption, modules, IPC, preemption, shell/files, cancellation,
read-only dirty mounts, drive removal/recovery and service-loss diagnostics in
both engines. See its `mlos/docs/runtime-stages.md` for limits and measured costs.
The full simulator suite currently passes 222/223; the pre-existing BoardLogic
representative-address fixture lacks a LOGIC section in examples/hw.cfg.
