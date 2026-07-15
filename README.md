# microlind-sim

Early C++ skeleton for a modular, cycle-ticked HD6309/MC6809 simulator with a pluggable bus.

![Microlind Simulator GUI](resources/screen.png)

## Structure
- `include/microlind/` public headers for bus, CPU, clock, simulator, logic helper.
- `src/` implementations.
- `src/devices/memory.cpp` simple RAM/ROM device.
- `src/devices/compact_flash.cpp` minimal CF-ATA storage device.
- `src/cli/` interactive CLI, image loading, hardware config parsing, and simulator setup.

## Build
```
cmake -S . -B build
cmake --build build
```
Run the CLI:
```
./build/microlind-sim-cli --6309 --rom path/to/bios.rom
./build/microlind-sim-cli --6309 --ihex --rom bios.hex
./build/microlind-sim-cli --6309 --srec --rom bios.s19
```

## GUI
An experimental Dear ImGui/SDL GUI is available as a separate executable beside
the CLI. It is disabled by default so the core simulator can still build without
GUI dependencies installed.

To build it, install SDL2 development files. CMake will fetch Dear ImGui into
the build tree:

```
cmake -S . -B build -DMICROLIND_BUILD_GUI=ON
cmake --build build --target microlind-sim-gui
```

You can use a local Dear ImGui checkout instead with
`-DMICROLIND_IMGUI_DIR=/path/to/imgui`, or change the fetched branch/tag with
`-DMICROLIND_IMGUI_GIT_TAG=...`.

The initial GUI is a simulator workbench with text path fields and browse
dialogs for ROM, hardware config, and CF image loading; run, pause, step,
step-over, run-until-address, run-until-return, registers, flags, disassembly,
memory inspection/write, stack inspection, serial RX/TX, breakpoints,
watchpoints, memory mapper display, instruction trace, mapped-device display,
and an event log.

## Hardware config
`examples/hw.cfg` maps the current microLind memory and I/O layout: ROM, RAM,
serial, CompactFlash, memory mapper windows, and optional PLD logic routing.
See [docs/hardware-config.md](docs/hardware-config.md) for the full syntax.

A minimal CompactFlash section looks like:

```
[CF]
IO_START_ADDRESS=0xF418
IO_END_ADDRESS=0xF41F
SECTORS=2048
IMAGE=cf.img
READ_ONLY=false
```

`IMAGE`, `SECTORS`, and `READ_ONLY` are optional. Without an image path, the CF
device uses volatile zero-filled storage. With `IMAGE`, the image size becomes
the disk size unless `SECTORS` is set as a larger minimum. The current model
implements the 8-byte ATA/CF register window, `IDENTIFY DEVICE`, PIO read/write
sector commands, read/write multiple, erase sectors, read verify, set features,
set multiple mode, diagnostics, and common idle/standby/check-power commands.

Raw disk images, such as files created with `dd`, can also be loaded at runtime:

```
loadcf path/to/cf.img
loadcf path/to/cf.img 4096
```

The optional sector count is a minimum size. Images that are not an exact multiple of 512 bytes are padded to the next sector.

### PLD logic
`hw.cfg` may optionally reference the board PLD source files. Relative paths are
resolved from the config file location:

```
[PLD_LOGIC]
SIGNAL_LOGIC=signal-logic.pld
MEMORY_LOGIC=mem-logic.pld
ADDRESS_LOGIC=address-logic.pld
BUS_MODE=route
```

When this section is present, simulator rebuilds validate configured ROM, RAM,
memory mapper, CompactFlash, and serial ranges against the decoded PLD logic.
The CLI prints validation diagnostics; GUI/session rebuilds add them to the
event log.

`BUS_MODE` controls how decoded PLD selects interact with the simulator bus:
`validate` keeps the normal range map and logs mismatches, `route` uses the PLD
selected device role for bus accesses, and `range` disables live PLD bus decode.
The example config uses `route`.

Two CLI commands are available for visual validation:

```
pldcfg examples/signal-logic.pld examples/mem-logic.pld examples/address-logic.pld
pldcheck examples/hw.cfg examples/signal-logic.pld examples/mem-logic.pld examples/address-logic.pld
```

`pldcfg` prints a partial `hw.cfg` generated from decoded PLD address ranges.
`pldcheck` validates an existing hardware config against the PLD decode. The
current parser supports the WinCUPL subset used by the project PLDs, including
`PIN` declarations, active-low `!` names, simple sum-of-products equations, and
binary/hex constants. Parsed active-low outputs are exposed as asserted logical
signals. See `docs/logic-plan.md` for details and parser limits.

## Current CPU coverage
- Core registers (plus 6309 E/F and MD), direct/extended/indexed addressing (common postbyte forms), DP register, CC flag updates for NZ and arithmetic flags on ALU ops.
- Implemented instructions with cycle counts: NOP, CLRA/CLRB, LDA/B (imm/direct/extended/indexed), LDD (imm/direct/extended/indexed), STA/B/D (direct/extended/indexed), BRA/BSR plus full conditional branches, JMP/JSR (direct/extended/indexed), RTS/RTI, SWI/SWI2/SWI3, CWAI, SYNC (stub), MUL, TFR/EXG, logical ops AND/OR/EOR/BIT on A/B (imm/direct/extended/indexed), arithmetic ADD/SUB/ADC/SBC/COMPARE on A/B (imm/direct/extended/indexed), 16-bit ADDD/SUBD/CMPD, LEA X/Y/U/S, 16-bit loads/stores for X/Y/U/S (imm/direct/extended/indexed) and compares for X/Y/U/S, stack ops PSHS/PULS/PSHU/PULU, accumulator and memory unary/shift ops (NEG/COM/LSR/ROR/ASR/ASL/ROL/DEC/INC/TST/CLR) across accumulator, direct, indexed, extended, misc ABX/SEX/ANDCC/ORCC/DAA.
- 6309 extensions wired: W/V/Q handling; LDQ/STQ, LDW/STW, ADDW/SUBW/CMPW/INCW/DECW/TSTW/CLRW; ADDE/F/SUBE/F/CMPE/F/CMPF; ADCD/SBCD/ORD; register-register ADDR/SUBR/CMPR/ADCR/SBCR/ORR; DIVQ with div-by-zero trap to FFF0; MULD; LDMD/SEXW; TFM variants; bit immediates AIM/OIM/EIM/TIM; bit transfer/logic ops BAND/BIAND/BOR/BIOR/BEOR/BIEOR/LDBT/STBT (direct).
- Unknown opcodes currently consume 1 cycle and do nothing; remaining TODO includes FIRQ/NMI line support, HD6309 native-mode interrupt/RTI details, precise cycle tables for each indexed form, additional 6309 edge cases as they appear, and proper CWAI/SYNC wait-state behavior. The microLind `$F404` IRQ register is modeled as a board-level IRQ source for instruction-boundary IRQ service and resumable IRQ stack/vector microcycles.

## Next steps
- Implement the full HD6309/MC6809 core with cycle-accurate micro-ops and per-instruction timing.
- Flesh out bus arbitration, FIRQ/NMI lines, and wait states.
- Add device modules: parallel I/O, video/sound stubs, and deeper CF timing/interrupt behavior.
- Model ATF22V10/ATF16V8 logic: parse equations and expose signal outputs per cycle; drive address decode and R/W logic from those chips.
- Add tests for bus overlap, memory behavior, and CPU instruction timing.
