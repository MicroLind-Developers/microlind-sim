# microlind-sim

Early C++ skeleton for a modular, cycle-ticked HD6309/MC6809 simulator with a pluggable bus and placeholder PAL/GAL logic support.

## Structure
- `include/microlind/` public headers for bus, CPU, clock, simulator, logic helper.
- `src/` implementations.
- `src/devices/memory.cpp` simple RAM/ROM device.
- `src/cli/main.cpp` minimal driver to tick the simulator and optionally load a ROM image.

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

## Current CPU coverage
- Core registers (plus 6309 E/F and MD), direct/extended/indexed addressing (common postbyte forms), DP register, CC flag updates for NZ and arithmetic flags on ALU ops.
- Implemented instructions with cycle counts: NOP, CLRA/CLRB, LDA/B (imm/direct/extended/indexed), LDD (imm/direct/extended/indexed), STA/B/D (direct/extended/indexed), BRA/BSR plus full conditional branches, JMP/JSR (direct/extended/indexed), RTS/RTI, SWI/SWI2/SWI3, CWAI, SYNC (stub), MUL, TFR/EXG, logical ops AND/OR/EOR/BIT on A/B (imm/direct/extended/indexed), arithmetic ADD/SUB/ADC/SBC/COMPARE on A/B (imm/direct/extended/indexed), 16-bit ADDD/SUBD/CMPD, LEA X/Y/U/S, 16-bit loads/stores for X/Y/U/S (imm/direct/extended/indexed) and compares for X/Y/U/S, stack ops PSHS/PULS/PSHU/PULU, accumulator and memory unary/shift ops (NEG/COM/LSR/ROR/ASR/ASL/ROL/DEC/INC/TST/CLR) across accumulator, direct, indexed, extended, misc ABX/SEX/ANDCC/ORCC/DAA.
- 6309 extensions wired: W/V/Q handling; LDQ/STQ, LDW/STW, ADDW/SUBW/CMPW/INCW/DECW/TSTW/CLRW; ADDE/F/SUBE/F/CMPE/F/CMPF; ADCD/SBCD/ORD; register-register ADDR/SUBR/CMPR/ADCR/SBCR/ORR; DIVQ with div-by-zero trap to FFF0; MULD; LDMD/SEXW; TFM variants; bit immediates AIM/OIM/EIM/TIM; bit transfer/logic ops BAND/BIAND/BOR/BIOR/BEOR/BIEOR/LDBT/STBT (direct).
- Unknown opcodes currently consume 1 cycle and do nothing; remaining TODO includes completing indirect indexed forms ([n,PC]/[n,R]) and PC-relative indirect, full interrupt entry/exit timing (FIRQ/NMI/IRQ vectors, reset vector), precise cycle tables for each indexed form, additional 6309 ops (DIVD, BITMD, PSHSW/PULSW/PULUW, RBIT if desired), and a proper SYNC wait model.

## Next steps
- Implement the full HD6309/MC6809 core with cycle-accurate micro-ops and per-instruction timing.
- Flesh out bus arbitration, interrupt lines, and wait states.
- Add device modules: XR88C92 serial (with terminal frontend), parallel I/O, IDE/CF, video/sound stubs.
- Model ATF22V10/ATF16V8 logic: parse equations and expose signal outputs per cycle; drive address decode and R/W logic from those chips.
- Add tests for bus overlap, memory behavior, and CPU instruction timing.
