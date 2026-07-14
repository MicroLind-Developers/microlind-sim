# PLD Logic Integration Plan

## Goal

Use the hardware logic source files as executable simulator inputs, or at least as validation inputs, so the simulator can prove that its memory, I/O, and control-signal behavior matches the real board logic.

The immediate target files are:

- `examples/signal-logic.pld`
- `examples/mem-logic.pld`
- `examples/address-logic.pld`

This should not become a full CUPL/GAL fitter. The simulator only needs to parse and evaluate the subset used by these files.

The current files use the WinCUPL/CUPL-style dialect. Other common PAL/GAL tools, such as GALasm/PALasm, use a simpler but different expression syntax. The implementation should keep dialect handling explicit so WinCUPL support does not block later GALasm/PALasm support.

## Why This Is Worth Doing

- The logic chips are part of the hardware specification, not just documentation.
- Address decoding and mapper behavior are easy places for simulator drift.
- Tests can validate the simulator against the same source used for hardware logic.
- The GUI can later expose decoded chip outputs, which helps debugging BIOS and hardware configuration issues.

## Non-Goals

- Do not emulate physical propagation delays in the first implementation.
- Do not parse arbitrary CUPL syntax beyond what the project uses.
- Do not generate JEDEC files or reproduce fitter behavior.
- Do not replace the current bus map until the parsed logic is tested and trusted.

## Supported PLD Subset

Start with the WinCUPL syntax already present in the project logic files:

- Header metadata:
  - `Name`
  - `PartNo`
  - `Date`
  - `Revision`
  - `Device`
- Pin declarations:
  - `PIN 1 = A10 ;`
  - `PIN 23 = !ROM_EN ;`
- Boolean equations:
  - `OUT = A & !B # C ;`
  - Parenthesized expressions.
- Operators:
  - `!` for NOT
  - `&` for AND
  - `#` for OR
  - `()` for grouping
- Comments:
  - `/* ... */`
- Case-insensitive keywords, case-preserving signal names.

Optional later support:

- Multi-line equations.
- Registered outputs if future logic files need them.
- More CUPL operators if they appear in real hardware files.

## Current User-Facing Integration

Hardware configs may optionally reference the PLD files:

```ini
[PLD_LOGIC]
SIGNAL_LOGIC=signal-logic.pld
MEMORY_LOGIC=mem-logic.pld
ADDRESS_LOGIC=address-logic.pld
BUS_MODE=route
```

Paths are resolved relative to the `hw.cfg` file.

When a loaded hardware config contains `[PLD_LOGIC]`, simulator rebuilds validate the configured ROM, RAM, mapper, CompactFlash, and serial ranges against the decoded PLD logic. CLI builds print the validation result; GUI/session rebuilds add the result to the session log.

`BUS_MODE` may be `range`, `validate`, or `route`. `range` keeps the classic range-map-only bus. `validate` keeps range routing but records a diagnostic when PLD decode selects a different mapped device role. `route` uses PLD-selected roles for ROM, RAM, mapper, CompactFlash, serial, and decoded expansion/device selects.

The CLI also exposes two manual inspection commands:

```text
pldcfg <signal.pld> <memory.pld> <address.pld>
pldcheck <hw.cfg> <signal.pld> <memory.pld> <address.pld>
```

`pldcfg` prints a partial `hw.cfg` generated from decoded PLD ranges. It is intended for visual validation, not as a complete board config: physical RAM size, backing RAM availability, ROM images, CF image paths, and some runtime details are not encoded in the PLDs.

`pldcheck` validates an existing `hw.cfg` against the PLD decode and reports address-tagged issues.

Active-low pin declarations such as `PIN 23 = !ROM_EN` are parsed as active-low pin metadata, while equations and decode results expose logical asserted signal names such as `ROM_EN`. That means `ROM_EN == true` in the simulator means "ROM select is asserted", not "the physical pin is high".

## Dialect Strategy

Support should be dialect-aware from the beginning, even though the first working dialect is WinCUPL.

Recommended dialect enum:

```cpp
enum class LogicDialect {
    Auto,
    WinCUPL,
    GALasm,
    PALasm,
};
```

The initial implementation should support:

- `LogicDialect::WinCUPL`
- `LogicDialect::Auto`, which should detect the current project files as WinCUPL from headers such as `Name`, `Device`, and `PIN`.

Later GALasm/PALasm support should reuse the same AST and evaluator, but have its own tokenizer/parser rules. That keeps the boolean evaluation and board-level decode independent of source syntax.

Dialect differences to plan for:

- WinCUPL uses operators like `!`, `&`, and `#`.
- GALasm/PALasm commonly use different equation/operator characters and often have a simpler source structure.
- Active-low notation can differ between dialects.
- Header and pin declaration formats are not portable between dialects.

The parser should avoid normalizing source text with ad hoc replacements before tokenization. Each dialect should tokenize its own operators and declarations into the shared AST.

## Proposed Architecture

### 1. Parser Layer

Replace the current placeholder parser in `src/logic.cpp` with a real parser that produces an AST.

Suggested public types in `include/microlind/logic.hpp`:

```cpp
namespace microlind::logic {

enum class NodeKind {
    Signal,
    Not,
    And,
    Or,
};

struct ExprNode {
    NodeKind kind;
    std::string signal;
    std::vector<ExprNode> children;
};

struct Pin {
    int number{};
    std::string signal;
    bool active_low{};
};

struct Equation {
    std::string output;
    bool output_active_low{};
    ExprNode expression;
};

struct LogicDeviceDescription {
    std::string name;
    std::string device;
    std::vector<Pin> inputs;
    std::vector<Pin> outputs;
    std::vector<Equation> equations;
};

ParseResult parse_pld(std::string_view source, LogicDialect dialect = LogicDialect::Auto);

} // namespace microlind::logic
```

The exact structure can change during implementation, but the parser should preserve enough information to evaluate equations and report useful errors.

### 2. Evaluator Layer

Add an evaluator that takes signal input values and returns output values.

Suggested API:

```cpp
struct EvalContext {
    std::unordered_map<std::string, bool> signals;
};

struct EvalResult {
    std::unordered_map<std::string, bool> outputs;
};

EvalResult evaluate(const LogicDeviceDescription& device, const EvalContext& context);
```

Important behavior:

- Active-low pin declarations should be normalized clearly.
- Equations should use logical signal names, not physical voltage names, unless explicitly documented otherwise.
- Missing input signals should return a parse/evaluation error rather than silently defaulting.

### 3. Hardware Decoder Layer

Build a small board-level helper that evaluates the three logic devices together.

Suggested model:

```cpp
struct BoardSignals {
    uint16_t address{};
    bool rw{};
    bool e{};
    bool q{};
    bool ba{};
    bool bs{};
    bool breq{};
    uint8_t mapper_bits{}; // AM14-AM21 or equivalent mapper output.
};

struct DecodeResult {
    bool mem_rd{};
    bool mem_wr{};
    bool rom_en{};
    bool raml_en{};
    bool ramh_en{};
    bool ramx_en{};
    bool io_en{};
    bool map_rd{};
    bool cf_en{};
    bool ser_en{};
    bool mem_en{};
    uint8_t bank_select{};
};
```

This layer should know how to map CPU/bus state into the PLD signal names:

- Address bits: `A0` through `A15`.
- Mapper bits: `AM19`, `AM20`, `AM21` and any future bank signals.
- CPU control signals: `RW`, `E`, `Q`, `BA`, `BS`, `BREQ`.
- Derived signals from `signal-logic.pld`: `MEM_RD`, `MEM_WR`, `RD`, `WR`.
- Address decode outputs from `address-logic.pld`.
- Memory decode outputs from `mem-logic.pld`.

### 4. Validation Layer

Before using PLD evaluation in runtime bus access, add tests that compare the parsed logic against known board behavior.

Recommended tests:

- Parser accepts all three project PLD files.
- Parser rejects malformed expressions with useful error messages.
- `address-logic.pld` decodes known ranges:
  - `$F400-$F403` -> `MEM_EN`
  - `$F418-$F41F` -> `CF_EN`
  - `$F430-$F43F` -> `SER_EN`
  - `$F500-$F7FF` -> `EXP_EN`
- `mem-logic.pld` decodes:
  - ROM region.
  - low RAM region.
  - high RAM region.
  - expansion RAM region.
  - I/O region.
  - bank select behavior for reads vs writes.
- `signal-logic.pld` derives:
  - `MEM_RD`
  - `MEM_WR`
  - `RD`
  - `WR`
  - `BAVAIL`
- Board-level decode matches the current simulator map for representative addresses from `examples/hw.cfg`.

### 5. Runtime Integration

Use the PLD evaluator for validation first. Runtime integration should happen only after validation tests are stable.

Phase 1 runtime use:

- Keep current `Bus::map_device()` model.
- Use PLD decode to generate or validate mapped ranges during simulator build.
- Warn or fail when `hw.cfg` mappings disagree with PLD-derived decode.

Phase 2 runtime use:

- Add a board decoder object that can be queried on each bus access.
- Let the bus route reads/writes through decoded chip-select outputs.
- Preserve `peek8()` behavior for side-effect-free inspection.

Phase 3 runtime use:

- Expose decoded PLD outputs in the GUI.
- Add a logic analyzer/debug panel for address, control signals, and chip-select outputs.

## Implementation Milestones

### Milestone 1: Parser and AST

- Add explicit dialect selection with `LogicDialect::Auto` and `LogicDialect::WinCUPL`.
- Replace `parse_simple_logic()` with `parse_pld()`.
- Parse metadata, pins, and equations.
- Add unit tests using the three project PLD files.
- Add tests proving the current project files auto-detect as WinCUPL.
- Keep old API only if needed for compatibility.

### Milestone 2: Evaluator

- Evaluate `!`, `&`, `#`, and parentheses.
- Add missing-signal diagnostics.
- Add unit tests for small hand-written logic snippets.
- Add unit tests for selected equations from each PLD file.

### Milestone 3: Board Decode Model

- Add a board-level decoder that loads all three PLD descriptions.
- Convert bus/address/control state into input signals.
- Return a `DecodeResult`.
- Add tests for known ranges and control-signal combinations.

### Milestone 4: Simulator Validation

- Compare PLD-derived decode results with the current hardcoded simulator mapping.
- Add tests for `examples/hw.cfg`.
- Decide whether mismatches should be warnings or hard errors.

### Milestone 5: Optional Runtime Routing

- Route bus access through the board decoder.
- Keep existing map-based routing available as a fallback.
- Add GUI visibility for decoder outputs.

## Risks And Decisions

- Active-low naming can be confusing. The parser must document whether `MEM_EN` means the logical asserted signal or the physical pin level.
- The current simulator maps address ranges directly. PLD-derived decode works more like chip-select routing. The first integration should validate, not replace.
- If future PLD files use registered outputs or more CUPL features, the parser must fail loudly until support is added.
- Per-cycle timing should remain out of scope until functional decode is correct.

## Recommended Next Step

Start with Milestone 1 and Milestone 2 together:

- Implement a minimal tokenizer/parser for the current WinCUPL PLD syntax.
- Make dialect selection explicit in the API.
- Implement boolean evaluation.
- Add tests that parse and evaluate known outputs from `signal-logic.pld`, `mem-logic.pld`, and `address-logic.pld`.

That gives the project immediate value: hardware logic files become testable simulator inputs without destabilizing the bus implementation.
