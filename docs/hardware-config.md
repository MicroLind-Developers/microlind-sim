# Hardware Config Reference

`hw.cfg` describes the simulator's board-level memory map and optional PLD
logic integration. The current example is `examples/hw.cfg`.

## Format

The file is an INI-style text file:

```ini
[SECTION]
KEY=value
```

Section and key names are case-insensitive. Blank lines are ignored. Lines that
start with `#` or `;` are comments. Numeric values may be decimal, `0x` hex, or
`$` hex.

Relative paths are resolved from the directory containing the config file.

Inline comments are safest after numeric values only. Path values should not
have trailing comments because the parser treats the whole value as the path.

Unknown sections and unknown keys are ignored.

## ROM

Each `[ROM]` section defines one read-only mapped range. Multiple `[ROM]`
sections are allowed.

```ini
[ROM]
START=0xE000
END=0xF3FF

[ROM]
START=0xF800
END=0xFFFF
```

Keys:

- `START`: first CPU address in the ROM range.
- `END`: last CPU address in the ROM range.

When a ROM image is loaded, bytes whose absolute image addresses fall inside a
ROM range are copied into that range. Unfilled ROM bytes default to `0xFF`.

## RAM

`[RAM]` defines the writable memory area and optional banked backing store.

```ini
[RAM]
START=0x0000
END=0xDFFF
BANK_SIZE=16384
AVAILABLE=524288
```

Keys:

- `START`: first CPU address in RAM.
- `END`: last CPU address in RAM.
- `BANK_SIZE`: bank/window size in bytes. Use `16384` for 16 KiB windows.
- `AVAILABLE`: total backing RAM in bytes.

If `BANK_SIZE` and `AVAILABLE` are set together with `[MEMORY_MAPPER]`, the
simulator builds banked RAM. Without mapper settings, RAM is mapped as a flat
writable memory device.

## Serial

`[SERIAL]` maps the XR88C92 serial device.

```ini
[SERIAL]
IO_START_ADDRESS=0xF430
IO_END_ADDRESS=0xF43F
IRQ_LEVEL=1
```

Keys:

- `IO_START_ADDRESS`: first serial I/O register address.
- `IO_END_ADDRESS`: last serial I/O register address.
- `IRQ_LEVEL`: optional microLind IRQ controller level asserted by the serial
  device when an enabled XR88C92 interrupt is pending. Defaults to `1`.

The XR88C92 output port also models the microLind RGB LED wiring: setting
`STCR_SOPR`/OP4 turns red on, OP5 turns green on, and OP6 turns blue on.
Writing the same bits to `SPCR_ROPR` turns them off.

## CompactFlash

`[CF]` or `[COMPACT_FLASH]` maps the CompactFlash/ATA register window.

```ini
[CF]
IO_START_ADDRESS=0xF418
IO_END_ADDRESS=0xF41F
SECTORS=512
IMAGE=sim.img
READ_ONLY=false
```

Keys:

- `IO_START_ADDRESS`: first CF register address.
- `IO_END_ADDRESS`: last CF register address.
- `IMAGE` or `IMAGE_PATH`: optional raw disk image path.
- `SECTORS`: optional minimum sector count.
- `READ_ONLY`: `true`, `yes`, or `1` opens the image read-only.

Without `IMAGE`, the CF register window is mapped but no media is loaded and
register reads return `$FF`. With `IMAGE`, the image size becomes the disk size
unless `SECTORS` is set as a larger minimum. Disk images that are not an exact
multiple of 512 bytes are padded to the next sector.

## Parallel I/O

`[PARALLEL]` or `[PAR]` maps the W65C22/VIA-style parallel I/O device.

```ini
[PARALLEL]
IO_START_ADDRESS=0xF420
IO_END_ADDRESS=0xF42F
IRQ_LEVEL=2
```

Keys:

- `IO_START_ADDRESS`: first parallel I/O register address.
- `IO_END_ADDRESS`: last parallel I/O register address.
- `IRQ_LEVEL`: optional microLind IRQ controller level asserted by the
  parallel device when an enabled W65C22 interrupt is pending. Defaults to `2`.

The initial W65C22 model exposes port A/B output latches, input pins, data
direction registers, ACR/PCR, IFR/IER, shift register storage, and basic
timer-driven interrupt flags.

## Video / VDC

`[VIDEO]` or `[VDC]` maps the MOS 8563/8568-style VDC two-register CPU
interface. The VDC owns 64 KiB of private video RAM that is accessed through
the VDC data register, not through the normal CPU memory map.

```ini
[VIDEO]
IO_START_ADDRESS=0xF440
IO_END_ADDRESS=0xF441
VRAM_SIZE=65536
```

Keys:

- `IO_START_ADDRESS`: VDC control/status register address.
- `IO_END_ADDRESS`: VDC data register address. The first implementation expects
  this to be `IO_START_ADDRESS + 1`.
- `VRAM_SIZE`: documented private VDC RAM size. The simulator currently models
  64 KiB.

The initial model ignores exact VDC timing and reports ready immediately. The
GUI VDC Display window snapshots the 80x25 text and attribute planes plus the
8 KiB character-generator bank selected by register `$1C`. It composes those
bytes into a native-pixel RGBA framebuffer at a 25 Hz snapshot refresh rate.
The display panel offers fit-to-panel and 1x-4x zoom modes. CRT aspect mode
doubles the displayed vertical pixel height while leaving framebuffer and PNG
dimensions unchanged.

## Memory Mapper

`[MEMORY_MAPPER]` maps bank registers and, optionally, the CPU windows they
control.

```ini
[MEMORY_MAPPER]
BANK_0_REGISTER=0xF400
BANK_1_REGISTER=0xF401
BANK_2_REGISTER=0xF402
BANK_3_REGISTER=0xF403
WINDOW_0=0x0000-0x3FFF
WINDOW_1=0x4000-0x7FFF
WINDOW_2=0x8000-0xBFFF
WINDOW_3=0xC000-0xDFFF
BUS_SIGNALS_PROVIDER=AM14, AM15, AM16, AM17, AM18, AM19, AM20, AM21
```

Keys:

- `BANK_0_REGISTER` through `BANK_3_REGISTER`: mapper register addresses.
- `WINDOW_0` through `WINDOW_3`: CPU address ranges for each banked window.
- `WINDOW_N_START` and `WINDOW_N_END`: alternate spelling for window ranges.
- `BUS_SIGNALS_PROVIDER`: currently documented board metadata; parsed but not
  used by the simulator.

When explicit `WINDOW_N` ranges are present, each window is mapped separately.
If no windows are present, the simulator derives windows from the RAM range and
`BANK_SIZE`.

## Interrupt Controller

When a hardware config is used, the simulator maps the microLind IRQ register
at `0xF404`. The address is decoded by the project PLD as `IRQ_EN`.

Register layout:

- Bits 0-3: pending IRQ level.
- Bits 4-7: IRQ mask level.

Reads return `(mask << 4) | pending`. Writes update the high-nibble mask; the
low nibble is ignored. The CPU IRQ line is asserted when the pending IRQ level
is higher than the mask level. The CPU accepts the IRQ at an instruction
boundary when `CC.I` is clear and vectors through `$FFF8/$FFF9`.

## PLD Logic

`[PLD_LOGIC]` or `[LOGIC]` connects the hardware config to parsed WinCUPL PLD
source files.

```ini
[PLD_LOGIC]
SIGNAL_LOGIC=signal-logic.pld
MEMORY_LOGIC=mem-logic.pld
ADDRESS_LOGIC=address-logic.pld
BUS_MODE=route
```

Keys:

- `SIGNAL_LOGIC` or `SIGNAL_LOGIC_PATH`: signal/control PLD path.
- `MEMORY_LOGIC` or `MEMORY_LOGIC_PATH`: memory decode PLD path.
- `ADDRESS_LOGIC` or `ADDRESS_LOGIC_PATH`: I/O address decode PLD path.
- `BUS_MODE`: live bus decode mode.

`BUS_MODE` values:

- `range`: classic range-map-only bus; PLD files are still available for
  explicit validation/generation commands.
- `validate`: keep range-map routing, but log when PLD decode selects a
  different device role.
- `route`: use the PLD-selected device role for bus accesses. Devices are still
  created from `hw.cfg`; PLD routing chooses between those mapped devices.

The simulator validates configured ROM, RAM, memory mapper, CompactFlash,
parallel, and serial ranges against the PLD decode during rebuild. CLI builds print
diagnostics, and GUI/session rebuilds add them to the event log.

## CLI Helpers

Generate a partial config from decoded PLD ranges:

```text
pldcfg examples/signal-logic.pld examples/mem-logic.pld examples/address-logic.pld
```

Validate an existing config against the PLD decode:

```text
pldcheck examples/hw.cfg examples/signal-logic.pld examples/mem-logic.pld examples/address-logic.pld
```

`pldcfg` is for visual validation. It cannot know runtime details such as ROM
image paths, CF image paths, installed RAM size, or whether optional devices are
implemented.
