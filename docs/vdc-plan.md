# VDC Graphics Device Implementation Plan

This plan adds a MOS 8563/8568-style VDC device to the simulator, backed by
internally addressable 64 KiB video RAM and displayed as an 80x25 monospaced
character window in the GUI.

The goal is to model the microLind-facing behavior first: the CPU only sees a
two-register I/O window, while all VDC register and video-memory access happens
through that window. Rendering can then evolve from a faithful text-mode view
toward more complete VDC timing and display behavior.

## Design Goals

- Add a board device mapped as `BusDeviceSelect::Video`.
- Expose only two CPU-visible registers:
  - control/status at the configured base address.
  - data at base address + 1.
- Maintain 64 KiB of private VDC RAM that is not mapped on the normal CPU bus.
- Implement enough VDC register behavior to support firmware text output.
- Provide a GUI VDC display window showing an 80x25 monospaced character view.
- Keep simulator/runtime threading safe:
  - simulation owns mutable device state.
  - GUI reads copied snapshots.
  - rendering resources remain on the GUI thread.
- Leave room for later support of attributes, custom character ROM/RAM, cursor,
  blink, smooth scrolling, graphical mode, and more accurate busy/vblank timing.

## Decisions

- Treat MOS 8563 and MOS 8568 as behaviorally identical in the simulator for
  now. The relevant hardware difference is that the later chip exposes the
  READY status on a physical pin, which does not change the CPU-visible model
  here.
- Use `$F440/$F441` as the VDC I/O range.
- Render text mode from display, attribute, and character-generator RAM into a
  native-pixel framebuffer.
- Keep the framebuffer compositor independent of SDL and ImGui so screenshots
  and the live display use identical pixels.
- Defer graphical mode. It is handled differently and is not needed for the
  current software target.
- Ignore exact VDC timing in the first implementation. The status register can
  report ready immediately, and the GUI display should update at 25 Hz.

## Device Model

Create `microlind::devices::Vdc8568`:

- `std::array<uint8_t, 0x10000> vram_`.
- `std::array<uint8_t, 0x25> regs_` for VDC registers `$00-$24`.
- `uint8_t selected_register_`.
- status flags:
  - ready / hblank bit.
  - vblank bit.
  - light pen bit.
  - update-ready bit.
  - display-enabled bit.
- optional busy counter, deferred until timing fidelity is needed.
- dirty region/version tracking for efficient GUI refresh.

The CPU-visible register behavior should follow the documented routines:

- write control: select the VDC internal register.
- read control: return status.
- write data: write value to selected VDC register.
- read data: read value from selected VDC register.

Special handling is needed for the internal data register `$1F` and update
address registers `$12/$13`:

- writes to `$1F` write to `vram_[update_address]`.
- reads from `$1F` read from `vram_[update_address]`.
- after a data transfer, increment update address according to register `$1B`
  when supported.
- keep ready/update-ready simple at first, with ready reported immediately.

## Initial Register Coverage

Implement these first because they drive an 80x25 text screen:

- `$06` vertical displayed.
- `$09` character total vertical.
- `$0C/$0D` display start address.
- `$0E/$0F` cursor position.
- `$12/$13` update address.
- `$14/$15` attribute start address.
- `$18/$19` scroll/text/attribute mode bits, initially stored and exposed.
- `$1A` foreground/background default color.
- `$1B` address increment.
- `$1C` character base address.
- `$1F` data register / VRAM access.

Other registers should be stored and readable even if their detailed behavior is
not implemented yet. Unknown/out-of-range selected registers should read `$FF`
or the stored value according to what firmware expects after we test real code.

## Hardware Config

Add a new config section:

```ini
[VIDEO]
IO_START_ADDRESS=0xF440
IO_END_ADDRESS=0xF441
VRAM_SIZE=65536
```

Optional future keys:

- `TYPE=mos8568`
- `CHAR_ROM=...`
- `COLUMNS=80`
- `ROWS=25`
- `CHAR_WIDTH=8`
- `CHAR_HEIGHT=16`

Parser changes:

- add `VideoConfig` to `HardwareConfig`.
- parse `[VIDEO]` or `[VDC]`.
- require a two-byte I/O window for the first model.
- include the device in `build_sim()`.
- include the device in PLD validation/generation if the address PLD exposes a
  `VID_EN`/`VDC_EN` signal later.

## GUI Display

Add a `Video` / `VDC Display` window:

- default size suitable for 80x25 text.
- monospaced character grid.
- optional border/status line with:
  - display start address.
  - attribute start address.
  - update address.
  - selected VDC register.
  - status byte.
  - dirty frame/version.
- View menu toggle and session persistence.

Rendering pipeline:

- snapshot the 8 KiB character-generator bank selected by register `$1C`.
- use 16 bytes per character and attribute bit 7 to select characters 256-511.
- derive cell and displayed-pixel geometry from registers `$09`, `$16`, and `$17`.
- compose text, RGBI attributes, reverse, underline, flash, and cursor pixels into
  a native-resolution RGBA framebuffer.
- upload the framebuffer to an SDL texture for the live display.
- write the same framebuffer directly for PNG screenshots.

## Threading Model

Do not render SDL/ImGui objects from a device thread. SDL renderer and ImGui
state should stay on the main GUI thread.

Recommended threading model:

- simulation thread:
  - owns `Vdc8568`.
  - mutates registers and VRAM during bus accesses.
  - produces immutable snapshots through `SimSession`.
- GUI thread:
  - pulls `RuntimeDebuggerSnapshot`.
  - renders the latest VDC snapshot.
  - creates/updates SDL textures if texture rendering is used.
- optional VDC compositor thread, later:
  - receives copied VDC snapshots or a copied dirty VRAM range.
  - converts text/attribute/charset data into a plain RGBA buffer.
  - never touches `SimSession`, `Bus`, `Vdc8568`, SDL renderer, or ImGui.
  - publishes completed pixel buffers to the GUI thread.

This keeps True Run compatible with the existing runtime design: the worker can
continue executing while the GUI displays the newest copied video state.

## Snapshot Shape

Add an app-layer snapshot, for example:

```cpp
struct VdcSnapshot {
    bool present{};
    uint16_t start{};
    uint16_t end{};
    uint8_t selected_register{};
    uint8_t status{};
    std::array<uint8_t, 0x25> registers{};
    uint16_t display_start{};
    uint16_t attribute_start{};
    uint16_t update_address{};
    uint16_t cursor_position{};
    uint8_t columns{80};
    uint8_t rows{25};
    uint64_t frame_version{};
    std::array<uint8_t, 80 * 25> chars{};
    std::array<uint8_t, 80 * 25> attrs{};
};
```

The snapshot should contain display-ready data rather than exposing direct VRAM
pointers. That avoids data races and keeps GUI code simple.

## Testing Strategy

Unit tests:

- control register selects internal VDC register.
- data register reads/writes selected internal register.
- update address high/low select VRAM address.
- data register `$1F` reads/writes private VRAM.
- update address increments correctly.
- `peek8()` does not clear status flags or advance internal state.
- status reads report the initial always-ready behavior.

App/session tests:

- hardware config parses `[VIDEO]`.
- simulator maps the VDC device.
- VDC snapshot reports presence, address range, registers, and screen bytes.
- session save/load preserves the VDC panel visibility state.

GUI-adjacent tests:

- snapshot extraction is side-effect free.
- True Run can update VDC state without direct GUI reads.
- display text derived from VRAM matches the expected 80x25 character matrix.

## Implementation Phases

### Phase 1: Core Device

- Add `include/microlind/devices/vdc.hpp`.
- Add `src/devices/vdc.cpp`.
- Implement two-register bus access.
- Implement internal VDC register storage.
- Implement 64 KiB private VRAM.
- Implement `$12/$13` update address and `$1F` VRAM data register.
- Add focused GTest unit coverage.

### Phase 2: Board Integration

- Add `VideoConfig` to hardware config.
- Parse `[VIDEO]` / `[VDC]`.
- Add VDC construction in `build_sim()`.
- Map the VDC as `BusDeviceSelect::Video`.
- Update `examples/hw.cfg` and `docs/hardware-config.md`.
- Add SimSession VDC snapshot support.

### Phase 3: GUI Display

- Add `show_video` to GUI state/session persistence.
- Add View menu entry.
- Add `draw_vdc_display()`.
- Snapshot display, attribute, and character-generator RAM.
- Render 80x25 display RAM characters into a native-pixel framebuffer.
- Render VDC cursor modes and scan lines.
- Update `FEATURE.md`.

### Phase 4: Runtime Snapshot Efficiency

- Add dirty version/range tracking in the VDC device.
- Avoid copying full 64 KiB VRAM every GUI frame.
- Snapshot only the displayed 80x25 chars/attrs plus registers.
- In True Run, publish video snapshots at a bounded refresh rate.
- The GUI VDC display should refresh at 25 Hz, independent of simulator speed.

### Phase 5: Better Rendering

- Add SDL texture-backed rendering if ImGui text rendering is not smooth enough.
- Add a GUI-owned texture and RGBA staging buffer.
- Add optional compositor thread only if profiling shows it is useful.
- Implement attributes:
  - foreground/background color.
  - reverse.
  - underline.
  - blink.
  - alternate charset.

### Phase 6: VDC Fidelity

- Audit against MOS 8563/8568 reference behavior.
- Improve ready/hblank/vblank/update-ready timing.
- Block fill registers are implemented; model block copy when firmware uses it.
- Model display-enable blanking.
- Extend character-generator addressing for modes using 32 bytes per character.
- Add tests from real BIOS routines.

## Risks

- Rendering from a separate thread can break SDL/ImGui assumptions. Keep SDL and
  ImGui on the GUI thread.
- Full VRAM copies every frame are easy but wasteful during True Run. Prefer a
  display snapshot and dirty version tracking.
- VDC busy timing can affect firmware loops. Start simple, but isolate timing so
  it can be improved without changing the bus/device API.
- Firmware must populate character-generator RAM before text glyphs appear,
  matching the VDC hardware rather than assuming a host character encoding.

## First Milestone

The first useful milestone is:

- `[VIDEO]` maps `$F440-$F441`.
- firmware can write text bytes into VDC VRAM through the VDC data register.
- the GUI shows an 80x25 VDC window with those bytes.
- tests prove register selection, VRAM access, snapshot extraction, and config
  parsing.
