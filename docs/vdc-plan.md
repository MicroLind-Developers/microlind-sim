# VDC Graphics Device Implementation Plan

This plan adds a MOS 8563/8568-style VDC device to the simulator, backed by
internally addressable 64 KiB video RAM and displayed as an 80x25 monospaced
character window in the GUI.

The goal is to model the microLind-facing behavior first: the CPU only sees a
two-register I/O window, while all VDC register and video-memory access happens
through that window. Rendering can then evolve from a faithful text-mode view
toward more complete VDC timing and display behavior.

## Current Status

The core device, board mapping, runtime snapshots, text compositor, SDL texture
display, and PNG screenshots are implemented. Text rendering includes RGBI
attributes, alternate characters, reverse, underline, blink, and cursor scan
lines. VRAM data transfers, block fill, and forward block copy are implemented,
including 16-bit address wrapping. The GUI obtains copied snapshots at 25 Hz.

The initial graphical-mode stages (7A-7D) are implemented: owned bitmap/color
snapshots, bounded register-derived geometry, scan-line composition, per-group
colors, global reverse, mode-aware GUI/PNG validation, and a firmware exercise.
Text output retains its existing fixed 80x25 snapshot geometry. Bitmap output
supports non-interlaced eight-pixel cells with no smooth scrolling, double width,
or semigraphics. Unsupported profiles return a diagnostic instead of a frame.
Bitmap cursor effects remain deferred; the text cursor is not overlaid on graphics.

Validation covers device extraction/invalidation, bitmap pixels and colors,
snapshot ownership/stride/wrapping, fill/copy, and Debug Run/True Run mode changes.
The GUI builds, and the assembled firmware exercise has been executed through
all three stages. PNG round trips and SDL software texture output match the
native framebuffer pixels. Interactive fit/zoom/aspect controls retain their
existing implementation; a desktop UI walkthrough remains a manual check.

The original phases below describe the text-mode foundation. The Graphical Mode
Implementation section records the bitmap design and acceptance criteria;
Later Graphics Fidelity lists the remaining work. Reset defaults now set `$01`
to 80 and `$1B` to zero. Frame allocations are capped at 2048x1024 (8 MiB RGBA).

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
- Implement graphical mode in stages, starting with a non-interlaced 640x200
  bitmap and then bitmap color attributes. Keep advanced raster behavior in a
  separate fidelity phase.
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
- after a data transfer, increment the update address by one. Register `$1B`
  controls display row stride, not CPU data transfers or block fill/copy.
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

The original text-only snapshot proposal was:

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
pointers. That avoids data races and keeps GUI code simple. The implemented
`VdcSnapshot` also contains character-generator RAM; bitmap payload changes are
specified below.

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
- Block fill and forward block copy are implemented, including address wrapping.
- Model display-enable blanking.
- Extend character-generator addressing for modes using 32 bytes per character.
- Add tests from real BIOS routines.

## Graphical Mode Implementation

### Scope and References

Implement the VDC's one-bit bitmap mode through the existing two-byte I/O
window. Firmware selects the mode through registers and populates private VRAM
using the existing data, fill, and copy operations. No new hardware-config
section or host graphics commands are needed.

Use the [Commodore 128 Programmer's Reference
Guide](https://www.pagetable.com/docs/Commodore%20128%20Programmer%27s%20Reference%20Guide.pdf),
chapter 10, pages 314-316 and 324-333, as the register reference. A searchable
[transcription of the guide](https://manualzz.com/doc/23964126/commodore-128-personal-computer-programmer-s-reference-guide)
is also available. Its bitmap introduction inconsistently mentions 640x400
alongside 16,000 bytes; use the explicit 640x200 memory layout for the first
milestone and verify interlace separately.

The first supported profile is non-interlaced 640x200 with eight pixels per
byte, full horizontal/vertical display within each cell, and smooth scrolling,
double width, and semigraphics disabled. Then add color attributes and other
validated non-interlaced dimensions. Interlaced video, raster-time register
changes, split screens, precise blanking, and busy timing remain follow-up work.

### Register and Memory Contract

| Register | Bitmap responsibility |
|---|---|
| `$19` bit 7 | Clear selects text; set selects bitmap. |
| `$19` bit 6 | Enable bitmap color attributes. |
| `$01` | Displayed byte columns. |
| `$06`, `$09` bits 4-0 | Row groups and scan lines per group. |
| `$0C/$0D` | Bitmap base address. |
| `$14/$15` | Bitmap attribute base address. |
| `$16/$17` | Horizontal/vertical cell display geometry; verify bitmap gating. |
| `$18` bit 6 | Global reverse; verify its bitmap interaction. |
| `$1A` | Global foreground in high nibble, background in low nibble. |
| `$1B` | Extra bytes skipped between bitmap scan lines and attribute rows. |
| `$08`, `$18/$19` remaining mode bits | Detect modes outside the initial profile. |

For the initial profile, let `C = R01`, `G = (R09 & 0x1F) + 1`, and
`H = R06 * G`. The frame is `C * 8` pixels wide and `H` pixels high.
Define `stride = C + R1B`. Read bitmap byte `(column, y)` from
`uint16_t(display_start + y * stride + column)`; bit 7 is the leftmost pixel.
With attributes enabled, read the corresponding color byte from
`uint16_t(attribute_start + (y / G) * stride + column)`.

Bitmap attributes use the low nibble for foreground and the high nibble for
background. They must have a separate decoder from text attributes. With
attributes disabled, use `$1A`. Confirm reverse, cursor, and display gating
against the reference before defining their bitmap behavior in tests.

### Phase 7A: Geometry and Snapshot Payload

- In `include/microlind/app/vdc_render.hpp` and `src/app/vdc_render.cpp`, add a
  shared mode/geometry decoder. Derive bitmap dimensions from registers and
  validate the initial profile, zero dimensions, payload sizes, and a documented
  framebuffer allocation limit before multiplying or allocating.
- In `include/microlind/devices/vdc.hpp` and `src/devices/vdc.cpp`, add a const
  VRAM range-copy method with 16-bit wrapping. It must not select registers,
  advance the update address, or change the frame version.
- Extend `VdcSnapshot` in `include/microlind/app/sim_session.hpp` with owned
  bitmap bytes and bitmap color bytes. Store visible bytes in scan-line order,
  excluding skipped bytes; store colors in row-group order. Keep the current
  text arrays for compatibility. Use the register decoder as the single source
  of mode and geometry rather than maintaining a second mutable mode flag.
- In `SimSession::vdc_snapshot()`, gather the payload appropriate to the mode.
  A 640x200 bitmap needs 16,000 bytes and, when enabled, 2,000 color bytes.
  Avoid copying character-generator RAM in bitmap mode. Keep payload ownership
  inside the snapshot and extraction under the existing runtime lock.
- Audit constructor defaults: `$01` currently has no 80-column default and
  `$1B` defaults to one. Define coherent reset values and require the demo to
  program every geometry/stride register it depends on. Preserve existing text
  behavior while moving toward register-derived geometry.
- Add frame-version invalidation for newly used geometry/stride registers,
  including `$01`, `$06`, `$08`, and `$1B`. Existing VRAM writes, fill, and copy
  must invalidate bitmap output just as they invalidate text output.

Exit criterion: a coherent, side-effect-free bitmap snapshot with tested
geometry, row stride, and wrapping, without changing existing text pixels.

### Phase 7B: Basic Bitmap Compositor

- Keep `render_vdc_framebuffer()` as the shared entry point. Dispatch to text
  or bitmap composition based on `$19` bit 7.
- Expand each bitmap byte into eight opaque RGBA pixels using `vdc_rgb()` and
  the two global colors. Bitmap pixels come directly from display VRAM; do not
  fetch glyphs or apply text underline, blink, or alternate-charset flags.
- Resolve global reverse and hardware cursor behavior during the reference
  audit. Keep any text cursor overlay confined to the text path unless bitmap
  cursor behavior has been established independently.
- Return an empty frame or a shared validation error for unsupported profiles
  and incomplete payloads. Ensure every allocated pixel is initialized and
  never index past a snapshot buffer.
- Keep composition independent of SDL, ImGui, and mutable device state.

Exit criterion: firmware can select bitmap mode and display a deterministic
640x200 two-color image, then switch back to the existing text mode.

### Phase 7C: Bitmap Color Attributes

- Add a bitmap color decoder using both nibbles of each attribute byte. Reuse
  the RGBI palette without calling `vdc_cell_style()`.
- Apply one color pair per byte column and `G` scan lines. Test transitions
  between row groups, attribute base relocation, and `$1B` stride independently
  from bitmap data.
- Verify that disabling attributes restores global `$1A` colors and that all
  eight attribute bits remain color bits. Reserve independent VRAM regions in
  the demo so the bitmap, colors, and any saved character set do not overlap.

Exit criterion: a full 640x200 bitmap with independent color pairs per 8x8 area
when `G = 8`, using the simulator's existing 64 KiB VRAM.

### Phase 7D: GUI, Screenshots, and Firmware Exercise

- In `src/gui/gui_memory_panels.cpp`, replace the unconditional text-cell
  validation with shared mode/payload validation. Let the texture resize from
  the composed frame. Show mode and pixel dimensions in the existing status
  line; retain fit, integer zoom, and CRT aspect controls.
- In `src/gui/gui_state.cpp`, make PNG validation accept either payload and
  keep screenshot pixels identical to live framebuffer pixels. PNG dimensions
  remain native dimensions, independent of GUI scaling.
- Preserve the 25 Hz snapshot polling and existing `GuiRuntime` locking. Start
  with complete visible-payload copies; measure copy/composition cost before
  adding a version-based cache. Any cache must also account for text blink and
  cursor animation when switching back to text.
- Add a repository-style firmware example that programs a complete bitmap
  profile, draws byte/scan-line boundary patterns, enables color attributes,
  clears with block fill, copies a region with block copy, and restores text.
  Treat fill counts as additional bytes after the initial `$1F` write; split
  larger copy operations into the existing count-sized transfers.
- Document the supported register profile and limitations in `docs/VDC_INFO.md`
  and announce bitmap support in `FEATURE.md` once implemented.

Exit criterion: the same firmware-generated image appears during normal Run
and True Run and exports correctly as PNG; switching modes requires no GUI
reconfiguration.

### Validation and Acceptance

- `tests/test_vdc.cpp`: test read-only VRAM extraction across `$FFFF`, unchanged
  register selection/update address/version, and invalidation for new display
  registers. Exercise fill and copy on bitmap memory through the I/O window.
- `tests/test_vdc_render.cpp`: test `0x80`, `0x01`, `0xAA`, and `0x55` pixel
  order; byte and scan-line boundaries; exact 640x200 dimensions; global colors;
  row-group colors; reverse after verification; and text/bitmap/text dispatch.
  Include absent-device, zero-size, unsupported-profile, and short-payload cases.
- `tests/test_sim_session.cpp`: program registers and VRAM through the mapped
  device; assert snapshot packing for nonzero base addresses, skipped bytes,
  independent attribute stride, and wrapping. Rendering must use only the
  copied snapshot, even after further device writes.
- `tests/test_gui_runtime.cpp`: extend the True Run VDC exercise to update
  bitmap VRAM and switch modes without incoherent snapshots.
- Build the GUI and manually exercise texture resizing, scale/aspect controls,
  and PNG export with the firmware example. Compare exported dimensions and
  pixels with the compositor output. Run the full existing CTest suite.

### Later Graphics Fidelity

After the initial milestones, verify and implement smooth-scroll offsets,
double-width pixels, semigraphic extension, and display gating with distinct
fixtures. Extend dimensions beyond the initial profile only with bounded
allocations and reference-backed geometry. Implement interlaced fields and
raster-time mode changes when a scan-line/timing model exists; a larger image
alone does not establish interlace support.

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
