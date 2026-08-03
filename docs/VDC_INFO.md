# MOS 8563 VDC
The VDC chip has 2 registers that can be read or written to. All communication takes place thru those 2 registers. Write to the first register is called Control and a read from it is Status. The second register is Data.

## Video memory
The VDC have 64kB memory attached to it that is not available from the regular system bus, but only thru the above mentioned registers. So to set where in the video memory the framebuffer is following routine is done:
``` asm
VDC_CONTROLL EQU $F440
VDC_DATA     EQU $F441

; write to a register in vdc.
; A = register to write to
; B = value to write to A
vdc_write:
    sta VDC_CONTROL
vdc_write_wait:
     tst VDC_CONTROL     ; copies bit 7 into the N flag
     bpl vdc_write_wait   ; loop while bit 7 is clear
     stb VDC_DATA

; read from a register in vdc.
; A = register to read from
; B = returns value in register
vdc_read:
    sta VDC_CONTROL
vdc_read_wait:
     tst VDC_CONTROL     ; copies bit 7 into the N flag
     bpl vdc_read_wait   ; loop while bit 7 is clear
     ldb VDC_DATA
```

## VDC Status flags
Reading the VDC status register returns a set of bit flags:


| Bit | Name | Description |
|---|---|---|
| 7 | Ready / H Blank | 0 = VDC is busy, 1 = VDC is ready to receive or send data. |
| 6 | V Blank | 0 = Active display, 1 = During vertical blanking period |
| 5 | L Pen | Indicates the light pen trigger status |
| 4 | Update Ready | 1 = RAM update operation is complete |
| 3 | Display Enabled | 1 = CRT display is active |

## VDC register map
These are the regissters available in the VDC:
| Register | Mnemonic | Bit 7 | Bit 6 | Bit 5 | Bit 4 | Bit 3 | Bit 2 | Bit 1 | Bit 0 | Description |
|---|---|---|---|---|---|---|---|---|---|
| $00 | H_TOT | HT7 | HT6 | HT5 | HT4 | HT3 | HT2 | HT1 | HT0 | Horizontal Total |
| $01 | H_DISP | HD7 | HD6 | HD5 | HD4 | HD3 | HD2 | HD1 | HD0 | Horizontal Displayed |
| $02 | HS_POS | HP7 | HP6 | HP5 | HP4 | HP3 | HP2 | HP1 | HP0 | Horizontal Sync Position |
| $03 | S_WIDTH | VW3 | VW2 | VW1 | VW0 | HW3 | HW2 | HW1 | HW0 | Vertical/Horizontal Sync Width |
| $04 | V_TOT | VT7 | VT6 | VT5 | VT4 | VT3 | VT2 | VT1 | VT0 | Vertical Total |
| $05 | V_ADJ | — | — | — | VA4 | VA3 | VA2 | VA1 | VA0 | Vertical Adjust |
| $06 | V_DISP | VD7 | VD6 | VD5 | VD4 | VD3 | VD2 | VD1 | VD0 | Vertical Displayed |
| $07 | VS_POS | VP7 | VP6 | VP5 | VP4 | VP3 | VP2 | VP1 | VP0 | Vertical Sync Position |
| $08 | IM | — | — | — | — | — | — | IM1 | IM0 | Interlace Mode |
| $09 | C_TOT_V | — | — | — | — | CTV4 | CTV3 | CTV2 | CTV1 | Character Total Vertical |
| $0A | CU_START | — | CM1 | CM0 | CS4 | CS3 | CS2 | CS1 | CS0 | Cursor Mode, Start Scan |
| $0B | CU_END | — | — | — | CE4 | CE3 | CE2 | CE1 | CE0 | Cursor End Scan Line |
| $0C | DISP_START_H | DS15 | DS14 | DS13 | DS12 | DS11 | DS10 | DS9 | DS8 | Display Start Address High Byte |
| $0D | DISP_START_L | DS7 | DS6 | DS5 | DS4 | DS3 | DS2 | DS1 | DS0 | Display Start Address Low Byte |
| $0E | C_POS_H | CP15 | CP14 | CP13 | CP12 | CP11 | CP10 | CP9 | CP8 | Cursor Position High Byte |
| $0F | C_POT_L | CP7 | CP6 | CP5 | CP4 | CP3 | CP2 | CP1 | CP0 | Cursor Position Low Byte |
| $10 | LP_V_POS | LPV7 | LPV6 | LPV5 | LPV4 | LPV3 | LPV2 | LPV1 | LPV0 | Light Pen Vertical Position |
| $11 | LP_H_POS | LPH7 | LPH6 | LPH5 | LPH4 | LPH3 | LPH2 | LPH1 | LPH0 | Light Pen Horizontal Position |
| $12 | U_ADDR_H | UA15 | UA14 | UA13 | UA12 | UA11 | UA10 | UA9 | UA8 | Update Address High Byte |
| $13 | U_ADDR_L | UA7 | UA6 | UA5 | UA4 | UA3 | UA2 | UA1 | UA0 | Update Address Low Byte |
| $14 | ATTR_START_H | AA15 | AA14 | AA13 | AA12 | AA11 | AA10 | AA9 | AA8 | Attribute Start Address High Byte |
| $15 | ATTR_START_L | AA7 | AA6 | AA5 | AA4 | AA3 | AA2 | AA1 | AA0 | Attribute Start Address Low Byte |
| $16 | CH_TOT_H | CTH3 | CTH2 | CTH1 | CTH0 | CDH3 | CDH2 | CDH1 | CDH0 | Character Total Horizontal, Character Display Horizontal |
| $17 | CH_TOT_V | — | — | — | CDV4 | CDV3 | CDV2 | CDV1 | CDV0 | Character Display Vertical |
| $18 | V_SCROLL | COPY | RVS | CBRATE | VSS4 | VSS3 | VSS2 | VSS1 | VSS0 | Vertical Smooth Scrolling |
| $19 | H_SCROLL | TEXT | ATR | SEMI | DBL | HSS3 | HSS2 | HSS1 | HSS0 | Horizontal Smooth Scrolling |
| $1A | COLOR | FG3 | FG2 | FG1 | FG0 | BG3 | BG2 | BG1 | BG0 | Foreground/Background color |
| $1B | ADD_INC | AI7 | AI6 | AI5 | AI4 | AI3 | AI2 | AI1 | AI0 | Address Increment per Row |
| $1C | CH_BASE | CB15 | CB14 | CB13 | RAM | — | — | — | — | Character Base Address |
| $1D | UL_SCAN | — | — | — | UL4 | UL3 | UL2 | UL1 | UL0 | Underline Scan Line |
| $1E | WC | WC7 | WC6 | WC5 | WC4 | WC3 | WC2 | WC1 | WC0 | Word Count |
| $1F | D_REG | DA7 | DA6 | DA5 | DA4 | DA3 | DA2 | DA1 | DA0 | Data Register |
| $20 | BL_START_H | BA15 | BA14 | BA13 | BA12 | BA11 | BA10 | BA9 | BA8 | Block Start Address High Byte |
| $21 | BL_START_L | BA7 | BA6 | BA5 | BA4 | BA3 | BA2 | BA1 | BA0 | Block Start Address Low Byte |
| $22 | DISP_EN_START | DEB7 | DEB6 | DEB5 | DEB4 | DEB3 | DEB2 | DEB1 | DEB0 | Display Enable Begin |
| $23 | DISP_EN_END | DEE7 | DEE6 | DEE5 | DEE4 | DEE3 | DEE2 | DEE1 | DEE0 | Display Enable End |
| $24 | REF_RATE | — | — | — | — | DRR3 | DRR2 | DRR1 | DRR0 | DRAM Refresh Rate |

## Block fill

Clear bit 7 of register `$18` to select block fill mode. Write the fill byte to
register `$1F`, then write the number of additional bytes to register `$1E` to
start the fill. A value of zero in `$1E` means 256 bytes. Both the normal `$1F`
write and the block fill advance the update address in registers `$12/$13`, so
firmware commonly writes `total_length - 1` to `$1E`.

## Attribute RAM
In the attribute ram each byte corresponds to a byte on the screen. This byte defines the characteristics of that character.

| Bit | Mnem | Descr. |
|---|---|---|
| 7 | ALT | Alternative character table |
| 6 | RVS | Inverted  (Colored background) |
| 5 | UL | Underligned (Cursor) |
| 4 | FLASH | Flashing character |
| 3 | Red | Red bit |
| 2 | Green | Green bit |
| 1 | Blue | Blue bit |
| 0 | Int | Light bit |
