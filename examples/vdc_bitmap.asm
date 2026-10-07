; microLind VDC bitmap exercise (6809/6309, ROM at $F800).
; Assemble: lwasm --format=srec --output=build/vdc_bitmap.srec examples/vdc_bitmap.asm
; Load with examples/hw.cfg and run. Change RAM $0000 from 0 to 1 to enable
; per-cell colors, then to 2 to return to a text screen of custom glyphs.
; Bitmap: $2000-$5E7F. Colors: $6000-$67CF. Text glyph: $8010-$801F.

VDC_CONTROL equ $F440
VDC_DATA    equ $F441
DEMO_STAGE  equ $0000

            org $F800
start:
            orcc #$50
            lds #$7FFF
            clr DEMO_STAGE
            ldx #profile
init:
            lda ,x+
            cmpa #$FF
            beq clear_bitmap
            ldb ,x+
            jsr vdc_write
            bra init

clear_bitmap:
            ldx #$2000
            ldy #16000
            clrb
            jsr fill_vram
            ldx #$2000
            jsr set_update
            ldy #200
            ldb #$AA
scan_line:
            pshs b
            lda #$1F
            ldb #$80           ; left edge of first byte
            jsr vdc_write
            ldb #$01           ; right edge of second byte
            jsr vdc_write
            puls b
            ldu #78
stripe:
            lda #$1F
            jsr vdc_write
            leau -1,u
            bne stripe
            comb
            leay -1,y
            bne scan_line

            ; Copy the first scan line to the second (80 bytes, no $1F write).
            ldx #$2050
            jsr set_update
            lda #$20
            ldb #$20
            jsr vdc_write
            lda #$21
            clrb
            jsr vdc_write
            lda #$18
            ldb #$80
            jsr vdc_write
            lda #$1E
            ldb #80
            jsr vdc_write
            lda #$18
            clrb
            jsr vdc_write
wait_colors:
            tst DEMO_STAGE
            beq wait_colors

            ldx #$6000
            ldy #2000
            ldb #$2F           ; white foreground, blue background
            jsr fill_vram
            ldx #$6000
            jsr set_update
            ldy #2000
            ldb #$F0           ; all foreground/background color combinations
color:
            lda #$1F
            jsr vdc_write
            incb
            leay -1,y
            bne color
            lda #$19
            ldb #$C0
            jsr vdc_write
wait_text:
            lda DEMO_STAGE
            cmpa #2
            blo wait_text

            ; Provide a custom glyph before returning to text.
            ldx #$8010
            jsr set_update
            ldx #glyph
            ldy #16
glyph_byte:
            ldb ,x+
            lda #$1F
            jsr vdc_write
            leay -1,y
            bne glyph_byte
            ldx #$0000
            ldy #2000
            ldb #1
            jsr fill_vram
            lda #$0C
            clrb
            jsr vdc_write
            lda #$0D
            jsr vdc_write
            lda #$19
            jsr vdc_write
done:
            bra done

; A = register, B = value. Preserves A/B and index registers.
vdc_write:
            sta VDC_CONTROL
ready:
            tst VDC_CONTROL
            bpl ready
            stb VDC_DATA
            rts

; X = VRAM update address. Preserves D and index registers.
set_update:
            pshs d
            tfr x,d
            pshs b
            tfr a,b
            lda #$12
            jsr vdc_write
            puls b
            lda #$13
            jsr vdc_write
            puls d
            rts

; X = destination, Y = total length, B = fill byte. Consumes Y, preserves B.
; Each chunk writes one byte normally, then at most 255 additional bytes.
fill_vram:
            cmpy #0
            beq fill_return
            jsr set_update
            pshs b
            lda #$18
            clrb
            jsr vdc_write
fill_chunk:
            lda #$1F
            ldb ,s
            jsr vdc_write
            cmpy #256
            bhs fill_full
            tfr y,d
            decb
            beq fill_end       ; don't write zero to $1E: it means 256 bytes
            lda #$1E
            jsr vdc_write
            bra fill_end
fill_full:
            lda #$1E
            ldb #255
            jsr vdc_write
            leay -256,y
            bne fill_chunk
fill_end:
            puls b
fill_return:
            rts

profile:
            fcb $01,80,$06,25,$08,0,$09,7,$0A,$20
            fcb $0C,$20,$0D,0,$14,$60,$15,0
            fcb $16,$78,$17,8,$18,0,$19,$80,$1A,$F0,$1B,0,$1C,$80
            fcb $FF
glyph:
            fcb $18,$24,$42,$7E,$42,$42,$42,0
            fcb 0,0,0,0,0,0,0,0

            org $FFF0
            fdb start,start,start,start,start,start,start,start
