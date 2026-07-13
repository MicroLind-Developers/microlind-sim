; Minimal microLind CF boot test.
;
; Reads sector 0 from the CompactFlash data register and writes the first
; four bytes to XR88C92 channel A. With examples/sim.img this should print:
;
;   TPLM
;
; Intended ROM placement: $FF00-$FFFF. Assemble as raw/Intel HEX/S-record
; according to your assembler's syntax.

CF_DATA     equ $F418
CF_ERROR    equ $F419
CF_SECCNT   equ $F41A
CF_LBA0     equ $F41B
CF_LBA1     equ $F41C
CF_LBA2     equ $F41D
CF_DRIVE    equ $F41E
CF_STATUS   equ $F41F
CF_COMMAND  equ $F41F

CF_CMD_READ equ $20
CF_ST_ERR   equ $01
CF_ST_DRQ   equ $08
CF_ST_BSY   equ $80

SER_MRA     equ $F430
SER_SRA     equ $F431
SER_CSRA    equ $F431
SER_CRA     equ $F432
SER_TXA     equ $F433

SER_TXRDY   equ $04

TMP_CHAR    equ $0000

            org $FF00

start:
            orcc #$50          ; Mask IRQ/FIRQ while we bring things up.
            lds  #$7FFF

            jsr  init_serial
            jsr  cf_read_lba0

            ldb  #4
print_loop:
            lda  CF_DATA       ; Repeated reads stream bytes from the sector.
            jsr  putc
            decb
            bne  print_loop

halt:
            bra  halt

cf_read_lba0:
            lda  #1
            sta  CF_SECCNT
            clra
            sta  CF_LBA0
            sta  CF_LBA1
            sta  CF_LBA2
            lda  #$E0          ; LBA mode, drive 0, LBA bits 24-27 = 0.
            sta  CF_DRIVE
            lda  #CF_CMD_READ
            sta  CF_COMMAND

cf_wait:
            lda  CF_STATUS
            bita #CF_ST_BSY
            bne  cf_wait
            bita #CF_ST_ERR
            bne  cf_error
            bita #CF_ST_DRQ
            beq  cf_wait
            rts

cf_error:
            lda  #'?'
            jsr  putc
            bra  halt

init_serial:
            ; The simulator only needs writes to SER_TXA, but these writes are
            ; close to a real XR88C92 channel-A bring-up sequence.
            lda  #$10          ; Reset MR pointer.
            sta  SER_CRA
            lda  #$13          ; 8 data bits, no parity.
            sta  SER_MRA
            lda  #$07          ; Normal mode, 1 stop bit.
            sta  SER_MRA
            lda  #$BB          ; Baud clock selector placeholder.
            sta  SER_CSRA
            lda  #$05          ; Enable RX and TX.
            sta  SER_CRA
            rts

putc:
            sta  TMP_CHAR
putc_wait:
            lda  SER_SRA
            bita #SER_TXRDY
            beq  putc_wait
            lda  TMP_CHAR
            sta  SER_TXA
            rts

            org $FFF0
            fdb start          ; 6309 illegal-instruction vector.
            fdb start          ; SWI3
            fdb start          ; SWI2
            fdb start          ; FIRQ
            fdb start          ; IRQ
            fdb start          ; SWI
            fdb start          ; NMI
            fdb start          ; RESET
