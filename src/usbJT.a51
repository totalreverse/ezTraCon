.globl _USB_jump_table
.globl _config

;--------------------------------------------------------------------------;
; USB Jump Table                                                           ;
; Not used in USB_POLL Mode!                                               ;
;--------------------------------------------------------------------------;

.area  USB_JT (ABS)        ; Absolute placement
.org   0x1b00              ; Place jump table at 0x1b00

_USB_jump_table:           ; autovector jump table

    ljmp  _sudav_isr       ; Setup Data Available (USED)
    .db 0
    ljmp  _sof_isr         ; Start of Frame   ----------- CURRENTLY UNUSED
    .db 0
    ljmp  _sutok_isr       ; Setup Data Loading ----------- CURRENTLY UNUSED
    .db 0
    ljmp  _suspend_isr     ; Global Suspend ----------- CURRENTLY UNUSED
    .db 0
    ljmp  _usbreset_isr    ; USB Reset ----------- CURRENTLY UNUSED
    .db 0
    ljmp  _ibn_isr         ; IN Bulk NAK interrupt ----------- CURRENTLY UNUSED
    .db 0
    ljmp  _ep0in_isr       ; Endpoint 0 IN ----------- CURRENTLY UNUSED
    .db 0
    ljmp  _ep0out_isr      ; Endpoint 0 OUT ----------- CURRENTLY UNUSED
    .db 0

    ; EP1 and EP2 mapped to same ISR - only one is in use at a time

    ljmp  _ep12in_isr       ; Endpoint 1 IN  - ANTFE-C Emulation ----------- CURRENTLY UNUSED
    .db 0
    ljmp  _ep12out_isr      ; Endpoint 1 OUT - ANTFE-C Emulation (USED)
    .db 0
    ljmp  _ep12in_isr       ; Endpoint 2 IN  ----------- CURRENTLY UNUSED
    .db 0
    ljmp  _ep12out_isr      ; Endpoint 2 OUT (USED)
    .db 0

_config:
    .db 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0

