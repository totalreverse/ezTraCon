/***************************************************************************
 *   EzTraCon - ezUSB Trainer Controller                                   *
 *                                                                         *
 *   Copyright (C) 2019 by Michael Hipp                                    *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#include "main.h"


// Divider 4
// Reload -20 = -0x14 = 0xEC = 256 - 24MHz/4/19200*2/32
// Reload for baud generation => 0xEC is 18750 baud => best match for 19200 (with Error -2.34%)
// (works also with 0xED: 19737 error ~ +2.80%)
#define RXTX_BAUDGENERATOR_TH1          (0xEC)

// Divider 12 for Bitbang!
// TX/RX: Reload (v = 256 - 24MHz/12/19200) -
//
// recommended is 0x98
#define TX_BITBANG_SAMPLING_TIMER       (0x98)
//
// 0x98 is 19230 (error +0.16%) - stability of 0x96 depends a lot on RX_BITBANG_SAMPLING_OFFSET and the
// implementation delay in ie0_isr
// => stable between 151..160 = 0x97..0xa0 ( 19048..19200 ),
// recommended is 0x98
#define RX_BITBANG_SAMPLING_TIMER       (0x98)
//
// Time between high/low edge and first sampling (= double baudrate) => 38400 => 0xCC
// with RX_BITBANG_SAMPLING_TIMER=0x98 stable between (0xC0 and 0xD4) value depends on the delay in ie0
// Recommended is 0xCC.
#define RX_BITBANG_SAMPLING_OFFSET      (0xCC)


/*********************************************************
 * serial tx/rx bitbang etc/pp
 ********************************************************/

#ifdef SERIAL_8N2
#define TX_BITS_PER_BYTE (11)
#else
#define TX_BITS_PER_BYTE (10)
#endif

// TXBUF_SIZE must larger than ((TXBUF_MAX_MSG_ACCEPT+2)*2+2) ->
// 29 for 0x40, 13 for 0x20:
// TXBUF_MAX_MSG_ACCEPT from HOST decoded without checksum
// sizeof(struct frameCommandFortius) == TXBUF_MAX_MSG_ACCEPT
#define TXBUF_MAX_MSG_ACCEPT 12
#define TXBUF_SIZE           ((TXBUF_MAX_MSG_ACCEPT+2)*2+2)
__pdata uint8_t txbuf[TXBUF_SIZE];
__xdata uint8_t txDecoded[TXBUF_MAX_MSG_ACCEPT];

// RXBUF_DECODED is main message size is 23 plus 2 bytes checksum
#define RXBUF_DECODED   (23+2)
// RXBUF_MAX_MSG we accept from brake (encoded)
// 1+(23+2)*2+1 = 52 largest known frame
// we do not store SOF and EOF bytes, so we save 2 bytes => 50
#define RXBUF_MAX_MSG   (RXBUF_DECODED*2)
__pdata uint8_t rxbuf[RXBUF_MAX_MSG+1];  // +1 to store byte before size check
__xdata uint8_t rxDecoded[RXBUF_DECODED];

#if defined(WITH_BITBANG) && !defined(WITH_ASM8051_TX) || defined(WITH_SERIAL0)
__data volatile uint8_t  txbufIdx;
#endif
__bit volatile txON;

#ifndef WITH_ASM8051_TX
__data volatile uint8_t txNext;
__data volatile uint8_t txNextBITSET;
__data volatile uint8_t txByte;
__data volatile uint8_t txBitCnt;
#endif

__bit volatile rxON;
__bit volatile rxEND;

//__data volatile uint8_t  rxlen;    // length of last received and valid decoded frame (incl. 2 bytes of checksum)
__data         uint16_t  rxChecksum;

__data volatile uint8_t  rxbufIdx; // valid if rxON = true or rxEND == true
__data uint8_t  rxByte;   // TODO not used for WITH_BITBANG && WITH_ASM8051_RX

#ifdef WITH_BITBANG
#ifndef WITH_ASM8051_RX
__data uint8_t   rxBitCnt = 0;
#endif
#endif

// statistics
#ifdef WITH_STATISTICS_SERIAL

__data uint8_t  count_tx_err;
__data uint16_t  count_rx_valid;  // valid frames received
__data uint8_t  count_rx_timeout_err;
__data uint8_t  count_rx_checksum_err;

__data volatile uint8_t  count_rx_overflow_err;
__data volatile uint8_t  count_rx_frame_err;
__data volatile uint8_t  count_rx_start_err;
__data volatile uint8_t  count_rx_stop_err;
__data volatile uint8_t  count_txrx_collision;
#endif

// global
__data volatile uint8_t timer2_serial_timeout;
__data uint8_t count_rx_err;
__bit  u_semaphore_rx_valid;
__bit  u_semaphore_rx_valid_4stateMachine;


/*****************************************************************************
 * SERIAL0 specific code (works only with solid blue head unit)
 ****************************************************************************/

#ifdef WITH_SERIAL0

inline void configureTimer1Baudgenerator(void) {
    RESET_TIMER1();

    TMOD  |= M11;       // GATE1=int-gate, CT1=Counter/Timer select, M11/M10=mode select (b00=13bit,b01=16bit,b10=8bit reload,b11=timer1 stopped)
    CKCON |= T1M;       // Divder 4
    TH1 = RXTX_BAUDGENERATOR_TH1;

    TR1 = true;        // Timer1 RUN
}

inline void disableSerial0(void) {
    ES0   = false;              // IRQ off
    SCON0 = 0b00000000;         // all off: SM0_0|SM1_0|SM2_0|REN_0|TB8_0|RB_8|TI_0|RI_0 = b00000000
}

inline void configureSerial0(void) {
    ES0   = false;              // IRQ off
    SCON0 = 0b00000000;         // all off: SM0_0|SM1_0|SM2_0|REN_0|TB8_0|RB_8|TI_0|RI_0 = b00000000

    // default is Baudgenerator for serial0
    configureTimer1Baudgenerator();

    RCLK = false;  // RCLK=0 selects Timer 1 overflow as Rx clock (0=Timer1, 1=Timer2)
    TCLK = false;  // TCLK=0 selects Timer 1 overflow as Tx clock (0=Timer1, 1=Timer2)

    // Serial 0
    PCON |= SMOD0;  // SMOD0=1 (PCON0.7) - When SMOD0 = 1, the baud rate for Serial Port 0 is doubled.

     //SM0_0|SM1_0|SM2_0=b010 => serial0 mode 1  => 8 Bit, 1 Start, 1 Stop, No Parity (8N1)
    SM1_0 = true;
#ifdef SERIAL_8N2
    SM0_0 = true;  // SM0_0|SM1_0|SM2_0=b110 => serial0 mode 3  => 8 Bit, 1 Start, 2 Stop, No Parity (8N2)
    TB8_0 = true;  // 9th = true => 2nd Stopbit
#endif

    REN_0 = true;   // REN_0=receive enabled,
    PS0 = true;     // 1=serial0 High Prio IRQ ON
    ES0 = true;     // enable Serial0 IRQ
}

/*****************************************************************************
 * Only the "solid blue" device can use the serial0 ports for communicating
 * with the motorbrake. It's the standard way to handle the motorbrake with
 * the solid blue head unit
 ****************************************************************************/

void serial0_isr(void)  __interrupt SI0_VECTOR {

    while (true) {
        if(RI_0) {
            RI_0=false;
            rxByte = SBUF0;

            // same (similar) code  in timer1_rx_bitbang
            if(rxEND) {
                STATISTICS_SERIAL(count_rx_overflow_err++);
            } else if(rxON) {
                // TODO check for unexpected T1941_START_OF_FRAME, log error, and start a new frame
                if(rxByte == T1941_END_OF_FRAME) {
                    // Discard EOF byte
                    rxEND = true;
                } else if(rxbufIdx >= RXBUF_MAX_MSG) {
                    STATISTICS_SERIAL(count_rx_overflow_err++);
                    rxON  = false;
                } else {
                    rxbuf[rxbufIdx++] = rxByte;
                }
            } else if(rxByte != T1941_START_OF_FRAME) {
                STATISTICS_SERIAL(count_rx_frame_err++);
            } else {
                // TODO timer2_rxTimeout watchdog drops frame if data is not expected (see handle_rx_brake())
                // Discard SOF byte - just set rxbufIdx to zero
                rxbufIdx = 0;
                rxON = true;
            }
        } else if(TI_0) {
            TI_0=false;
            if(txON) {
                if(txbuf[txbufIdx] == T1941_END_OF_FRAME) {
                    txON  = false;
                } else {
                    txbufIdx++;
                    SBUF0 = txbuf[txbufIdx];
                }
            }
        } else {
            break;
        }
    }

    STATISTICS_IRQ(count_serial0++);
}
#else
void serial0_isr(void)  __interrupt SI0_VECTOR { }
#endif

/*****************************************************************************
 * BITBANG specific code (works with solid blue and solid green head unit)
 ****************************************************************************/

#ifdef WITH_BITBANG

#ifdef WITH_ASM8051_RX

// rxBitCnt = 8     => r6 (maybe with 0-9 and tst: dec jnz"

// rxByte   = 0x00; => r5
// 'a' save/restore => r7
// rxBuf => r0      => @r0 = byte ; inc r0
//
// EXPECTS rxBuf in pData!! so we can access data with "movx .. @r0 ..."

void timer1_isr_bitbang_rx(void)   __interrupt TF1_VECTOR  __using(1)  __naked {

    __asm

    push    psw
    mov     psw,#0x08
    inc     _DPS          // switch to alternate DPTR ... only slightly faster than push/pop
    mov     r7,a

#ifdef WITH_STATISTICS_IRQ
    inc     _count_t1
#endif

    mov     dptr,#_PINSC
    movx    a,@dptr

#if PORTC_BIT_RX_COMMON != 2
#error Change bit
#else
    mov     c,acc.2
    // NOTE: !do not change CY flag until we processed it!
#endif

    // do not use cjne here, because this overwrites "CY" flag
    mov     a,r6
    jnz     timer1_rx_continue_bit

    // Case rxBitCnt = 0xff : Test start_bit != 0 (=CY set)  ==> err
    jc      timer1_rx_start_bit_err
    // Start-Bit is "0" => init bit shifter: rxByte = 0x00; rxBitCnt = 8;
    mov     r5,a        // a is zero here
    mov     r6,#0x09
    sjmp    timer1_end

timer1_rx_continue_bit:
    djnz    r6,timer1_rx_shift_bit

timer1_rx_byte:
    // one byte received including one stop bit
    clr     _TR1

    // if(stopbit == 0) { => error }
    jnc     timer1_rx_stop_bit_err
    // if(rxEND) { => error }
    jb      _rxEND,timer1_rx_overflow_err
    jnb     _rxON, timer1_rx_byte_start_frame

    // if(rxbuf != rxbuf+RXBUF_MAX_MSG) { => rx msg buffer overflow error }
    cjne    r5,#T1941_END_OF_FRAME,timer1_continue_frame

    // Frame complete with a T1941_END_OF_FRAME byte:
    setb    _rxEND
    mov     a,r0
    clr     c
    subb    a,#_rxbuf
    mov     _rxbufIdx,a
    sjmp    timer1_end

timer1_continue_frame:
    cjne    r0,#_rxbuf+RXBUF_MAX_MSG,timer1_write_byte
    sjmp    timer1_rx_overflow_err_clr

timer1_write_byte:
    mov     a,r5

#ifdef WITH_ASM8051_RXTX_PAGED_XDATA
    movx    @r0,a
#else
    mov     @r0,a
#endif

    inc     r0
    sjmp    timer1_end

timer1_rx_byte_start_frame:
    cjne    r5,#T1941_START_OF_FRAME,timer1_rx_byte_start_frame_err

    mov     r0,#_rxbuf
    setb    _rxON
    sjmp    timer1_end

    // main case : push bit to top of byte
    // rxByte >>= 1; rxByte |= (pins & PORTC_MASK_RX_COMMON ? 0x80 : 0x00)
    // Carry contains bit
timer1_rx_shift_bit:
    mov     a,r5
    rrc     a
    mov     r5,a

timer1_end:
    mov     a,r7
    inc     _DPS
    pop     psw
    reti

    // ==========================
    // Error cases
    // ==========================

timer1_rx_start_bit_err:
#ifdef WITH_STATISTICS_SERIAL
    inc     _count_rx_start_err
#endif
    sjmp    timer1_end

timer1_rx_stop_bit_err:
#ifdef WITH_STATISTICS_SERIAL
    inc _count_rx_stop_err
#endif
    clr _rxON
    sjmp    timer1_end

timer1_rx_overflow_err_clr:
    clr _rxON
timer1_rx_overflow_err:
#ifdef WITH_STATISTICS_SERIAL
    inc _count_rx_overflow_err
#endif
    sjmp    timer1_end

timer1_rx_byte_start_frame_err:
#ifdef WITH_STATISTICS_SERIAL
    inc     _count_rx_frame_err
#endif
    sjmp    timer1_end

    __endasm;
}

#else

void timer1_isr_bitbang_rx(void)  __interrupt TF1_VECTOR   {

    uint8_t pins = PINSC;

    STATISTICS_IRQ(count_t1++);

    if(rxBitCnt == 0) {
        if(pins & PORTC_MASK_RX_COMMON) {
            STATISTICS_SERIAL(count_rx_start_err++);
            rxON = false;
        } else {
            rxByte   = 0x00;
            rxBitCnt = 0x09;
        }
    } else if( (--rxBitCnt) == 0) {
        // TODO 2 stopbits handling - we just ignore 2nd bit by switching off TR1
        TR1 = false;
        if(!(pins & PORTC_MASK_RX_COMMON)) {
            STATISTICS_SERIAL(count_rx_stop_err++);
            rxON = false;
        } else if(rxEND) {
            // frame completed but not yet processed by user process
            STATISTICS_SERIAL(count_rx_overflow_err++);
        } else if(rxON) {
            // TODO check for unexpected T1941_START_OF_FRAME, log error, and start a new frame
            if(rxByte == T1941_END_OF_FRAME) {
                // Discard EOF byte
                rxEND = true;
            } else if(rxbufIdx >= RXBUF_MAX_MSG) {
                STATISTICS_SERIAL(count_rx_overflow_err++);
                rxON = false;
            } else {
                rxbuf[rxbufIdx++] = rxByte;
            }
        } else if(rxByte != T1941_START_OF_FRAME) {
            STATISTICS_SERIAL(count_rx_frame_err++);
        } else {
            // Discard SOF byte - just set rxbufIdx to zero
            // TODO timer2_rxTimeout watchdog drops frame if data is not expected (see handle_rx_brake())
            rxbufIdx = 0;
            rxON = true;
        }
    } else {
        rxByte >>= 1;
        if(pins & PORTC_MASK_RX_COMMON)
            rxByte |= 0x80;
    }


}

#endif

#ifdef WITH_ASM8051_TX
// uses r1,r2,r3,r4 of REG_SET #1 - so for same level ISR there is r0,r5,r6,r7 left
void timer0_isr_bitbang_tx(void)  __interrupt TF0_VECTOR  __using(1)  __naked {
    // DPTR1=OUTC
    // r1 = txbuf
    // r3 -> txNext
    // r2 -> txBitcnt
    // r4 -> txByte

    __asm

    push    psw
    mov     psw,#0x88     // switch to bank 1 and set CY bit
    inc     _DPS          // switch to alternate DPTR ... only slightly faster than push/pop
    xch     a,r3

    //mov     r0,#_OUTC
    mov     dptr,#_OUTC
    movx    @dptr,a       // NOTE "a" is 0x00 or 0xff (works only if "THE ONE" right PIN at PORTC is set to OUT)

    djnz    r2,timer0_byte
#ifdef WITH_ASM8051_RXTX_PAGED_XDATA
    movx    a,@r1
#else
    mov     a,@r1
#endif
    cjne    a,#T1941_END_OF_FRAME,timer0_new_byte

    clr     _ET0            // switch off timer0
    clr     _txON

    sjmp    timer0_end

timer0_new_byte:
    mov     r2,#TX_BITS_PER_BYTE        // txBitCnt = TX_BITS_PER_BYTE;
    inc     r1                          // txbuf++
    movx    a,@r1
    mov     _R1r4,a                     // txByte = *txbuf

    clr     a                           //  txNext = 0x0;
    sjmp    timer0_end

timer0_byte:
    //CY is set via PSW  setb    c                           // txByte |= 0x80
    mov     a,r4
    rrc     a                           // txByte |= 0x80 ; txByte >>= 1
    mov     r4,a
    subb    a,r4                        // txNext = (txByte & 0x01) ? 0xff : 0x00;

timer0_end:
#ifdef WITH_STATISTICS_IRQ
    inc     _count_t0                   // statistics
#endif

    xch     a,r3
    inc     _DPS
    pop     psw
    reti

    __endasm;
}

#else
void timer0_isr_bitbang_tx(void)  __interrupt TF0_VECTOR  {

    OUTC = txNext;

    if(txBitCnt > 0) {
        txNext = (txByte & 0x01) ? 0xff : 0x00;
#if 0
        // we have only ONE out pin on portC (the TX we want to write here), so this is not really necessary
        txNext &= txNextBITSET;
#endif
        txByte >>= 1;
        txByte |= 0x80; // feed with 'stopbits'
        txBitCnt--;
    } else  if(txbuf[txbufIdx] != T1941_END_OF_FRAME) {
        txBitCnt = TX_BITS_PER_BYTE;
        txbufIdx++;
        txByte = txbuf[txbufIdx];
        txNext = 0x0; // startbit
    } else {
        ET0 = false;
        txON = false;
    }

    STATISTICS_IRQ(count_t0++);
}
#endif

void ie0_isr_bitbang_rx(void) __interrupt IE0_VECTOR {
    // sync Timer1 with Rx => Load TL1 with TH1+(TH1>>1) to sample the PIN "in the middle" of the bit frame
    // Rx is also connected to PINC.2

    //if( (brake_state & BRAKE_RX_ENABLED_TYPE) ) {
        TR1 = false;
        TL1 = RX_BITBANG_SAMPLING_OFFSET;
        TF1 = false;
        TR1 = true;
    //}

    if(txON) {
        // it works - but we count the case
        STATISTICS_SERIAL(count_txrx_collision++);
    }
    STATISTICS_IRQ(count_ie0++);
}

// TODO used by main -> no inlining until we move it to a "serial.h"
inline void configureBitBang() {

    // NOTE: for bitbang PT0 and PT1 MUST have the same priority, because the implementation of 
    // Rx/Tx Bitbang does not allow to interrupt each other. 

    // -------------------------------------------------------------
    // Timer0 ISR is exclusively used by Bitbang TX
    // -------------------------------------------------------------
    RESET_TIMER0();

    PT0 = true;         // Timer0 High-Prio ON (bitbang) 
    TMOD |=  M01;
    TH0 = TX_BITBANG_SAMPLING_TIMER;
    TR0 = true;         // Timer0 RUN

    // ET0 IRQ is still switch off - we switch it on, when we start tx-bitbang transmission
    // -------------------------------------------------------------
    // Timer 1 RX (used for PAC/PWM in eddy current mode)
    // -------------------------------------------------------------
    RESET_TIMER1();

    PT1 = true;         // Timer1 High-Prio ON (bitbang)
    ET1 = true;         // irq ON - we controll TR1
    TMOD  |= M11;       // GATE1=int-gate, CT1=Counter/Timer select, M11/M10=mode select (b00=13bit,b01=16bit,b10=8bit reload,b11=timer1 stopped)

    TH1 = RX_BITBANG_SAMPLING_TIMER;
    TL1 = RX_BITBANG_SAMPLING_OFFSET;

    PX0 = true;            // IE0 Priority HIGH for BITBANG (RX) to detect startbit 
    INSTALL_ISR(TF1_VECTOR, &timer1_isr_bitbang_rx);
    INSTALL_ISR(IE0_VECTOR, &ie0_isr_bitbang_rx);

#ifdef WITH_ASM8051_RX
    R1r6 = 0x00;    // rxBitCnt initialize
#endif

    EX0 = true;
}
#endif


#ifdef WITH_MOTORBRAKE

#ifdef WITH_ASM8051
bool parity(uint8_t in) __naked  {
    in; // avoid warning
    __asm

    mov a,dpl
    mov c,_P
    ret

    __endasm;
}
#else
bool parity(uint8_t in) {
    in = (in>>4)^in;
    in = (in>>2)^in;
    in = (in>>1)^in;
    return in&1;
}
#endif

// call only from user space - not reentrant

#define CHECKSUM_PATTERN  0xc001
#define CHECKSUM_START    0x0000
// with 0x01 start of frame precoded: (0x0000^0xc001)^(0x01<<6)^(0x01<<7)
#define CHECKSUM_START_01 0xc0c1

uint16_t checksum1(__pdata uint8_t* buf, uint8_t bufsize) {
    uint16_t shiftreg = CHECKSUM_START_01;
    uint8_t tmp;
    while(bufsize-- > 0) {
        tmp = shiftreg ^ *buf++;
        shiftreg >>= 8;

        if(parity(tmp)) {
            shiftreg ^= CHECKSUM_PATTERN;
        }

        shiftreg ^= ((uint16_t) tmp ^ ((uint16_t) tmp <<1)) << 6;
    }
    return shiftreg;
}

#ifdef WITH_ASM8051
static uint8_t tx_Nibble2Char(uint8_t in) __naked  {
    in; // avoid warning
    __asm

    mov a,dpl
    anl a,#0xf
    add a,#0x30
    da  a
    jnb acc.6,tx_Nibble2Char_end
    inc a
tx_Nibble2Char_end:
    mov dpl,a
    ret

    __endasm;
}
#else
uint8_t tx_Nibble2Char(uint8_t nibble) {
    nibble &= 0xf;
    return nibble > 9 ? nibble + 0x41 - 10 : nibble + 0x30;
}
#endif


inline void encodeTx(__xdata uint8_t* tx_src, uint8_t txlen) {
    uint8_t  i,ptr=0;
    uint16_t chk;

    if(txlen > TXBUF_MAX_MSG_ACCEPT || !(brake_state & BRAKE_TX_ENABLED_TYPE)) {
        STATISTICS_SERIAL(count_tx_err++);
        return;
    }

    // bin2hex
    txbuf[ptr++] = T1941_START_OF_FRAME;
    for(i=0;i<txlen;i++) {
        uint8_t c = tx_src[i];
        txbuf[ptr++] = tx_Nibble2Char(c>>4);
        txbuf[ptr++] = tx_Nibble2Char(c);
    }
    chk = swap16(checksum1(&txbuf[1], ptr-1));
    for(i=4;i>0;i--) {
        txbuf[ptr+i-1] = tx_Nibble2Char(chk);
        chk >>= 4;
    }
    ptr += 4;
    txbuf[ptr++] = T1941_END_OF_FRAME;
}


/*
 * a standard data frame has 52 bytes:
 *   SOF-BYTE-0x01[1] | HEXCODEED-MESSAGE[2*23] | HEXCODEDED-CHECKSUM[2*2] | EOF_BYTE-0x17[1]
 *
 * Note: The 0x1 SOF (start-of-frame-byte) and 0x17 EOF (end-of-frame) bytes are discarded
 *       in the RX-function. They are NOT hexcoded.
 *       We process here only the remaining HEX-coded part
 *
 * Minimum size is 4 bytes (just checksum, no msg)
 * Maximum size is 50 - (note: the "version"-Frame (answer on 0x02000000 command) is shorter)
 * odd sizes are are not valid (but we do not check - checksum will fail)
 *
 * => max decoded frame is 25 bytes (23 message + 2 checksum)
 *
 * 46 data bytes "hex coded"    => 23 bytes data
 *  4 checksum bytes "hex coded" =>  2 bytes checksum
 *
 * return 'valid' bool: true = OK, false=ERROR
 */
inline bool decodeRx(__xdata uint8_t *ptr) {
    uint8_t  b,c,i;
    uint16_t sentChecksum;

    b = 0; // omit warning

    if(rxbufIdx < 4 || rxbufIdx > 50) {
        return false;
    }

    // hex2bin : magic: ( 'ascii' | 0x10) % 39 - 9
    // '0'-'9' => 0x0-0x9
    // 'A'-'F' => 0xa-0xf
    // 'a'-'f' => 0xa-0xf
    //for(si=1;si<rxbufIdx-1;si++) {
    for(i=0;i<rxbufIdx;i++) {
        c = rxbuf[i];
        // 0 remains 0 (to omit the 0x02-initial command bug)
        if(c != 0) {
            c = (c | 0x20) % 39 - 9;
        }
        if(c & 0xf0)
            goto rxDataWithError;

        b <<= 4;
        b += c;
        if(i & 1) {
            *ptr++ = b;
        }
    }

    // sentChecksum = (b<<8)|lastB; // ((__xdata uint16_t*)(rxDecoded+(rxbufIdx>>1)))[-1];
    sentChecksum = ((__xdata uint16_t*)ptr)[-1];
    rxChecksum = checksum1(&rxbuf[0], rxbufIdx-4);

    if(rxChecksum == sentChecksum) {
        STATISTICS_SERIAL(count_rx_valid++);

        return true;
    }
rxDataWithError:
    count_rx_err++; // general error count
    STATISTICS_SERIAL(count_rx_checksum_err++);
    return false;
}

/*****************************************************************************
 * splitted in "prepare" and "send"
 * We prepare a message when we got it from the host. We do not want to
 * block the OUT2BUF -> copy and encode
 * The send is delayed if there is another tx going on or if we want
 * to do a 'steering' measurment
 ****************************************************************************/

void serial_command_tx(uint8_t cmdSize) {

    encodeTx(txDecoded, cmdSize);

#if defined(WITH_BITBANG) && !defined(WITH_ASM8051_TX) || defined(WITH_SERIAL0)
    txbufIdx = 0x00;  // Not used for bitbang with assembler
#endif
    txON = true;

#ifdef WITH_BITBANG

#  ifdef WITH_ASM8051_TX
    R1r1  = (uint8_t)txbuf;
    R1r2  = TX_BITS_PER_BYTE;
    R1r3  = 0x0;  // startbit
    R1r4  = T1941_START_OF_FRAME;
#  else
    txBitCnt = TX_BITS_PER_BYTE;
    txNext       = 0x0;  // startbit
    txNextBITSET = headunit_device & HEADUNIT_SOLIDGREEN_TYPE ? PORTC_MASK_MAGNET : PORTC_MASK_TX;
    txByte = T1941_START_OF_FRAME;
#  endif

    // TR0 allready running: just start IRQ-Handling
    TF0 = false;
    ET0 = true;

#elif defined(WITH_SERIAL0)

    SBUF0 = T1941_START_OF_FRAME;

#endif

    // maximum time after we expect a full received valid  answer:
    // in normal operation: worst case is about 41ms - so 60ms is enough
    timer2_serial_timeout = TIMER2_RX_TIMEOUT;
}

void serial_handle_rx() {
    if(rxEND) {
        if(decodeRx(rxDecoded)) {
            u_semaphore_rx_valid = true;
            u_semaphore_rx_valid_4stateMachine = true;
        }
        timer2_serial_timeout = 0x00;
        rxON  = false;
        rxEND = false;
    } else if(rxON && timer2_serial_timeout == 0x00) {
        // rx timeout - reset rx state
        count_rx_err++; // general error count
        STATISTICS_SERIAL(count_rx_timeout_err++);
        rxON  = false;
        rxEND = false;
    }
}


#endif

