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

#ifndef __MAIN_H
#define __MAIN_H

#include <stdint.h>
#include <stdbool.h>

#define EZTRACON_8051

#include "math.h"

#include "reg_ezusb.h"
#include "usb.h"
#include "i2c.h"

#include "blinkcode.h"
#include "antplus.h"
#include "tacx.h"
#include "trainer.h"
#include "serial.h"
#include "eddycurrent.h"
#include "models.h"

// ========================================
// special development defines  => use Makefile to configure
// ========================================
//#define WITH_ANALYZER
//#define WITH_DEVELOPMENT
//#define WITH_DEVELOPMENT_SERIAL
//#define WITH_STATISTICS_GENERAL
//#define WITH_STATISTICS_IRQ
//#define WITH_STATISTICS_SERIAL
// ========================================

// ========================================
// functional defines => use Makefile to configure
// ========================================
// #define WITH_EMULATE_USB_ANTFE_C
// #define WITH_USB_ID_T1942_SOLID_BLUE
// #define WITH_USB_ID_T1902_SOLID_GREEEN
// #define WITH_EDDYCURRENT
// #define WITH_MOTORBRAKE
// #define WITH_BITBANG
// #define WITH_SERIAL0
// #define SERIAL_8N2
// ========================================

// use 2-6 bytes commands an 21 bytes answers of the T1902
// TODO #define FORCE_LEGACY_PROTOCOLL

#define WITH_ASM8051
#define WITH_ASM8051_TX
#define WITH_ASM8051_RX
#define WITH_ASM8051_RXTX_PAGED_XDATA

// --------------------------------------------

// #define DEBUG_EC_DRY_RUN

#define WITH_HEARTRATE
#define WITH_BLINKCODE
#define WITH_STEERING

// --------------------------------------------

#ifndef WITH_MOTORBRAKE
#undef WITH_BITBANG
#undef WITH_SERIAL0
#endif

#ifdef WITH_STATISTICS_SERIAL
# define STATISTICS_SERIAL(a) a
#else
# define STATISTICS_SERIAL(a)
#endif

#ifdef WITH_STATISTICS_GENERAL
# define STATISTICS_GENERAL(a) a
#else
# define STATISTICS_GENERAL(a)
#endif

#ifdef WITH_STATISTICS_IRQ
# define STATISTICS_IRQ(a) a
#else
# define STATISTICS_IRQ(a)
#endif

// --------------------------------------------


// ==============================================
// states of connected brake
// ==============================================

// BRAKE_RX_ENABLED_TYPE: we listen to RX (for motorbrake)
// note: this is a state for AUTODETECT and MOTORBRAKE. If we are in EDDY_CURRENT this
// PIN is to count the wheel signals for speed
#define BRAKE_RX_ENABLED_TYPE           (0x08)
// BRAKE_POWERLINE_TYPE: signals on powerline PIN
#define BRAKE_POWERLINE_TYPE            (0x10)
// EDDYCURRENT_TYPE: powerline with stable frequency
#define BRAKE_EDDYCURRENT_TYPE          (0x20)
// BRAKE_TX_ENABLED_TYPE: we enable TX as OUT with active LOW
#define BRAKE_TX_ENABLED_TYPE           (0x40)
// ERROR_TYPE: illegal state change
#define BRAKE_ERROR_TYPE                (0x80)


#define BRAKE_ERROR                     (BRAKE_ERROR_TYPE      | 0x01)
#define BRAKE_AUTODETECT                (BRAKE_RX_ENABLED_TYPE | 0x01)
#define BRAKE_POWERLINE_DETECTED        (BRAKE_POWERLINE_TYPE  | 0x01)
#define BRAKE_EDDYCURRENT_50HZ          (BRAKE_POWERLINE_TYPE  | BRAKE_EDDYCURRENT_TYPE | 0x01)
#define BRAKE_EDDYCURRENT_60HZ          (BRAKE_POWERLINE_TYPE  | BRAKE_EDDYCURRENT_TYPE | 0x02)
#define BRAKE_MOTORBRAKE                (BRAKE_RX_ENABLED_TYPE | BRAKE_TX_ENABLED_TYPE  | 0x01)
#define BRAKE_MOTORBRAKE_TESTING        (BRAKE_RX_ENABLED_TYPE | BRAKE_TX_ENABLED_TYPE  | 0x02)

// TODO: not yet implemented: Passive modes sends a single test command to motorbrake when starting
// and then waits for any kind of reaction from motorbrake ("zerobyte", regualar answer)
// or a powerline signal from eddy current
// #define BRAKE_PASSIVE                   (BRAKE_RX_ENABLED_TYPE | 0x02)
// #define BRAKE_PASSIVE_TESTING           (BRAKE_RX_ENABLED_TYPE | BRAKE_TX_ENABLED_TYPE  | 0x03)

// autodetect:
// 1. if we have a powerline signal, we never switch to motorbrake mode. If we detect a powerline signal while
//    being in (full) motorbrake mode, we switch to "error" ... to reset error state, you have to reload the
//    firmware. If we detect powerline signal while being in motorbrake_testing, we switch to BRAKE_POWERLINE_DETECTED
// 2. if powerline frequency is 50Hz or 60HZ we switch to (valid detected) "eddy current" mode
// 3. to detect the motorbrake, from time to time we switch from BRAKE_AUTODETECT to BRAKE_MOTORBRAKE_TESTING
//    and send a 0x2,0x0,0x0,0x0 command on the Tx wire. If we do not get an answer within 100ms we switch back
//    to BRAKE_AUTODETECT. If we we got a valid answer we switch to motorbrake mode.
// 3b. the motorbrake seems to send a 0x00 about 5 seconds after switch on, so another strategy is to wait
//    for the 0x00 byte and switch to "testing" after the receive. In case the motorbrake is still switched on when
//    loading the firmware, we can send a few initial testing messages (if not case 1) in the beginning and go
//    to "wait for (any) brake" mode after
// Note: we ignore every Rx signal, if we are not in a BRAKE_TX_ENABLED_TYPE state (testing or valid motorbrake), 
//       because we do not expect RX data without sending a command to the brake
// 4. going to USB-suspend (currently not supported) we switch BRAKE_AUTODETECT and try to redetect brake after
//    resume
//

extern  __data volatile  uint8_t brake_state;

// ==============================================
// supported head units
// ==============================================

// solid-green type head units (not "soylent green" :-) control motorbrake/magnetic field on PINC.7 (no serial0 connected)
#define HEADUNIT_SOLIDGREEN_TYPE        (0x10)
// solid-blue type head units control motorbrake/magnetic field on PINC.1 (the serial0-TX PINC)
#define HEADUNIT_SOLIDBLUE_TYPE         (0x20)

// T1902 iMagic, T1942 Fortius
#define HEADUNIT_UNKNOWN                (0x00)
#define HEADUNIT_T1902                  (HEADUNIT_SOLIDGREEN_TYPE | 0x00)
#define HEADUNIT_T1942                  (HEADUNIT_SOLIDBLUE_TYPE  | 0x00)

extern  __data           uint8_t headunit_device;

#ifndef WITH_USB_POLL
extern void USB_jump_table(void)        __interrupt USB_VECTOR;  /* 0x43 USB sub-interrupts */
#endif

extern void sudav_isr(void)     __interrupt;
extern void sof_isr(void)       __interrupt;
extern void sutok_isr(void)     __interrupt;
extern void suspend_isr(void)   __interrupt;
extern void usbreset_isr(void)  __interrupt;
extern void ibn_isr(void)       __interrupt;
extern void ep0in_isr(void)     __interrupt;
extern void ep0out_isr(void)    __interrupt;
extern void ep12in_isr(void)    __interrupt;
extern void ep12out_isr(void)   __interrupt;

// ==============================================
// analyzer / info / statistics
// ==============================================

extern __xdata __at 0x0000 volatile uint8_t xdata[0x8000];
extern __idata __at 0x0000 volatile uint8_t idata[0x0100];
extern __code  __at 0x0000 volatile uint8_t  code[0x1b40];

#ifdef WITH_ANALYZER
extern void handle_analyzer(bool writeDataPage);
#endif

#ifdef WITH_STATISTICS_GENERAL
extern __idata uint8_t count_in_frames12;
extern __idata uint8_t count_out_frames12;
extern __idata uint8_t count_out_frames_skipped;
extern __idata uint8_t count_steering;
#endif

#ifdef WITH_STATISTICS_IRQ
extern  __data volatile  uint8_t count_ep12_in;
extern  __data volatile  uint8_t count_ep12_out;
extern  __data volatile  uint8_t count_sudav_isr;
extern  __data volatile  uint8_t count_ie0;
extern  __data volatile  uint8_t count_ie1;
extern  __data volatile  uint8_t count_t0;
extern  __data volatile  uint8_t count_t1;
extern  __data volatile  uint8_t count_serial0;
#endif

#ifdef WITH_STATISTICS_SERIAL
extern  __data uint8_t  count_tx_err;
extern  __data uint16_t count_rx_valid;
extern  __data uint8_t  count_rx_timeout_err;
extern  __data uint8_t  count_rx_checksum_err;
extern  __data volatile uint8_t  count_rx_overflow_err;
extern  __data volatile uint8_t  count_rx_frame_err;
extern  __data volatile uint8_t  count_rx_start_err;
extern  __data volatile uint8_t  count_rx_stop_err;
extern  __data volatile uint8_t  count_txrx_collision;
extern  __data volatile uint8_t  count_double_buffer_error;
#endif

// ==============================================

extern __bit  volatile          event_timer2_expired;
extern __bit  volatile          event_1000ms_expired;
extern __bit  volatile          event_125ms_expired;
extern __data          uint8_t  timer2_lastPinsC;
extern __bit  volatile          blinkcode_force_red;
extern __bit  volatile          blinkcode_force_green;

// ==============================================

extern __data uint8_t heart_bpm;

// ==============================================

extern  __data volatile uint32_t count_t2;

#ifndef WITH_EMULATE_USB_ANTFE_C
// nailed to IN2BUF
extern __xdata __at 0x7E00 struct frame classic_inbuf_fortius;
// nailed to OUT2BUF
extern __xdata __at 0x7DC0 struct frameCommandFortius classic_outbuf_fortius;
extern __xdata __at 0x7DC0 struct frameCommandImagic  classic_outbuf_imagic;   // legacy
#endif

// register Bank 1 direct data access (used to init ASM bitbang)
uint8_t __data __at 0x08 R1r0;
uint8_t __data __at 0x09 R1r1;
uint8_t __data __at 0x0a R1r2;
uint8_t __data __at 0x0b R1r3;
uint8_t __data __at 0x0c R1r4;
uint8_t __data __at 0x0d R1r5;
uint8_t __data __at 0x0e R1r6;
uint8_t __data __at 0x0f R1r7;

// we do not need the full precision, so we use the reduced timer 
// 24MHZ/12 = 2MHz with 4096 = 0x1000 as start value (TH2/TL2 = -0x1000 = 0xF000), 
// This simplifies many time based runtime calculations, if we only need
// TH2 precision (which is still about 0.1ms) 
// TIMER2_PER_SECOND is "488.28125" - a Timer2 'click' happens every 2.048ms 
#define TIMER2_COUNTER     (-0x1000)
#define TIMER2_LOW      LO8(TIMER2_COUNTER)
#define TIMER2_HIGH     HI8(TIMER2_COUNTER)
#define TIMER2_TL2_IS_ZERO

// TIMER2_MHZ(2) * 1000000(MHz) * 1.0(sec) / TIMER2_COUNTER = 488.28125 = ~488
#define TIMER2_PER_SECOND    488
// 61.03515625 = ~61
#define TIMER2_PER_1_8_SEC   (TIMER2_PER_SECOND>>3)
// TIMER2_MHZ(2) * 1000000(Mz) * 60.0(sec) / TIMER2_COUNTER = 29296.875 = ~29297
#define TIMER2_PER_MINUTE    (29297)   
// about 60ms
#define TIMER2_RX_TIMEOUT    (29)

// ===============================================================================

// all values in timer2-"clicks"
#define HR_DEADTIME     (80)
// max.-time is min.-heart-bpm: more than 2 seconds => less than 30 BPM -> set to '0'
#define HR_MAX_TIME     (TIMER2_PER_SECOND*2)
#define HR_MIN_VALUE    (0)
// minimum time is maximum heart-bpm: less than 1/4 seconds => more than 240 BPM -> clip
#define HR_MIN_TIME     (TIMER2_PER_SECOND/4)
#define HR_MAX_VALUE    (240)

// ===============================================================================

#define CAD_DEADTIME    (80)
// max-time is min-pedaling-rpm: more than 2 seconds => less than 30 RPM pedaling -> set to '0'
#define CAD_MAX_TIME    (TIMER2_PER_SECOND*2)
#define CAD_MIN_VALUE   (0)
// min time is max-pedaling-rpm: less than 1/4 seconds => more than 240 RPM pedaling -> clip
#define CAD_MIN_TIME    (TIMER2_PER_SECOND/4)
#define CAD_MAX_VALUE   (240)

// ===============================================================================

// Timer0: STOP, Overflow-Reset, IRQ=OFF
// GATE0=int-gate, CT0=Counter/Timer select,
// M01/M00=mode select (b00=13bit)
// Divider 12 for Timer0
#define RESET_TIMER0() {                \
        TR0 = false;                    \
        TF0 = false;                    \
        ET0 = false;                    \
                                        \
        TMOD &= ~(M01|M00|GATE0|CT0);   \
        CKCON &= ~T0M;  }

// Timer1: STOP, Overflow-Reset, IRQ=OFF
// GATE1=int-gate, CT1=Counter/Timer select,
// M11/M10=mode select (b00=13bit)
// Divider 12 for Timer1
#define RESET_TIMER1() {                \
        TR1 = false;                    \
        TF1 = false;                    \
        ET1 = false;                    \
                                        \
        TMOD &= ~(M11|M10|GATE1|CT1);   \
        CKCON &= ~T1M;  }

#endif

#define swap16(num) ((num>>8)|(num<<8))
#define swap32(num) ((num>>24)|(num<<24)|((num>>8)&0xff00)|((num<<8)&0xff0000))

// #define INSTALL_ISR(vector,function) *((__xdata uint16_t *)((vector)*8+4)) = swap16((uint16_t)(function));
#define INSTALL_ISR(vector,function)  { *((__xdata uint8_t *)((vector)*8+4)) = HI8(function); *((__xdata uint8_t *)((vector)*8+5)) = LO8(function); }
