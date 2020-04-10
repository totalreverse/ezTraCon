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
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *l
 ***************************************************************************/

#include "main.h"

/*
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
!!!!!!                                                                 !!!!!!
!!!!!!           This firmware is highly experimental!                 !!!!!!
!!!!!!                 USE AT YOUR OWN RISK !!                         !!!!!!
!!!!!!                                                                 !!!!!!
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
!                                                                           !
! This 'magic' firmware, can handle all combinations of brakes (motorbrake  !
! or magnetic brake) and a 'classic' head units (solid green, solid blue).  !
!                                                                           !
! Caution: Always connect head unit and brake before you switch on the      !
! brake and the head unit. The connection is TTL logic not RS232.           !
!                                                                           !
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

Supported brakes: 
ONLY 230V/50Hz EU brakes are tested
US versions (110V/60Hz) are NOT tested (but may work) 

T1601 - old eddy current brake 
T1901 - the new(er) eddy current brake NOT tested, but should work  
T1941       motor brake


Supported headunits:
  T1902 (solid green)
  T1942 (solid blue)

===========================================================================
PORT A (same for Imagic & Fortius)
===========================================================================

OUTA PINA.4 0x10 LED Red
OUTA PINA.5 0x20 LED Green

===========================================================================
PORT B (same for Imagic & Fortius)
===========================================================================

similar to imagic: BIT 4-7 Buttons (IN), BIT 0-3 "Gameport"  OUT/IN
PINB/OUTB 0x0f : 555/585 Timer for Gameport/Steering (steering is Bit #2 = mask 0x04)
PINB 0xf0 : Buttons (enter: 0x10, down: 0x20, up: 0x40, cancel: 0x80)

===========================================================================
PORT C
===========================================================================

PINSC.7 magnetic field switch (or :Tx) (only "solid green" T1902) - 1:on 0:off
PINSC.6 not connected / unknown
PINSC.5 Heartbeat                 (alt function: connected to 'T0')
PINSC.4 Cadence (on T1901)        (alt function: connected to 'T1')
PINSC.3 Powerline "zero crossing" (alt function: connected to INT1)
PINSC.2 Wheel (or: Rx)            (alt function: connected to INT0) (see note 1)
PINSC.1 connected to Tx0 (only on "solid blue" T1942)
PINSC.0 connected to Rx0 (only on "solid blue" T1942) (see note 1)

Note 1: on the "solid blue" headunit the "blue" wire is connected to the PINSC.2 and PINSC.0!

 n#   dir            Dev    function
------------------------------------------------------------------------------
PINC0 IN    Serial0  B      Rx Alt-Function (OUT) - for T1941  motorbrake communication

PINC1 OUT   Serial0  B      Tx Alt-Function - for T1941 motorbrake communication
                            misused as magnetic field PAC/PWM/RPC when used with T1901 eddy current brake
PINC2 IN    INT0     B/G    Wheel-Sensor (only when powered) for T1901-EddyCurrent-Brake

PINC3 IN    INT1     B/G    Power-Line-Frequency zero crossing (only when powered) T1901

PINC4 IN    T0       B/G    CAD on T1901 brake, works also with unpowered T1901 (detected by sampling)

PINC5 IN    T1       B/G    HR  - works without brake connected (detected by sampling)

PINC6 (IN)                  Voltage/Power from brake (without load: 15-18V (T1601) or 5.8V (T1942))
                            unused for host powered devices

PINC7 OUT              G    "PAC/PWM/RPC-controls" magnetic field of T1901 eddy current.
                            misused as TX when used with T1941 motorbrake

headunit backside socket
     __|^^^|__
   _|         |_
   |           |  cable  |  connect to PINC# (H/L: signal default when port is 'IN')
   |           |  wire   |       solid-blue         solid-green
   |6 5 4 3 2 1|  ------------------------------------------------
    | | | | | |__ white  |          4  (H)            4  (H)  (3.3V TTL)
    | | | | |____ black  |         --- (L)           --- (L)  (mass?)
    | | | |______ red    |          1  (L)            7  (L)
    | | |________ green  |          3  (L)            3  (L)  (sinus (AC) ~15V internally converted to 0V/~5V)
    | |__________ yellow |             --- V_Brake ---   (Voltage/Power from brake 15-18V (T1901) or 5.8V (T1942))
    |____________ blue   |     0 and 2 (H)            2  (H)  (3.3v TTL?)

One can see, that on both devices the important wires are connected to a pin
at the CPU. Unfortunately the serial communication wire of the motorbrake
is not connected to the serial0-Tx/Rx pins in the 'solid-green' headunit
The only way to control the motorbrake with the solid-green is to emulate
a serial signal with "bitbanging" on PINC.2 (INT0) for Rx and PINC.7 for Tx,
which is time critical and more expensive - but possible.

==============================================================================
TIMERS (how we use it)
==============================================================================

Bitbang Mode:      Timer 0 is TX-Timer, 
                   Timer1 is RX-Timer (Together with ie0 for syncing)

Serial Mode:       Timer 1 is Baudgenerator/Timer for serial 0 (Motorbrake) 

Eddy Current Mode: Timer 1 (together with powerline ie0-interrupt) triggers start of PAC/PWM/RPC (Magnetic on) 
                   in first or second half of the powerline sinus and then controlls the length of the impuls.
                   (Timer0 currently unused in Eddy Current mode)

Timer2:       main application timer currently running with 2 MHz and a "downcount" of 0x1000 
              => 2000000/4096 = 488.28125 interrupts per second = 2.048 ms per Timer 2 interrupt

if you increase the sampling frequency, you get a more precise timing, but ~2ms is good enough for most.

OLD:
Timer1a : (Motorbrake): Baud generator (with Overflow-IRQ off) or Rx bitbang with IRQ
Timer1b : (Eddy Current Brake): PAC controller  - in sync with IE1 (with Overflow-IRQ on)
Timer2  : (both) beat is 2.048ms = 1/512 sec. with reload - IRQ on
          (both) also used to count ticks for steering


Note: some uint8_t data fields may overflow if you increase or decrease the number of events. For example:
  count_CADTDeadtime
  count_HRDeadtime
  timer2_serial_timeout
  timer2_125ms_countdown

==============================================================================
T1941 serial communication:
==============================================================================

The commands and answers all look like "4-bytes-header" | Msg-Data | "2-bytes-checksum"
the length of the msg is coded in the second byte of the header (header[1])
initial Motorbrake command:
02 00 00 00
   => short answer from T1941 brake (and what T1904 says instead)
    T1941[24+16+2]: 03 0c 00 00 65 09 00 00 ba c4 77 18 08 0c 00 00 [c4 70]
    T1904[24+40]:   03 0c 00 00 02 19 00 00 00 00 00 00 00 00 00 00 35 69 00 00 00 00 02 55 64 [00...]

Note: If you send 0x02,0x00,0x00,0x00 commands to early (while switching on the brake), the brake 
initialization is incomplete and the brake sends the following answer:
1-2 times:
 01 46 46 30 32 30 30 30 30 43 44 46 46 39 44 38 31 17
    => ff 02 00 00 cd ff
and then:
 01 30 33 30 43 30 30 30 30 36 35 30 39 30 30 30 30 42 41
 43 34 37 37 31 38 30 38 30 43 00 00 00 00 44 46 36 42 17
                               ^^^^^^^^^^^
Instead of converted zeros "30 30 30 30" the brake sends real zeros "00 00 00 00"
A (valid) checksum is calculated based on "00 00 00 00"

standard Motorbrake command:

|    Header   |  Message ....
|  0  1  2  3 |     4        5     |        6         |  7 |   8  |    9   |      10            11
| 01 08 01 00 |  LOAD_LSB LOAD_MSB | PEDALSENSOR_ECHO | 00 | MODE | WEIGHT | CALIBRATE_LSB CALIBRATE_MSB

LOAD depends on the operating mode
  0. Off-Mode:       set to zero
  1. Force-Mode:     load = (power - offsetPower) / scaleLoad / v_raw - offsetLoad 
                     A force of 1 Newton [N] corresonds to a load of about ~137. 
  3. Calibrate-Mode: load = f_speed(speed)      with f ~= speed(kph) * 290            with speed = [0..20++] (be careful)

CALIBRATE adds/reduce the resistance to compensate the friction of the system in Force mode
    Off-Mode:          set to zero (but probably value does not matter?)
    Force mode: 
      total_load_offset = offsetLoad - calibrate
      default CALIBRATE = 0x0410 - 
      If you increase the calibration value, the trainer reduce the (real) force.
      As long as "load - calibrate = const", the trainer has a constant resistance
    Calibrate-Mode:    set to zero (but probably value does not matter?)

   PEDALSENSOR_ECHO:
       set Bit 0 (=0x1) to echo the pedal-sensor event from bit 0 of byte 42 of brake answer frame
       if the motorbrake does not get an echo for some time, the brake stops

   WEIGHT (virtual flywheel):
       The weight is not riders weight, because it does no have an effect on the 
       force when riding at constant speed. It's a (virtual) flywheel weight, which has only an
       effect if the speed changes (adds some force if you accelerate and gives back
       some energy, if you stop pedalling). 
       
       Off-Mode:       probably value does not matter
       Force-Mode:     Use "0x0a" (small weight), if you have your own simulation modell. 
                       Do not use values under 0x0a. Use higher values to simulate 
                       the acceleration of a mass. This does not simulate a weight riding
                       uphill.                
       Calibrate-Mode: probably value does not matter

   MODE:
       Off-Mode:           set to "0"
       Force-Mode:         set to "2" 
       Calibrate-Mode:     set to "3"

   => long answer from brake (and what T1904 says instead)
    T1941[24+23+1]: 03 13 02 00 00 00 00 00 00 00 f0 61 00 00 00 00 80 06 00 00 00 00 02 [e5] // one byte checksum fragment
    T1904[24+40]:   03 13 02 00 00 00 00 00 00 00 00 00 0f 04 0f 04 43 a8 00 00 00 00 02 55 64 [00...]
    T1932[24+40]:   TODO

illegal message (if you need a very short testmessage to detect motorbrake):
01
   => the middle part and the checksum can vary
    T1941[24+6+2] : ff 02 14 00 ce ff [48 4d]    -> little endian is 02FF, 0014, FFCE [cksm]
    T1904[24+49]  : normal answer
    T1932[24+40]:   TODO

Alternatives:
  * Raspi or clones
  * Arduino or clones ( !! ATTENTION !! standard arduino signal voltage is 5V which is (probably) too much for the brake)
  * a simple TTL-USB cable, if you have the motorbrake and do not need the HR-signal (HR is a head-unit feature).
    => original signal TTL level is below 3v ! ==> use a voltage divider if you have 5v - probably 3.3v is safe

*/

/*
==============================================================================
ezUsb Interrupts
==============================================================================

 See Manual an2131_trm Table 9-1. EZ-USB Interrupts:
New P 8051 Interrupt (IRQ name)     SourceVector          (hex) Natural Priority
    IE0 (0)                         INT0# Pin               03      1
    TF0 (1)                         Timer 0 Overflow        0B      2
    IE1 (2)                         INT1# Pin               13      3
    TF1 (3)                         Timer 1 Overflow        1B      4
    RI_0 & TI_0  i(4)               UART0 Rx & Tx           23      5
    TF2 (5)                         Timer 2 Overflow        2B      6
P   Resume (PFI) (6)                WAKEUP# Pin or USB Core 33      0
P   RI_1 & TI_1 UART1 (7)           Rx & Tx                 3B      7
P   USB (INT2) (8)                  USB Core                43      8
P   I C (INT3) (9)                  USB Core                4B      9
--
P   IE4  (10)                       IN4 Pin                 53      10
P   IE5  (11)                       INT5# Pin               5B      11
P   IE6  (12)                       INT6 Pin                63      12

USB,I2C,TIMER2 IRQs are set to LOW prio
T0,T1,IE0,IE1,serial0 IRQs are set to HIGH prio

*/

/*
==============================================================================
USB control
==============================================================================

The following information is from the ezUsb technical reference

============
usb disconnect
============
The logic for the DISCON and DISCOE bits is shown in Figure 5-2. . This
arrangement allows connecting the 1,500-ohm resistor directly between the DISCON# pin
and the USB D+ line (Figure 5-3)

============
IRQ reset(mass?)
============
Note
Any USB ISR should clear the 8051 INT2 interrupt request bit before clearing any of the
EZ-USB endpoint IRQ bits, to avoid losing interrupts. Interrupts are discussed in more
detail in Chapter 9, "EZ-USB Interrupts."
Individual interrupt request bits are cleared by writing “1” to them to simplify code. For
example, to clear the endpoint 2-IN IRQ, simply write “0000100” to IN07IRQ. This will
not disturb the other interrupt request bits. Do not read the contents of IN07IRQ, logi-
cal-OR the contents with 01, and write it back. This clears all other pending interrupts
because you are writing “1”s to them.

============
control msg
============

8051 clears HSNAK bit (writes 1 to it)
or sets the STALL bit.

As with BULK endpoints, the endpoint zero byte count registers must be loaded
to ACK the data transfer stage of a CONTROL transfer

The HSNAK bit is used to hold off completing the CONTROL transfer until the device
has had time to respond to a request. For example, if the host issues a Set_Interface
request, the 8051 performs vari(mass?)ous housekeeping chores such as adjusting internal modes
and re-initializing endpoints. During this time the host issues handshake (STATUS stage)
packets to which the EZ-USB core responds with NAKs, indicating “busy.” When the
8051 completes the desired operation, it sets HSNAK=1 (by writing a “1” to the bit) to ter-
minate the CONTROL transfer. This handshake prevents the host from attempting to use
a partially configured interface.

Note
To indicate an endpoint stall on endpoint zero, set both EP0STALL and HSNAK bits.
Setting the EP0STALL bit alone causes endpoint zero to NAK forever because the host
keeps the control transfer pending
*/

/*
From sdcc manual:

  __bit, __code, __idata, __data, __pdata, __xdata, __sfr / __sfr16 / __sfr32 / __sbit

pointer physically in internal ram pointing to object in external ram
  __xdata unsigned char * __data p;

pointer physically in external ram pointing to object in internal ram
  __data unsigned char * __xdata p;
pointer physically in code rom pointing to data in xdata space
  __xdata unsigned char * __code p;

pointer physically in code space pointing to data in code space
  __code unsigned char * __code p;

generic pointer physically located in xdata space
  unsigned char * __xdata p;

generic pointer physically located in default memory space
  unsigned char * p;

the following is a function pointer physically located in data space
  char (* __data fp)(void);
*/

/*********************************************************
 * general
 ********************************************************/

// state is a combinaton of brake-type and brake-state
__data volatile uint8_t brake_state, last_brake_state = 0xff;           
__data  uint8_t headunit_device;


// Trainer 'simulation' state
__pdata struct trainer trainer;


#ifdef WITH_HEARTRATE

__data uint8_t heart_bpm;

__data uint16_t count_hr_timeout;
__data uint16_t count_hr_last;
__bit  lastHRPin;
__bit  volatile event_hr;
__bit  volatile hrStalled;

#define HEARTRATE() (heart_bpm)
#else
#define HEARTRATE() (0)
#endif

#ifdef WITH_EDDYCURRENT
// CAD is EDDY_CURRENT only

__data uint16_t count_cad_timeout;
__data uint16_t count_cad_last;
__bit  lastCADPin;
__bit  volatile event_cad;
__bit  volatile cadStalled;
__bit  event_pedal_detect;
__data uint8_t eddycurrent_cadence;

#endif

// reduce initial interval to 'interleave' timer125 and timer1000 (so they never occur together)
__data volatile uint8_t timer2_125ms_countdown  = TIMER2_PER_1_8_SEC>>1;
__data volatile uint16_t timer2_1000ms_countdown = TIMER2_PER_SECOND;

__bit volatile event_timer2_expired;
__bit volatile event_125ms_expired;
__bit volatile event_1000ms_expired;

// timer2 related
// count_t2 with uint32_t as master counter, overflows after 3 months (with TIMER2_PER_SECOND=512)
__data volatile uint32_t count_t2;

__data lastPinB = PORTB_MASK_BUTTONS;

#ifdef WITH_STEERING

// we sync with Timer2 and count T2-ticks
// => CPU burns max. 1 ms (with four clocks/cycle => ~6000 cycles) in waitloop 
// minimum is about ~1500 cycles - maybe we should use this time for some computations ??
// Gameport timing - stop measurment after 8*256  TIMER2 counts (24MHz/12 = 2MHz => ~1 ms) -
#define AXIS_TIMEOUT_AFTER_TH2 ((TIMER2_HIGH)+8)
// divide measured T2 time by (2^AXIS_TIMERSHIFT) and ...
#define AXIS_TIMER_SHIFT (2)   
// ... correct measured value by AXIS_TIMER_OFFSET
#define AXIS_TIMER_OFFSET (0)   

// average measured values AXIS_AVERAGE times (AXIS_AVERAGE must be pow-of-2 (2,4,8,...))!
#define AXIS_AVERAGE (4)  

// the value we report, if we have a timeout (= steering unplugged)
#define AXIS_TIMEOUT_VALUE_T1942       (2000)
#define AXIS_TIMEOUT_VALUE_T1932_T1904 (2573)
#define AXIS_TIMEOUT_VALUE AXIS_TIMEOUT_VALUE_T1932_T1904

__data uint16_t steering = AXIS_TIMEOUT_VALUE;
__pdata uint16_t steering_avg[AXIS_AVERAGE];
__data uint8_t steering_idx;
__bit volatile start_steering;
__bit volatile steering_running;
__bit schedule_do_steering;

#endif

/*********************************************************
 ********************************************************/

#ifdef WITH_STATISTICS_GENERAL
// General statistics
__idata uint8_t count_in_frames12;
__idata uint8_t count_out_frames12;
__idata uint8_t count_out_frames_skipped;
#ifdef WITH_STEERING
__idata uint8_t count_steering;
#endif
__idata uint8_t count_lost_powerline_freq_error; 
#endif

#ifdef WITH_STATISTICS_IRQ
// IRQ Statistics (= "activity indicator" for development)
__data volatile uint8_t count_ep12_in;
__data volatile uint8_t count_ep12_out;
__data volatile uint8_t count_sudav_isr;
__data volatile uint8_t count_t0;        // timer0 ovfl irq
__data volatile uint8_t count_t1;        // timer1 ovfl irq
__data volatile uint8_t count_ie0;       // int0 ie0-irq (triggered by wheel)
__data volatile uint8_t count_ie1;       // general ie1 counter for statistics (triggered by powerline)
__data volatile uint8_t count_serial0;
#endif

/*********************************************************
 ********************************************************/

#ifndef WITH_EMULATE_USB_ANTFE_C

__bit eddycurrent_version_answer;

// nailed to IN2BUF
__xdata __at 0x7E00 struct frame classic_inbuf_fortius;
// nailed to OUT2BUF
__xdata __at 0x7DC0 struct frameCommandFortius classic_outbuf_fortius;
__xdata __at 0x7DC0 struct frameCommandImagic  classic_outbuf_imagic;   // legacy

__data uint8_t classic_outbuf_fortius_size;
#endif

#ifdef WITH_MOTORBRAKE
__bit motorbrake_send_cmd;
__bit motorbrake_send_test;
#endif

// note: ezusb allows access to unified __code/__xdata under same address
__xdata __at 0x0 volatile uint8_t xdata[0x8000];
__code  __at 0x0 volatile uint8_t  code[0x1b40];
__idata __at 0x0 volatile uint8_t idata[0x0100];

// ================================================================================
// ISR functions, it is important to define them in same file as the main() function
// ================================================================================

// ie0:    used for eddy current (wheel signal) and serial bitbang  (tx sync)
// ie1:    used only for eddy current (powerline zero crossing)
// timer1: used for eddy current (PAC/PWM/RPC offset/length) and serial bitbang  (rx impuls length)
#ifdef WITH_EDDYCURRENT
extern void ie0_isr_eddycurrent(void)       __interrupt IE0_VECTOR;  /* 0x03 external interrupt 0 */
extern void ie1_isr_eddycurrent(void)       __interrupt IE1_VECTOR;  /* 0x13 external interrupt 1 */
extern void timer1_isr_eddycurrent(void)    __interrupt TF1_VECTOR;  /* 0x1b timer 1 */
#else
extern void ie0_isr_bitbang_rx(void)        __interrupt IE0_VECTOR;  /* 0x03 external interrupt 0 */
extern void ie1_isr_empty(void)             __interrupt IE1_VECTOR;  /* 0x13 external interrupt 1 */
extern void timer1_isr_bitbang_rx(void)     __interrupt TF1_VECTOR;  /* 0x1b timer 1 */
#endif

// timer0: used for serial bitbang  (tx impuls length)
#ifdef WITH_BITBANG
extern void timer0_isr_bitbang_tx(void)     __interrupt TF0_VECTOR;
#endif

// plain vanilla serial implementation
#ifdef WITH_SERIAL0
extern void serial0_isr(void)               __interrupt SI0_VECTOR;  /* 0x23 */
#endif

// timer2: main timer
extern void timer2_isr(void)                __interrupt TF2_VECTOR;  /* 0x2b timer 2 */

/*****************************************************************************
 * ie1 signals powerline zero crossing, if the unit is connected to
 * an eddy current brake (and the brake is plugged in)
 * PINC.3 not used for the motorbrake.
 ****************************************************************************/

#ifndef WITH_EDDYCURRENT
void ie1_isr_empty(void)      __interrupt  {
    // if we have an eddy current brake, this is a good point to see it switched OFF
    // (if everything works ok, the magnetic field is still switched off at this point)
    // note : in motorbrake-mode we never expect this IRQ
    OUTC = 0x00;
    semaphore_ie1_powerline = true;
    is_alive_ie1_powerline  = true;     // deadman-switch checked every 1000sec

    STATISTICS_IRQ(count_ie1++);
}
#endif

/*
 * Timer 2 is (always) our general main application timer
 *
 * a little discussion about how to measure the parameters we need (HR, CAD,
 * Powerline-Frequency and speed):
 *
 * The main problems are:
 *    1. you should not miss an event,
 *    2. the complexity of the measurement and the post processing
 *    3. the error of the method
 *    4. how "fast" is the method
 *    5. what happens if there are no more events coming in
 *    6. signal noise 
 *
 * We have two strategies:
 *   1. Measure the time between two events or 
 *   2. count the number of events within a fixed time interval. 
 * 
 * The second method is some kind of implicit averaging.
 * Averaging reduces the error but the "responsiveness" of the parameter is worse.
 * i.e. if you want to know the beats per minute of your heart you can measure
 * one minute (and simply count the number of beats) but than you average over
 * one minute. That's not what you want.
 *
 * #1: powerline and speed (wheel) is connected to an interrupt
 *     it's fairly simple to measure the time or just count the number
 *     of events. 
 *     CAD and HR is available as signal on port B. We have to sample the PINs
 *     The precision depends on the sampling frequency. 2ms is enough to not
 *     miss an event.
 *
 * Generally, the most precise method is to measure the time between two events, 
 * but then you need some kind of "timeout" or "fade out", if no signal is detected 
 * for some time.
 *
 * Currently we use we both strategies:
 * 
 *   For events with a higher frequency (speed, powerline) we count the
 *   number of events per second.
 *   Additionally, we now measure the time between two wheel interrupts,
 *   to get a more precise and faster value to calculate the right value
 *   to control the eddy current PWM timing. This timing is probably
 *   even precise enough to get some kind of left/right pedal balance
 *   if we sync it with the pedal signal.
 * 
 *   For events with (potentially) low frequencies (heart BPM, cadence RPM)
 *   we measure the time between two events (sampled with timer2 frequency)
 *   and set the value (bpm, rpm) to "0", if there is no signal detected over 
 *   two seconds (no fade out).
 * 
 *   Especially for the HR signal, which is very sensitive to EM noise, we
 *   added a 'deadtime'. After receiving a high-low or low-high level change
 *   we ignore every level-change for a specific time interval.
 */
void timer2_isr(void) __interrupt  {

    uint8_t pinsC;

    if(start_steering) {
        if(!steering_running) {
            steering_running = true;

            // TODO even if we immediately start the steering meassurment, other 
            // (same level or higher) interrupts can delay the start. So we should 
            // save at least the TL2 value to correct the steering meassurment
            OEB  = PORTB_MASK_GAMEPORT_AXES ^ PORTB_MASK_GAMEPORT_AXIS2;
        }
    } else {
        steering_running = false;
        OEB  = PORTB_MASK_GAMEPORT_AXES;
    }

    pinsC         =  PINSC;

    // HR, CAD is active low
#ifdef WITH_HEARTRATE
    // very simple ... more robust implementation against interference is necessary!
    {
        bool hrPin = (pinsC & PORTC_MASK_HR) ? true : false;
        if(count_hr_timeout > HR_MAX_TIME) {
            if(!hrStalled) {
                hrStalled = true;
                event_hr = true;
            }    
        } else {
            count_hr_timeout++;
        }

        if(lastHRPin != hrPin) {
            if(!hrPin) {
                // Edge High->Low 
                if(count_hr_timeout >= HR_MIN_TIME) {
                    count_hr_last = count_hr_timeout;
                    event_hr = true;
                    count_hr_timeout = 0;
                }
            }
        }
        lastHRPin = hrPin;
    }
#endif

#ifdef WITH_EDDYCURRENT
    // very simple ... more robust implementation against interference is necessary!
    {
        bool cadPin = (pinsC & PORTC_MASK_CAD) ? true : false;
        if(count_cad_timeout > CAD_MAX_TIME) {
            if(!cadStalled) {
                cadStalled = true;
                event_cad = true;
            }    
        } else {
            count_cad_timeout++;
        }

        if(lastCADPin != cadPin) {
            if(!cadPin) {
                // Edge High->Low 
                if(count_cad_timeout >= CAD_MIN_TIME) {
                    count_cad_last = count_cad_timeout;
                    event_cad = true;
                    count_cad_timeout = 0;
                }
            }
        }
        lastCADPin = cadPin;
    }
#endif

    // TODO: "interleave" the events, so no more than one event happens at the same time
    if(timer2_serial_timeout > 0) {
        timer2_serial_timeout--;
    }

    timer2_125ms_countdown--;
    if(timer2_125ms_countdown == 0) {
        event_125ms_expired = true;
        timer2_125ms_countdown = TIMER2_PER_1_8_SEC;
    }

    timer2_1000ms_countdown--;
    if(timer2_1000ms_countdown == 0) {
        event_1000ms_expired = true;
        timer2_1000ms_countdown = TIMER2_PER_SECOND;
#ifdef WITH_EDDYCURRENT
        event_1000ms_eddycurrent_powerline = true;
#endif
    }
    
    TF2 = false;
    count_t2++;
    event_timer2_expired = true;
}


#ifdef WITH_STEERING
/*****************************************************************************
 * ! BUSY WAITING !
 * Timer2 triggers a pulse on the used PINB and measures the time until the 
 * PIN becomes HIGH again. It's the 'gameport' way to achive an A/D conversion
 * Every interrupt disturbes the measurment, so we sync the measurment with
 * timer2 and stop serial communication
 * TODO also sync with Wheeling-ISR and Powerline+PAC/PWM/RPC-Timer-ISR
 *      or at least detect IRQ and drop "steerings" with measuring errors
 ****************************************************************************/

static void steering_handle() {
    bool first = true;

    while(true) {
        if(TH2 >= AXIS_TIMEOUT_AFTER_TH2) {
            if(first)
                return;
            steering = AXIS_TIMEOUT_VALUE;
            break;
        }
        if(PINSB & PORTB_MASK_GAMEPORT_AXIS2) {            
            uint8_t i,tl2,th2;
            // TODO not the best way to stop timer2 - but it works for now
            EA=false;  // we do not want any interrupts while TR2 is stopped 
            TR2=false;
            tl2=TL2;
            th2=TH2;
            TR2=true;
            EA=true;
            if(first)
                return;

#ifdef TIMER2_TL2_NON_ZERO
            th2 -= TIMER2_HIGH;
            if(tl2 < TIMER2_LOW)
                th2--;
            tl2 -= TIMER2_LOW;
#else
            th2 &= 0x0f; // works only if TL2 is initialized with 0x00
#endif
            steering_avg[steering_idx] =  ((uint16_t)th2<<8) | tl2;
            steering_idx++;
            steering_idx &= (AXIS_AVERAGE-1);
            steering = 0; 
            for(i=0;i<AXIS_AVERAGE;i++)
                steering += steering_avg[i];
            schedule_do_steering = false;
            start_steering = false;

            steering >>= AXIS_TIMER_SHIFT;
            steering +=  AXIS_TIMER_OFFSET;

            STATISTICS_GENERAL(count_steering++);
            return;
        }
        first = false;
    }

    schedule_do_steering = false;
    start_steering = false;
    STATISTICS_GENERAL(count_steering++);
}
#endif


/*****************************************************************************
 * Configure I/O Ports
 * depends on operation mode, brake-type and headunit-type
 ****************************************************************************/

inline void io_init(void) {

    PORTACFG = 0x00;
    OEA      = PORTA_MASK_LEDS;
    OUTA     = 0x00;

    // Buttons PortB Bit4-7 - IN
    PORTBCFG = 0x00;
    OEB      = PORTB_MASK_GAMEPORT_AXES; // "gameport" is OUT by default
    OUTB     = 0x00;

    // Port C config depends on operation mode: see configurePortC
    PORTCCFG = 0x00;
    OEC      = 0x00;
    OUTC     = 0x00;
}

inline void configurePortC(void) {

    OEC      = 0x00;
    OUTC     = 0x00;
    PORTCCFG = 0x00;

    if(headunit_device & HEADUNIT_SOLIDGREEN_TYPE) {
        // Green unit do not use alt-functions and the only "OUT" pin we need for motorbrake and eddy-current is PINC.7
        // "solid green unit": PINC7 to "OUT" for  PAC/PWM/RPC for magnetic field and TX
        if(brake_state & BRAKE_TX_ENABLED_TYPE) {
            OUTC = PORTC_MASK_MAGNET;           // TX is active low -> default is high
        }
        if(brake_state & (BRAKE_TX_ENABLED_TYPE | BRAKE_EDDYCURRENT_TYPE) ) {
            OEC = PORTC_MASK_MAGNET;
        }
    } else if(headunit_device & HEADUNIT_SOLIDBLUE_TYPE) {
        // always set Rx on "alternate" - this disables the OUT feature on PINC.0 - PINC.0 is readable anyway
        PORTCCFG = PORTC_MASK_RX;

        if(brake_state & BRAKE_TX_ENABLED_TYPE) {
            // solid blue as motorbrake
            // let Tx-alt Disabled on PINC.1 so we can use it as self controlled "OUT" and set it to High (Tx is active LOW)
#ifdef WITH_BITBANG
            OUTC = PORTC_MASK_TX;
#elif defined(WITH_SERIAL0)
            PORTCCFG |= PORTC_MASK_TX;
#endif
        }
        if(brake_state & (BRAKE_TX_ENABLED_TYPE | BRAKE_EDDYCURRENT_TYPE)) {
            // PINCS.1 is always an output (ignored if TX is controlled by serial0 alt function)
            OEC      = PORTC_MASK_TX;
        }
    }
}


/*****************************************************************************
 *
 ****************************************************************************/

inline void buttons_handle() {
#ifdef WITH_EMULATE_USB_ANTFE_C
    uint8_t changed, pinsb = PINSB;

    changed = pinsb ^ lastPinB;
    lastPinB &= changed;

    if(lastPinB & PORTB_MASK_BUTTON_UP) {
        trainer.calibration += 25;
    }
    if(lastPinB & PORTB_MASK_BUTTON_DOWN) {
        trainer.calibration -= 25;
    }
    if(lastPinB & PORTB_MASK_BUTTON_CANCEL) {
        trainer.calibration = 0;
    }
    if(lastPinB & PORTB_MASK_BUTTON_ENTER) {
        trainer.calibration = T1942_CALIBRATE_DEFAULT_RAW;
    }

    lastPinB = pinsb;
#endif
}

/*****************************************************************************
 initialize_hardware()
 ****************************************************************************/

inline void initialize_hardware() {
    EA = true;      // Enable all interrupt

    // ---------------------------
    // in the beginning: timers and serial0 is OFF
    // ---------------------------

    ES0 = false; // disable Serial0 IRQ

    // ---------------------------
    // STOP Timer0/1, disable IRQs set Divider to 12, default mode 0
    // ---------------------------
    TR0 = false; TF0 = false; ET0 = false;
    TR1 = false; TF1 = false; ET1 = false;
    // GATEx=int-gate, CTx=Counter/Timer select, Mx1/Mx0=mode select (b00=13bit,b01=16bit,b10=8bit reload,b11=Two 8-bit counters (timer1 reduced func))
    TMOD = 0x00;
    // Divider 12 for Timer0/1
    CKCON &= ~(T0M|T1M);

    // -------------------------------------------------------------------------
    // -------------------------------------------------------------------------
    // TIMER2: init general application timer
    // -------------------------------------------------------------------------
    // -------------------------------------------------------------------------
    TR2 = false;   // all off
    TF2 = false;   // overlfow reset
    ET2 = false;   // TIMER2 IRQ OFF

    CKCON &= ~(T2M);    // divider 24MHz/12

    // EXEN2 - Timer 2 external enable. EXEN2=1 enables capture or reload to occur as a result of
    // a high-to-low transition on the T2EX pin, if Timer 2 is not generating baud rates for the serial
    // port. EXEN2=0 causes Timer 2 to ignore all external events on the T2EX pin.
    EXEN2 = false;

    // C/T2 - Counter/Timer select. When C/T2 = 1, Timer 2 is clocked by high-to-low transitions on the T2 pin.
    // When C/T2 = 0 in modes 0, 1, or 2, Timer 2 is clocked by CLK24/4 or CLK24/12, depending on the state of T2M (CKCON.5).
    // When C/T2 = 0 in mode 3, Timer 2 is clocked by CLK24/2, regardless of the state of CKCON.5.
    CT2 = false;

    // CP/RL2 - Capture/reload flag. When CP/RL2=1, Timer 2 captures occur on high-to-low transitions  of the T2EX pin, if EXEN2 = 1.
    // When CP/RL2 = 0, auto-reloads occur when Timer 2 overflows or when high-to-low transitions occur on the T2EX pin, if EXEN2 = 1.
    // If either RCLK or TCLK is set to 1, CP/RL2 will not function and Timer 2 will operate in auto-reload mode following each overflow.
    CPRL2 = false;

    RCAP2H = TIMER2_HIGH;
    RCAP2L = TIMER2_LOW;
    TH2 = TIMER2_HIGH;
    TL2 = TIMER2_LOW;

    ET2 = true;     // TIMER2 IRQ ON
    PT2 = false;    // TIMER2 High-Prio OFF (reset default)
    TR2 = true;

    // -------------------------------------------------------------------------
    // -------------------------------------------------------------------------
    // Setup INT0 Wheel and RX-Bitbang (common settings)
    // -------------------------------------------------------------------------
    // -------------------------------------------------------------------------    
    IT0 = true;            // TCON.0 IE0 : EDGE triggered - INT0 is detected on falling edge, when IT0 = true
    IE0 = false;

    // -------------------------------------------------------------------------
    // -------------------------------------------------------------------------
    // Setup INT1 for Powerline zero crossing (exclusively for eddy current brake)
    // -------------------------------------------------------------------------
    // -------------------------------------------------------------------------

    // PX1 Priority HIGH. Maybe LOW is better, so steering is not disturbed and a little delay of
    // INT1 (because T2 takes some time) is not critical 
    PX1 = true;            

    IT1 = true;            // TCON.2 IE1 : EDGE triggered - INT1 is detected on falling edge, when IT1 = 1
    IE1 = false;
    EX1 = true;            // IE.1   Enable external interrupt
}

/*****************************************************************************
 initialize_data()
 ****************************************************************************/

inline void initialize_data() {
    brake_state = BRAKE_AUTODETECT;
    // brake_state = BRAKE_PASSIVE_TESTINGs
    // brake_state = BRAKE_MOTORBRAKE;

    // not using "else" here saves a few code bytes - (and runtime performance does not matter here)
    headunit_device = HEADUNIT_UNKNOWN;
    if(eeprom.productID == USB_ID_T1902_SOLID_GREEEN) {
        headunit_device = HEADUNIT_T1902;
    } 
    if(eeprom.productID == USB_ID_T1942_SOLID_BLUE_NOINIT) {
        headunit_device = HEADUNIT_T1942;
    } 
}

#ifndef WITH_EMULATE_USB_ANTFE_C
inline void classic_init() {
    uint8_t i;
    for(i=0;i<sizeof(classic_inbuf_fortius);i++)
        ((__xdata uint8_t *)&classic_inbuf_fortius)[i] = 0x00;

    classic_inbuf_fortius.unit.deviceSerial     = eeprom.deviceSerial;
    classic_inbuf_fortius.unit.year_production  = eeprom.year_production;

    //classic_inbuf_fortius.unit.fixed1 = 0x0000;
    classic_inbuf_fortius.unit.fixed2   = 0x0108; // also seen 0x0104 - maybe some kind of firmware version?
    //classic_inbuf_fortius.unit.fixed3 = 0x0000;
    //classic_inbuf_fortius.unit.unused1 = classic_inbuf_fortius.unit.unused2 = classic_inbuf_fortius.unit.unused3 = classic_inbuf_fortius.unit.unused4 = 0x00;
    //classic_inbuf_fortius.unit.rxErrorCount = 0x00;
    classic_inbuf_fortius.unit.axis1 = AXIS_TIMEOUT_VALUE;
}

#ifdef WITH_EDDYCURRENT
void classic_eddycurrent_prepare_ep_inbuf(void) {

    if(eddycurrent_version_answer) {
        *((uint32_t *)&classic_inbuf_fortius.brake.s.header) = 0x00000c03;
        classic_inbuf_fortius.brake.s.firmwareVersion = 0x00001902;
        classic_inbuf_fortius.brake.s.serialNumber    = 0x00000000;
        classic_inbuf_fortius.brake.s.version2        = 0x0000;
        classic_inbuf_fortius.brake.s.unused          = 0x0000;
        classic_inbuf_fortius.brake.s.checksumLSB     = 0x35;  // dummy checksum  
        classic_inbuf_fortius.brake.s.checksumMSB     = 0x69;  // dummy checksum  
    } else {
        // T1902
        *((uint32_t *)&classic_inbuf_fortius.brake.s.header) = 0x00021303;
#ifdef WITH_EXTENDED_PROTOCOL
        EA = false;
        // Development extension
        classic_inbuf_fortius.brake.l.resistance_now = eddycurrent_resistance_now;
        EA = true;
#endif
        classic_inbuf_fortius.brake.l.distance     = eddycurrent_wheel_total;
        classic_inbuf_fortius.brake.l.rawSpeed     = eddycurrent_speed;
        classic_inbuf_fortius.brake.l.cadence      = eddycurrent_cadence;
        classic_inbuf_fortius.brake.l.events       = event_pedal_detect;
        classic_inbuf_fortius.brake.l.currentResistance = classic_inbuf_fortius.brake.l.avgResistance = eddycurrent_calculated_power;
        classic_inbuf_fortius.brake.l.checksumLSB  = 0x55;  // dummy checksum  
        classic_inbuf_fortius.brake.l.checksumMSB  = 0x64;  // dummy checksum  

#ifdef WITH_EXTENDED_PROTOCOL
        // Development extension
        {  
            // Access to wheel_timestamps_next_ptr is atomic - in case of overrun we loose 
            // the entire buffer
            uint8_t tmp = wheel_timestamps_next_ptr;
            uint8_t num = (tmp - wheel_timestamps_last_ptr) & EDDYCURRENT_TIMESTAMPS_MASK;
            uint8_t i;
            if(num > 6)
                num = 6;
            for(i=0;i<num;i++) {
                classic_inbuf_fortius.brake.l.wheel_timestamps[i] = wheel_timestamps[(wheel_timestamps_last_ptr + i) & EDDYCURRENT_TIMESTAMPS_MASK];
            }
            wheel_timestamps_last_ptr = tmp;
            classic_inbuf_fortius.brake.l.wheel_timestamps_num = num;
        }
#endif

        event_pedal_detect = false;
    }               
    eddycurrent_version_answer = false;
}
#endif

inline void classic_protocol_handle() {
        // headunit T1942 connected to the motorbrake sends 24 bytes, if there is no
        // feedback from the brake, and 48 bytes, if there is feedback. Feedback is
        // always triggered by a command sent to the motorbrake. So we should
        // distinguish betweeen:
        //   case 1: no command, no answer from MotorBrake -> 24 bytes
        //   case 2: command received for eddy current -> emulate 48 bytes
        //   case 3: answer received from motorbrake -> 48 bytes
        // additional note 1: T1904 always sends 64 bytes (even without Tx commands)
        //                    dunno what 1932 does..
        // additional note 2: full emulation of "case 1" is not really necessary,
        //                    because host software does not expect it
        //
        // headunits T1932 and T1904 both send 64 bytes long frames and 24 bytes short frames,
        // if a eddy current magnetic is detected T1932 and T1904 always send 64 bytes
        // Version packet for eddy current is 02,19,0,0, 0,0,0,0 0,0,0,0
        // T1932 and T1904 connected to a motor brake seems to "hold" the last frame from the
        // motorbrake for about 1 second, while the T1942 firmware only returns "long frames", if
        // the users sends commands to the headunit


        // --------------------------------------------------------
        // prepare a frame to host if tx-queue ('in'-endpoint) is not busy
        // --------------------------------------------------------

        if(!(IN2CS & EPBSY)) {
            bool in2buf_largeFrame = false;

            classic_inbuf_fortius.unit.rxErrorCount = count_rx_err;
            classic_inbuf_fortius.unit.heartRate    = HEARTRATE();
            classic_inbuf_fortius.unit.buttons      = (PINSB>>4)^0xf;  // not working: (~PINSB)>>4 and (~(uint8_t)PINSB)>>4
            classic_inbuf_fortius.unit.axis1        = steering;

            if(false) {
            }
# ifdef WITH_EDDYCURRENT
            else if(brake_state & BRAKE_EDDYCURRENT_TYPE) {
                classic_eddycurrent_prepare_ep_inbuf();
                in2buf_largeFrame = true;
            }
# endif
# ifdef WITH_MOTORBRAKE
            else if(brake_state == BRAKE_MOTORBRAKE && u_semaphore_rx_valid) {
                uint8_t i;
                u_semaphore_rx_valid = false;
                // 25 bytes to catch the second checksum byte
                for(i=0;i<sizeof(classic_inbuf_fortius.brake.l);i++)
                    classic_inbuf_fortius.brake.buf[i] = rxDecoded[i];
                in2buf_largeFrame = true;
                if(trainer.fake != 0)
                    classic_inbuf_fortius.brake.l.currentResistance = trainer.fake;
            }
# endif
            IN2BC = in2buf_largeFrame ? TACX_FRAME_SIZE_LONG : TACX_FRAME_SIZE_SHORT;

            STATISTICS_GENERAL(count_in_frames12++);
        }

        // --------------------------------------------------------
        // handle a frame from host if rx-queue ('out'-endpoint) is not busy 
        // --------------------------------------------------------

#ifdef WITH_USB_POLL
        if(!(OUT2CS & EPBSY)) {
#else
        if(Semaphore_EP12_out) {
            Semaphore_EP12_out = false;
#endif

            STATISTICS_GENERAL(count_out_frames12++);

            classic_outbuf_fortius_size = OUT2BC;

            if(classic_outbuf_fortius_size >= T1941_CONTROL_COMMAND_SIZE 
                && classic_outbuf_fortius.h.header32 == T1941_CONTROL_COMMAND_HEADER) { 
                // Command: 0x01,0x08,0x01,0x00 

                trainer.calibration  = classic_outbuf_fortius.calibrate;
                trainer.rawLoad      = classic_outbuf_fortius.force;
                trainer.pedalecho    = classic_outbuf_fortius.pedalecho;

                if(classic_outbuf_fortius.mode == T1941_CONTROL_COMMAND_MODE_FORCE) {
                    if(classic_outbuf_fortius.flywheel == T1941_CONTROL_COMMAND_FLYWHEEL_WEIGHT_OFF) {
                        trainer.mode         = TRAINER_MODE_POWER;
                    } else {
                        // TODO currently ignored
                        // trainer.slope        = (classic_outbuf_fortius.force) + (classic_outbuf_fortius.force<<6) + 19600;
                        trainer.flywheel     = classic_outbuf_fortius.flywheel;
                        trainer.mode         = TRAINER_MODE_SIMULATION;
                    }
                } else if(classic_outbuf_fortius.mode == T1941_CONTROL_COMMAND_MODE_CALIBRATE){
                    trainer.mode = TRAINER_MODE_CALIBRATE;
                } else {
                    trainer.mode = TRAINER_MODE_NONE;
                }
#ifdef WITH_MOTORBRAKE
                motorbrake_send_cmd = true;
#endif
            } else if(classic_outbuf_fortius_size == T1941_VERSION_COMMAND_SIZE 
                        && classic_outbuf_fortius.h.header32 == T1941_VERSION_COMMAND_HEADER) {   
                // Command: 0x02,0x00,0x00,0x00
                if(false) {
                }
#ifdef WITH_MOTORBRAKE
                else if(brake_state == BRAKE_MOTORBRAKE) {
                    motorbrake_send_test = true;
                } 
#endif
#ifdef WITH_EDDYCURRENT
                else if(brake_state & BRAKE_EDDYCURRENT_TYPE) {
                    eddycurrent_version_answer = true;
                    // TODO do nothing ?
                }
#endif
            }

            // re-arm OUT2 EP for the next message from the host
            OUT2BC = 0x0;
        }
}
#endif

#ifdef WITH_EMULATE_USB_ANTFE_C
void antplus_prepare_motor_cmd(void) {

    __xdata struct frameCommandFortius *cmdBrk = (__xdata struct frameCommandFortius *)txDecoded;

    *((uint32_t *)cmdBrk) = T1941_CONTROL_COMMAND_HEADER;   // little endia 0x01,0x08,0x01,0x00

    cmdBrk->force     = 0;
    cmdBrk->zero0     = 0;
    cmdBrk->mode      = T1941_CONTROL_COMMAND_MODE_NONE;
    cmdBrk->flywheel  = T1941_CONTROL_COMMAND_FLYWHEEL_WEIGHT_OFF;

    cmdBrk->calibrate = trainer.calibration;
    cmdBrk->pedalecho = trainer.pedalecho;
    trainer.pedalecho = 0;

    if(trainer.mode == TRAINER_MODE_CALIBRATE) {
        // TODO fixed speed 20km/h = 0x16a3 ?
        // cmdBrk->force     = 0x16a3;
        cmdBrk->force  = 0x16a3;
        cmdBrk->mode   = T1941_CONTROL_COMMAND_MODE_CALIBRATE;
    } else if(trainer.mode == TRAINER_MODE_POWER) {
        // reduce initial force if speed is under about 5kph
        cmdBrk->force = 1000;
        if(trainer.rawSpeed > (5*290)) { 
             cmdBrk->force = antPower2load();
        } 
        cmdBrk->mode   = T1941_CONTROL_COMMAND_MODE_FORCE;
    } else if(trainer.mode == TRAINER_MODE_BASIC_RESISTANCE) {
        uint8_t percent = trainer.page48.total_resistance;
        if(percent > 200)
            percent = 200; // scale = 0.5 -> 200 is 100%
        
        cmdBrk->force = 50 * percent;
        // Special case mode = TRAINER_MODE_POWER
        cmdBrk->mode   = T1941_CONTROL_COMMAND_MODE_FORCE;
    } else if(trainer.mode == TRAINER_MODE_SIMULATION) {
        // ANT+ describes a simple physical model:
/*
Total resistance [N] = Gravitational Resistance + Rolling Resistance + Wind Resistance
Gravitational Resistance [N] = (Equipment Mass + User Mass) x Grade(%)/100 x 9.81
With the assumption of a total 
weight of about 80 kg 
and without rolling and wind resistance together with the simple assumption 
Load = Force [N] * 137 
(the result of a fitting in ergo mode) one get:
Load = 137 * 80 * 9.81 * slope_in_percent / 100
SlopeScale = 137 * 80 * 9.81 / 100 = 1075
*/

        int16_t slope  = trainer.page51.grade - 20000;
        cmdBrk->force  = 13 * slope; // simple simulation
        cmdBrk->mode   = T1941_CONTROL_COMMAND_MODE_FORCE;
    } else {
        // TODO check ranges
        //cmdBrk->force  = 0;
        //cmdBrk->mode   = 0;
        //cmdBrk->weight = 0x52; // 72kg rider + 10kg bike
    }
}
#else // WITH_EMULATE_USB_ANTFE_C
void classic_prepare_motor_cmd(void) {
     __xdata struct frameCommandFortius *cmdBrk = (__xdata struct frameCommandFortius *)txDecoded;

    *((uint32_t *)cmdBrk) = T1941_CONTROL_COMMAND_HEADER;   // little endia 0x01,0x08,0x01,0x00

    cmdBrk->force     = trainer.rawLoad;
    cmdBrk->pedalecho = trainer.pedalecho;
    cmdBrk->zero0     = 0;
    cmdBrk->mode      = T1941_CONTROL_COMMAND_MODE_NONE;
    cmdBrk->flywheel  = trainer.flywheel;
    cmdBrk->calibrate = trainer.calibration;

    if(trainer.mode == TRAINER_MODE_CALIBRATE) {
        // TODO fixed speed 20km/h = 0x16a3 ?
        // cmdBrk->force     = 0x16a3;
        // cmdBrk->weight    = 0x52;
        // cmdBrk->calibrate = 0x0000;
        cmdBrk->mode   = T1941_CONTROL_COMMAND_MODE_CALIBRATE;
    } else if(trainer.mode == TRAINER_MODE_POWER) {
        // Special case mode = TRAINER_MODE_POWER
        cmdBrk->mode     = T1941_CONTROL_COMMAND_MODE_FORCE;
        cmdBrk->flywheel = T1941_CONTROL_COMMAND_FLYWHEEL_WEIGHT_OFF;
    } else if(trainer.mode == TRAINER_MODE_SIMULATION) {
        cmdBrk->mode   = T1941_CONTROL_COMMAND_MODE_FORCE;
    } else {
        //cmdBrk->force  = 0;
        //cmdBrk->mode   = 0;
        //cmdBrk->weight = 0x52; // 72kg rider + 10kg bike
    }
}
#endif

/*****************************************************************************
 ****************************************************************************
 MAIN
 ****************************************************************************
 ****************************************************************************/

void main(void) {
    // Ensure CLK24 (NOTE: not really necessary, because ezusb an2131 is always 24mhz)
    CPUCS |= CLK24OE;
    // CKCON[2..0]: zero wait states for 'eXternal' memory
    CKCON = 0x00;

    io_init();
    usb_init();
    usb_renumber();
#ifdef WITH_EMULATE_USB_ANTFE_C
    antplus_init();

#endif
    i2c_read_eeprom();
    initialize_data();
#ifndef WITH_EMULATE_USB_ANTFE_C
    classic_init();
#endif
    initialize_hardware();

    // ---------------------------
    // Main 'polling' Loop
    // ---------------------------

    while(true) {

        // --------------------------------------------------------------------
        // USB handling - polling or irq-semaphore triggered
        // --------------------------------------------------------------------

        usb_handle_setup_data();

        // --------------------------------------------------------------------
        // Steering
        // --------------------------------------------------------------------

        //  Tx-BITBANGing disturbs - so no timing until finish of Tx
        if((timer2_serial_timeout == 0) && schedule_do_steering) {
#ifdef WITH_STEERING
            start_steering = true;
            if(steering_running)
                steering_handle();
#endif
        }

        // --------------------------------------------------------------------
        // Main host communication handling
        // --------------------------------------------------------------------

#ifdef WITH_EMULATE_USB_ANTFE_C
        antplus_handle();
#else
        classic_protocol_handle();
#endif

        // --------------------------------------------------------------------
        // Main motorbrake handling
        // --------------------------------------------------------------------

#ifdef WITH_MOTORBRAKE
        serial_handle_rx();

#ifdef WITH_EMULATE_USB_ANTFE_C        
        if(u_semaphore_rx_valid) {
            __xdata struct frameBrakeLong *answerBrk = (__xdata struct frameBrakeLong *)rxDecoded;
            uint8_t idx = trainer.countAnswers & 0x3; // for avg. watt - 4 values 
            
            u_semaphore_rx_valid = false;

            trainer.pedalecho        |= answerBrk->events & T1941_EVENTS_PEDALSENSOR_MASK;
            trainer.rawSpeed          = answerBrk->rawSpeed;
            trainer.currentResistance = answerBrk->currentResistance;
            trainer.cadence           = answerBrk->cadence;

            trainer.countAnswers++;
            trainer.avgWatt -= trainer.watts[idx];
            trainer.watts[idx] = load2power();
            trainer.avgWatt += trainer.watts[idx];
        }
#endif

        if((brake_state & BRAKE_TX_ENABLED_TYPE) && (timer2_serial_timeout == 0) && !schedule_do_steering) {
            if(motorbrake_send_test) {
                motorbrake_send_test = false;

                *((uint32_t *)txDecoded) = T1941_VERSION_COMMAND_HEADER; // little endia 0x02,0x00,0x00,0x00
                serial_command_tx(4);

            } else if(motorbrake_send_cmd) {
                motorbrake_send_cmd = false;

#ifdef WITH_EMULATE_USB_ANTFE_C     
                antplus_prepare_motor_cmd();
#else
                classic_prepare_motor_cmd();
#endif // WITH_EMULATE_USB_ANTFE_C

                serial_command_tx(12);
            }

        }

        if(brake_state & (BRAKE_RX_ENABLED_TYPE|BRAKE_TX_ENABLED_TYPE)) {
            // blinkcode_force_green = txON || rxON;
        }
#endif //  WITH_MOTORBRAKE


        // --------------------------------------------------------------------
        // Main eddy current handling
        // --------------------------------------------------------------------

#ifdef WITH_EDDYCURRENT
        // Every loop "round" we calculate and adjust the power ... (TODO limit ?)
        {
            int16_t new_resistance = EDDYCURRENT_OFF_VALUE;
            if(eddycurrent_wheeling) {
                eddycurrent_calculated_power = eddycurrent_power_mul13();
                if(trainer.mode == TRAINER_MODE_POWER) {
                    // eddycurrent_resistance_now (volatile!) or eddycurrent_resistance_target ? 
                    new_resistance = eddycurrent_resistance_target;
                    if(eddycurrent_calculated_power < trainer.rawLoad && eddycurrent_resistance_target < EDDYCURRENT_MAX_VALUE) {
                        new_resistance += EDDYCURRENT_STEP_VALUE;
                    } else if (eddycurrent_resistance_target > EDDYCURRENT_MIN_VALUE){
                        new_resistance -= EDDYCURRENT_STEP_VALUE;
                    }                    
                } else if(trainer.mode == TRAINER_MODE_SIMULATION) {
                        // TODO Experimental => take slope/2 as magnetic brake value (0x10..0x80..0xf0) 
                        //                      value is multiplied by 6 (for 50Hz) or 5 (for 60 Hz) to get
                        //                      the "length" of the PAC pulse. Negativ values reduce force from 
                        //                      default (static) magnetic field. Positive values increase the force.
                        //                      => with (current) limits at -0x700 and 0x700 the allowed 'real' slope 
                        //                         is about -5.5% to 5.5% (because the factor is 10*5*13 = 650)                        
                    new_resistance = EDDYCURRENT_OFF_VALUE + (((int16_t)trainer.rawLoad)>>1) ;
                }
            } else {
                eddycurrent_calculated_power = 0;
            }
            eddycurrent_set_resistance(new_resistance);
        }
        {
            uint8_t tmp,delta;

            tmp = eddycurrent_wheel;
            delta = tmp - eddycurrent_wheel_last;
            if(delta != 0) {
                eddycurrent_wheel_last = tmp;
                eddycurrent_wheel_total += delta;
            }
        }

        if(event_cad) {
            eddycurrent_cadence = 0;
            if(!cadStalled) {    
                event_pedal_detect = true;
                eddycurrent_cadence = TIMER2_PER_MINUTE / count_cad_last;
            }
            event_cad = false;
        }
#endif // WITH_EDDYCURRENT
        if(event_hr) {
            heart_bpm = 0;
            if(!hrStalled) {                
                // TODO if code already includes 32 bit long div, casting to uint32_t may reduce code
                heart_bpm = TIMER2_PER_MINUTE / count_hr_last;
            }
#ifdef WITH_EMULATE_USB_ANTFE_C
            {
                __xdata heart_rate_t *hrpage = (__xdata heart_rate_t *) &xdata[(uint16_t)&antplus_page_answers_static.heart_rate];
                hrpage->heart_beat_count++;

                hrpage->previous_heart_beat_event_time = hrpage->heart_beat_event_time;
                // convert timer 1/488s -> 1/1024s
                hrpage->heart_beat_event_time += mul16x16(count_hr_last<<2,34380)>>16;
                hrpage->computed_heart_rate  = heart_bpm;    // 12 bit ... (4 bit nibble) + Trainer Status Bit Field
            }
#endif
            event_hr = false;
            blinkcode_force_red_single = true;
        }



        // --------------------------------------------------------------------
        // 8 Hz main loop hook
        // --------------------------------------------------------------------

        if(event_125ms_expired) {
            event_125ms_expired = false;

#ifdef WITH_EMULATE_USB_ANTFE_C
            antplus_8Hz_hook();
# ifdef WITH_MOTORBRAKE
#  if 1 
            // TODO enable if we have ANT+ detected
            motorbrake_send_cmd = true;
#  endif
# endif
#endif // WITH_EMULATE_USB_ANTFE_C

            schedule_do_steering = true;
            blinkcode_shift();

#ifdef WITH_EDDYCURRENT
            //            eddycurrent_8Hz_hook();
#endif
        }

        // ======================================================================
        // Brake-state handling ... TODO add a real state machine
        // ======================================================================

        // Events:
        //
        // Powerline-Interrupt detected    => expect EDDY-CURRENT =>  test for powerline-frequency
        // Valid-Motorbrake-Frame-Received => MOTORBRAKE detected
        //
        // Run-Command-From-Host received
        // tx-testing-finished
        // ex: isAlive-Deadman-Switch-Wheeling expired
        // isAlive-Deadman-Switch-powerline expired
        // powerline-event
        // Powerline-frequency changed
        //   => from unstable to stable
        //   => from, stable to unstable
        //
        // There are two reasons to change the brake state - an active event or a timeout after a missing event.
        // Timeouts are detected with "watchdogs" every second.
        // The ONLY place where we change brake state is HERE. All other parts just set a semaphore, which is
        // handled here
        if(semaphore_ie1_powerline) {
            semaphore_ie1_powerline = false;
            if(brake_state == BRAKE_MOTORBRAKE) {
                // the motorbrake should never trigger this IRQ and PINC.7 (the switch for the magnetic).
                // Problem: the switch is ON in motorbrake-mode because TX is active LOW (and inactive high)
                // so switch motorbrake mode OFF and log an error. The only allowed state transition between
                // MOTORBRAKE and EDDYCURRENT is via BRAKE_AUTODETECT and BRAKE_MOTORBRAKE_TESTING
                brake_state = BRAKE_ERROR;
            } else if(!(brake_state & BRAKE_POWERLINE_TYPE)) {
                brake_state = BRAKE_POWERLINE_DETECTED;
            }
        }

        if(u_semaphore_rx_valid_4stateMachine) {
            u_semaphore_rx_valid_4stateMachine = false;
            if(brake_state & BRAKE_RX_ENABLED_TYPE) {
                brake_state = BRAKE_MOTORBRAKE;
            } else if(brake_state & BRAKE_EDDYCURRENT_TYPE){
                brake_state = BRAKE_ERROR;
            }
        }

#ifdef WITH_MOTORBRAKE
        if(!txON && brake_state == BRAKE_MOTORBRAKE_TESTING && !motorbrake_send_test) {
            brake_state = BRAKE_AUTODETECT;
        }
#endif

        // --------------------------------------------------------------------
        // 1 Hz main loop hook
        // --------------------------------------------------------------------

        if(event_1000ms_expired) {
            event_1000ms_expired = false;

            if(is_alive_ie1_powerline) {
                is_alive_ie1_powerline = false;
#ifdef WITH_EDDYCURRENT

                // "count_wheel_delta" is zero in motorbrake state 
                eddycurrent_speed = (uint16_t)eddycurrent_wheel_total - eddycurrent_wheel_total_last;
                eddycurrent_wheel_total_last = (uint16_t)eddycurrent_wheel_total;
                eddycurrent_wheeling = eddycurrent_speed > EDDYCURRENT_MIN_WHEEL_COUNT;
                if(eddycurrent_speed > 5) {
                    eddycurrent_speed *= 24; // * 24 does not match perfect
                } else {
                    eddycurrent_speed = 0;
                }            
                eddycurrent_powerline_detect();
#else           
                // switch to error state if firmware cannot handle Eddy current
                brake_state = BRAKE_ERROR;
#endif            
            } else if(brake_state & BRAKE_POWERLINE_TYPE) {
                brake_state = BRAKE_AUTODETECT;
            } 
#ifdef WITH_MOTORBRAKE
            else if(brake_state == BRAKE_AUTODETECT) {
                brake_state = BRAKE_MOTORBRAKE_TESTING;
                motorbrake_send_test = true;
            }
#endif
        }


        // --------------------------------------------------------------------
        // reconfigure brake after brake-state change
        // --------------------------------------------------------------------

        if(brake_state != last_brake_state) {
            last_brake_state = brake_state;

            // TODO reload ISR, timer, etc only on 'major' state changes, when it is necessary

            // ---------------------------
            // Motorbrake:
            //    TIMER0 for BITBANG_TX "baud" timer    with ISR
            //    IE0 for level change detect and sync
            //
            //    TIMER1 for
            //         serial0 baudgenerator (RX/TX)    without ISR
            //         or BITBANG_RX "baud" timer       with ISR
            //
            // Eddy Current:
            //    TIMER1 as "PAC/PWM/RPC" timer (for pulse offset and pulse length) with ISR
            //    IE0    as wheel turning detector/counter with ISR
            //    IE1    as powerline detector/counter     with ISR
            // ---------------------------

            TR1 = false;        // stop Timer1
            TF1 = false;        // overflow IRQ reset
            ET1 = false;        // irq OFF

            EX0 = false;
            ET0 = false;

            txON = false;
            rxON = false;

#ifdef WITH_EDDYCURRENT
            eddycurrent_set_resistance(EDDYCURRENT_OFF_VALUE);
#endif

#ifdef WITH_SERIAL0
            disableSerial0();
#endif
            configurePortC();

            if(false) {
            }
#ifdef WITH_BITBANG
            else if( (brake_state & BRAKE_RX_ENABLED_TYPE)) {
                configureBitBang();
            }
#elif defined(WITH_SERIAL0)
            else if( (brake_state & BRAKE_RX_ENABLED_TYPE)) {
                configureSerial0();
            }
#endif
#ifdef WITH_EDDYCURRENT
            else if(brake_state & BRAKE_EDDYCURRENT_TYPE) {
                // Timer1 and IE0-ISR controlltimer1_isr_bitbang_rxs the 'PAC/PWM/RPC'-signal
                eddycurrent_configure();
            }
#endif

            switch(brake_state) {
#ifdef WITH_MOTORBRAKE
                case (uint8_t)BRAKE_MOTORBRAKE:
                    blinkcode_green(0b1001000000000000);
                    break;
#endif
                case (uint8_t)BRAKE_POWERLINE_DETECTED:
                    blinkcode_green(0b1010101010101010);
                    break;
#ifdef WITH_EDDYCURRENT
                case (uint8_t)BRAKE_EDDYCURRENT_50HZ:
                    blinkcode_green(0b1001001000000000);
                    break;
                case (uint8_t)BRAKE_EDDYCURRENT_60HZ:
                    blinkcode_green(0b1001001001000000);
                    break;
#endif
                case (uint8_t)BRAKE_ERROR:
                    blinkcode_green(0x5555);
                    blinkcode_red(0xaaaa);
                    break;
                default:
                    blinkcode_green(0);
                    break;
            }
        }

        // --------------------------------------------------------------------
        // LED blink code handling (together with blinkcode_shift() in 8Hz handler)
        // --------------------------------------------------------------------

        blinkcode_handle();

        // --------------------------------------------------------------------
        // Button handling
        // --------------------------------------------------------------------
        buttons_handle();

    }

}
