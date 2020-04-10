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


__bit volatile  semaphore_ie1_powerline;
__bit volatile  is_alive_ie1_powerline;


// General ideas to improve the code
// * switch IRQ handling "back to user mode" after the time critical part is done, so another
//   time critical IRQ is handled ASAP
//
// Ideas to optimze steering: 
// * measure steering as often as possible and throw away measurements, if we detect
//   an IRQ in the "final wait loop"
// * try to predict steering state change (last value plus/minus a window) and use a Timer (reuse Timer2?)
//   to enable the "final wait loop" (switched back to user mode)
//   => increase frequency of Timer2 and switch to "final wait loop" if we expect a steering 
//   => if we missed the bit -> enter "final wait loop" one Timer2 IRQ earlier

#ifdef WITH_EDDYCURRENT

// --------------------------------------------
// powerline related
// --------------------------------------------
__data uint8_t  count_powerline;       // counter to detect powerline frequency (per interval (here 1sec) => Hz)
__data volatile uint8_t eddycurrent_powerline_freq;
__data uint8_t eddycurrent_detect_state;
__data uint8_t count_same_Hz;

// set by 1Hz sub-timer in timer2
__bit volatile event_1000ms_eddycurrent_powerline;

__bit  volatile          eddycurrent_timer1_switch_to_state_B;
__data volatile uint16_t eddycurrent_timer1_magneticTime;

// --------------------------------------------

__data volatile int16_t eddycurrent_resistance_now;
__data int16_t eddycurrent_resistance_target;
__data int16_t eddycurrent_calculated_power;

// --------------------------------------------
// wheel related 
// --------------------------------------------

// when detected by powerline zero crossing with 50/60 Hz
// 0=frequency not detected, 1=50Hz, 2=60Hz
__bit  volatile eddycurrent_wheeling;
__data uint16_t eddycurrent_speed;

__data volatile uint8_t eddycurrent_wheel;        // incremented in ie0 
__data uint8_t eddycurrent_wheel_last;            // delta = wheel - wheel_last
__idata uint32_t eddycurrent_wheel_total;         // count wheel signals for total distance
__idata uint16_t eddycurrent_wheel_total_last;    // eddycurrent_speed = total - total_last;

__data uint16_t wheel_timestamp_last, wheel_timestamp_delta;  // used in ie0 

#ifdef WITH_EXTENDED_PROTOCOL
__pdata volatile uint16_t wheel_timestamps[EDDYCURRENT_TIMESTAMPS_SIZE];
__data volatile uint8_t wheel_timestamps_next_ptr;
__data uint8_t wheel_timestamps_last_ptr;
#endif

/*****************************************************************************
 * Timer 1 interrupt:
 * ------------------
 * 1a. IRQ Timer1 is used for eddy current timer handling
 *    state a) to wait for the "second half" of the powerline sinus
 *             The timer is started in ie1 (and changes here to state b)
 *    state b) to time the length of the "PWM-high-signal" for the magnetic
 *             field. Timer1 is startet and stopped here. After stopping
 *             we disable the timer.
 *
 *  state a) and b) are controlled by the 'switch_to_state_B' flag
 *
 * 1b. second use case is the as a clock for serial communication with
 *    the motorbrake:
 *
 *   a) as the baudgenerator for the serial0  (with IRQ switched off) or
 *
 *   b) to sample PINC.2 for bitbang-Rx-emulation
 *   (that's the only way the "solid green" device can drive the motorbrake)
 *
 * IE0 Interrupt
 * -------------
 * 2a. ie0_isr counts every "round" of the metal-wheel in the eddy current brake
 * 2b. triggers Rx High-Low detection when operating in Bitbang-Rx mode to
 *    sync Timer1 with the bit-rx-stream (main RX sampling is done in Timer1
 *    IRQ)
 ****************************************************************************/

void timer1_isr_eddycurrent(void)  __interrupt  {

    TR1  = false; // Running=OFF

    if(eddycurrent_timer1_switch_to_state_B) {
        eddycurrent_timer1_switch_to_state_B = false;
        // We write the whole byte to OUTC (instead of ANDing the bit)
        // -> works, because we have exaclty one 'OUT'-PORTC-Pin at a time
        // (in this case the magentic on PINC.1 ("solid blue") or PINC.7 ("solid green"))
        // PINC depends on the unit-type ... write whole byte (see note above)
        TH1 = HI8(eddycurrent_timer1_magneticTime);
        TL1 = LO8(eddycurrent_timer1_magneticTime);
        TR1 = true;
#ifndef DEBUG_EC_DRY_RUN
        OUTC = (headunit_device & HEADUNIT_SOLIDGREEN_TYPE) ? PORTC_MASK_MAGNET : PORTC_MASK_TX;
#endif 
    } else {
        OUTC = 0x00;
    }

    STATISTICS_IRQ(count_t1++);
}

void ie0_isr_eddycurrent(void) __interrupt {
    // I have a very old T1601 (Cycleforce Basic) where
    //   - 40 interrupts per second correspond to 1 m/s 
    //   => a speed of 1 m/s corresponds to a time difference of about 25 ms between two interrupts
    //   => ~25 m/s = ~90 km/h ==> delta_t ~= 1 ms ==> Timer precission is (2MHZ >> 4) 
    //   => 1ms = 2048 = 0x800 timer-events
    //   => wheel_timestamp_delta is then 0x800>>4 = 0x80 (precision 7 bit)

    {
        bool tf2;
        uint8_t th2, th2_tmp, tl2 ;
        uint8_t t2_lsb ;
        uint16_t timestamp;

        // NOTE: implementation expects same priority of ie0_ir and t2_ir, so they cannot interrupt each other

        th2 = TH2;
        tf2 = TF2;
        tl2 = TL2;
        th2_tmp = TH2;
        if(th2 != th2_tmp) {
            // TH2 rollover happend, just between reading TH2 and TL2 => read TF2 again and use new th2
            // we expect, that this happens fast enough, so no more rollover happens
            tl2 = TL2;
            tf2 = TF2;
            th2 = th2_tmp;
        }
        // t = (th2,tl2) - (- 0x1000)  = (th2,tl2)+0x1000 - 

        th2 = (th2<<4) | (tl2>>4);
        t2_lsb = count_t2;
        if(tf2)
            t2_lsb++;

        timestamp = th2 + ((uint16_t)t2_lsb<<8);

        wheel_timestamp_delta = timestamp - wheel_timestamp_last;
        wheel_timestamp_last  = timestamp;

#ifdef WITH_EXTENDED_PROTOCOL
        {
            uint8_t next = wheel_timestamps_next_ptr;
            timestamp &= 0x7fff; // only 15 bits for timer -- the MSB is used for CAD-detect
            if(PINSC & PORTC_MASK_CAD)
                timestamp |= 0x8000;
            wheel_timestamps[next] = timestamp;
            next++;
            next &= EDDYCURRENT_TIMESTAMPS_MASK;
            if(next != wheel_timestamps_last_ptr) {
                wheel_timestamps_next_ptr = next;
            }
        }
#endif
    }

    eddycurrent_wheel++; 
    STATISTICS_IRQ(count_ie0++);
}


/*
experimental: P_13 = f("eddycurrent_resistance_now", "wheel_timestamp_delta") 

             a             b          c            d              e
guess1 = (85,         1.52,       1.0,         14.3058742, 0.000811613017)
guess2 = (74.8380347, 1.56142716, 0.735762312, 13.7915949, 0.00107680030 )
guess4 = (79,         1.65213394, 1.09196131,  12.5882402, 0.000545285698)
guess5 = (75.4133716, 1.59032650, 0.819433457, 13.4871348, 0.000999338404)
guess6 = (75.3500839, 1.58474161, 0.817179504, 13.5293908, 0.00100103945 )

we use guess2 ..
*/

#define eX  (1694)                  // int(8 * 6 * e * 256 * 128)  # unsigned  9+3 bit
#define dX  (14123)                 // int(8 * d * 128)            # unsigned 12+3 bit
#define dX2 (574)                  // dX-((EDDYCURRENT_OFF_VALUE*eX)>>8))   
#define cX_  (48219)                 // int(256 * 256 * c)            # unsigned 16 bit
#define cX__  (6027)                 // int(256 * 64 * c)            # unsigned 16 bit
#define cX  (753)                   // int(256 * 4 * c)            # unsigned 10 bit
#define bX  (3198)                  // int(8 * 256 * b)            # unsigned  9+3 bit
#define aX1 (1123)                  // int(90/6*a)                   # unsigned 11 bit  # -brk/a + 90 => -brk/6 * a*90/6
#define aX2 (1261129)               // int(8100*a*a/6/6) (aX1*aX1) # unsigned 21 bit
#define aX3 (19706)                 // (aX2 * 4) >> 8              # unsigned 15 bit

// "#counts per second" * ~24 * ~290 = speed in km/h
// wheel_timestamp_delta in units of 2MHz/16 = 1/(2000000/16) seconds = 1/125000 seconds = 8us
// the brake triggers 4 signals per metal wheel turn - 
// R_fly_metal_wheel is ~16mm ==> circumference is ~0.10m ==> 4 signals in 0.10 m is 1 signal 
// in 0.025 meters ==> 0.025 * 125000 =  3125
//
// maybe TURN_LENGTH = 3125 (or 3052 with another aproximation) would be better 
// but we did the fitting with 2940, so this value is "inside" the parameters a,b,c,d,e

// "classic rawspeed" = 125000 / wheel_timestamp_delta * ~24 = 8 * (TURN_LENGTH) / wheel_timestamp_delta

#define TURN_LENGTH (128*2940L)     // s=v*t --- V=TURN_LENGTH / wheel_timestamp_delta
#define SCALE (13)                  // (wrong?) protocol sends power*13 as "currentLoad" 

// z1(brk) = d + brk * e       => "linear correction"
// z2(brk) = c*sin(brk/a)+b    
// p = z1 * speed + z2 * speed^2
//    or:
// F = z1 + z2 * speed
// p = F * speed
//

uint16_t eddycurrent_power_mul13() {

    __bit negate = false;

    // bool increaseForce = true;
    int16_t brk;   // max 11 bit + 1 bit sign 
    int32_t tmp32;
    uint16_t z1;    // max 14 (s)bit int16_t z1;

#define EC_MUL_Z2_16BIT 

#ifdef EC_MUL_Z2_16BIT
    uint16_t z2;    // Attention - sdcc has problems with: "uint16_t a = (uint32_t) b >> 8" other shift values work
#else
    int32_t z2;
#endif


    uint16_t tmp16;
    uint16_t speed; // max 11 bit (unsinged) for typical speeds 
    uint16_t wheel;

    // with timer2 at 2MHz and division by 16:
    //   wheel_timestamp_delta is about   0x60 for ~107 km/h  
    //   wheel_timestamp_delta is about   0x80 for 80 km/h  
    //   wheel_timestamp_delta is about 0x2000 for 1.5 km/h 
    EA = false;
    brk   = eddycurrent_resistance_now;
    wheel = wheel_timestamp_delta;
    EA = true;

    if(wheel == 0)
        return 0;

    // 0x60 is > 105 km/h
    tmp16 = 0x60;
    if(wheel > 0x60)
        tmp16 = wheel;
#ifdef EC_CHECK_LIMTS       
    if(brk > 0x1000)
        brk = 0x1000;
#endif

    // speed: max. 12 bit unsinged for typical speeds 
    // (with wheel_timestamp_delta value = 0x60, it is a little lesser then 0x1000)
    speed = (TURN_LENGTH) / tmp16;      

    // !! without (uint32_t) sdcc shifts the 32bit wrong to 16bit (?!) !!
    z1 = (uint32_t) (mul16x16(brk,eX) >> 8);      // 12bit+12bit=24bit >>8 => 16 bit  (max 0x1000*3387 = 53280)    
    z1 += dX2;                          // 16 bit => 16 bit  (max 53280+1149)

    brk -= EDDYCURRENT_OFF_VALUE;       // signed 11+1 = 12 bit

    if(brk < 0) {
        brk = -brk;
        negate = true;
    }

    brk -= aX1;                         // 11(+1) = signed 12 bit with zero check => 11 bit (unsigned)
    
    if(brk < 0)                        
        brk = -brk;                     // for unsigned mul 

    tmp32 = mul16x16(brk,brk);          // 2*11 = 22 bit (unsigned)
    tmp16 = tmp32>>8;                   // 22 bit usigned >>8  => 14 bit unsigned  
    tmp16 += aX3;                       // 14 ubit + 15 ubit   => 16 bit unsigned

    tmp32 = aX2 - tmp32;                // z2 = [0...aX2] (because tmp32 <= aX2)
    if(tmp32 < 0) {
        tmp32 = -tmp32;
        negate = !negate;
    }

    tmp32 <<= 8;      
    z2 = tmp32 / tmp16;                 // tmp16 = [aX3 ... aX3+(ax2/256)] result fits into 14 bit - max 0x3fff

    z2 = mul16x16(z2,cX__) >> 16;       // z2-shift-11 + cX-shift-16 => shift-27

    if(negate)
        z2 = -z2;

    z2 += bX;                           // "shift 11" => Fixed wrong z2 shift (former 8 now 11)

#if defined(EC_MUL_Z2_16BIT)

    z2 = mul16x16(z2,speed) >> 9;       // s11+s8 => s19 --- s19-8  => s11
    z1 >>= 1;
    // z2 can use 16 bits (unsigned) here , z1 fits into 13 bit
    z2 += z1;                           // 14/15 bit + 15bit ->  15/16 bit (for high speeds -> 16 bit)

#else
    z2 = mul16x16(z2,speed) >> 8;       // s11+s8 => s19 --- s19-8  => s11

    // z2 can use 17 bits (unsigned) here , z1 fits into 14 bit
    z2 += z1;                           // 14/15 bit + 15bit ->  15/16 bit (for high speeds -> 16 bit)
    z2 >>= 1;
#endif

    z2 = mul16x16(z2,speed) >> 16;

    return mul16x16(z2,13);
}


void ie1_isr_eddycurrent(void)      __interrupt  {
    // if we have an eddy current brake, this is a good point to see it switched OFF
    // (if everything works ok, the magnetic field is still switched off at this point)
    // note : in motorbrake-mode we never expect this IRQ
    OUTC = 0x00;

    if(TR1) {
        // TR1 still switched ON => should never happen
        // not thread safe, but we do not care - next try will probably work
        brake_state = BRAKE_ERROR;
        TR1 = false;
    } 
    
    eddycurrent_resistance_now = EDDYCURRENT_OFF_VALUE;
    if(brake_state & BRAKE_EDDYCURRENT_TYPE 
                && eddycurrent_wheeling
                && eddycurrent_resistance_target != EDDYCURRENT_OFF_VALUE) {
        int16_t absCurrentForce;
        uint8_t mul = EDDYCURRENT_FACTOR_50HZ;
        __bit increaseForce = true;
        
        absCurrentForce = eddycurrent_resistance_target - EDDYCURRENT_OFF_VALUE;
        if(absCurrentForce < 0) {
            increaseForce = false;
            absCurrentForce = -absCurrentForce;
        }

        if(brake_state == BRAKE_EDDYCURRENT_50HZ) {
            if(increaseForce) {
                TH1 = HI8(EDDYCURRENT_PHASE1_DELAY_50HZ);
                TL1 = LO8(EDDYCURRENT_PHASE1_DELAY_50HZ);
            } else {
                TH1 = HI8(EDDYCURRENT_PHASE2_DELAY_50HZ);
                TL1 = LO8(EDDYCURRENT_PHASE2_DELAY_50HZ);
            }
            //eddycurrent_timer1_magneticTime = absCurrentForce * EDDYCURRENT_FACTOR_50HZ;
        } else {
            if(increaseForce) {
                TH1 = HI8(EDDYCURRENT_PHASE1_DELAY_60HZ);
                TL1 = LO8(EDDYCURRENT_PHASE1_DELAY_60HZ);
            } else {
                TH1 = HI8(EDDYCURRENT_PHASE2_DELAY_60HZ);
                TL1 = LO8(EDDYCURRENT_PHASE2_DELAY_60HZ);
            }
            mul = EDDYCURRENT_FACTOR_60HZ;
        }

        eddycurrent_timer1_magneticTime = absCurrentForce * mul;
        eddycurrent_timer1_switch_to_state_B = true; // when timer1 overflows -> switch to stateB
        TF1 = false;
        TR1 = true;
        eddycurrent_resistance_now = eddycurrent_resistance_target;
    }

    count_powerline++;                  // for powerline frequency detect
    if(event_1000ms_eddycurrent_powerline) {
        event_1000ms_eddycurrent_powerline = false;
        eddycurrent_powerline_freq = count_powerline;
        count_powerline = 0x00;
    }

    semaphore_ie1_powerline = true;
    is_alive_ie1_powerline  = true;     // deadman-switch checked every 1000sec

    STATISTICS_IRQ(count_ie1++);
}


/*****************************************************************************
 * EddyCurrent-Timer starts with powerline-zero-crossing ..
 * values for TH1 and TL1 depend on the desired resistence.
 *  A Magnetic field in the "first half" of the sinus-period increases resistence
 *  and magnetic field in the "second half" decrease it.
 ****************************************************************************/

void eddycurrent_configure(void) {
    RESET_TIMER1();

    // TODO _ 
    // to increase steering quality we could switch T2 to HIGH prio in eddy current
    // so the start time is more precise - or use (unused) Timer0 for that
    // PT1 = false;           // Timer1 Prio LOW
    // PX1 = false;           // INT1 Prio LOW
    // PT2 = true;              

    PX0 = false;          // IE0 Priority LOW, so T2 and INT0 can easily use same memory (count_wheel)

    INSTALL_ISR(TF1_VECTOR, &timer1_isr_eddycurrent);
    INSTALL_ISR(IE0_VECTOR, &ie0_isr_eddycurrent);

    ET1 = true;         // here irq always ON - we controll TR1
    TMOD  |= M10;       // 16bit timer -- GATE1=int-gate, CT1=Counter/Timer select, M11/M10=mode select (b00=13bit,b01=16bit,b10=8bit reload,b11=timer1 stopped)

    EX0 = true;         // EC-mode: wheel
}

/*****************************************************************************
 * The detection is slow (4-5 seconds) but simple 
 ****************************************************************************/

void eddycurrent_powerline_detect(void) {
    uint8_t state = 0;

    if(eddycurrent_powerline_freq >= 45) {
        if(eddycurrent_powerline_freq < 55)
            state = BRAKE_EDDYCURRENT_50HZ;
        else if(eddycurrent_powerline_freq < 65)
            state = BRAKE_EDDYCURRENT_60HZ;
    }

    if(brake_state == state) {
        return;
    }

    if(eddycurrent_detect_state == state) {
        count_same_Hz++;
        if(count_same_Hz >= EDDYCURRENT_MIN_STABLE_POWERLINE_FREQ)  {  
            brake_state = state;
        }
    } else {
        count_same_Hz = 0;
        brake_state = BRAKE_AUTODETECT;
    }
    eddycurrent_detect_state = state;
}

void eddycurrent_set_resistance(int16_t new_resistance) {
    EX1 = false;
    eddycurrent_resistance_target = new_resistance;
    EX1 = true;
}

#endif

