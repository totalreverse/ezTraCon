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

#ifndef __EDDYCURRENT_H
#define __EDDYCURRENT_H

extern __bit volatile  semaphore_ie1_powerline;
extern __bit volatile  is_alive_ie1_powerline;

extern __bit  volatile eddycurrent_wheeling;
extern __bit volatile event_1000ms_eddycurrent_powerline;


//extern __data volatile uint16_t count_wheel_delta;
extern __data  uint16_t eddycurrent_speed;               // eddycurrent_speed = total - total_last;
extern __data volatile uint8_t eddycurrent_wheel;        // incremented in ie0 
extern __data uint8_t eddycurrent_wheel_last;            // delta = wheel - wheel_last
extern __idata uint32_t eddycurrent_wheel_total;         // count wheel signals for total distance
extern __idata uint16_t eddycurrent_wheel_total_last;    // count wheel signals for total distance 

#define EDDYCURRENT_TIMESTAMPS_SIZE (16)
#define EDDYCURRENT_TIMESTAMPS_MASK (EDDYCURRENT_TIMESTAMPS_SIZE-1)
extern __pdata volatile uint16_t wheel_timestamps[EDDYCURRENT_TIMESTAMPS_SIZE];
extern __data volatile uint8_t wheel_timestamps_next_ptr;
extern __data uint8_t wheel_timestamps_last_ptr;

// ie1 realted:
extern volatile __data uint8_t eddycurrent_powerline_freq;
// extern __data volatile uint8_t  count_powerline;       // counter to detect powerline frequency (per interval (here 1sec) => Hz)

extern __data int16_t eddycurrent_resistance_last;
extern __data volatile int16_t eddycurrent_resistance_now;
extern __data int16_t eddycurrent_resistance_target;
extern __data int16_t eddycurrent_calculated_power;

extern void eddycurrent_configure(void);
extern void eddycurrent_powerline_detect(void);
extern void eddycurrent_set_resistance(int16_t);
extern void eddycurrent_8Hz_hook();
extern uint16_t eddycurrent_power_mul13();

// Minimal counts (per second) before we detect a valid wheel speed (23 is about 2 km/h)
#define EDDYCURRENT_MIN_WHEEL_COUNT           (23)

// NOTE: "0x80" is the EDDYCURRENT_OFF_VALUE for the old T1902 protocol (1 Byte command). 
//       Because we now can use the 16 bit values from the T1942 protocol we work with
//       more precise values (factor 1<<4 = 16)

#define EDDYCURRENT_SHIFT                     (4)
#define EDDYCURRENT_MIN_STABLE_POWERLINE_FREQ (3)
#define EDDYCURRENT_OFF_VALUE                 (0x80<<EDDYCURRENT_SHIFT)
#define EDDYCURRENT_STEP_VALUE                ((1<<EDDYCURRENT_SHIFT)>>1)
#define EDDYCURRENT_MIN_VALUE                 (0x10<<EDDYCURRENT_SHIFT)
#define EDDYCURRENT_MAX_VALUE                 (0xf0<<EDDYCURRENT_SHIFT)

// State B timings (depending on 24MHz/12):
// EDDYCURRENT_FACTOR_60HZ = (EDDYCURRENT_FACTOR_50HZ*5/6)
#define EDDYCURRENT_FACTOR_50HZ            (-96>>4)
#define EDDYCURRENT_FACTOR_60HZ            (-80>>4)

// State A timings (depending on 24MHz/12): 
// Phase 1 (to increase force) starts with a little delay of about 0.1 ms
// Phase 2 (to decrease/compensate force) starts after a half period (50Hz => 10ms, 60Hz => 8.333ms) plus a little delay of 0.1 ms
#define EDDYCURRENT_PHASE1_DELAY_50HZ      (-200)
#define EDDYCURRENT_PHASE1_DELAY_60HZ      (-166)
//#define EDDYCURRENT_PHASE1_DELAY_60HZ      (EDDYCURRENT_PHASE1_DELAY_50HZ*5/6)
#define EDDYCURRENT_PHASE2_DELAY_50HZ      (-20200)
#define EDDYCURRENT_PHASE2_DELAY_60HZ      (-16833)
//#define EDDYCURRENT_PHASE2_DELAY_60HZ      (EDDYCURRENT_PHASE2_DELAY_50HZ*5/6)

#endif

