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

#ifndef __TRAINER_H
#define __TRAINER_H

#include "antplus.h"

#define T1942_SLOPE_MIN             (20000-500)
#define T1942_SLOPE_MAX             (20000+2000)

// 289.75  -- convert km/h to "raw_speed" 
// For antplus we want units of 0.001 m/s -> Scale is .9586808551
#define T1942_SPEED_SCALE_ANT       (245)
#define T1942_SPEED_SCALE_RAW       (290)

// (T1942_CALIBRATE_DEFAULT + 8) * T1942_SCALE_CALIBRAET
#define T1942_CALIBRATE_DEFAULT_RAW (1040)
 

// =============================================================================================
// Trainer operation modes
// =============================================================================================

#define TRAINER_MODE_NONE              ((uint8_t)0x00)
// The trainer controls the force to achive a constant power even if the speed changes
#define TRAINER_MODE_POWER             ((uint8_t)0x01)

/* ANT FEC: Percentage of maximum resistance to be applied.
 "The total resistance field allows [...] to set the resistance to be applied [..] This field is
  transmitted as a percentage of the maximum resistance [...] is capable of applying.
  Note that the maximum resistance [...] may be variable. For example the
  maximum resistance that a trainer can apply often varies based on the current cycling speed. In this case [...] 
  apply the requested resistance as a percentage of the current maximum."
  T1941: mode=2, weight=0xa - fixed value: we use percent * 128, so 100% is "12800" - this is a target power of 1000W at about 35 kmh ...
*/
#define TRAINER_MODE_BASIC_RESISTANCE  ((uint8_t)0x02)
// simulation mode - Simulation of descent if possible 
#define TRAINER_MODE_SIMULATION        ((uint8_t)0x03)
// not supported yet
#define TRAINER_MODE_CALIBRATE         ((uint8_t)0x04)

struct trainer {
    uint16_t rawLoad;        // T1941 protocol load value 

    uint8_t  pedalecho;
    uint16_t rawSpeed;
    uint16_t currentResistance;
    uint8_t  cadence;
    uint8_t  mode;           // our cmds 0-3

    uint16_t calibration;    // t1941 cmd
    uint8_t  flywheel;       // t1941 cmd

    uint8_t  countAnswers;
    uint16_t avgWatt;
    uint16_t watts[4];

    // DEBUG: fake fixed power/resitance via Debug-Console
    uint16_t fake;

    // ---------------------
    // ANT buffer
    // ---------------------
    // TODO
    /*
    uint16_t speed;                 // Instantaneous (real) speed: units 0.001 m/s, set to 0xFFFF if invalid
    uint8_t  heart_rate;            // Instantaneous heart rate, 0xFF indicates invalid, unit: 1 bpm
    uint8_t  distance_traveled;	    // Accumulated value of the distance traveled since start of workout: unit: 1 meter
    */

    // CHECKME
    // Physics: F_total = F_gravity + F_roll + F_wind
    // with slope as angle "a":  v_total = v_bike + v_wind,  Wind_Resistance_Coefficient = f(c_w * A * density)
    // F_gravitiy = g_earth * mass_total * sin(a)
    // F_roll = f_roll * g_earth * mass_total * cos(a)
    // F_wind = Drafting_Factor * Wind_Resistance_Coefficient(c_w * A * const) * v_total^2
    
    struct page48 {
        // 6.8.1 Data Page 48 (0x30) - Basic Resistance
        // -------------------------------------------
        // Byte  Description
        //  0    Data Page Number           48 (0x30)
        // 1-6   Reserved                   0xff,0xff,0xff,0xff,0xff,0xff  (reserved for future use)
        //  7    Percentage of maximum resistance to be applied.
        //       Units: 0.5&
        //       Range: 0 - 100%
        // "The total resistance field allows the open display to set the resistance to be applied by the fitness equipment. This field is
        //  transmitted as a percentage of the maximum resistance that the fitness equipment is capable of applying.
        //  Note that the maximum resistance that fitness equipment is capable of applying may be variable. For example the
        //  maximum resistance that a trainer can apply often varies based on the current cycling speed. In this case the fitness
        //  equipment shall apply the requested resistance as a percentage of the current maximum."
        // trainer.resistance = ((uint16_t)antplus_recv_buf[4+7]);  // resistance value
        uint8_t reserved[3];
        uint8_t total_resistance;
    } page48;
    struct page49 {
        // 6.8.2 Page 67 of 96 Data Page 49 (0x31) - Target Power
        // -------------------------------------------
        // Byte  Description
        //  0    Data Page Number           49 (0x31)
        // 1-5   Reserved                   0xff,0xff,0xff,0xff,0xff  (reserved for future use)
        // 6,7   Target Power, word, little endian
        //       Units: 0.25W
        //       Range: 0 - 4000W
        // trainer.power = *((uint16_t *)&antplus_recv_buf[4+6]);
        uint8_t reserved[2];
        uint16_t target_power;
    } page49;
    struct page50 {
        // 6.8.3 Data Page 50 (0x32) - Wind Resistance
        // -------------------------------------------
        // Byte  Description
        //  0    Data Page Number           Page 50 (0x32)
        // 1-4   Reserved                   0xff,0xff,0xff,0xff  (reserved for future use)
        //  5    Wind Resistance Coefficient
        //       Product of Frontal Surface Area, Drag Coefficient and Air Density. Use default value: 0xFF
        //          Units: 0.01 kg/m
        //          Range: 0.00 – 1.86 kg/m
        //  6    Wind Speed
        //       Speed of simulated wind acting on the cyclist.
        //        (+) Head Wind
        //        (–) Tail Wind
        //         Units  1 km/h
        //         Range -127 – +127 km/h
        //       Use default value: 0xFF
        //  7    Drafting Factor
        //       Simulated drafting scale factor
        //       Use default value: 0xFF
        //         Units: 0.01
        //         Range: 0 – 1.00

        uint8_t reserved[1];
        // Product of Frontal Surface Area, Drag Coefficient and Air Density. 
        // Use default value: 0xFF (ant fec recommends 0.51 kg/m as default)
        // unit=0.01 kg/m , range 0.00 – 1.86 
        // Wind Resistance Coefficient [kg/m] = Frontal Surface Area [m2] x Drag Coefficient x Air Density [kg/m3]
        uint8_t wind_resistance_coefficient;  
        
        // Speed of simulated wind acting on the cyclist.
        // Use default value: 0xFF (then use "0 km/h" wind)
        // unit = 1 km/h, range -127 – +127 (PLUS is Head Wind, MINUS is Tail Wind)
        // Relative Speed [m/s] = Bicycle Speed + Wind Speed
        // Wind Resistance [N] = (0.5 Wind Resistance Coefficient x (Relative Speed / 3.6)**2 ) x Drafting Factor
        uint8_t wind_speed; 
    
        // Simulated drafting scale factor (dimensionless)
        // Use default value: 0xFF (then use factor "1.0")
        // unit = 0.01 , range 0 – 1.00
        uint8_t drafting_factor;
    } page50;
    struct page51 {
        // Table 6-43. Data Page 51 Format - Track Resistance 0x33 = 51
        // -------------------------------------------
        // Byte  Description
        //  0    Data Page Number          51 (0x33)
        // 1-4   Reserved                   0xff,0xff,0xff,0xff  (reserved for future use)
        // 5-6   Grade (Slope) 2 Bytes little endian Grade of simulated track
        //          Units 0.01%
        //          Range -200.00% – % 200.00%
        //          Invalid, use default value: 0xFFFF
        //          Example:
        //             0x0000 -200.00%
        //             0x9C40 +200.00%
        //             0x4E20    0.00%
        //   7   Coefficient of Rolling Resistance between bicycle tires and track terrain (dimensionless)
        //         Use default value: 0xFF
        //         Units: 5x10^-5
        //         Range 0.0 – 0.0127

        uint8_t reserved[1];
        // Grade/Slope 
        // Use default value: 0xFFFF (the set slope to 0% - flat track)
        // Units 0.01%, Range -200.00% – 200.00%
        // Simulated Grade (%) = (Raw Grade Value x 0.01%) – 200.00%
        // Gravitational Resistance [N] = (Equipment Mass + User Mass) x Grade/100 x 9.81
        uint16_t grade;

        // Coefficient of rolling resistance between bicycle tires and track terrain (dimensionless)
        // Use default value: 0xFF (then use "0.004" (asphalt road))
        // units 5x10**-5 ,  0.0 – 0.0127
        // Rolling Resistance [N] = (Bicycle Mass + Cyclist Mass) x Coefficient of Rolling Resistance x 9.8
        uint8_t coef_roll_res; 
    } page51;

    struct page55 {        
        uint8_t  page_number_55;
        uint16_t user_weight;					// The user weight entered on the display. units 0.01kg, range 0-655.34kg, Invalid: 0xFFFF
        uint8_t  reserved[1];
        uint8_t  bicycle_wheel_diameter_offset:4; // Offset applied to Bicycle Wheel Diameter. units 1mm, range 0-9, Invalid / No Offset: 0xF
        uint8_t  bicycle_weight:12;				// The bicycle weight entered on the display. units: 0.05kg, range 0-50kg. Invalid: 0xFFF
        uint8_t  bicycle_wheel_diameter;		// The bicycle wheel diameter entered on the display. units 0.01m, range 0-2.54m
        uint8_t  gear_ratio;
    } page55;

};


extern __pdata struct trainer trainer;

#endif
