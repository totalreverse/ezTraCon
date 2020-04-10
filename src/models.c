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

/*

power = offsetPower + scaleLoad * (offsetLoad + load) * rawSpeed
load = (power - offsetPower) / scaleLoad / v_raw - offsetLoad
load = (F_simulation * v - offsetPower) / scaleLoad / v_raw - offsetLoad

offsetPower = -35.4
scaleLoad = .0000070529
scaleLoad<<32 = 30292
1/scaleLoad = 141786 = 2*3*3*7877
offsetLoad = 851

*/

/** antPower is in units of 0.25 => fixed point 14.2 type */

uint16_t antPower2load(void) {

    // target_power units 0.25 W => fixed point 14.2  =>
    // 35.4*4 =  ~142, 141786/4 = ~35447
    uint16_t load = mul16x16(trainer.page49.target_power + 142, 35447) / trainer.rawSpeed;
    
    if(load < 851) 
        return 0;
    return load - 851;
}


uint16_t load2power(void) {

    uint16_t t2,load = trainer.currentResistance;
    uint32_t t1;

    // negative values => 0
    if(load >= 0x8000) {
        return 0;
    }

    load += 851;

    // Check me: power = l + 851 * rawspeed * 1/c1 - 35
    t1 = mul16x16(load, 30292); // 30292 = (1/scaleLoad)<<32
    t1 = mul16x16(t1>>16, trainer.rawSpeed);

    t2 = t1>>16;

    if(t2 < 35) {
        return 0;
    }
    return t2 - 35;
}