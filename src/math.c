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

/** unsigned 16x16 => unsigned 32 bit multiplication 
 * takes less than 70 cycles (67? including lcall and ret)
 * 
 * uses 
 *   a,b,dpl,dph for computing and IN/OUT of parameters
 *   direct mem _mul16x16_PARM_2 (word) for param 2
 *   r4,r5       for storing temporary values
 */
#ifdef WITH_ASM8051

uint32_t mul16x16(uint16_t a, uint16_t b) __naked {
    a*b;   // make compiler happy

  __asm

    mov  a,dpl                     ; A0

    mov  b,(_mul16x16_PARM_2+0)    ; B0
    mul  ab              ; A0 * B0
    mov  r5,b            ; r5=C1 

    xch  a,dpl           ; => C0=a , a=A0

    mov  b,(_mul16x16_PARM_2+1)
    mul  ab              ; A0 * B1
    add  a,r5
    mov  r5,a            ; r5=C1
    clr  a
    addc a,b             ; 

    xch  a,dph           ; dph=C2, a=A1

    mov  r4,a            ; save r4=A1
    mov  b,(_mul16x16_PARM_2+0)
    mul  ab              ; A1 * B0        
    add  a,r5            ; += C1

    xch  a,dph           ; => C1=a , a=C2  
    addc a,b
    mov  r5,a            ; r5=C2
    clr  a
    rlc  a

    xch  a,r4            ; r4=C7, a=A1
    mov  b,(_mul16x16_PARM_2+1)

    mul  ab              ; A1 * B1
    add  a,r5
    xch  a,b             ; => C2
    addc a,r4            ; => C3
    ret

  __endasm;
}

#else

uint32_t mul16x16(uint16_t a, uint16_t b) {
  return (uint32_t)a*(uint32_t)b;
}

#endif
