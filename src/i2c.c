/***************************************************************************
 *   Copyright (C) 2012 by Johann Glaser <Johann.Glaser@gmx.at>            *
 *                                                                         *
 *   Modifications for EzTraCon Copyright (C) 2019 by Michael Hipp         *
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

#include <stdbool.h>
#include "main.h"
#include "reg_ezusb.h"
#include "i2c.h"

__xdata struct eeprom eeprom;

void i2c_read_eeprom(void) {

#define stRecvFirst 0x01
#define stReceiving 0x02
#define stSending   0x04
#define stStop      0x08
#define stEndSend   0x10
#define stExit      0x20

#define EEPROM_ADR  (0x00)
#define EEPROM_SIZE (0x10)

  uint8_t i2c_adr = (0x50 << 1);
  uint8_t i2c_state  = stSending;

  while (true) {
    uint8_t i2c_count = 0;

    if (I2CS & I2C_STOP)
        continue;

    I2CS  = I2C_START;
    I2DAT = i2c_adr;
    i2c_adr++; // next is (0x50 << 1)|0x1

    while (true) {

        if(!(I2CS & DONE))
            continue;

        // check for bus error and for missing NACK
        if ( (I2CS & BERR) || (!(i2c_state & stReceiving) && (!(I2CS & ACK)))) {
            i2c_state = stStop|stExit;  // triggers stop in state machine
        }
        if (i2c_count == (EEPROM_SIZE-2)) {
            // set LASTRD for the second-last byte
            I2CS |= LASTRD;
        }
        if (i2c_count == (EEPROM_SIZE-1)) {
            // set STOP bit for the last byte -> will generate I2C stop condition after it was received
            i2c_state = stReceiving|stStop|stExit;
        }

        if(i2c_state & stStop)
            I2CS |= I2C_STOP;
        if(i2c_state & stReceiving)
            ((__xdata uint8_t *)&eeprom)[i2c_count++] = I2DAT;

        if(i2c_state & stEndSend)
            break;
        if(i2c_state & stExit)
            return;

        if(i2c_state & stRecvFirst) {
            // read from I2DAT and discard the value -> initiate first burst of 9 SCL
            // pulses to clock in the first byte from the slave
            if (I2DAT)
                ;
            i2c_state = stReceiving;
        }
        if(i2c_state == stSending) {
            I2DAT = EEPROM_ADR;
            i2c_state = stStop|stEndSend;
        }
    }

    i2c_state = stRecvFirst;
  }

}
