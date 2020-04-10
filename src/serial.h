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

#ifndef __SERIAL_H

#define __SERIAL_H


// ==============================================
// tx/rx communication stuff
// ==============================================

extern __bit  volatile         rxON;
extern __bit  volatile         txON;
extern __data volatile uint8_t timer2_serial_timeout;

extern __data uint8_t count_rx_err;
extern __bit          u_semaphore_rx_valid;
extern __bit          u_semaphore_rx_valid_4stateMachine;

extern __xdata uint8_t rxDecoded[];
extern __xdata uint8_t txDecoded[];

// extern void encodeTx(__xdata uint8_t* tx_src, uint8_t msglen_src);

extern void serial_handle_rx();
extern void serial_command_tx(uint8_t size);

extern void configureBitBang();
extern void configureSerial0(void);
extern void disableSerial0(void);

#endif
