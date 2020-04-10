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

extern __pdata uint8_t txbuf[];
extern __pdata uint8_t rxbuf[];

#if defined(WITH_BITBANG) && !defined(WITH_ASM8051_TX) || defined(WITH_SERIAL0)
extern  __data volatile  uint8_t txbufIdx;
#endif

extern  __data volatile  uint8_t txNext,txBitCnt,txNextBITSET,txByte;
extern  __data volatile   int8_t rxBitCnt;
extern  __data volatile  uint8_t rxByte;
extern  __data volatile  uint8_t rxbufIdx;
extern  __data          uint16_t rxChecksum;
extern  __bit volatile          rxEND;

// working data
extern __data volatile uint8_t  count_powerline;       // counter to detect powerline frequency (per interval (here 1sec) => Hz)
extern __code struct usb_all_descriptors all_descriptors; // usb_device_descriptor device_descriptor;

#ifdef WITH_ANALYZER
void handle_analyzer(bool writeDataPage) {

    uint8_t  len  = setup_data.wLength > 0x40 ? 0x40 : setup_data.wLength;
    uint16_t idx  = setup_data.wIndex;
    uint16_t addr = setup_data.wValue;
    bool     useINBUF = setup_data.bmRequestType & USB_DIR_IN;

    uint16_t i;

    // TODO: arm the OUT0 endpoint in advance
    if(!writeDataPage && !useINBUF) {
        OUT0BC =  0;
        return;
    }

#define inbuf   IN0BUF
#define outbuf  OUT0BUF

    switch(setup_data.bRequest) {
        case (uint8_t)0x80:
            // application / ports
            inbuf[0] = headunit_device;
            inbuf[1] = brake_state;
            inbuf[2] = (uint8_t) txbuf;
            inbuf[3] = (uint8_t) rxbuf;
            inbuf[4] = (uint8_t) trainer;
            inbuf[5] = (uint8_t) eeprom;
            inbuf[6] = (uint8_t) LO8(&all_descriptors.device_descriptor);
            inbuf[7] = (uint8_t) HI8(&all_descriptors.device_descriptor);

            *((uint32_t *)&inbuf[16])  = count_t2;

#ifdef WITH_STATISTICS_IRQ
            inbuf[20]  = count_t0;
            inbuf[21]  = count_t1;
            inbuf[22]  = count_serial0;
            inbuf[23]  = count_ie0;
            inbuf[24]  = count_ie1;

            inbuf[25]  = count_ep12_in;
            inbuf[26]  = count_ep12_out;
            inbuf[27]  = count_sudav_isr;
#endif

#ifdef WITH_STATISTICS_GENERAL
            inbuf[28] = count_steering;
            inbuf[29] = count_in_frames12;
            inbuf[30] = count_out_frames12;
            inbuf[31] = count_out_frames_skipped;
#endif

#ifdef WITH_EDDYCURRENT
            inbuf[32] = count_powerline;
            inbuf[33] = eddycurrent_wheel_total & 0xff;
#endif

#ifdef WITH_STATISTICS_SERIAL
            inbuf[40] = count_rx_overflow_err;
            inbuf[41] = count_rx_checksum_err;
            inbuf[42] = count_rx_frame_err;
            inbuf[43] = count_txrx_collision;
            inbuf[44] = count_rx_start_err;
            inbuf[45] = count_rx_stop_err;
            inbuf[46] = HI8(count_rx_valid);
            inbuf[47] = LO8(count_rx_valid);
            inbuf[48] = count_tx_err;

#ifdef WITH_BITBANG
#ifdef WITH_ASM8051_RX
            inbuf[49] = R1r6;
#else
            inbuf[49] = rxBitCnt;
#endif
#endif
            inbuf[50] = rxByte;
            inbuf[51] = rxbufIdx;
            inbuf[52] = rxON | (rxEND<<1);

            inbuf[55] = HI8(rxChecksum);
            inbuf[56] = LO8(rxChecksum);
#endif  // WITH_STATISTICS_SERIAL

#ifndef WITH_EMULATE_USB_ANTFE_C            
            inbuf[57] = classic_inbuf_fortius.brake.l.checksumMSB;
            inbuf[58] = classic_inbuf_fortius.brake.l.checksumLSB;
#endif

#ifdef WITH_STATISTICS_SERIAL
#if defined(WITH_BITBANG) && !defined(WITH_ASM8051_TX) || defined(WITH_SERIAL0)
            inbuf[59]  = txbufIdx;
#endif
#ifndef WITH_ASM8051_TX
            {
                extern __data volatile uint8_t txNext;
                extern __data volatile uint8_t txNextBITSET;
                extern __data volatile uint8_t txByte;
                extern __data volatile uint8_t txBitCnt;
                inbuf[60]  = txByte;
                inbuf[61]  = txNext;
                inbuf[62]  = txBitCnt;
                inbuf[63]  = txON;
            }
#else
                inbuf[60]  = R1r1;
                inbuf[61]  = R1r2;
                inbuf[62]  = R1r3;
                inbuf[63]  = R1r4;
#endif
#endif // WITH_STATISTICS_SERIAL
            break;

        case (uint8_t)0x88:
            // read or write xdata
            for(i=0;i<len;i++) {
                if(useINBUF) {
                    inbuf[i] = xdata[addr+i];
                } else {
                    // because we load the OUT0BC to late, the buffer contains the data from the last write
                    xdata[addr+i] = outbuf[i];
                }
            }
            break;
        case (uint8_t)0x89:
            // read or write idata
            for(i=0;i<len;i++) {
                if(useINBUF) {
                    inbuf[i] = idata[addr+i];
                } else {
                    // because we load the OUT0BC to late, the buffer contains the data from the last write
                    idata[addr+i] = outbuf[i];
                }
            }
            break;
        case (uint8_t)0x8a:
            for(i=0;i<len;i++) {
                inbuf[i] = code[addr+i];
            }
            break;

#ifdef WITH_DEVELOPMENT
        // case (uint8_t)0x8c:
        // {
        //     extern uint16_t wheel_timestamp_delta;

        //     *((uint16_t *)inbuf) = idx;
        //     *((uint16_t *)(inbuf+2)) = addr;
        //     eddycurrent_resistance_now = addr;
        //     wheel_timestamp_delta = idx;
        //     *((uint16_t *)(inbuf+4)) = eddycurrent_power_mul13();
        //     break;
        // }
        case (uint8_t)0x8b:
            switch(addr) {
                case 2:
                    trainer.currentResistance = idx;
                    break;
                case 6:
                    trainer.fake = idx;
                    break;
                case 7:
                    {
                        // renumber via interface

                        *((uint16_t *)&xdata[(uint16_t)&all_descriptors.device_descriptor.idProduct]) = idx;
                        usb_renumber();
                    }
                    break;
#ifdef WITH_EMULATE_USB_ANTFE_C
                case 0:
                    xdata[(uint16_t)(&page_answers_static.manufacturer_information.manufacturer_id)] = idx;
                    break;
                case 1:
                    xdata[(uint16_t)(&page_answers_static.manufacturer_information.model_id)] = idx;
                    break;
                case 3:
                    xdata[(uint16_t)(&page_answers_static.general_fe.speed)] = idx;
                    break;
                case 4:
                    xdata[(uint16_t)(&page_answers_static.general_fe.heart_rate)] = idx;
                    xdata[(uint16_t)(&page_answers_static.specific_trainer.instantaneous_cadence)] = idx >> 8;
                    break;
                case 5:
                    xdata[(uint16_t)(&page_answers_static.product_information.sw_revision)] = idx;
                    break;

#endif
            }
            break;
#endif

    }

    if(useINBUF)
        IN0BC =  len;

}
#endif
