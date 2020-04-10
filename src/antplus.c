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

#ifdef WITH_EMULATE_USB_ANTFE_C

#include "main.h"
#include "antplus.h"


#define ANTPLUS_2_SECONDS_AT_4Hz (8)

#define ANT_STAGE_BUF_SIZE (64)
#define ANT_STAGE_BUF_EXTENDED_SIZE (ANT_STAGE_BUF_SIZE+32)

__data  uint8_t antplus_stage_ptr;
__data  uint8_t antplus_stage_ptr2send;
__xdata uint8_t antplus_stage_buf[ANT_STAGE_BUF_EXTENDED_SIZE];

// PREPARE_MAX = 9+1+ANT_EXT_MESG_DEVICE_ID_FIELD_SIZE(4)+ANT_EXT_MESG_RSSI_FIELD_SIZE(4)+ANT_EXT_MESG_TIME_STAMP_FIELD_SIZE(2)
// MSG_MAX = 1+1+PREPARE_MAX+1 (header,len,mesg,checksum)
// = 20
__pdata uint8_t antplus_scratch_buf[ANT_PREPAREBUF_MAX_SIZE];
// sync(1) + len(1) + channel(1) + ANTPLUS_MAX_LEN_RX_USED + checksum(1)
__pdata uint8_t antplus_recv_buf[ANTPLUS_MAX_LEN_RX_USED+4]; // +4 = sync,len,msgid,checksum


struct antplus_channel_id {
    uint16_t device_id;
    uint8_t  device_type;
    uint8_t  trans_type;
};

static __xdata uint8_t antplus_tx_buffer[ANTPLUS_MAX_CHANNEL][8];

// we store first byte of network key
static __pdata uint8_t antplus_network[ANTPLUS_MAX_NETWORK];

// network, 0x03 status (unassigned, assigned, searching, tracking)
static __pdata uint8_t antplus_channel_status[ANTPLUS_MAX_CHANNEL];
static __pdata uint8_t antplus_channel_status2[ANTPLUS_MAX_CHANNEL];  // (2 network bits)<<2 , (Channel Type nibble)<<4, tx_ready bit0, tx_ack bit1
static __pdata struct antplus_channel_id antplus_channel_ids[ANTPLUS_MAX_CHANNEL];
static __pdata uint8_t antplus_channel_timeout[ANTPLUS_MAX_CHANNEL];

#ifdef WITH_ANTPLUS_SEARCH_TIMEOUT_EMULATION
__pdata uint8_t  antplus_channel_hp_search_timeout[ANTPLUS_MAX_CHANNEL];
__pdata uint8_t  antplus_channel_lp_search_timeout[ANTPLUS_MAX_CHANNEL];
#endif

__bit antplus_cont_scanning;
__bit antplus_8Hz_4Hz_toggle;
__bit antplus_page50_51_toggle;

__data uint8_t antplus_channels_opened;
__data uint8_t antplus_msg_cnt;
// as seen by channel status req (0x52)  0xF0 Mask (0x00-0x60) channeltype, 0xc0 :
__data uint8_t antplus_lib_state;

__idata uint8_t antplus_page70_answer;
__idata uint8_t antplus_timestamp_high = 0x55; // += 16(<<8) every 8Hz call => 32768 Hz
__idata uint8_t antplus_elapsed_time;
__idata uint16_t antplus_accumulated_power;

void antplus_init(void) {
    trainer.calibration = T1942_CALIBRATE_DEFAULT_RAW;
}

/**
 * len = prepare_buf_size - 1 ==> is size-attribute of antplus message
 * Total antplus message is len +4 (for sync-, len-, channel#- and checksum-byte)
 * antplus_scratch_buf has to be in __pdata 
 * ASM to squeeze some code bytes
 */
#ifdef WITH_ASM8051
static  void antplus_prepare(uint8_t len)   {
        len;
        __asm   

        ;  uint8_t s = antplus_stage_ptr + len + 4;
        mov     a,dpl
        mov     r7,a
        mov     r5,_antplus_stage_ptr
        add     a,r5
        add     a,#0x04                  
        mov     r6,a

        ; ptr = _antplus_stage_buf + _antplus_stage_ptr
        mov     a,r5
        add     a,#_antplus_stage_buf
        mov     dpl,a
        clr     a
        addc    a,#(_antplus_stage_buf >> 8)
        mov     dph,a

        ;  if(s > ANT_STAGE_BUF_EXTENDED_SIZE) return
        add     a,#0xff - 0x60
        jc      _antplus_prepare_end     

        mov     a,r6
        mov     _antplus_stage_ptr,a
        add     a,#0xff - 0x40
        jc      _antplus_prepare_l1

        mov     _antplus_stage_ptr2send,r6   ; _antplus_stage_ptr2send = s;

_antplus_prepare_l1:

        // TX_SYNC
        mov     a,#0xA4
        movx    @dptr,a
        inc     dptr
        mov     r4,a
        // LEN 
        mov     a,r7
        movx    @dptr,a
        inc     dptr
        xrl     ar4,a

        ; for(i=0;i<=len;i++) {
        inc     r7
        mov     ar1,#_antplus_scratch_buf
_antplus_prepare_loop:
        ; !! antplus_scratch_buf has to be in __pdata !!!
        movx    a,@r1        ; uint8_t tmp = antplus_scratch_buf[i]; 
        inc     r1
        movx    @dptr,a
        inc     dptr
        xrl     ar4,a
        djnz    r7,_antplus_prepare_loop

        mov     a,r4         ; *ptr++ = xor;
        movx    @dptr,a
_antplus_prepare_end:
        ret

    __endasm;
}

#else
static void antplus_prepare(uint8_t len) {
    __xdata uint8_t *ptr;
    uint8_t i,xor;
    uint8_t s = antplus_stage_ptr + len + 4;

    ptr = antplus_stage_buf + antplus_stage_ptr;

    if(s > ANT_STAGE_BUF_EXTENDED_SIZE)
        return;
    if(s <= 64)
        antplus_stage_ptr2send = s;
    antplus_stage_ptr = s;

    xor = MESG_TX_SYNC;
    *ptr++ = xor;

    *ptr++ = len;
    xor ^= len;
    for(i=0;i<=len;i++) {
        uint8_t tmp = antplus_scratch_buf[i];
        *ptr++ = tmp;
        xor ^= tmp;
    }
    *ptr++ = xor;
    
}
#endif

void antplus_prepare_event(uint8_t channel, int8_t event, uint8_t msgId) {
    if(event < 0)
        return;

    antplus_scratch_buf[0] = MESG_RESPONSE_EVENT_ID;
    antplus_scratch_buf[1] = channel;
    antplus_scratch_buf[2] = msgId;
    antplus_scratch_buf[3] = event;
    antplus_prepare(3);
}

#ifdef WITH_LOAD_DATA_TO_UPPER_MEM
__code __at 0x1b00 static uint8_t antplus_request_answers_static[0x2a] = {
#else
__code static uint8_t antplus_request_answers_static[0x2a] = {
#endif
 0x0b+1, MESG_VERSION_ID,                'A', 'P', '2', 'U', 'S', 'B', '1', '.', '0', '5', 0x00, // 0x3E ant version
 // 0x54: capabilities RESP: [0x54] 08 03 00 ba 30 00 -> 08 channels, 03 networks, 00=no rx/tx limits,
 0x06+1, MESG_CAPABILITIES_ID,           0x08, 0x03, 0x00, 0xba, 0x36, 0x00, // 08 03 00 ba 36 00
 0x02+1, MESG_STACKLIMIT_ID,             0xad, 0x00,                      // 0x55 MESG_STACKLIMIT_ID
 0x04+1, MESG_GET_SERIAL_NUM_ID,         0xf4, 0x2b, 0x25, 0x12,          // 0x61 serial num NO
 0x01+1, MESG_FLASH_PROTECTION_CHECK_ID, 0x01,                            // 0xA7 flash protect
 0x05+1, MESG_GET_GRMN_ESN_ID,           0xa0, 0xa1, 0xa2, 0xa3, 0xa4,    // 0xC6 Garmin ESN (No?)
 0x00
};

#ifdef WITH_LOAD_DATA_TO_UPPER_MEM
__code __at 0x1b00+0x2a static uint8_t antplus_fec_page_order[] = {
#else
__code static uint8_t antplus_fec_page_order[] = {
#endif
  0x10,0x19,
  0x10,0x19,
  0x10,0x19,
  0x10,0x19,
  0x10,0x19,
  0x10,0x19,
  0x10,0x19,
  0x10,0x19,
  
  0x50,0x51
};

#ifdef WITH_LOAD_DATA_TO_UPPER_MEM
__code __at 0x1b40 struct page_answers_static  antplus_page_answers_static =
#else
__code struct page_answers_static  antplus_page_answers_static =
#endif
{
    {
        0x04,                                           // page_number 0x04 or 0x84  bit 7 is toggle bit
        0xff,                                           // uint8_t    manufacturer_specific
        0x0000,                                         // uint16_t   previous_heart_heat_event_time 
        0x0000,                                         // uint16_t   heart_heat_event_time
        0x00,                                           // uint16_t   heart_beat_count;
        0x00,                                           // uint16_t   computed_heart_rate;
    },
    {
        ANT_FE_MANUFACTURERS_ID_DATA_PAGE,              // page_number 0x50 page 80
        0xffff,                                         // uint16_t   reserved = 0xffff
        0x01,                                           // uint8_t    hw_revision 
        ANTPLUS_MANUFACTURER_ID_QUARQ,                  // uint16_t   manufacturer_id; // 0xFF development
        ANTPLUS_PRODUCT_ID_TACX_T2080_GENIUS,           // uint16_t   model_number;
    },
    {
        ANT_FE_PRODUCT_INFORMATION_DATA_PAGE,           // page_number 0x51 page 81
        0xff,                                           // uint8_t reserved; // 0xFF
        0x00,                                           // uint8_t sw_revision_minor;
        0x03,                                           // uint8_t sw_revision_major;
        ANTPLUS_DEVICE_ID,                              // uint32_t serial_number;
    },
    {
        ANT_FE_CAPABILITIES_DATA_PAGE,                  // page_number 0x36 page 54
        0xffffffff,                                     // reserved, always set to 0xFFFFFFFF
        100,                                            // The maximum applicable resistance of the trainer 0 – 65534N. Unit 1 Newton. Invalid: 0xFFFF
                                                        // => Power=Force*Velocity => W=N*m/s => ie: 1000W = 100N * 10 m/s (100N with 10m/s=36km/h)
        ANT_FE_CAPABILITIES_SUPPORTS_BASIC_RESISTANCE_MASK
         | ANT_FE_CAPABILITIES_SUPPORTS_TARGET_POWER_MASK
         | ANT_FE_CAPABILITIES_SUPPORTS_SIMULATION_MASK,    // capabilities_bit_field 
    },
    {
        ANT_FE_COMMAND_STATUS_DATA_PAGE,                // page_number 0x47 page 71
        0xff,                                           // last_received_command_id;  0xFF means no command rxed
        0xff,                                           // sequence_number;     
        0xff,                                           // command_status;  0 pass, 1 fail 2 not supported, 3 rejected 4 pending, 5-254 reserved, 255 uninitialized
        { 0xff,0xff,0xff,0xff },                        // data pertaining to the last command
    },
    {
        ANT_FE_GENERAL_FE_DATA_PAGE,                    // page_number 0x10 page 16 "ANT_FE_GENERAL_FE_DATA_PAGE"
        ANT_FE_EQUIPMENT_TYPE_TRAINER,                  // uint8_t 0x19, 25 == trainer
        0xff,                                           // Accumulated value of the elapsed time since start of workout, unit: 0.25,  overroll 64 seconds
        0xff,                                           // Accumulated value of the distance traveled since start of workout: unit: 1 meter, rollover 256m
        0xffff,                                         // Instantaneous speed: units 0.001 m/s, set to 0xFFFF if invalid
        0xff,                                           // Instantaneous heart rate, 0xFF indicates invalid, unit: 1 bpm
        ANT_FE_STATE_IN_USE,                            // FE State Bit Field      // 3 in use
        ANT_FE_CAPABILITIES_HR_DATA_SOURCE_EM,          // Capabilities Bit Field: 5khz EM (bit0/1=2), FE do not transmits distance (bit2=0), real speed (bit3=0)
    },
    {
        ANT_FE_SPECIFIC_TRAINER_DATA_PAGE,              // page_number 0x19 page 25
        0xff,                                           // Event counter increments with each information update
        0xff,                                           // crank cadence 0-254 rpm or 0xFF for invalid 
        0xffff,                                         // Accumulated power: units 1 watt
        0xfff,                                          // instantaneous power: units 1 watt. 0xFFF indicates BOTH instantaneous and accumulated power are invalid
        ANT_FE_STATUS_BICYCLE_POWER_CALIBRATION_REQUIRED_MASK,    // Trainer Status Bit Field :4;        // TEST: calibrate required
        ANT_FE_POWER_LIMIT_OPERATING_AT_TARGET_POWER,             // Flags Bit Field Description :4;
        ANT_FE_STATE_IN_USE,                            // FE State Bit Field      // 3 in use
    },
    0x00
};


static bool antplus_request_answer(uint8_t msgID) {
    uint8_t i,len;
    __xdata uint8_t *from =  (__xdata uint8_t *) antplus_request_answers_static;
    __pdata uint8_t *to;

    while(1) {
        len = *from++;
        if(len == 0x00)
            break;
        if(msgID == *from) {
            to =  antplus_scratch_buf;
            for(i=0;i<len;i++)
                *to++ = *from++;
            antplus_prepare(len-1);
            return 1;
        }
        from += len;
    }
    return 0;
}

static bool antplus_page_answer(uint8_t msgID) {
    uint8_t i;
    __xdata uint8_t *from =  (__xdata uint8_t *) antplus_page_answers_static;
    __pdata uint8_t *to = antplus_scratch_buf+2;

    while(1) {
        uint8_t pId = *from;
        if(pId == 0x00)
            break;
        if(pId == msgID) {
            for(i=8;i!=0;i--)
                *to++ = *from++;
            return 1;
        }
        from += 8;
    }
    return 0;
}

/**
 * Stores data from HOST in tx buffer for later processing if our channel is active (TRACKING or cont scan mode)
 */
static void antplus_copy_page_p2x( __xdata uint8_t *to,__pdata uint8_t *from) {
    uint8_t i;
    for(i=8;i!=0;i--)
        *to++ = *from++;
}

static void antplus_store_tx_2_scratch(uint8_t channelNo) {
    uint8_t i;
    __pdata uint8_t *to = antplus_scratch_buf;
    __xdata uint8_t *from = (__xdata uint8_t *) &antplus_tx_buffer[channelNo];
    for(i=8;i!=0;i--)
        *to++ = *from++;
}

static void antplus_handle_tx_fec(int channelNo) { 
    uint8_t pageNr;

    antplus_store_tx_2_scratch(channelNo);

    pageNr = antplus_scratch_buf[0];

    // TODO complete implementation of Command 0x30..0x33, 0x37, 0x47

    // Command we accept:
    //   Request Command Data Page
    //   Control  - Basic Resistance (PARTIALLY)
    //   Control  - Target Power
    //   Control  - Wind Resistance  (NOT YET)
    //   Control  - Track Resistance (PARTIALLY)

    if(pageNr >= ANT_FE_BASIC_RESISTANCE_DATA_PAGE && pageNr <= ANT_FE_TRACK_RESISTANCE_DATA_PAGE) {
        __xdata command_status_t *command_status = (__xdata command_status_t *) &xdata[(uint16_t)&antplus_page_answers_static.command_status];
        command_status->last_received_command_id = pageNr;
        // For commands that do not specify a sequence number (e.g. control pages 48 - 51), 
        // this value shall be incremented by the fitness equipment for each supported command received
        command_status->sequence_number++;
        command_status->command_status           = 0;        // 0 pass, 1 fail, 2 not supported, 3 rejected, 4 pending,

        ((__xdata uint32_t*)&trainer.page48)[pageNr-ANT_FE_BASIC_RESISTANCE_DATA_PAGE] = *((__pdata uint32_t *)(&antplus_scratch_buf[4]));

        // TODO Optimze: We could check command_status->last_received_command_id instead of "mode"
        trainer.mode  = TRAINER_MODE_SIMULATION; // Simulationmode
        if(pageNr == ANT_FE_BASIC_RESISTANCE_DATA_PAGE) {
            trainer.mode  = TRAINER_MODE_BASIC_RESISTANCE;
        }  
        if(pageNr == ANT_FE_TARGET_POWER_DATA_PAGE) {
            trainer.mode  = TRAINER_MODE_POWER;
        }
    } 
    if(pageNr == ANT_FE_USER_CONFIGURATION_DATA_PAGE) {
        antplus_copy_page_p2x(( __xdata uint8_t *)&trainer.page55, antplus_scratch_buf);
        //  ((uint32_t*))[0] =  *((uint32_t *)(antplus_scratch_buf+0+0));
        //  ((uint32_t*)&trainer.page55)[1] =  *((uint32_t *)(antplus_scratch_buf+0+4));
    } 
    if(pageNr == ANT_FE_REQUEST_DATA_PAGE && antplus_scratch_buf[7] == 0x01) {
        // Common Data Page 70 (0x46) – Request Data Page
        // 0xff, 0xff, 0xff, 0xff, number-of-transmission-response, requestd-page, 0x01 (request data page)
        antplus_page70_answer = antplus_scratch_buf[6];
    }
}

void antplus_set_channel_id(uint16_t device_id, uint8_t device_type, uint8_t trans_type, uint8_t channelNo) {
  __pdata struct antplus_channel_id *channelId = antplus_channel_ids + channelNo;

  channelId->device_id    = device_id;
  channelId->device_type  = device_type & 0x7f; // without pairing bit
  channelId->trans_type   = trans_type & 0xff;
}

static int8_t antplus_match_slave_channel(uint16_t device_trans_type) {
    int8_t channelScan = -1, channelTrack = -1;
    uint8_t i;

    // NOTE: Pairing Bit not supported
    // TODO check device is "running"

    for(i=0;i<ANTPLUS_MAX_CHANNEL;i++) {
        __pdata struct antplus_channel_id *channelId = antplus_channel_ids + i;
        __bit jokermatch = true;
        __bit match = true;

        // Only Slave channels count for RX
        if(antplus_channel_status2[i] & CHANNEL_MASK_MASTER) {
            continue;
        }
        // TODO antplus freq-band 57 check - we currently ignore frequency
        if(antplus_network[ (antplus_channel_status2[i]>>2) & (CHANNEL_MASK_NETWORK>>2) ] != ANTPLUS_NETWORK_KEY_LSB ) {
            continue;
        }

#ifdef ANTPLUS_DEVICE_ID_IS_STATIC
        if(channelId->device_id != ANTPLUS_DEVICE_ID) {
#else
        if(channelId->device_id != device_trans_type) {
#endif
            match = false;
            if(channelId->device_id != 0)
                jokermatch = false;
        }
        if(channelId->device_type != (uint8_t)device_trans_type) {
            match = false;
            if(channelId->device_type != 0)
                jokermatch = false;
        }
        if(channelId->trans_type != (uint8_t)(device_trans_type>>8)) {
            match = false;
            if(channelId->trans_type != 0)
                jokermatch = false;
        }
        //;

        if(antplus_channel_status[i] == STATUS_SEARCHING_CHANNEL) {
            if(jokermatch && channelScan < 0) {
                channelScan = i;
            }
        } else if(antplus_channel_status[i] == STATUS_TRACKING_CHANNEL || antplus_cont_scanning) {
            if(match) {
                channelTrack = i;
                break;
            }
        }
    }

antplus_match_channel_end:
    // channel id or mask (of channel 0) must match in cont scan mode, too
    if(antplus_cont_scanning) {
        return channelScan != 0 ? -1 : (channelTrack >= 0 ? channelTrack : 0);
    }

    if(channelTrack < 0 && channelScan >= 0) {
#ifdef ANTPLUS_DEVICE_ID_IS_STATIC
        antplus_set_channel_id(ANTPLUS_DEVICE_ID,LO8(device_trans_type),HI8(device_trans_type), channelScan);
#else
        antplus_set_channel_id(device_trans_type,LO8(device_trans_type),HI8(device_trans_type), channelScan);
#endif
        channelTrack = channelScan;
        antplus_channel_status[(uint8_t)channelTrack] = STATUS_TRACKING_CHANNEL; // channel to "tracking"
    }

    if(channelTrack >=0 ) { 
        antplus_channel_timeout[channelTrack] = 0;
    }

    return channelTrack;
}

void antplus_timeout_handling() {
    uint8_t i;

    for(i=0;i!=ANTPLUS_MAX_CHANNEL;i++) {
        if(antplus_channel_status[i] == STATUS_TRACKING_CHANNEL) {
            int8_t eventId = -1;
            if(antplus_channel_status2[i] & CHANNEL_MASK_MASTER) {
                eventId = EVENT_TX;
                if(antplus_channel_status2[i] & CHANNEL_MASK_TX_ACK) {
                    antplus_channel_status2[i] &= ~(CHANNEL_MASK_TX_ACK);
                    eventId = EVENT_TRANSFER_TX_FAILED;
                }
            } else if(antplus_channel_timeout[i] == 0) {
                antplus_channel_timeout[i] = ANTPLUS_2_SECONDS_AT_4Hz;
            } else {
                eventId = EVENT_RX_FAIL;
                antplus_channel_timeout[i]--;
                if(antplus_channel_timeout[i] == 0) {
                    eventId = EVENT_RX_FAIL_GO_TO_SEARCH;
                    antplus_channel_status[i] = STATUS_SEARCHING_CHANNEL;
                }
            }
            antplus_prepare_event(i,eventId,1);
        } else if(antplus_channel_status[i] == STATUS_SEARCHING_CHANNEL) {
            // TODO RX search timeout handling
            // TODO Channel timeout and channel close triggers "dual event" 0x01 & 0x07
        }
    }
}

void antplus_prepare_rx_msg(int8_t channelNo,uint8_t device_type, uint8_t transfer_type) {
    uint8_t p;

    // Stop sending pages if buffer is too full 
    if(antplus_stage_ptr >= ANT_STAGE_BUF_SIZE || channelNo < 0)
        return;

    antplus_scratch_buf[0] = MESG_BROADCAST_DATA_ID;
    antplus_scratch_buf[1] = channelNo;

    p = 0;
    if(antplus_lib_state > 0x00) {
        __pdata uint8_t *to = antplus_scratch_buf + 10;
        *to++ = antplus_lib_state;
        //antplus_scratch_buf[10] = antplus_lib_state;
        if(antplus_lib_state & 0x80) {
            // MESG_DEVICE_ID_FIELD
#ifdef ANTPLUS_DEVICE_ID_IS_STATIC            
            *to++ = LO8(ANTPLUS_DEVICE_ID);
            *to++ = HI8(ANTPLUS_DEVICE_ID);
#else
            *to++ = device_type;
            *to++ = transfer_type;
#endif
            *to++ = device_type;
            *to++ = transfer_type;
            p += ANT_EXT_MESG_DEVICE_ID_FIELD_SIZE;
        }
        if(antplus_lib_state & 0x40) {
            // MESG_RSSI_FIELD (rx level in dbm)
            *to++ = 0x10;
            *to++ = 0x00;
            *to++ = 0x68;
            *to++ = 0x00;
            p += ANT_EXT_MESG_RSSI_FIELD_SIZE;
        }
        if(antplus_lib_state & 0x20) {
            // MESG_TIME_STAMP_FIELD
            *to++ = 0x55;                   // timerLow - fix;
            *to++ = antplus_timestamp_high; // timerLow;
            p += ANT_EXT_MESG_TIME_STAMP_FIELD_SIZE;
        }
        p++;
    }
    antplus_prepare(p+9);
}



void antplus_8Hz_hook(void) {
    uint8_t page;
    int8_t channelNo;

    // main beat is 8Hz - we switch between HR and FEC so every device "sends" with 4Hz
    //
    // TODO (probably this will not fit into memory at the same time)
    // Other antplus devices:    
    //   Bike (pure) Speed and Cad
    //   Bike (pure) Power
    //   Bike pedal index 
    //
    // Non Antplus (different freq.-band, default network key):
    //   Tacx Steering 
    //   Tacx Control Unit

    
    antplus_timestamp_high += 0x08;
    if(antplus_timestamp_high & 0x08) {
        antplus_timeout_handling();

        // ---------------
        // MATCH Heart Belt
        // ---------------
        channelNo = antplus_match_slave_channel(ANTPLUS_HR_DEVICE_TYPE|(ANTPLUS_TRANS_TYPE_01<<8));
        if(channelNo >= 0) {
            // TODO page 0x04 gets written in IRQ - protect access with lock or disable Timer2 IRQ or with dirty flag and retry
            antplus_page_answer(0x04);
            antplus_scratch_buf[2] = 0x04 | (antplus_timestamp_high & 0x40)<<1;
            antplus_prepare_rx_msg(channelNo,ANTPLUS_HR_DEVICE_TYPE, ANTPLUS_TRANS_TYPE_01);
        }
        return;
    }



    {
        //__xdata heart_rate_t *hrpage = (__xdata heart_rate_t *) &xdata[(uint16_t)&antplus_page_answers_static.heart_rate];
        __xdata general_fe_t *page10 = (__xdata general_fe_t *) &xdata[(uint16_t)&antplus_page_answers_static.general_fe];        
        // if(wheeling)
            antplus_elapsed_time++;            
            page10->elapsed_time = antplus_elapsed_time; // units 0.25 = 4Hz
            page10->speed = trainer.rawSpeed;            // units 0.001 m/s - fortius raw matchs almost
            //page10->heart_rate = hrpage->computed_heart_rate;     // units: bpm
            // TODO
            // Accumulated value of the distance metres traveled since start of workout
            //antplus_scratch_buf[5] = trainer.totalDistance;
            //antplus_scratch_buf[6] = trainer.speed;     // current speed lo
            //antplus_scratch_buf[7] = trainer.speed>>8;  // current speed hi
            //antplus_scratch_buf[8] = trainer.heartRate; // current Heartrate
    }
    {
        uint16_t power;
        __xdata specific_trainer_t *page19 = (__xdata specific_trainer_t *) &xdata[(uint16_t)&antplus_page_answers_static.specific_trainer];
        // Update page 0x19, 25, ANT_FE_SPECIFIC_TRAINER_DATA_PAGE
        page19->update_event_count++;
        page19->instantaneous_cadence = trainer.cadence;
        // xdata[(uint16_t)(&page_answers.specific_trainer.instantaneous_cadence)] = ...
        
        //antplus_accumulated_power   += power;
        //page19->accumulated_power    = antplus_accumulated_power;
        page19->instantaneous_power  = (trainer.avgWatt>>2) & 0xfff;    // 12 bit ... (4 bit nibble) + Trainer Status Bit Field - units 1 W
    }
    

    // ---------------
    // MATCH Fitness Equipment FEC
    // ---------------
    channelNo = antplus_match_slave_channel(ANTPLUS_FEC_DEVICE_TYPE|(ANTPLUS_TRANS_TYPE_05<<8));

    if(channelNo < 0)
        return;


    if(antplus_channel_status2[channelNo] & (CHANNEL_MASK_TX_ACK|CHANNEL_MASK_TX_READY)) {
        uint8_t eventId = EVENT_TX;
        if(antplus_channel_status2[channelNo] & CHANNEL_MASK_TX_ACK)         
            eventId = EVENT_TRANSFER_TX_COMPLETED;
        antplus_prepare_event(channelNo,eventId,1);
        antplus_channel_status2[channelNo] &= ~(CHANNEL_MASK_TX_ACK|CHANNEL_MASK_TX_READY);

        antplus_handle_tx_fec(channelNo);

        // TODO channel based processing of data send from HOST to device
        // TODO page70 is only for ANTFE_C device

        if(antplus_page_answer(antplus_page70_answer)) {
            antplus_page70_answer = 0;
            goto antplus_8Hz_hook_end;
        }

    }

    page = antplus_fec_page_order[antplus_msg_cnt];
    antplus_msg_cnt++;
    if(antplus_msg_cnt == sizeof(antplus_fec_page_order))
        antplus_msg_cnt = 0;
    // switch(antplus_msg_cnt) {
    //     case 17:
    //         antplus_msg_cnt = 0;
    //         antplus_page50_51_toggle = !antplus_page50_51_toggle;
    //     case 16:
    //         page = ANT_FE_PRODUCT_INFORMATION_DATA_PAGE;
    //         if(antplus_page50_51_toggle) {
    //             page = ANT_FE_MANUFACTURERS_ID_DATA_PAGE;
    //         }
    //         break;
    //     default:
    //         page = ANT_FE_SPECIFIC_TRAINER_DATA_PAGE;
    //         if(antplus_msg_cnt & 0x1) {
    //             page = ANT_FE_GENERAL_FE_DATA_PAGE;
    //         }
    //         break;
    // }
    antplus_page_answer(page);

    // Handling for cont_scanning
    // 1. if mask of channel 0 matchs send RX stuff on channel0
    // 2. if we have an exact match on a non zero channel we use this to confirm pending TX ot TX_ACK 

antplus_8Hz_hook_end:
    if(antplus_cont_scanning)
        channelNo = 0;

    antplus_prepare_rx_msg(channelNo,ANTPLUS_FEC_DEVICE_TYPE, ANTPLUS_TRANS_TYPE_05);
}

/**********************************************************************************
 **********************************************************************************
 **** MAIN communication handler für USB
 **********************************************************************************
 **********************************************************************************/
void antplus_handle(void) {

    uint8_t msgID,channelNo,msgContent0,msgContent1;
    int8_t answer,event = -1;

    // Paket received and enough space in buffer for answer?
#ifdef WITH_USB_POLL
    if(!(OUT1CS & EPBSY)) {
#else
    if(Semaphore_EP12_out) {
        Semaphore_EP12_out = false;
#endif

        STATISTICS_GENERAL(count_out_frames12++);

#ifdef WITH_DEVELOPMENT_LOG_ANT
        {
            static __idata uint16_t antplus_log_ctr = 0;
            uint8_t i,len = OUT1BUF[1] + 4;
            __xdata uint8_t *ptr = xdata+0x2000+antplus_log_ctr;
            if(len <= 8) {
                len = 8;
            } else {
                len = 16;
            }
            for(i=0;i<len;i++) {
                ptr[i] = OUT1BUF[i];
            }
            antplus_log_ctr += len;
            antplus_log_ctr &= 0x7ff;
        }
#endif

        // -----------------------------------------------------------
        // Verify frame structure
        // Note: we do not care about real message size as long as the length field is smaller 
        // than 33 (max paket is 37) and checksum is OK (Dynastream 0x1008 works same)
        // -----------------------------------------------------------

        {
            uint8_t errcode,xor,mlen,val;
            uint8_t i,len = OUT1BC;

            if(OUT1BUF[0] != MESG_TX_SYNC) {
                errcode = 0x00;
                goto antplus_pre_check_error_jmp;
            }
            mlen = OUT1BUF[1];
            if(mlen > ANTPLUS_MAX_LEN_RX) {
                errcode = 0x03;
                goto antplus_pre_check_error_jmp;
            }

            // 1. The dynastream ant dongle uses the data (but not the checksum) from the last 
            // commmand if a new command is shorter then signaled in "mlen". So, for
            // 'perfect' emulation (of too short messages) and to save some code bytes (__pdata), 
            // we copy the message vom USB OUTBUF without checksum to a second buffer (antplus_recv_buf).
            // 2. calc and verify checksum 'en passant' ...
            xor = MESG_TX_SYNC ^ mlen;
            for(i=0;i<=mlen;i++) {
                val = OUT1BUF[2+i];
                xor ^= val;
                if(i < ANTPLUS_MAX_LEN_RX_USED+4-2)
                    antplus_recv_buf[2+i] = val;
            }
            if(OUT1BUF[2+i] != xor) {
                errcode = 0x02;
antplus_pre_check_error_jmp:
#if 1
                // the antplus protocol allows zero padding after the checksum - but (at least) the linux driver cuts the
                // data into two frames. The ANTPLUS-USB-Sticks sends a MESG_SERIAL_ERROR_ID frame back
                // we ignore it for test purposes - it floods the log files
                // TODO using linux ttyUSB devices, the padding zeros are sent
                // in a new frame. Ignore leading "0x00" bytes -> but antstick triggers error, too
                for(i=0;i<len;i++) {
                    if(OUT1BUF[i] != 0)
                        goto not_all_zero_check_passed;
                }
                goto antplus_pre_check_no_error_jmp;
not_all_zero_check_passed:
#endif
                antplus_scratch_buf[0] = MESG_SERIAL_ERROR_ID;
                antplus_scratch_buf[1] = errcode;
                if(len > ANTPLUS_ERR_BOUNCE_MAX_LEN+1)
                    len = ANTPLUS_ERR_BOUNCE_MAX_LEN+1;
                for(i=len+1;i>0;i--) {
                    (antplus_scratch_buf+2-1)[i] = (OUT1BUF-1)[i];
                }
                antplus_prepare(len);
antplus_pre_check_no_error_jmp:                
                goto antplus_outbuf_end;
            }
        }

        msgID            = antplus_recv_buf[2];
        msgContent0      = antplus_recv_buf[4];
        msgContent1      = antplus_recv_buf[5];

        channelNo        = antplus_recv_buf[3] & 0x1f;
        if(channelNo >= ANTPLUS_MAX_CHANNEL)
            goto antplus_response_invalid_message;

        // -----------------------------------------------------------
        // Verify Data
        // -----------------------------------------------------------

        // c=channel CMD
        // z=zero channel CMD
        // n=network CMD
        //      0x40..0x47  0x48..0x4f  0x50..0x57  0x58..0x5f  0x60..0x67  0x68..0x6f  0x70..0x75
        // HOST  0cccccnz,   zczccccc    cc0z0000   0ccz0[ccc]   c00c0ccc    00c000z0    cc000c  


        answer           = RESPONSE_NO_ERROR; // answer with response OK

        {
        uint8_t state2 = antplus_channel_status2[channelNo],i,channelStatus = antplus_channel_status[channelNo];
        switch((uint8_t)msgID) {
            // Config Messages
            case MESG_UNASSIGN_CHANNEL_ID:      // 0x41:
                if(channelStatus != STATUS_ASSIGNED_CHANNEL) {
                    goto antplus_response_channel_wrong_state;
                }
                antplus_channel_status[channelNo] = STATUS_UNASSIGNED_CHANNEL;
                goto antplus_response_answer;

            case MESG_ASSIGN_CHANNEL_ID:        // 0x42:
                // ANT Message Protocol and Usage, Rev 5.1 says:
                // "This Assign Channel command should be issued before any other channel 
                // configuration messages, and before the channel is opened. Assigning a 
                // channel sets all of the other configuration parameters to their defaults."

                // Prio 1 Check: Channel State
                if(channelStatus != STATUS_UNASSIGNED_CHANNEL) {
                    goto antplus_response_channel_wrong_state;
                }
                // Prio 2 Check: Network
                if(msgContent1 >= ANTPLUS_MAX_NETWORK) {
                    // error 'invalid_network_number' 0x29
                    answer = INVALID_NETWORK_NUMBER;
                    goto antplus_response_answer;
                }
                *((uint32_t *) &antplus_channel_ids[channelNo]) = 0x0;
                // we do not care about Reset of "Channel Period", "RF Frequency", "TX Power" ...
                antplus_channel_status[channelNo] = STATUS_ASSIGNED_CHANNEL;
                antplus_channel_status2[channelNo] = (msgContent0 & 0xf0) | (msgContent1<<2);
#ifdef WITH_ANTPLUS_SEARCH_TIMEOUT_EMULATION
                antplus_channel_hp_search_timeout[channelNo] = 10;  // *2.5 seconds
                antplus_channel_lp_search_timeout[channelNo] = 2;   // *5 seconds
#endif
                goto antplus_response_answer;

            case MESG_NETWORK_KEY_ID:          // 0x46
                if(channelNo >= ANTPLUS_MAX_NETWORK)
                    goto antplus_response_invalid_message;
                // here channelNo is networkNo
                antplus_network[channelNo] = msgContent0;
                goto antplus_response_answer;

            case MESG_SYSTEM_RESET_ID:         // 0x4a
                // 0x4a:  // Reset answer
                antplus_lib_state = 0;
                for(i=0;i!=ANTPLUS_MAX_CHANNEL;i++) {
                    antplus_channel_status[i] = STATUS_UNASSIGNED_CHANNEL;
                }
                antplus_channels_opened = 0;
#if ANTPLUS_MAX_NETWORK == 3
                antplus_network[0] = antplus_network[1] = antplus_network[2] = 0;
#else
                for(i=0;i!=ANTPLUS_MAX_NETWORK;i++) {
                    antplus_network[i] = 0;
                }
#endif
                antplus_cont_scanning = false;

                antplus_scratch_buf[0] = MESG_STARTUP_MESG_ID;
                antplus_scratch_buf[1] = 0x20; // or 0x00 for initial reset
                antplus_prepare(1);
                goto antplus_outbuf_end;

            case MESG_OPEN_RX_SCAN_ID:          // 0x5b
                // 1. scan (always opens channel ZERO - provided channelNo does not matter)
                // 2. transfer type does not matter - dynastream 0x1008 switch to STATUS_SEARCHING_CHANNEL 
                //    even as master or master_tx_only
                if(antplus_channels_opened > 0) {
                    // error need to 'close all channels' for RxScan : 0x19
                    answer = CLOSE_ALL_CHANNELS;
                    goto antplus_response_answer;
                }
                // State check and change for channel0 (even if channelNo is non 0 here) but response is with current channelNo
                if(antplus_channel_status[0] != STATUS_ASSIGNED_CHANNEL) {
                    goto antplus_response_channel_wrong_state;
                }
                antplus_cont_scanning = true;
                antplus_channels_opened++;
                antplus_channel_status[0] = STATUS_SEARCHING_CHANNEL; // switch channel-ZERO to "open in searching mode" 
                antplus_channel_timeout[0] = 0; // infinite
                goto antplus_response_answer;

            case MESG_OPEN_CHANNEL_ID: // 0x4b
                // open channel
                if(antplus_cont_scanning || channelStatus != STATUS_ASSIGNED_CHANNEL) {
                    goto antplus_response_channel_wrong_state;
                }
                antplus_channels_opened++;
                antplus_channel_status[channelNo] = STATUS_SEARCHING_CHANNEL;
                antplus_channel_timeout[channelNo] = 0; // TODO infinite search time
                if(state2 & CHANNEL_MASK_MASTER) {
                    antplus_channel_status[channelNo] = STATUS_TRACKING_CHANNEL;
                }
                goto antplus_response_answer;

            case MESG_CLOSE_CHANNEL_ID: // 0x4c
                // close channel DUAL answer (OK and closed)
                if(channelStatus < STATUS_SEARCHING_CHANNEL) {
                    goto antplus_response_channel_wrong_state;
                }
                antplus_channels_opened--;
                antplus_channel_status[channelNo] = STATUS_ASSIGNED_CHANNEL;
                event = EVENT_CHANNEL_CLOSED;
                if(antplus_cont_scanning) {
                    antplus_cont_scanning = false;
                }
                goto antplus_response_answer;

            case MESG_CHANNEL_ID_ID:   // 0x51
                // (set)channelId
                antplus_set_channel_id( ((uint16_t)msgContent1<<8)|(uint16_t)msgContent0, antplus_recv_buf[6], antplus_recv_buf[7], channelNo);
                goto antplus_response_answer; // antplus_MESG_SERIAL_NUM_SET_CHANNEL_ID_ID_end;                
            case MESG_SERIAL_NUM_SET_CHANNEL_ID_ID:   // 0x65
                antplus_set_channel_id(ANTPLUS_DEVICE_ID, msgContent0, msgContent1, channelNo);
                goto antplus_response_answer;

            case MESG_RX_EXT_MESGS_ENABLE_ID:  // 0x66
                 // conf Ext Rx: 66 Filler00 00|!00
                if(msgContent0 == 0)
                    antplus_lib_state &= 0x7f;
                else
                    antplus_lib_state |= 0x80;
                goto antplus_response_answer;

            case MESG_ANTLIB_CONFIG_ID:      // 0x6e
                // Lib (for extended message) 6E Filler00 00|20|40|80, (and 01|21|41|81)
                if( (msgContent0 & ~0xe1) == 0)
                    antplus_lib_state = msgContent0;
                else
                    answer = INVALID_PARAMETER_PROVIDED;
                goto antplus_response_answer;
            case MESG_REQUEST_ID:           // 0x4d
                antplus_scratch_buf[0] = msgContent0;
                antplus_scratch_buf[1] = channelNo;
                switch((uint8_t)msgContent0) {
                    case MESG_CHANNEL_ID_ID: // 0x51 channel ID
                        *((uint32_t *)&antplus_scratch_buf[2]) = *((uint32_t *)&antplus_channel_ids[channelNo]);
                        antplus_prepare(5);
                        goto antplus_outbuf_end;
                    case MESG_CHANNEL_STATUS_ID:
                        // ATTENTION we CORRUPT channelStatus - but that is OK here at the end
                        if(channelStatus != 0)
                            channelStatus |= state2 & ~(0x3);  // we (miss)use lower two bits for tx state
                        // 0x52 channel status
                        antplus_scratch_buf[2] = channelStatus;
                        antplus_prepare(2);
                        goto antplus_outbuf_end;
                }
                if(antplus_request_answer(msgContent0))
                    goto antplus_outbuf_end;
                goto antplus_response_invalid_message;

            // (non burst) data messages: 0x4E, 0x4F
            case MESG_ACKNOWLEDGED_DATA_ID:      // 0x4f: ack message
            case MESG_BROADCAST_DATA_ID:         // 0x4e: broadcast mesg.
                // Dynastream 0x1008 seems to have a bug when you trigger an ACK message
                // in ASSIGNED (not opend) RX-Mode - the state remains in "TX-Pending" state
                // and every try to send an 0x4e or 0x4f message results in a TRANSFER_IN_PROGRESS
                if(antplus_channel_status[channelNo] == STATUS_UNASSIGNED_CHANNEL
                    && (!antplus_cont_scanning || channelNo == 0)) {
                    // In cont scanning, sending on non-zero channel is allowed - channel-state does not matter
                    goto antplus_response_channel_wrong_state;
                }                

                if(state2 & CHANNEL_MASK_TX_ACK) {
                    answer = TRANSFER_IN_PROGRESS;
                    goto antplus_response_answer;
                } 
                if(msgID == MESG_ACKNOWLEDGED_DATA_ID)
                    state2 |= CHANNEL_MASK_TX_ACK;
                state2 |= CHANNEL_MASK_TX_READY;
                antplus_channel_status2[channelNo] = state2;

                answer = -1; //  'trigger' "no answer"

                antplus_copy_page_p2x((__xdata uint8_t *) &antplus_tx_buffer[channelNo], antplus_recv_buf+4);
                goto antplus_response_answer;

            default: {
                __code static uint8_t valid[] = { 
                    MESG_BURST_DATA_ID,                 // 0x50
                    MESG_RADIO_CW_INIT_ID,              // 0x53
                    MESG_ID_LIST_ADD_ID,       	        // 0x59
                    MESG_ID_LIST_CONFIG_ID,    	        // 0x5a
                    MESG_CHANNEL_RADIO_TX_POWER_ID,     // 0x60
                    MESG_SET_LP_SEARCH_TIMEOUT_ID,      // 0x63
                    MESG_RADIO_CONFIG_ALWAYS_ID,        // 0x67
                    0x6a,
                    MESG_AUTO_FREQ_CONFIG_ID,           // 0x70
                    MESG_PROX_SEARCH_CONFIG_ID,         // 0x71
                    MESG_SET_SEARCH_CH_PRIORITY_ID,   	// 0x75
                    MESG_UNLOCK_INTERFACE_ID,           // 0xAD
                };
                if(msgID >= 0x41 && msgID <= 0x51)
                    goto antplus_response_answer;
                for(i=0;i!=sizeof(valid);i++)
                    if(msgID == valid[i])
                        goto antplus_response_answer;
            } break;            
        }
        }

antplus_response_invalid_message:                
        answer = INVALID_MESSAGE;
        goto antplus_response_answer;


antplus_response_channel_wrong_state:
        answer = CHANNEL_IN_WRONG_STATE;

antplus_response_answer:
        antplus_prepare_event(channelNo, answer, msgID);
        antplus_prepare_event(channelNo, event, 1);

antplus_outbuf_end:
        // re-arm OUT1 EP for the next message from the host
        OUT1BC = 0x0;
    }

    if(!(IN1CS & EPBSY)) {
        if(antplus_stage_ptr2send > 0) {
            uint8_t i;

            for(i=0;i<antplus_stage_ptr2send;i++) {
                IN1BUF[i] = antplus_stage_buf[i];
            }            
            IN1BC = antplus_stage_ptr2send;
            antplus_stage_ptr2send = 0;
            for(;i<antplus_stage_ptr;i++,antplus_stage_ptr2send++) {
                antplus_stage_buf[antplus_stage_ptr2send] = antplus_stage_buf[i];
            }
            antplus_stage_ptr = antplus_stage_ptr2send;
            STATISTICS_GENERAL(count_in_frames12++);
#if 0
        } else {
            // I had problems under Linux with an ASM1042 USB bridge/hub: One of 
            // two IN data frames were dropped, if there was a "read" from the host
            // without a frame "in the pipeline". 
            // Maybe this was a LibUSB-0.1 problem. If you need to use libUSB-0.1 
            // implementation or if you notice communication deadlocks try to 
            // uncomment this region.
            IN1BC = 0;
#endif
        }
    }
}

#endif

