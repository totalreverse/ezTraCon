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

#ifndef __ANTPLUS_H

#define __ANTPLUS_H

extern void antplus_init(void);
extern void antplus_handle(void);
extern void antplus_8Hz_hook(void);

#define ANTPLUS_MAX_CHANNEL (8)
#define ANTPLUS_MAX_NETWORK (3)

// Limits of Dynastream ANTUSB2 stick:
// Max. size of frames we accept from the host
#define ANTPLUS_MAX_LEN_RX      (0x21)
#define ANTPLUS_MAX_LEN_RX_USED (8)
// Max. size of bounced data after error
#define ANTPLUS_ERR_BOUNCE_MAX_LEN (0x1f)

// Tacx Motorbrakes T2900=Flux, T2800=Neo, T2780=Bushido (smart), T2080=Genius Smart
// Tacx Eddycurrent (magentic) T2180=Vortex, T2240=Flow   (T2160, T2170 auch Vortex??)
// Tacx manual magnetic T2400=Satori
// Tacx Steering T2420 (Black Track), T2430 (NEO Track)
// Tacx Control T2022
#define ANTPLUS_MANUFACTURER_ID_TACX               ((uint16_t)0x0059)
#define ANTPLUS_MANUFACTURER_ID_QUARQ              ((uint16_t)0x0007)
#define ANTPLUS_PRODUCT_ID_TACX_T2800_NEO          ((uint16_t)0x0af0)
#define ANTPLUS_PRODUCT_ID_TACX_T2080_GENIUS       ((uint16_t)0x0820)
#define ANTPLUS_PRODUCT_ID_QUARQ_DZERO             ((uint16_t)0x2507)

// ---------------------------------------------------------------------
// Old Tacx ANT devices do not conform to Ant+ or Ant+ FE-C.
// They use channel #60, the default network-key and a different format
// Default Channel Period is 4096 (0x1000) -- Transfer Type = 0x01
// ---------------------------------------------------------------------
#define ANT_TACX_DEVICE_TYPE_BUSHIDO_BRAKE     (0x51)
#define ANT_TACX_DEVICE_TYPE_BUSHIDO_DISPLAY   (0x52)
#define ANT_TACX_DEVICE_TYPE_GENIUS_BRAKE      (0x53)

// Steering (BlackTrack 0x54): Channel-Period 2048 (0x0800)
// 4E/4F [channel] 00 [msb lsb] [ff] [00 00 00 00]
// msb,lsb = steering value between [-512...512]
#define ANT_TACX_DEVICE_TYPE_BLACKTRACK        (0x54)

// Remote Control (the blue round device with 4+1 buttons)
// Master broadcast messages:
// [channel] 00 00 [0b_0000_URDL] 00 00 00 00
// [U]p, [R]ight, [D]own, [L]eft
#define ANT_TACX_DEVICE_TYPE_CURSOR_CONTROL    (0x55)

// Flux (0x56) Channel-Period = 8192 (0x2000)
#define ANT_TACX_DEVICE_TYPE_FLUX              (0x56)


// ---------------------------------------------------------------------
// ANT+ sport equipment send always with ANT+ network key on channel 57
// ---------------------------------------------------------------------

#define MAX_POWER_T1941                 (1000)

#define ANTPLUS_DEVICE_ID_MSN           ((uint8_t)0x0)
#define ANTPLUS_DEVICE_ID               ((uint16_t)0xC001)

#define ANTPLUS_FEC_DEVICE_TYPE         ((uint8_t)0x11)
#define ANTPLUS_TRANS_TYPE_05           ((uint8_t)0x05)
#define ANTPLUS_TRANS_TYPE_01           ((uint8_t)0x01)
// #define ANTPLUS_CHANNEL_ID              (((uint32_t)ANTPLUS_DEVICE_ID)|((uint32_t)(ANTPLUS_DEVICE_ID_MSN&0xf)<<28)|((uint32_t)ANTPLUS_TRANS_TYPE<<24)|((uint32_t)ANTPLUS_DEVICE_TYPE<<16))

#define ANTPLUS_NETWORK_KEY_MSL         ((uint32_t)0x45c372bd) // [B9 A5 21 FB]  BD 72 C3 45
#define ANTPLUS_NETWORK_KEY_LSL         ((uint32_t)0xfb21a5b9) //  B9 A5 21 FB  [BD 72 C3 45]
#define ANTPLUS_NETWORK_KEY_LSB         ((uint8_t)0xb9)

// Bike-power
#define ANTPLUS_PWR_DEVICE_TYPE			(0x0b)
#define ANTPLUS_PWR_PERIOD  			(8182)
// HR monitor
#define ANTPLUS_HR_DEVICE_TYPE			(0x78)
#define ANTPLUS_HR_PERIOD	  		    (8070)
// Speed und Cad 
#define ANTPLUS_SUC_DEVICE_TYPE			(0x79)
#define ANTPLUS_SUC_PERIOD	  		    (8086)
// Cad 
#define ANTPLUS_CAD_DEVICE_TYPE			(0x7a)
#define ANTPLUS_CAD_PERIOD   			(8102)
// Speed
#define ANTPLUS_SPD_DEVICE_TYPE			(0x7b)
#define ANTPLUS_SPD_PERIOD  			(8118)

// ---------------------------------------------------------------------
// Tacx 'legacy' ANT devices send on several channels with default network key
// ---------------------------------------------------------------------

// ANT_NO_NETWORK_KEY_MSL 0x00000000
// ANT_NO_NETWORK_KEY_LSL 0x00000000
// ANT_NO_NETWORK_KEY_LSB 0x00

// Tacx Vortex Brake T2171 (non smart)
#define ANT_VORTEX_BRAKE_CHANNEL		(66)       // 0x42
#define ANT_VORTEX_BRAKE_PERIOD	        (0x2000)   // =4hZ
#define ANT_VORTEX_BRAKE_DEVICE_TYPE    (0x3d)     // no pairing bit
// Tacx T2172 light-blue, T2162 yellow-green
#define ANT_VORTEX_DISPLAY_CHANNEL		(78)       // 0x4e
#define ANT_VORTEX_DISPLAY_PERIOD	    (0x0f00)   // =~8.53 Hz
#define ANT_VORTEX_DISPLAY_DEVICE_TYPE  (0x3e)     // no pairing bit
// Tacx Bushido Brake T1981 
#define ANT_BUSHIDO_BRAKE_CHANNEL		(60)       // 0x4e
#define ANT_BUSHIDO_BRAKE_PERIOD	    (0x1000)   // =8 Hz
#define ANT_BUSHIDO_BRAKE_DEVICE_TYPE   (0x51)     // no pairing bit
// Tacx Bushido Display T1982
#define ANT_BUSHIDO_DISPLAY_CHANNEL		(60)       // 0x3c
#define ANT_BUSHIDO_DISPLAY_PERIOD	    (0x1000)   // =8 Hz
#define ANT_BUSHIDO_DISPLAY_DEVICE_TYPE (0x52)     // no pairing bit
// Tacx Genius Brake T2021 (non smart)
#define ANT_GENIUS_BRAKE_CHANNEL	 	(60)       // 0x3c
#define ANT_GENIUS_BRAKE_PERIOD	        (0x1000)   // =8 Hz
#define ANT_GENIUS_BRAKE_DEVICE_TYPE    (0x53)     // no pairing bit
// Tacx Blacktrack Steering  T2420 (NEO track T2430.)
#define ANT_BLACKTRACK_CHANNEL        	(60)       // 0x3c
#define ANT_BLACKTRACK_PERIOD	        (0x0800)   // =16 Hz
#define ANT_BLACKTRACK_DEVICE_TYPE      (0x54)     // no pairing bit
// Tacx Genius "Cursor Control"  T2022
#define ANT_CURSOR_CTRL_CHANNEL		    (60)       // 0x3c
#define ANT_CURSOR_CTRL_PERIOD	        (0x1000)   // =8 Hz
#define ANT_CURSOR_CTRL_DEVICE_TYPE     (0x55)     // no pairing bit
// Flux Brake (non smart - never release?)
#define ANT_FLUX_BRAKE_CHANNEL		    (66)       // 0x42
#define ANT_FLUX_BRAKE_PERIOD	        (0x2000)   // =4 Hz
#define ANT_FLUX_BRAKE_DEVICE_TYPE      (0x56)     // no pairing bit
// Flow Brake (non smart - never release?)
#define ANT_FLOW_BRAKE_CHANNEL		    (66)       // 0x42
#define ANT_FLOW_BRAKE_PERIOD	        (0x2000)   // =4 Hz
#define ANT_FLOW_BRAKE_DEVICE_TYPE      (0x5f)     // no pairing bit

typedef struct heart_rate_s {
	uint8_t  page_number;						// 0x04 - bit 7 is toggle bit
	uint8_t  manufacturer_specific;				// note: receiver shall not interpret this field
	uint16_t previous_heart_beat_event_time;    // time of the previous valid heart beat event. unit 1/1024 sec
	uint16_t heart_beat_event_time;				// time of the last valid heart beat event. unit 1/1024 sec
	uint8_t  heart_beat_count;					// increments with each heart beat event.
	uint8_t  computed_heart_rate;				// bpm , 1-255, 0x00 invalid value
} heart_rate_t;

typedef struct ant_main_page16_s {			
	uint8_t  page_number;					// Page 16 / 0x10 - General FE Data Page
	uint8_t  equipment_type_bit_field;		// see Table 6-8. Equipment Type Bit Field Description
	uint8_t  elapsed_time;					// Accumulated value of the elapsed time since start of workout, unit: 0.25, 
	uint8_t  distance_traveled;				// Accumulated value of the distance traveled since start of workout: unit: 1 meter
	uint16_t speed;							// Instantaneous speed: units 0.001 m/s, set to 0xFFFF if invalid
	uint8_t  heart_rate;					// Instantaneous heart rate, 0xFF indicates invalid, unit: 1 bpm
// TODO enforce right bitorder (DIY)
	uint8_t  capabilities_it_field:4;		// Table 6-9. Page 16 Capabilities Bit Field:
	uint8_t  fe_state_bit_field:4;			// see FE State Bit Field (ANT_FE_STATE_*)
} ant_main_page16_t, general_fe_t;

typedef struct ant_trainer_page25_s {
	uint8_t  page_number;					// Page 25 / 0x19
	uint8_t  update_event_count;			// Event counter increments with each information update
	uint8_t  instantaneous_cadence; 		// Crank cadence. 0xFF indicates invalid
	uint16_t accumulated_power;				// Accumulated power: units 1 watt
// TODO enforce right bitorder (DIY)
	uint16_t instantaneous_power:12;		// instantaneous power: units 1 watt. 0xFFF indicates BOTH instantaneous and accumulated power are invalid
	uint8_t  trainer_status_bit_field:4;	// see Table 6-28 : Trainer Status Bit Field
	uint8_t  flags_bit_field:4;				// see Table 6-29. Flags Bit Field Description
	uint8_t  fe_state_bit_field:4;			// see FE State Bit Field (ANT_FE_STATE_*)
} ant_trainer_page25_t, specific_trainer_t;

typedef struct ant_data_page54_s {
	uint8_t  page_number;					// Page 54 / 0x36
	uint32_t reserved;						// reserved, always set to 0xFFFFFFFF
	uint16_t maximum_resistance; 			// The maximum applicable resistance of the trainer 0 – 65534N. Unit 1 Newton. Invalid: 0xFFFF
	uint8_t  capabilities_bit_field;		// see Table 6-47. Capabilities Bit Field
} ant_data_page54_t, fe_capabilities_t;

typedef struct ant_data_page55_s {
	uint8_t  page_number;					// Page 55 / 0x37
	uint16_t user_weight;					// The user weight entered on the display. units 0.01kg, range 0-655.34kg, Invalid: 0xFFFF
	uint8_t  reserved;						// 0xFF (reserved for future use)
// TODO enforce right bitorder (DIY)	
	uint8_t  bicycle_wheel_diameter_offset:4; // Offset applied to Bicycle Wheel Diameter. units 1mm, range 0-9, Invalid / No Offset: 0xF
	uint16_t  bicycle_weight:12;				// The bicycle weight entered on the display. units: 0.05kg, range 0-50kg. Invalid: 0xFFF
	uint8_t  bicycle_wheel_diameter;		// The bicycle wheel diameter entered on the display. units 0.01m, range 0-2.54m
	uint8_t  gear_ratio;					// Front:Back Gear Ratio entered on the Gear Ratio display. Units: 0.03, range: 0.03 – 7.65, Invalid: 0x00
} ant_data_page55_t, user_configuration_t;

typedef struct ant_common_page71_s {
	uint8_t page_number;					// Page 71 / 0x47
	uint8_t last_received_command_id; 		// Indicates data page number of the last SUPPORTED control page received (i.e. page 48-51)
	uint8_t sequence_number;				// 0-254: Sequence number used by Slave in last received command request
	uint8_t command_status;					// Command status:
											// 0 = Pass: command received and processed successfully
											// 1 = Fail: command received and processed unsuccessfully
											// 2 = Not Supported (FE shall not use this value)
											// 3 = Rejected – e.g. due to invalid/unregistered remote 
											// 4 = Pending: command received and not yet processed
											// 5-254 = Reserved – Do not send or interpret
											// 255 = Uninitialized (Never received a command)
	union {
		uint32_t	data;
		struct command48_basic_resistance {
			uint8_t	reserved[3];			// Set to 0xFF,0xFF,0xFF
			uint8_t total_resistance;		
		} basic_resistance;

		struct command49_target_power {
			uint8_t	reserved[2];			// Set to 0xFF,0xFF
			uint16_t target_power;	
		} target_power;

		struct command50_wind_resistance {
			uint8_t	reserved;				// Set to 0xFF
			uint8_t wind_resistance_coefficient;
			uint8_t wind_speed;
			uint8_t drafting_factor;
		} wind_resistance;

		struct command51_track_resistance {
			uint8_t	reserved;				// Set to 0xFF
			uint16_t grade; 				// slope
			uint8_t coefficient_of_rolling;
		} track_resistance;

	} data;
} ant_common_page71_t, command_status_t;


/* Manufacturer’s Information: Page 80 / 0x50 */
typedef struct ant_common_page80_s {
	uint8_t	   page_number;		// Page 80 / 0x50
	uint16_t   reserved;		// always set to 0xFFFF
	uint8_t    hw_revision;		// Hardware revision 
	uint16_t   manufacturer_id;	// set 0xFF for development
	uint16_t   model_number;	// by manufacturer
} ant_common_page80_t, manufacturers_identification_t;

/* Product Information: Page 81 / 0x51  */
typedef struct ant_common_page81_s {
	uint8_t page_number;		// Page 81 / 0x51 
	uint8_t reserved;			// always set to 0xFF
	uint8_t sw_revision_minor;	// set to 0xFF if unused
	uint8_t sw_revision_major;	// set to 0xFF if unused
	uint32_t serial_number;		// lower 32 bits, set to 0xFFFFFFFFF if unused
} ant_common_page81_t, product_information_t;

/* Battery Status: Page 82 / 0x52 */
typedef struct ant_common_page82_s {
	uint8_t page_number;		// Page 82 / 0x52 
	uint8_t reserved;			// always set to 0xFF
	uint8_t bat_identifier;		// Bits 0-3 Number of Batteries, Bits 4-7: Identifier, Set to 0xFF if not used.
	uint8_t operating_time[3];	// 24 bit value : units: 2 or 16 seconds 
	uint8_t bat_voltage;		// Value = 0 – 255 (0x00 – 0xFF). 0=0%, 255 = 100%
	uint8_t descriptor;			// Descriptive Bit Field
} ant_common_page82_t;

struct page_answers_static {
    heart_rate_t                    heart_rate;
    manufacturers_identification_t  manufacturers_information;
    product_information_t           product_information;
    fe_capabilities_t               fe_capabilities;
    command_status_t                command_status;
    general_fe_t                    general_fe;
    specific_trainer_t              specific_trainer;
    uint8_t                         end;    
};

#ifdef WITH_LOAD_DATA_TO_UPPER_MEM
extern __code __at 0x1b40 struct page_answers_static antplus_page_answers_static;
#else
extern __code struct page_answers_static antplus_page_answers_static;
#endif

/* Pages for Bicycle Power Sensor Types: 
      See Ant+ Device Profile Bicycle Power Page 52 ff 
Page#| Descritpion                              | Direction 
0x01 | Calibration Messages All   			    | S <=> D    
0x02 | Get/Set Parameters 						| S <=> D   
0x03 | Measurement Output 						| S  => D
0x10 | Power Only 								| S  => D
0x11 | Torque At Wheel 							| S  => D
0x12 | Torque At Crank 							| S  => D
0x13 | Torque Effectiveness & Pedal Smoothness	| S  => D
0x20 | Crank Torque-Frequency Message 			| S  => D
*/

#define MESG_TX_SYNC                            ((uint8_t)0xA4)
#define MESG_RX_SYNC                            ((uint8_t)0xA5)

#define ANT_EXT_MESG_DEVICE_ID_FIELD_SIZE       ((uint8_t)4)
#define ANT_EXT_MESG_RSSI_FIELD_SIZE            ((uint8_t)4)
#define ANT_EXT_MESG_TIME_STAMP_FIELD_SIZE      ((uint8_t)2)

// = 20
#define ANT_PREPAREBUF_MAX_SIZE      			((uint8_t)9+1+ANT_EXT_MESG_DEVICE_ID_FIELD_SIZE+ANT_EXT_MESG_RSSI_FIELD_SIZE+ANT_EXT_MESG_TIME_STAMP_FIELD_SIZE)

#define ANT_CLOCK_FREQUENCY                     ((uint32_t)32768)

#define MESG_VERSION_ID                         ((uint8_t)0x3E)
#define MESG_RESPONSE_EVENT_ID                  ((uint8_t)0x40)

#define MESG_UNASSIGN_CHANNEL_ID       			((uint8_t)0x41)
#define MESG_ASSIGN_CHANNEL_ID                  ((uint8_t)0x42)
#define MESG_CHANNEL_MESG_PERIOD_ID             ((uint8_t)0x43)
#define MESG_CHANNEL_SEARCH_TIMEOUT_ID          ((uint8_t)0x44)
#define MESG_CHANNEL_RADIO_FREQ_ID              ((uint8_t)0x45)
#define MESG_NETWORK_KEY_ID                     ((uint8_t)0x46)
#define MESG_RADIO_TX_POWER_ID                  ((uint8_t)0x47)
#define MESG_RADIO_CW_MODE_ID   				((uint8_t)0x48)
#define MESG_SEARCH_WAVEFORM_ID   				((uint8_t)0x49)
#define MESG_RADIO_CW_INIT_ID   				((uint8_t)0x53)

#define MESG_SYSTEM_RESET_ID                    ((uint8_t)0x4A)
#define MESG_OPEN_CHANNEL_ID                    ((uint8_t)0x4B)
#define MESG_CLOSE_CHANNEL_ID                   ((uint8_t)0x4C)
#define MESG_REQUEST_ID                         ((uint8_t)0x4D)
#define MESG_BROADCAST_DATA_ID                  ((uint8_t)0x4E)
#define MESG_ACKNOWLEDGED_DATA_ID               ((uint8_t)0x4F)
#define MESG_BURST_DATA_ID                      ((uint8_t)0x50)
#define MESG_CHANNEL_ID_ID                      ((uint8_t)0x51)
#define MESG_CHANNEL_STATUS_ID                  ((uint8_t)0x52)
#define MESG_RADIO_CW_INIT_ID   				((uint8_t)0x53)
#define MESG_CAPABILITIES_ID                    ((uint8_t)0x54)
#define MESG_STACKLIMIT_ID                      ((uint8_t)0x55)

#define MESG_ID_LIST_ADD_ID   					((uint8_t)0x59)
#define MESG_ID_LIST_CONFIG_ID   				((uint8_t)0x5A)

#define MESG_OPEN_RX_SCAN_ID                    ((uint8_t)0x5B)
#define MESG_CHANNEL_RADIO_TX_POWER_ID   		((uint8_t)0x60)
#define MESG_GET_SERIAL_NUM_ID                  ((uint8_t)0x61)
#define	MESG_RX_EXT_MESGS_ENABLE_ID             ((uint8_t)0x66)
#define MESG_RADIO_CONFIG_ALWAYS_ID				((uint8_t)0x67)

#define MESG_ANTLIB_CONFIG_ID                   ((uint8_t)0x6E)
#define MESG_STARTUP_MESG_ID                    ((uint8_t)0x6F)
#define MESG_SET_LP_SEARCH_TIMEOUT_ID           ((uint8_t)0x63)
#define MESG_SERIAL_NUM_SET_CHANNEL_ID_ID       ((uint8_t)0x65)

#define MESG_AUTO_FREQ_CONFIG_ID   				((uint8_t)0x70)
#define MESG_PROX_SEARCH_CONFIG_ID   			((uint8_t)0x71)
#define MESG_SET_SEARCH_CH_PRIORITY_ID   		((uint8_t)0x75)

#define MESG_FLASH_PROTECTION_CHECK_ID          ((uint8_t)0xA7)
#define MESG_UNLOCK_INTERFACE_ID				((uint8_t)0xAD)
#define MESG_SERIAL_ERROR_ID                    ((uint8_t)0xAE)

#define MESG_GET_GRMN_ESN_ID                    ((uint8_t)0xC6)
#define MESG_SET_USB_INFO_ID                    ((uint8_t)0xC7)

// Response event
#define RESPONSE_NO_ERROR                       ((uint8_t)0x00)
#define EVENT_RX_SEARCH_TIMEOUT   				((uint8_t)0x01)
#define EVENT_RX_FAIL   						((uint8_t)0x02)
#define EVENT_TX   								((uint8_t)0x03)
#define EVENT_TRANSFER_RX_FAILED   				((uint8_t)0x04)
#define EVENT_TRANSFER_TX_COMPLETED             ((uint8_t)0x05)
#define EVENT_TRANSFER_TX_FAILED   				((uint8_t)0x06)
#define EVENT_CHANNEL_CLOSED                    ((uint8_t)0x07)
#define EVENT_RX_FAIL_GO_TO_SEARCH   			((uint8_t)0x08)
#define EVENT_CHANNEL_COLLISION   				((uint8_t)0x09)
#define EVENT_TRANSFER_TX_START   				((uint8_t)0x0A)
#define EVENT_TRANSFER_NEXT_DATA_BLOCK   		((uint8_t)0x11)
#define CHANNEL_IN_WRONG_STATE              	((uint8_t)0x15)
#define CHANNEL_NOT_OPENED   					((uint8_t)0x16)
#define CHANNEL_ID_NOT_SET   					((uint8_t)0x18)
#define CLOSE_ALL_CHANNELS                      ((uint8_t)0x19)
#define TRANSFER_IN_PROGRESS   					((uint8_t)0x1F)
#define INVALID_NETWORK_NUMBER                  ((uint8_t)0x29)
#define INVALID_MESSAGE                         ((uint8_t)0x28)
#define INVALID_LIST_ID							((uint8_t)0x30)
#define INVALID_PARAMETER_PROVIDED              ((uint8_t)0x33)

#define STATUS_CHANNEL_STATE_MASK               ((uint8_t)0x03)
#define STATUS_UNASSIGNED_CHANNEL               ((uint8_t)0x00)
#define STATUS_ASSIGNED_CHANNEL                 ((uint8_t)0x01)
#define STATUS_SEARCHING_CHANNEL                ((uint8_t)0x02)
#define STATUS_TRACKING_CHANNEL                 ((uint8_t)0x03)

#define CHANNEL_TYPE_SLAVE          			((uint8_t)0x00)  
#define CHANNEL_TYPE_MASTER          			((uint8_t)0x10) 
#define CHANNEL_TYPE_SHARED_SLAVE    			((uint8_t)0x20)
#define CHANNEL_TYPE_SHARED_MASTER   			((uint8_t)0x30)
#define CHANNEL_TYPE_SLAVE_RX_ONLY   			((uint8_t)0x40)
#define CHANNEL_TYPE_MASTER_TX_ONLY  			((uint8_t)0x50)
#define CHANNEL_MASK_MASTER          			((uint8_t)0x10) 
#define CHANNEL_MASK_SHARED          			((uint8_t)0x20) 
#define CHANNEL_MASK_ONEWAY_ONLY       			((uint8_t)0x40) 
// OUR Special defines
#define CHANNEL_MASK_NETWORK          			((uint8_t)0x0c) 
#define CHANNEL_MASK_TX_READY          			((uint8_t)0x01) 
#define CHANNEL_MASK_TX_ACK          			((uint8_t)0x02) 

// ANT+ Device Profile Fitness Equipment (thisisant.com D00001231 Rev 4.1 page 35ff)

// Calibration Pages 1,2
#define ANT_FE_CALIBRATION_REQUEST_RESPONSE_PAGE  	0x01
#define ANT_FE_CALIBRATION_IN_PROGRESS_PAGE 		0x02

// General Main Data Pages 16-18 (0x10-0x12)
#define ANT_FE_GENERAL_FE_DATA_PAGE      			0x10
#define ANT_FE_GENERAL_SETTINGS_PAGE     			0x11
#define ANT_FE_GENERAL_FE_METABOLIC_DATA_PAGE     	0x12

// FE Specific Main Data Pages 19-26 (0x13-0x1a)
// pages 27 to 47 are reserved for future data page definitions
#define ANT_FE_SPECIFIC_TREADMILL_DATA_PAGE     	0x13
#define ANT_FE_SPECIFIC_ELLIPTICAL_DATA_PAGE     	0x14
#define ANT_FE_SPECIFIC_STATIONARY_BIKE_DATA_PAGE   0x15
#define ANT_FE_SPECIFIC_ROWER_DATA_PAGE   			0x16
#define ANT_FE_SPECIFIC_CLIMBER_DATA_PAGE   		0x17
#define ANT_FE_SPECIFIC_NORDIC_SKIER_DATA_PAGE   	0x18
#define ANT_FE_SPECIFIC_TRAINER_DATA_PAGE     		0x19
#define ANT_FE_SPECIFIC_TRAINER_TORQUE_DATA_PAGE	0x1A

// Control Pages 48-51  (0x30-0x33)
#define ANT_FE_BASIC_RESISTANCE_DATA_PAGE			0x30
#define ANT_FE_TARGET_POWER_DATA_PAGE				0x31
#define ANT_FE_WIND_RESISTANCE_DATA_PAGE			0x32
#define ANT_FE_TRACK_RESISTANCE_DATA_PAGE			0x33

// On Demand Data Pages (0x36, 0x37)
// The request page (common page 70=0x46) may be used to request capabilities
#define ANT_FE_CAPABILITIES_DATA_PAGE 				0x36
#define ANT_FE_USER_CONFIGURATION_DATA_PAGE			0x37

#define ANT_FE_REQUEST_DATA_PAGE         			0x46
#define ANT_FE_COMMAND_STATUS_DATA_PAGE       		0x47

// Background Data Pages (common pages 80 and 81)
#define ANT_FE_MANUFACTURERS_ID_DATA_PAGE           0x50
#define ANT_FE_PRODUCT_INFORMATION_DATA_PAGE        0x51

#define ANT_FE_SUBFIELD_DATA_PAGE                   0x54


// ANT_FE_GENERAL_FE_DATA_PAGE (Page 0x10 / 16) Attribute Definitions 
// Byte 1 - Bits 0-4 (Bits 5-7 are reserved and should not be interpreted.)
// Table 6-8. Equipment Type Bit Field Description
#define ANT_FE_EQUIPMENT_TYPE_GENERAL			10
#define ANT_FE_EQUIPMENT_TYPE_TREADMILL			19
#define ANT_FE_EQUIPMENT_TYPE_ELLIPTICAL		20
#define ANT_FE_EQUIPMENT_TYPE_STATIONARY_BIKE 	21
#define ANT_FE_EQUIPMENT_TYPE_ROWER				22
#define ANT_FE_EQUIPMENT_TYPE_CLIMBER			23
#define ANT_FE_EQUIPMENT_TYPE_NORDIC_SKIER		24
#define ANT_FE_EQUIPMENT_TYPE_TRAINER			25

// Table 6-47. Capabilities Bit Field 
// Data Page 54 (0x36) – FE Capabilities  --- Attribute Definitions 
// Byte 7 Capabilities Bit Field
#define ANT_FE_CAPABILITIES_SUPPORTS_BASIC_RESISTANCE_MASK	0x01
#define ANT_FE_CAPABILITIES_SUPPORTS_TARGET_POWER_MASK		0x02
#define ANT_FE_CAPABILITIES_SUPPORTS_SIMULATION_MASK		0x04

// Table 6-9. Page 16 Capabilities Bit Field: HR-Bits 0/1, Distance-Bit 2, VSpeed-Bit 3
#define ANT_FE_CAPABILITIES_HR_DATA_SOURCE_HAND_CONTACT 	0x3
#define ANT_FE_CAPABILITIES_HR_DATA_SOURCE_EM			 	0x2
#define ANT_FE_CAPABILITIES_HR_DATA_SOURCE_ANTPLUS		 	0x1
#define ANT_FE_CAPABILITIES_HR_DATA_SOURCE_UNKNOWN		 	0x0
#define ANT_FE_CAPABILITIES_DISTANCE_TRAVELED_ENABLED	 	0x4
#define ANT_FE_CAPABILITIES_VIRTUAL_SPEED_FLAG			 	0x8

// Several data pages use "FE State Bit Field"
#define ANT_FE_STATE_ASLEEP		1
#define ANT_FE_STATE_READY		2
#define ANT_FE_STATE_IN_USE		3
#define ANT_FE_STATE_FINISHED	4	

// Table 6-28 : Trainer Status Bit Field : Bit 3 reserved for future use - set to "0"
#define ANT_FE_STATUS_BICYCLE_POWER_CALIBRATION_REQUIRED_MASK	(1)
#define ANT_FE_STATUS_RESISTANCE_CALIBRATION_REQUIRED_MASK		(2)
#define ANT_FE_STATUS_USER_CONFIGURATION_REQUIRED_MASK			(4)

// Table 6-29 : Target Power Limits Flags (in Bit0 & 1) - Bit  2,3 Reserved for future use. Set to 0.
#define ANT_FE_POWER_LIMIT_OPERATING_AT_TARGET_POWER	0 
#define ANT_FE_POWER_LIMIT_CYCLINGSPEED_TOO_LOW			1
#define ANT_FE_POWER_LIMIT_CYCLINGSPEED_TOO_HIGH		2
#define ANT_FE_POWER_LIMIT_REACHED_UNDETERMINED			3


#endif
