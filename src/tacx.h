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

#ifndef __TACX_H

#define __TACX_H

#define PORTA_MASK_LED_RED     (1<<5)
#define PORTA_MASK_LED_GREEN   (1<<4)
#define PORTA_MASK_LEDS        (PORTA_MASK_LED_RED|PORTA_MASK_LED_GREEN)

#define GREEN_ON()             (OUTA |= PORTA_MASK_LED_GREEN)
#define RED_ON()               (OUTA |= PORTA_MASK_LED_RED)
#define GREEN_OFF()            (OUTA &= ~PORTA_MASK_LED_GREEN)
#define RED_OFF()              (OUTA &= ~PORTA_MASK_LED_RED)

#define PORTB_MASK_GAMEPORT_AXIS0   (1<<0)
#define PORTB_MASK_GAMEPORT_AXIS1   (1<<1)
#define PORTB_MASK_GAMEPORT_AXIS2   (1<<2)
#define PORTB_MASK_GAMEPORT_AXIS3   (1<<3)
#define PORTB_MASK_GAMEPORT_AXES    (PORTB_MASK_GAMEPORT_AXIS0|PORTB_MASK_GAMEPORT_AXIS1|PORTB_MASK_GAMEPORT_AXIS2|PORTB_MASK_GAMEPORT_AXIS3)
// Buttons are 'active low'
#define PORTB_MASK_BUTTON_ENTER     (1<<4)
#define PORTB_MASK_BUTTON_DOWN      (1<<5)
#define PORTB_MASK_BUTTON_UP        (1<<6)
#define PORTB_MASK_BUTTON_CANCEL    (1<<7)
#define PORTB_MASK_BUTTONS          (PORTB_MASK_BUTTON_ENTER|PORTB_MASK_BUTTON_DOWN|PORTB_MASK_BUTTON_UP|PORTB_MASK_BUTTON_CANCEL)

#define PORTC_BIT_RX                (0)
#define PORTC_BIT_TX                (1)
#define PORTC_BIT_WHEEL             (2)
// PORTC_BIT_WHEEL = PORTC_BIT_RX_COMMON
#define PORTC_BIT_RX_COMMON         (2)
#define PORTC_BIT_POWERLINE         (3)
#define PORTC_BIT_CAD               (4)
#define PORTC_BIT_HR                (5)
#define PORTC_BIT_UNKNOWN6          (6)
#define PORTC_BIT_MAGNET            (7)
#define PORTC_BIT_GREEN_UNIT_OUT    (PORTC_BIT_MAGNET)
#define PORTC_BIT_BLUE_UNIT_OUT     (PORTC_BIT_TX)

#define PORTC_MASK_RX                (1<<PORTC_BIT_RX)
#define PORTC_MASK_TX                (1<<PORTC_BIT_TX)
#define PORTC_MASK_WHEEL             (1<<PORTC_BIT_WHEEL)
#define PORTC_MASK_RX_COMMON         (1<<PORTC_BIT_RX_COMMON)
#define PORTC_MASK_POWERLINE         (1<<PORTC_BIT_POWERLINE)
#define PORTC_MASK_CAD               (1<<PORTC_BIT_CAD)
#define PORTC_MASK_HR                (1<<PORTC_BIT_HR)
#define PORTC_MASK_UNKNOWN6          (1<<PORTC_BIT_UNKNOWN6)
#define PORTC_MASK_MAGNET            (1<<PORTC_BIT_MAGNET)
#define PORTC_MASK_GREEN_UNIT_OUT    (1<<PORTC_BIT_GREEN_UNIT_OUT)
#define PORTC_MASK_BLUE_UNIT_OUT     (1<<PORTC_BIT_BLUE_UNIT_OUT)

#define T1941_START_OF_FRAME  (0x01)
#define T1941_END_OF_FRAME    (0x17)

// status byte at position '18' in answer frame from brake (18+24 = Byte 42 in USB-answer-frame to host)
#define T1941_EVENTS_PEDALSENSOR_MASK   (0x01)
#define T1941_EVENTS_BRAKESTOP_MASK     (0x04)

// Head-Unit <==> Host communication frame sizes
#define TACX_FRAME_SIZE_T1942       (48)
#define TACX_FRAME_SIZE_T1932_T1904 (64)
#define TACX_FRAME_SIZE_LONG        TACX_FRAME_SIZE_T1932_T1904
#define TACX_FRAME_SIZE_SHORT       (24)

struct frameHeadUnit {
    uint16_t    deviceSerial;       //  0...1   hex-serial of YOUR head-unit (see sticker on backside)
    uint16_t    fixed1;             //  2...3   0x0000 (T1904 has 0x0005)
    uint16_t    fixed2;             //  4...5   0x0108 (little endian) - other firmwares sometimes 0x0104, (T1904: 0x0001)
    uint16_t    fixed3;             //  6...7   0x0000
    uint8_t     year_production;    //  8       production year of YOUR head unit  (see sticker on backside: first value - maybe after product#)
    uint8_t     unused0;            //  9       random artefact from uncleared IN2BUF?
    uint8_t     unused1;            // 10       random artefact from uncleared IN2BUF?
    uint8_t     unused2;            // 11       random artefact from uncleared IN2BUF?
    uint8_t     heartRate;          // 12       in BPM 
    uint8_t     buttons;            // 13
    //uint8_t     rxErrorCode;      // 14       0xfb= RX-Checksum Error, 0xfe=RX-bufferoverrun [in Frame from Motorbrake]
    uint8_t     unused3:1;          // 14.0     zero
    uint8_t     heartDetect:1;      // 14.1     bit is 'in sync' with RED LED
    uint8_t     unused4:6;          // 14.2..14.6 zero
    uint8_t     rxErrorCount;       // 15       counts motorbrake serial-communication receive-errors
    uint16_t    axis0;              // 16..17   typcially 'floating' - we set it to 0x07d0
    uint16_t    axis1;              // 18..19   steering (uncalibrated) ... typically between 0x0300 and 0x0580 (T1904), 0x0220-0x0540 (T1942)
    uint16_t    axis2;              // 20..21   unused: often 0x07d0 (timeout of gameport 'A/D-conversion')
    uint16_t    axis3;              // 22..23   unused: often 0x07d0 (timeout of gameport 'A/D-conversion')
};

// using the motorbrake, the frame is just a copy of the data frame we receive from the motorbrake
// but to emulate the same structure with the eddy current brake we have to know at least
// some fields of the frame.
//
// 1. There is a "long" frame when we send a standard 0x01 0x08 0x01 0x00 [0x....] control command to the motorbake
// 2. There is a "short" frame when we send a 0x02 0x00 0x00 0x00 version command to the motorbake.
// 3. And there is a very short 'error' answer, if we send corrupted data (with a valid checksum)
// 4. Finally, we do not get an answer if we send utter junk (with or without a valid checksum).
//
// The format of the "long" frame was not reversed by me, it's from the GoldenCheetah source
//

//                Cmd    Size Page  Zero
// --------------------------------------------------------------------
// Host => Brake: 0x01, 0x08, 0x01, 0x00 [0..7]  [control command]
// Brake => Host: 0x03, 0x13, 0x02, 0x00 [0..18] [motor-brake data frame answer]
// Brake => Host: 0xFF, 0x02, PageBounce, ZeroBounce, 0xce, 0xff [error code if page or zero is wrong]
// --------------------------------------------------------------------
// Host => Brake: 0x02, 0x00, 0x00, 0x00         [request version&serial]
// Brake => Host: 0x03, 0x0c, 0x00, 0x00 [0..11] [motor-brake version/serial answer]
// Brake => Host: 0xFF, 0x02, PageBounce, ZeroBounce, 0xcc, 0xff [error code if page or zero is wrong]
// --------------------------------------------------------------------
// Note: All other Command values (not 0x01 or 0x02) are ignored

struct frameHeader {
        uint8_t command;
        uint8_t size;
        uint8_t datapage;
        uint8_t zero;       // never seen anything else than 0x00
};

struct frameBrakeLong {
    struct frameHeader header;      // 24..27

    uint32_t    distance;           // 28..31  total distance
    uint16_t    rawSpeed;           // 32..33  wheel speed in kmh ~= speed / 290

    uint16_t    unknown34_35;       // 34..35  increases if you accelerate, bit 0..2 always zero
    uint16_t    avgResistance;      // 36..37  Avg. resistance? Changes similar to 38..39 on T1904
    uint16_t    currentResistance;  // 38..39 

    // Off:       set to zero
    //
    // Ergo:      load = f_power(power)     
    //            power = [0..1000]
    //            with f_power ~= currentResistance / 128866 * rawSpeed
    // 
    // Slope:     load = f_resistance(slope,Ds) 
    //            slope = [-5..20], Ds = -0.4 or Ds = 0 ??
    //            with f_resistance ~= (slope(%) + Ds) * (2*5*130) * F_unknown_correction(weight, speed, incline)
    // 
    // Calibrate: load = f_speed(speed)     
    //            speed = [0..20++] (be careful!!) 
    //            with f_speed ~= speed(kph) * 290

    uint16_t    targetLoad;         // 40..41 - bounced value of selectedLoad?
    uint8_t     events;             // 42  - 0x01 pedal-sensor event, 0x04 brake-stops event
    uint8_t     unknown43;          // 43
    uint8_t     cadence;            // 44 in RPM
    uint8_t     unknown45;          // 45 (value > 0x00 when accelerating )
    uint8_t     modeEcho;           // 46 motor brake echos the mode-byte from command-frame
    uint8_t     checksumLSB;        // 47 fragemented checksum
    // Standard 48 byte frame ends here - we added checksumMSB
    uint8_t     checksumMSB;        // 48 fragemented checksum
    // inoffical development extension
    uint16_t    resistance_now;
    uint8_t     wheel_timestamps_num;
    uint16_t    wheel_timestamps[6];
};


struct frameBrakeShort {
    struct frameHeader header;
    uint32_t    firmwareVersion;     // 0.x.y.c
    uint32_t    serialNumber;        // tt-YY-#####  (tt=41 (T1941), YY=year, ##### brake individual serial)
    uint16_t    version2;            // Date of firmware DD.MM or MM.DD?? (together with YY in the serial?)
    uint16_t    unused;              // ??
    uint8_t     checksumLSB;
    uint8_t     checksumMSB;
};

struct frameBrakeError {
    struct frameHeader header;
    uint16_t    errorcode;  // always 0xffce ?
    uint8_t     checksumLSB;
    uint8_t     checksumMSB;
};


struct frame {
    struct frameHeadUnit   unit;   // size 24 (bytes 0..23)
    union {
        uint8_t buf[25+15];        // bytes 24..48 + padding is 49..63
        struct frameBrakeLong  l;  // size 25
        struct frameBrakeShort s;
        struct frameBrakeError e;
    } brake;
};


#define T1941_VERSION_COMMAND_SIZE                 (4)
#define T1941_VERSION_COMMAND_HEADER               (0x00000002)

#define T1941_CONTROL_COMMAND_SIZE                 (12)
#define T1941_CONTROL_COMMAND_HEADER               (0x00010801)
#define T1941_CONTROL_COMMAND_MODE_NONE            (0)
#define T1941_CONTROL_COMMAND_MODE_FORCE           (2)
#define T1941_CONTROL_COMMAND_MODE_CALIBRATE       (3)
#define T1941_CONTROL_COMMAND_FLYWHEEL_WEIGHT_OFF  (0x0a)

//
// Mode=2: targetLoad = ~137 * targetForce in (N)
//   trainer load is a function of targetLoad, flywheelLoad and calibrationOffset 
//   virtual flywheel is a function of acceleration and configured flywheel weight
//   calibration is an negative offset to the selected load
//
// power = ..
//
// totalForce = trainer load + brakeError(velocity) + systemLoss
// systemLoss is the force of the transmission 
//
// Mode=3: load = targetSpeed * 289.75
/*

Yes, the weight changes the behavior of the trainer but it does not 
'correct' a targetLoad for a given 'slope'.
If the trainer would simulate a real mass riding up a hill, then changing the 
weight should have an effect on the force for a given slope. But there is no change of the trainer force, 
if you change the mass while riding with constant speed for a given slope (= constant load).
The trainer just do not know something about 'slopes' and there is 
no 'ergo mode' or 'slope mode'. There is only a 'calibration mode' (mode=0x3, targetLoad 
controls the speed of the trainer) and a 'force mode' (mode=0x2, targetLoad controls the force of the trainer).

But let me explain:

I have done some tests today (without a power meter) and now I am fairly sure, 
that the 'weight' value controls only some kind of "virtual fly wheel".

Values under 0x0a results in some strange effects but at least for values above 
0x0a, changing the 'weight' has an effect on the 'responsiveness' of the trainer, 
if you change the speed (acceleration != 0)
For higher weights, the trainer is less responsive, if you increase the speed 
(acceleration > 0) and on the other side the trainer runs a little longer if you stop pedaling (acceleration < 0) .

The trainer seems to simulate a fly wheel, which needs some extra power to 
'virtual accelerate' if you accelerate the trainer and gives back some (virtual kinetic) energy if you reduce the speed.
Internally the trainer adds some force to the 'targetLoad', if you increase 
the speed and reduces the force if you decrease the speed.

After this test I probably should change the documentation of the answer 
frame (byte offsets for T1942 answer frames - subtract 24 bytes for raw T1941 answers):

Byte 40/41 is the raw (bounced) "targetLoad" from the command.
Byte 38/39 is the real load (targetLoad +/- virtualFlyWheelLoad)
Byte 36/37 is the some kind of avg. (real) load
Byte 34/35 is a function of the speed-difference (maybe just a simple delta 
between two 'rawSpeed' values for a given time ==> +/- acceleration)

Byte 34/35 probably is (together with the 'weight' value of the command) 
the input value to compute the VirtualFlyWheelLoad:

    realLoad = targetLoad + flyWheelLoad(acceleration, weight)

As long as you do not change the speed (acceleration = 0) , the values for 
targetLoad, realLoad and (after a while) the avgRealLoad shows the same values, 
because the effect of the virtual fly wheel is "0". The 'weight' value has NO effect now.

Result:

    In 'force mode=0x2', trainer 'load' is always linear to a force (maybe 
    with a little offset for calibration).

As I wrote above, the unit of "1 load" is equivalent to about 137 N. Negative 
values are possible. In this case the trainer further accelerates if you stop pedalling.

As long as you do not know the effect of the virtual fly wheel, it is probably 
a good idea to reduce the 'weight' to a small value and just simulate all 
of physics by your own. Probably weight=0x0a is a good choice.


 I did another run.

    With the assumption speed = rawSpeed / scale the fitting gave a value of scale=286.7.
    With the assumption speed = (rawSpeed - offset) / scale the fitting for the same run gave me offset=-60 and scale=288.7.

A comparison with TTS4 confirms these values. Strange.

What kind of powerback/brake do you have? Is it a US (110V / 60Hz) or EU (230V / Hz) ?

My powerback says:
firmwareVersion= 00.00.09.65
serial= 410502330 (Tacx T1941 Year 2005 #02330)
Date= 0c.08 Unknown= 00.00

I've also done some further investigations.

The calibration value in byte 10,11 of the command frame is a simple offset to the brake-load.
A load of "1000" with calibration 0 results in the same force as a load of 2000 with calibration 1000.
So, real brake load is: realLoad = commandLoad - calibration.

The problem with the brake calibration is a drift when the brake, or the wheel, or whatever gets warm.

I made a calibration run after a very long warm-up. At the end of the warm-up the brake gave calibration values of about 0x430. The motor was warm (but not hot) at the end.
The calibration run itself was done with calibration_0=0 and with the assumption of

power = a + b1_0 * rawSpeed + c1 * rawSpeed * commandLoad

The fitting gave the following values:
a ~= -35.4
b1_0 ~= .01334
c1 ~= .0000070529

So, you can use these values to estimate the power for a given load (with calibration=0 in byte 10,11) and after a similar calibration run (speed=20 kph , load=0x430). At least the force (loss) of turning the rear wheel should be the same .
The loss between your pedal and the rear wheel can be different and cannot be compensated with a calibration run of the brake.

Or if you assume a calibration_0 <> 0:

power = a + (b1_0 + c1 * ( commandLoad - calibration_0 ) * rawSpeed

I.e with calibration_0=1040 (the standard tacx value)

power = a + b1_1040 * rawSpeed + c1 * commandLoad * rawSpeed

a ~= -35.4
b1_1040 ~= 0,006
c1 ~= .0000070529

With theses values you can estimate the power of a given load (but now with calibration=1040 in byte 10,11).

--

There is still a drift over time. After some time my power meter shows smaller values than the software model and the brake is very warm.
To correct this, one have to reduce the calibration (byte 10,11 in the command) by about "25" for 1 Watt at 20 kph. The brake increase the force (without increasing the power values of the software model).

Maybe a temperature sensor at the brake may help to compensate the drift.
*/
struct frameCommandFortius {
    // 02 00 00 00
    //     or
    // 01 08 01 00
    union {
        struct frameHeader header;
        uint32_t header32;
    } h;

    uint16_t force;      // 0..1 : targetLoad (in force mode (=2)) - targetSpeed (in calibrate mode (=3)) 
    uint8_t  pedalecho;  // 2 
    uint8_t  zero0;      // 3
    uint8_t  mode;       // 4 IDLE/NONE=0, FORCE/RESISTANCE=2, CALIBRATE/SPEED=3
    uint8_t  flywheel;   // 5 virtual flywheel - use 0x0a (almost no flywheel) for ergo  
    uint16_t calibrate;  // 6..7 used in mode=2: calibratedLoad = targetLoad - calibrationOffset 
};


// Legacy:
struct frameCommandImagic {
    // brake load/resistance
    uint8_t  load;

    //  Stopwatch Control (0x01/0x04/0x05 pause, 0x00/other values: start)
    //    3 bits (0-2) are relevant
    //           0x01 STOP/START  'isRunning'
    //           0x02 AUTOSTOP  if 'Wheel' < 20
    //           0x04 AUTOSTART if 'Wheel' >= 20
    //  To force a start without wheeling you need to a) start the Stopwatch (Bit0=1) and
    //  and b) disable autostopping (Bit1=0). Bit2 does not matter in this case
    //
    //  Auto-Start/Stop is achieved with 0x06
    //  0x01/0x04/0x05 pause, 0x00/other values: start/continue
    uint8_t  autostartstop;

    // optional: stopwatch (re)set values (different on older firmwares?)
    uint8_t  stopwatchSecHigh;
    uint8_t  stopwatchSecMid;
    uint8_t  stopwatchSecLow;
    uint8_t  stopwatchHundreds;
};

struct eeprom {
    uint8_t  loadType;   // 0xB0 -> reduced USB descriptor, 0xB2 => load firmware from eeprom
    uint16_t vendorID;
    uint16_t productID;
    uint16_t deviceID;
    uint8_t  unused[6];
    uint8_t  year_production;
    uint16_t deviceSerial;
};


#endif
