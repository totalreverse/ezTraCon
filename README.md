# EzTraCon - [Ez]Usb [Tra]ainer [Con]trol firmware

EZusb TRAiner CONtroller - An alternative firmware ONLY for Tacx "solid" green (T1902) or "solid" blue (T1942) head units.

## Preface

This firmware is highly experimental!<p>
**USE AT YOUR OWN RISK - You've been warned !!**<p>
Only tested with **230V, 50Hz** trainers.<p>
**I recommend to use the firmware(s) only if you're able to solve basic problems for your own. Use the source!**

## Intro

`ezTraCon` is an alternative firmware for the old "solid" head units, that can control both *classic* Tacx brakes: 
* The (Fortius) motorbrake and 
* the (Flow, i-Magic) magnetic brakes. 

For the communication with the host computer, the firmware offers two different options.
* the _classic_ 12/48 bytes protocol of the T1942 firmware and an
* experimental ANT+ emulation of the Dynastream ANT+ usb dongle (device id 0x1008) 

Note: **The firmware does not support the newer 'blue ring' (T1932) and 'green ring' (T1904) head units **. 

## Main tools and sources of information

Probably the project would not exists without these free tools and information:

* Tools, Compiler
    * sdcc [http://sdcc.sourceforge.net](http://sdcc.sourceforge.net)
    * sdas8051 - 8051 assembler of sdcc
    * fxload 
    * python (with libusb)
    * zadig (driver handling for Windows)

* ezUSB core firmware
    * EzUSB firmware  [https://github.com/hansiglaser/ezusb-firmware](https://github.com/hansiglaser/ezusb-firmware)

* ezUSB general documentation
    * Cypress [AN2121 Technical Referance Manual](http://www.cypress.com/file/101046/download)

## Other projects and sources of information

* Project Wiki
    * [tty1941 Wiki](https://github.com/totalreverse/ttyT1941/wiki) has a lot of information about Tacx trainer protocols.  

* Other EZ-USB projects    
    * EZ-USB board on Linux [https://www.danielclemente.com/placa_pi/index.en.html](https://www.danielclemente.com/placa_pi/index.en.html)
    * EZUSB2131  [http://ezusb2131.sourceforge.net/](http://ezusb2131.sourceforge.net/)

* ANT information
    * Main source for ANT+ information [https://www.thisisant.com/](https://www.thisisant.com/)

* Eddy Current Trainer 
    * The following link has some (really helpful) information about controlling a FLOW brake 
  [https://kluedo.ub.uni-kl.de/frontdoor/deliver/index/docId/1662/file/AmI_Bericht_Nr._5.pdf](https://kluedo.ub.uni-kl.de/frontdoor/deliver/index/docId/1662/file/AmI_Bericht_Nr._5.pdf) (in german language).

* Tacx trainer information and investigations
    * https://www.tacx.com/
    * https://forums.zwift.com/t/support-for-old-non-ant-trainers-and-bikes/1679/8
    * http://forum.tacx.com/viewtopic.php?t=34282
    * http://www.powercurvesensor.com/cycling-trainer-power-curves/
    * http://athletictechreview.com/2016/12/05/tacx-vortex-smart-review-part-2-spin-down/
    * https://github.com/totalreverse/ttyT1941/wiki

* Related software projects
    * GoldenCheetah [Project](http://www.goldencheetah.org/) Software: [Github](https://github.com/GoldenCheetah)
    * DIY head unit for TACX Flow [Tacx Flow Ant+ Conversion](https://hackaday.io/project/164276-tacx-flow-ant-conversion)
                  
    * FortiusANT [https://github.com/WouterJD/FortiusANT](https://github.com/WouterJD/FortiusANT)
    * Antifier   [https://github.com/john-38787364/antifier](https://github.com/john-38787364/antifier)
    * Antbridge  [https://github.com/pepelkod/AntBridge](https://github.com/pepelkod/AntBridge)
    * pyBushido  [https://github.com/jovial/PyBushido](https://github.com/jovial/PyBushido)
    * BleTrainerControl [https://github.com/jedla22/BleTrainerControl](https://github.com/jedla22/BleTrainerControl)

    * Cylcismo Projet [https://github.com/fluxoid-org/CyclismoProject/wiki/Tacx-Genius](https://github.com/fluxoid-org/CyclismoProject/wiki/Tacx-Genius:)

    * Budget- & Beagletrainer (example for Spindown calibration) and remote control 

        * https://budgettrainerbuild.wordpress.com/
        * https://github.com/dresco/BudgetTrainer
        * https://github.com/dresco/BeagleTrainer/blob/master/src/BeagleTrainer.c

    * ANT+ Adapter for Cycleops    [https://www.blinkyme.com/wp-content/hostinger-page-cache/category/cycleops-pro300pt-to-ant/_index.html](https://www.blinkyme.com/wp-content/hostinger-page-cache/category/cycleops-pro300pt-to-ant/_index.html)



## Build 

### Linux 

The build is tested with the plain vanilla packages from Ubuntu 18.04. 

 * sdcc 3.5.0 #9253 (Apr  3 2018) (Linux)
 * sdas8051 (sdas Assembler V02.00 + NoICE + SDCC mods  (Intel 8051))
 * make 

The build uses sdcc and sdas8051 to build the firmware. The d52 disassembler is not necessary to build the firmware. To install them on an Ubuntu system use the apt tool. 

`
  sudo apt install make sdcc d52
`

To build all variants of the firmware just type

`
make
`

It's a constant fight to squeeze as much functionality into
the firmware. The ezUSB memory is small and unfortunately, not everything fits into memory at once. I rewrote some functions in assembler, but it still was not possible to put all protocols into one firmware and for the ANTPLUS emulation it was even necessary to split the firmware into a version for magnetic brakes and one for motor brakes. 

Note: At least all firmwares support both 'solid' head units. 


* ezTraCon_Classic.hex<p>
  is the standard firmware, which supports both brake types (auto detection) and talks the classic Tacx Fortius protocol. Power model for eddy current brakes is experimental.

* ezTraCon_Classic_Motor.hex<p>
  is a reduced firmware, which supports the motor brake 
  and talks the classic Tacx Fortius protocol. There is some space left in memory
  to add some debugging and analyzing functionality.

* ezTraCon_Classic_Eddy.hex<p>
  is a reduced firmware, which supports the magnetic (eddy current)  brake 
  and talks the classic Tacx Fortius protocol. There is some space left in memory
  to add some debugging and analyzing functionality. Power model for eddy current brakes is experimental.

* ezTraCon_Ant_Eddy.hex<p>
  is an experimental firmware for the magnetic brake that emulates an ANTPLUS USB stick, so you can use it with software only talking ANT+ FEC (fitness equipment control). 
  *****I do not recommend to use this firmware!***

* ezTraCon_Ant_Motor.hex<p>
  is an experimental firmware for the motor brake that emulates an ANTPLUS USB stick so you can use it with software only talking ANT+ FEC (fitness equipment control). 
  Implemented operation modes are BASIC_RESISTANCE and POWER. Do not use SIMULATION mode.

## TODO and Known Bugs 

* use at your own risk. 

* There are lot of known problems. The whole firmware is still highly experimental. 

* I do not have much experience using the firmware under Windows and no experience with MacOS. All development and testing was done under Linux. 

* Eddy current handling is **really very experimental**.


## Linux: udev rules

To access the USB-device under linux, add these rules to your udev configuration

Rules for ANT+ USB sticks i.e. /etc/udev/rules.d/90-dynastream.rules
```
ATTRS{idVendor}=="0fcf", ATTRS{idProduct}=="1004", MODE="0666"
ATTRS{idVendor}=="0fcf", ATTRS{idProduct}=="1008", MODE="0666"
ATTRS{idVendor}=="0fcf", ATTRS{idProduct}=="1009", MODE="0666"
```

Rules for solid head unit i.e. /etc/udev/rules.d/90-tacxheadunit.rules
```
ATTRS{idVendor}=="3561", ATTRS{idProduct}=="1902", MODE="0666"
ATTRS{idVendor}=="3561", ATTRS{idProduct}=="1942", MODE="0666"
ATTRS{idVendor}=="3561", ATTRS{idProduct}=="e8be", MODE="0666"
```

Do not forget to reload the udev rules with 
```
sudo udevadm control --reload-rules && udevadm trigger
```
If you now plug in your device it can be accessed by every user. 

### auto load firmware 

If you copy the appropriate firmware to /lib/firmware/ and fxload is 
under /sbin/fxload, add the following rule to /etc/udev/rules.d/90-tacxheadunit.rules to auto load the firmware, every time you plug in your green head unit (product = 1902).

```
ATTRS{idVendor}=="3561", ATTRS{idProduct}=="1902", RUN+="/sbin/fxload -I /lib/firmware/ezTraCon_Classic.hex -D $root/$name" 
```

## Using 'stand alone'

Load the appropriate firmware. 

On linux you can use the 'fxload' tool. Please note, that you need the right permission
to open the USB device (see above).

```
 sudo apt install fxload
```
then<p>
```
fxload -D /dev/bus/usb/..path_to_your_usb_device -I firmware.ihx
```
You can find the right USB device with<p>
```
lsusb -d 3561:
```

You can get some more inforamation with 
```
lsusb -d 3561: -v | head -20
```

Note: After loading the firmware, the USB device number changes!

## Using with GoldenCheetah

GoldenCheetah currently only knows the solid blue head unit. So, if you have a solid green head unit, you have to patch GoldenCheetah. 
Alternatively, you can load the classic firmware or the ANT+ firmware in advance  with 'fxload' or let linux "udev" load the firmware for you (see above). 
If you have the solid blue head unit, there is no need to load the firmware with fxload. 
GoldenCheetah can load the firmware for you. But you still need the permission to open the USB device!

#### Known problem with GoldenCheetah and ANT+ emulation

GoldenCheetah deadlocks in ANT+ mode, if compiled with libusb-0.1 (at least under linux). 
You must compile GoldenCheetah with lixusb-1.0 to savely use it with the ANT+ firmware. 
If you have to use libusb-0.1 try send 0 bytes USB data frames instead of NAK
by uncommenting the IN1BC=0 at the end of antplus.c source file. 
But this is only a dirty work around.

### Manual brake calibration in ANT+ mode

The brake has a drift over time. Warm up the brake for about 10 minutes before doing a calibration run. In ANT+ mode you can change the calibration with the UP and DOWN buttons. 
"OK" resets the calibration to the default value 0x410 = 1040. Cancel sets the calibration to "0" which results in a higher resistance (lower value means less correction of external forces). So pressing the "UP" button reduce the force for a given target power and pressing the "DOWN" button increases the force. 

* If your power meter readings are to low press "DOWN"
* If your power meter readings are to high press "UP"


### Lights 

* Red light indicates a signal from the heart rate monitor

* Autodetect mode: one short green blink every 2 seconds

* detected motor brake : two green blinks every 2 seconds

* green light is flickering: ongoing communication between head unit and motor brake 

* constant green blinking lights : eddy current power line signal available but frequency not yet detected 

* detected eddy current brake 50 Hz : three green blinks

* detected eddy current brake 60 Hz : four green blinks

* fast alternate blinking of red and green light : *ERROR*. Switch off the brake! Needs firmware reload to leave this state.

### Troubleshooting

* No yellow light
  * No head unit power: check USB connection

* Software cannot detect brake (no green lights)
  * check if brake switched on
  * check cables
  * reload firmware
  * you're not using the apropriate firmware for your brake

* Motorbrake detected but stops after a few seconds 
  * CAD sensor not working (echo signal) - check CAD sensor cable
  * check status lights  
  * recheck power cable of brake
  * recheck connection between head unit and brake

