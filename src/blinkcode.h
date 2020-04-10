

#ifndef __BLINKCODE_H
#define __BLINKCODE_H

extern __bit  volatile           blinkcode_force_red;
extern __bit  volatile           blinkcode_force_red_single;
extern __bit  volatile           blinkcode_force_green;

extern void blinkcode_red(uint16_t code);
extern void blinkcode_green(uint16_t code);
extern void blinkcode_handle();        
extern void blinkcode_shift();

#endif