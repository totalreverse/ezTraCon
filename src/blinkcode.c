
#include "main.h"

__idata uint16_t blinkcode_red_loop;
__idata uint16_t blinkcode_red_single;
__idata uint16_t blinkcode_green_loop;
__idata uint16_t blinkcode_green_single;

__bit  volatile  blinkcode_force_red;
__bit  volatile  blinkcode_force_red_single;
__bit  volatile  blinkcode_force_green;

// #pragma callee_saves blinkcode_red

void blinkcode_red(uint16_t code) {
    blinkcode_red_single = blinkcode_red_loop = code;
}

void blinkcode_green(uint16_t code) {
    blinkcode_green_single = blinkcode_green_loop = code;
}

void blinkcode_handle() {
    if((blinkcode_force_green || (blinkcode_green_single & (1<<15)) )) {
        GREEN_ON();
    } else {
        GREEN_OFF();
    }
    if((blinkcode_force_red   || (blinkcode_red_single & (1<<15)) )) {
        RED_ON();
    } else {
        RED_OFF();
    }
}

void blinkcode_shift() {
    // rotate 'loop' and shift the lower bit of the loop to single
    blinkcode_green_loop   = (blinkcode_green_loop  <<1) | (blinkcode_green_loop>>15);
    blinkcode_green_single = (blinkcode_green_single<<1) | (blinkcode_green_loop&0x1);
    // rotate 'loop' and shift the lower bit of the loop to single
    blinkcode_red_loop     = (blinkcode_red_loop  <<1)   | (blinkcode_red_loop>>15);
    blinkcode_red_single   = (blinkcode_red_single<<1)   | (blinkcode_red_loop&0x1);

    if(blinkcode_force_red_single) {
        blinkcode_red_single |= (1<<15);
        blinkcode_force_red_single = false;
    }
}
