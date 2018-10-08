#ifndef _ENCODER_H_
#define _ENCODER_H_

#include "HardwareTimer.h"

#define ENCODER_L_TIMER 3
#define ENCODER_R_TIMER 2

#define PPR_ENCODER  0xffff

class encoder{
    public:
        encoder(int timerN);

        int16_t getEncoderCounter();
        void encoderclear();

        void enalbe();
        void disable();
        int getState();
        uint8_t getDirection();

    private:
        HardwareTimer *_timerEncoder;
};











#endif

