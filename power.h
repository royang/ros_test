#ifndef _POWER_H_
#define _POWER_H_

#include "stdint.h"

#define VOLTAGE_SAMPLE_PIN  6

class power{
    public:
        power(uint16_t pin);

        uint16_t getVoltage();
    private:
        uint16_t _voltageValue;
        uint16_t _pin;
};


#endif
