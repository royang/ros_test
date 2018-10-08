#include "Arduino.h"

#include "power.h"

power::power(uint16_t pin){
    _pin = pin;
    pinMode(pin, INPUT_ANALOG);
}

uint16_t power::getVoltage(){
    _voltageValue = analogRead(_pin);
}