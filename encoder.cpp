#include "encoder.h"
#include "Arduino.h"



#define ENCODER_ENABLE_PIN PB15

encoder::encoder(int timerN){
    if(timerN == 3){
        afio_remap(AFIO_REMAP_TIM3_FULL);
    }

    _timerEncoder = new HardwareTimer(timerN);
    
    _timerEncoder->setMode(0, TIMER_ENCODER); 
    _timerEncoder->setMode(1, TIMER_ENCODER); 
    _timerEncoder->pause(); 
    _timerEncoder->setPrescaleFactor(1); 
    _timerEncoder->setOverflow(PPR_ENCODER);    
    _timerEncoder->setCount(0);         
    _timerEncoder->resume();  

    pinMode(ENCODER_ENABLE_PIN, OUTPUT);
    digitalWrite(ENCODER_ENABLE_PIN, LOW);
}

int16_t encoder::getEncoderCounter(){
    return (int16_t)(_timerEncoder->getCount());
}

void encoder::encoderclear(){
    _timerEncoder->setCount(0);
}

void encoder::enalbe(){
    digitalWrite(ENCODER_ENABLE_PIN, HIGH);   
}

void encoder::disable(){
    digitalWrite(ENCODER_ENABLE_PIN, LOW);
}

int encoder::getState(){
    return digitalRead(ENCODER_ENABLE_PIN);
}

uint8_t encoder::getDirection(){
    return _timerEncoder->getDirection();
}