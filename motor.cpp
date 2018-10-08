#include "Arduino.h"
#include "motor.h"

#define MOTOR_DRIVER_ENABLE_PIN PB15

motor::motor(int timerN, int forwardPin, int backwardPin)
{
    _motorTimer = new HardwareTimer(timerN);

    _timer = timerN;
    _forwardPin = forwardPin;
    _backwardPin = backwardPin;

    pinMode(MOTOR_DRIVER_ENABLE_PIN, OUTPUT);
    this->disable();

    _motorTimer->setPrescaleFactor(71);
    _motorTimer->setOverflow(PPR_PWM);
    _motorTimer->setCount(0);
    _motorTimer->resume();
}

void motor::enable(){
    digitalWrite(MOTOR_DRIVER_ENABLE_PIN, HIGH);
}

void motor::disable(){
    digitalWrite(MOTOR_DRIVER_ENABLE_PIN, LOW);
}

int motor::getState(){
    return digitalRead(MOTOR_DRIVER_ENABLE_PIN);
}

void motor::speedUpdate(int speed){
    if(speed > 0){
        pinMode(_forwardPin, PWM);
        pwmWrite(_forwardPin, speed);

        pinMode(_backwardPin, OUTPUT);
        digitalWrite(_backwardPin, LOW);
    }else{
        pinMode(_backwardPin, PWM);
        pwmWrite(_backwardPin, -speed);

        pinMode(_forwardPin, OUTPUT);
        digitalWrite(_forwardPin, LOW);        
    }
}