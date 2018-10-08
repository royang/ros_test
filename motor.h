#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "HardwareTimer.h"

#define MOTOR_L_FORWARD_PIN PE14
#define MOTOR_L_BACKWARD_PIN  PE13

#define MOTOR_R_FORWARD_PIN PB8
#define MOTOR_R_BACKWARD_PIN    PB9

#define MOTOR_L_TIMER 1
#define MOTOR_R_TIMER 4

#include "HardwareTimer.h"

#define PPR_PWM 1024

class motor{
	public:
		motor(int timerN, int forwardPin, int backwardPin);

        void enable();
        void disable();
        int getState();

        void speedUpdate(int speed);


	private:
        int _timer;
        int _forwardPin;
        int _backwardPin;
        HardwareTimer *_motorTimer;

};

#endif //_MOTOR_H_
