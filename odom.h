#ifndef _ODOM_H_
#define _ODOM_H_

#include "PID_v1.h"
#include "motor.h"
#include "encoder.h"
#include "global.h"

class moveUnit {
    public:
        moveUnit(int timerN, int forwardPin, int backwardPin, int encoderTimer, double kp, double ki, double kd, int sampleTime=SAMPLE_TIME);

        void setInput(double input){
            _input = input;
        }
        double getInput(){
            return _input;
        }

        double getOutput(){
            return _output;
        }

        void setSetPoint(double setPoint){
            _setPoint = setPoint;
        }
        double getSetPoint(){
            return _setPoint;
        }

        void setKp(double kp){
            _kp = kp;
        }
        double getKp(){
            return _kp;
        }
        void setKi(double ki){
            _ki = ki;
        }
        double getki(){
            return _ki;
        }
        void setKd(double kd){
            _kd = kd;
        }
        double getKd(){
            return _kd;
        }

        void moveUnitCompute();
        void moveUnitSetTuning(double kp, double ki, double kd);
        
    private:
        unsigned int _sampleTime;

        motor *_motor;
        encoder *_encoder;
        PID *_pid;

        double _input=0;
        double _output=0;
        double _setPoint=0;

        double _kp;
        double _ki;
        double _kd;

        int _motorForwardPin;
        int _motorBackwardPin;

        int16_t _lastEncoderCnt;
};

#endif
