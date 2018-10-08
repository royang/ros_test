#include "odom.h"
#include "ros.h"

extern ros::NodeHandle nh;

moveUnit::moveUnit(int motorTimer, int motorForwardPin, int motorBackwardPin, int encoderTimer, double kp, double ki, double kd, int sampleTime){
    _motorForwardPin = motorForwardPin;
    _motorBackwardPin = motorBackwardPin;
    _kp = kp;
    _ki = ki;
    _kd = kd;
    _sampleTime = sampleTime;

    _motor = new motor(motorTimer, motorForwardPin, motorBackwardPin);
    _motor->enable();

    _encoder = new encoder(encoderTimer);
    _encoder->enalbe();

    _pid = new PID(&_input, &_output, &_setPoint, _kp, _ki, _kd, DIRECT);

    _pid->SetOutputLimits(-PPR_PWM, PPR_PWM);
    _pid->SetMode(AUTOMATIC);
    _pid->SetSampleTime(sampleTime);
}

void moveUnit::moveUnitCompute(){
    int16_t currentEncoderCnt=0;
    uint8_t currentEncoderDirection=0;
    char buf[128]={0};

    currentEncoderCnt = _encoder->getEncoderCounter();
    _encoder->encoderclear();
    if((currentEncoderCnt > UNCONTROL_ENCODER_SPEED_THERSHOLD) || (currentEncoderCnt < -UNCONTROL_ENCODER_SPEED_THERSHOLD)){
        _input = _lastEncoderCnt;
    }else{
        _input = _lastEncoderCnt = currentEncoderCnt;
    }

    _pid->Compute();

    if((_output > PPR_PWM) || (_output < -PPR_PWM)){
        _output = 0;
    }

    _motor->speedUpdate(_output);    
}

void moveUnit::moveUnitSetTuning(double kp, double ki, double kd){
    _pid->SetTunings(kp, ki, kd);
}