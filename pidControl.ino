#include "Arduino.h"
#include "HardwareTimer.h"
#include "PID_v1.h"
#include "ros.h"
#include "rosserial_arduino/Adc.h"
#include "std_msgs/String.h"

#define PPR2 1024

double Setpoint=30, Input , Output;
uint16_t LastEncoder=0, CurrentEncoder=0;

int PPR = 65535;
HardwareTimer timer_3(3);
HardwareTimer timer_2(2);
PID myPID(&Input, &Output, &Setpoint, 5, 1, 0.1, DIRECT);

ros::NodeHandle nh;
rosserial_arduino::Adc adc_msg;
std_msgs::String info;
ros::Publisher  pub("pid", &adc_msg);
ros::Publisher chatter("info", &info);

char buf[128] = {0};

void messageCb(std_msgs::String& str){
  
}

void setup() {
  nh.initNode();
  nh.advertise(pub);
  nh.advertise(chatter);

  Serial.begin(115200);
  
  pinMode(32, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(2, PWM);

//configure timer as encoder
  timer_3.setMode(0, TIMER_ENCODER); //set mode, the channel is not used when in this mode. 
  timer_3.pause(); //stop... 
  timer_3.setPrescaleFactor(1); //normal for encoder to have the lowest or no prescaler. 
  timer_3.setOverflow(PPR);    //use this to match the number of pulse per revolution of the encoder. Most industrial use 1024 single channel steps. 
  timer_3.setCount(0);          //reset the counter. 
  timer_3.setEdgeCounting(TIMER_SMCR_SMS_ENCODER3); //or TIMER_SMCR_SMS_ENCODER1 or TIMER_SMCR_SMS_ENCODER2. This uses both channels to count and ascertain direction. 
  timer_3.resume();                 //start the encoder... 

  timer_2.setMode(2, TIMER_PWM);
  timer_2.setPrescaleFactor(71);
  timer_2.setOverflow(PPR2);
  timer_2.setCount(0);
  timer_2.resume();

  pwmWrite(2, 0); 

  myPID.SetOutputLimits(0, PPR2);
  myPID.SetMode(AUTOMATIC);
}
unsigned long interval=0; //variable for status updates... 

void loop() {

  if (millis() - interval >= 30) { 
     interval = millis(); //update interval for user. 
     
     CurrentEncoder = timer_3.getCount();
     if(CurrentEncoder < LastEncoder){
        Input = PPR - LastEncoder + CurrentEncoder;
     }else{
        Input = CurrentEncoder - LastEncoder;
     }
     if(Input > 100){
       digitalWrite(32, !digitalRead(32));
       snprintf(buf, sizeof(buf), "cur:%d last:%d Input:%d\n", CurrentEncoder, LastEncoder, Input);
       info.data = buf;
       chatter.publish(&info);
       nh.spinOnce();
     }
     LastEncoder = CurrentEncoder;

     myPID.Compute();
     
     pwmWrite(2, Output);

    adc_msg.adc0 = (uint16_t)Input*30;
    adc_msg.adc1 = (uint16_t)Output;
    adc_msg.adc2 = (uint16_t)Setpoint*30;
    pub.publish(&adc_msg);
    nh.spinOnce();
  }
}
