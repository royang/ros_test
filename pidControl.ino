#include "Arduino.h"
#include "HardwareTimer.h"
#include "PID_v1.h"
#include "ros.h"
#include "rosserial_arduino/Adc.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Vector3.h"
#include "string.h"

#define PPR2 1024
#define PPR  65535

void messageCb(const geometry_msgs::Vector3 &ctlMsg);
void setPointCb(const std_msgs::String &val);

double Setpoint=30, Input , Output;
uint16_t LastEncoder=0, CurrentEncoder=0;

HardwareTimer timer_3(3);
HardwareTimer timer_2(2);

PID myPID(&Input, &Output, &Setpoint, 5, 1, 0.1, DIRECT);

geometry_msgs::Vector3 rqt_var;

ros::NodeHandle nh;
ros::Publisher pub("pid", &rqt_var);

std_msgs::String setPointVal;
ros::Subscriber<std_msgs::String> setPointSub("/setPntSub", &setPointCb);

ros::Subscriber<geometry_msgs::Vector3> sub("/pidCtrl", &messageCb);

char buf[64]="Arduino init ok";

void setPointCb(const std_msgs::String &val){
  Setpoint = atoi(val.data);
  memset(buf, 0, sizeof(buf));

  snprintf(buf, sizeof(buf), "new setPoint：%d", (int)Setpoint);
  
  nh.loginfo(val.data);
  nh.loginfo(buf);
}

void messageCb(const geometry_msgs::Vector3 &ctlMsg){
  digitalWrite(32, !digitalRead(32));

  memset(buf, 0, sizeof(buf));
  snprintf(buf, sizeof(buf), "kp:%.2f ki:%.2f kd:%.2f", ctlMsg.x, ctlMsg.y, ctlMsg.z);
  nh.loginfo(buf);
  nh.spinOnce();

  timer_2.pause();
  myPID.SetTunings(ctlMsg.x, ctlMsg.y, ctlMsg.z);
  timer_2.resume();
}

void setup() {
  nh.initNode();
  nh.advertise(pub);
  nh.subscribe(sub);
  nh.subscribe(setPointSub);
  
  pinMode(32, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(2, PWM);

  timer_3.setMode(0, TIMER_ENCODER); 
  timer_3.pause(); 
  timer_3.setPrescaleFactor(1); 
  timer_3.setOverflow(PPR);    
  timer_3.setCount(0);         
  timer_3.setEdgeCounting(TIMER_SMCR_SMS_ENCODER3); 
  timer_3.resume();                

  timer_2.setMode(2, TIMER_PWM);
  timer_2.setPrescaleFactor(71);
  timer_2.setOverflow(PPR2);
  timer_2.setCount(0);


  pwmWrite(2, 0); 

  myPID.SetOutputLimits(0, PPR2);
  myPID.SetMode(AUTOMATIC);

  delay(1000);

  timer_2.resume();
}

unsigned long interval=0; 

void loop() {
  if (millis() - interval >= 30) { 
     interval = millis();
     
     CurrentEncoder = timer_3.getCount();
     if(CurrentEncoder < LastEncoder){
        Input = PPR - LastEncoder + CurrentEncoder;
     }else{
        Input = CurrentEncoder - LastEncoder;
     }
    //  if(Input > 100){
      //  digitalWrite(32, !digitalRead(32));
      //  snprintf(buf, sizeof(buf), "cur:%d last:%d Input:%d\n", CurrentEncoder, LastEncoder, Input);
      //  nh.loginfo(buf);
    //  }
     LastEncoder = CurrentEncoder;

     myPID.Compute();
     
     pwmWrite(2, Output);

    rqt_var.x = Input*30;
    rqt_var.y = Output;
    rqt_var.z = Setpoint*30;
    pub.publish(&rqt_var);
    nh.spinOnce();
  }
}
