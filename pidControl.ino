#include "Arduino.h"
#include "HardwareTimer.h"

#include "ros.h"
#include "rosserial_arduino/Adc.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Twist.h"
#include "string.h"

#include "global.h"
#include "odom.h"

void pidSetCb(const geometry_msgs::Vector3 &ctlMsg);
void setPointCb(const std_msgs::String &val);
void cmdVelCb(const geometry_msgs::Twist &twist);

moveUnit motorL(MOTOR_L_TIMER, MOTOR_L_FORWARD_PIN, MOTOR_L_BACKWARD_PIN, ENCODER_L_TIMER, 5, 1, 0, 30);
moveUnit motorR(MOTOR_R_TIMER, MOTOR_R_FORWARD_PIN, MOTOR_R_BACKWARD_PIN, ENCODER_R_TIMER, 5, 1, 0, 30);

geometry_msgs::Vector3 displayValue;

ros::NodeHandle nh;
ros::Publisher pubValue("pubValueDisplay", &displayValue);

std_msgs::String setPointVal;
ros::Subscriber<std_msgs::String> subSetPoint("/subSetPoint", &setPointCb);
ros::Subscriber<geometry_msgs::Vector3> subPIDSet("/subPIDSet", &pidSetCb);
ros::Subscriber<geometry_msgs::Twist> subTwist("/turtle1/cmd_vel", &cmdVelCb);

char buf[64]="Arduino init ok";
uint32_t moveTime=0;
uint8_t twistSetFlag=0;

void setPointCb(const std_msgs::String &val){
  int setPoint = atoi(val.data);
  motorL.setSetPoint(setPoint);
  motorR.setSetPoint(setPoint);

  memset(buf, 0, sizeof(buf));
  snprintf(buf, sizeof(buf), "new setPoint：%d", setPoint);
  nh.loginfo(buf);
  nh.spinOnce();
}

void pidSetCb(const geometry_msgs::Vector3 &ctlMsg){
  memset(buf, 0, sizeof(buf));
  snprintf(buf, sizeof(buf), "kp:%.2f ki:%.2f kd:%.2f", ctlMsg.x, ctlMsg.y, ctlMsg.z);
  nh.loginfo(buf);
  nh.spinOnce();

  motorL.moveUnitSetTuning(ctlMsg.x, ctlMsg.y, ctlMsg.z);
  motorR.moveUnitSetTuning(ctlMsg.x, ctlMsg.y, ctlMsg.z);
}

void cmdVelCb(const geometry_msgs::Twist &twist){
  double tmp=0;

  moveTime = millis();
  twistSetFlag = 1;

  if(twist.linear.x){
    tmp = SPEED_K * twist.linear.x;
    motorL.setSetPoint(tmp);
    motorR.setSetPoint(tmp);

    memset(buf, 0, sizeof(buf));
    snprintf(buf, sizeof(buf), "linear speed %lf", tmp);
    nh.loginfo(buf);
  }else if(twist.angular.z){
    tmp = SPEED_K * twist.angular.z * WHEEL_RADIUS_M;
    motorL.setSetPoint(tmp);
    motorR.setSetPoint(-tmp);   

    memset(buf, 0, sizeof(buf));
    snprintf(buf, sizeof(buf), "angular speed %lf", tmp);
    nh.loginfo(buf);
  }
}

void setup() {
  nh.initNode();
  nh.advertise(pubValue);
  nh.subscribe(subPIDSet);
  nh.subscribe(subSetPoint);
  nh.subscribe(subTwist);
}

unsigned long interval=0; 

void loop() {
  if (millis() - interval >= SAMPLE_TIME) { 
    interval = millis();

    if((twistSetFlag) && (millis() - moveTime > STEP_TIME)){
      twistSetFlag = 0;
      motorL.setSetPoint(0);
      motorR.setSetPoint(0);
    }
    
    motorL.moveUnitCompute();
    motorR.moveUnitCompute();

    displayValue.x = motorL.getInput();
    displayValue.y = motorR.getInput();
    displayValue.z = motorL.getSetPoint();
    pubValue.publish(&displayValue);
    nh.spinOnce();
  }
}
