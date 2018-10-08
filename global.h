#ifndef _GLOBAL_H_
#define _GLOBAL_H_

#define SAMPLE_TIME 30
#define STEP_TIME   500

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

#define UNCONTROL_ENCODER_SPEED_THERSHOLD  1000

#define WHEEL_RADIUS_M  0.021
#define WHEEL_WIDTH_M   0.15

#define TICKS_ENCODER_CYCLE 720

#define SPEED_K 8.185   //163.7/1000*5


#endif
