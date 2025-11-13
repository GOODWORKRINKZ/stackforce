#include <Arduino.h>
#include "SF_SERVO.h"
#include "gait.h"
#include "kinematics.h"
#include "config.h"
SF_Servo servos = SF_Servo(Wire);
extern Node LeftFront;
extern Node LeftBack;
extern Node RightFront;
extern Node RightBack;
extern uint16_t servoFrontLeftFront, servoFrontLeftRear, servoFrontRightFront, servoFrontRightRear;
extern uint16_t servoBackLeftFront, servoBackLeftRear, servoBackRightFront, servoBackRightRear;
extern float t;
void setup() 
{
    Wire.begin(1, 2, 400000UL);
    servos.init();
    servos.setAngleRange(0, 300);
    servos.setPluseRange(500, 2500);
}
/*
mode : TROT / PACE / BOUND / STAND
freq: 0 ~ 0.01
height: 0 ~ 60
stride: 50 ~ -50
*/
void loop()
{
    if (t >= Ts) t = 0;
    gaitMode(PACE, 0.003, 30, 50);
    inverseKinematics(LeftFront, RightFront,LeftBack, RightBack);
    servos.setEightServoAngle(  3, servoFrontLeftFront, SERVO_OFFSET_3, 
                                4, servoFrontLeftRear,  SERVO_OFFSET_4, 
                                2, servoFrontRightFront,SERVO_OFFSET_2, 
                                1, servoFrontRightRear, SERVO_OFFSET_1, 
                                7, servoBackLeftFront,  SERVO_OFFSET_7,
                                8, servoBackLeftRear,   SERVO_OFFSET_8,
                                6, servoBackRightFront, SERVO_OFFSET_6,
                                5, servoBackRightRear,  SERVO_OFFSET_5);
}
