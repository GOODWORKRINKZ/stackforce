#ifndef _KINEMATICS_H
#define _KINEMATICS_H

#include <Arduino.h>
#include "SF_SERVO.h"

#include "gait.h"

#define L1 60
#define L2 100
#define L3 100
#define L4 60
#define L5 40

typedef struct 
{
    float alphaLeft;    // 左腿α角度
    float betaLeft;     // 左腿β角度
    float alphaRight;  // 右腿α角度
    float betaRight;   // 右腿β角度
}JointAngles;

void inverseKinematics(Node LeftFront, Node RightFront, Node LeftBack, Node RightBack);

#endif