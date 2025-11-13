#ifndef _GAIT_H
#define _GAIT_H

#include <Arduino.h>

#define pi      3.1415926   //π
#define faai    0.5         //占空比和周期  此占空比的指的是摆动相占周期的时间
#define Ts      1 

#define HEIGHT_MAX     130
#define BASE_HEIGHT    110
#define X_ZERO         20

#define STABLE_SWITCH    1

typedef struct 
{
    float x;
    float y;
} Node;


typedef struct 
{
  float xSwing;    // 前半周期生成的摆动相的x坐标
  float ySwing;    // 前半周期生成的摆动相的y坐标
  float xSupport;  // 前半周期生成的支撑相的x坐标
  float ySupport;  // 前半周期生成的支撑相的x坐标
  float sigma;     // 轨迹生成三角函数中的相位
}GaitPhasesState;

typedef struct
{
  float pitch;//mpu pitch作用
  float roll;//mpu  roll作用
}StableHeight;

typedef enum
{
  TROT = 1,
  PACE,
  BOUND,
  STAND,
}MODE_SET;
#define constrainValue(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt))) // 限幅函数
void gaitMode(MODE_SET mode, float freq, uint8_t height, int8_t stride);
#endif