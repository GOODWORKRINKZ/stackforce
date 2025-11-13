#include "gait.h"

Node LeftFront  = {0, 70};
Node LeftBack   = {0, 70};
Node RightFront = {0, 70};
Node RightBack  = {0, 70};
StableHeight StableHeightAdjust;

float t = 0;    
GaitPhasesState  gaitGenerator(bool select, uint8_t height, int8_t stride)
{
  GaitPhasesState phaseState;
  
  if(select == 0)         phaseState.sigma = 2 * pi * t / (faai * Ts);               // 前半周期轨迹生成三角函数中的相位
  else if (select == 1)   phaseState.sigma = 2 * pi * (t - faai* Ts)/ (faai * Ts);   // 前半周期轨迹生成三角函数中的相位

  //摆动xy
  phaseState.xSwing = stride * ((phaseState.sigma - sin(phaseState.sigma)) / (2 * pi)) - stride / 2;
  phaseState.ySwing = HEIGHT_MAX - height * (1 - cos(phaseState.sigma)) / 2;
  //支撑x
  phaseState.ySupport = HEIGHT_MAX;
  phaseState.xSupport = stride / 2  - stride * ((phaseState.sigma - sin(phaseState.sigma)) / (2 * pi));
  return phaseState;
}

void trot(float freq, uint8_t height, int8_t stride)
{
  t = t + freq;
  if (t <= Ts * faai)
  {
    GaitPhasesState phaseState = gaitGenerator(0, height, stride);

    LeftFront.y  = phaseState.ySwing   + STABLE_SWITCH * -StableHeightAdjust.pitch + STABLE_SWITCH * StableHeightAdjust.roll;
    LeftFront.y  = constrainValue(LeftFront.y,  70, 130);
    RightFront.y = phaseState.ySupport + STABLE_SWITCH * -StableHeightAdjust.pitch - STABLE_SWITCH * StableHeightAdjust.roll;
    RightFront.y = constrainValue(RightFront.y, 70, 130);
    LeftBack.y   = phaseState.ySupport - STABLE_SWITCH * -StableHeightAdjust.pitch + STABLE_SWITCH * StableHeightAdjust.roll;
    LeftBack.y   = constrainValue(LeftBack.y,   70, 130);
    RightBack.y  = phaseState.ySwing   - STABLE_SWITCH * -StableHeightAdjust.pitch - STABLE_SWITCH * StableHeightAdjust.roll;
    RightBack.y  = constrainValue(RightBack.y,  70, 130);
    
    LeftFront.x  =  phaseState.xSwing   + X_ZERO; 
    RightFront.x =  phaseState.xSupport + X_ZERO;
    LeftBack.x   =  phaseState.xSupport - X_ZERO;
    RightBack.x  =  phaseState.xSwing   - X_ZERO;
  }
  else if (t > Ts * faai && t <= Ts)
  {
    GaitPhasesState phaseState = gaitGenerator(1, height, stride);

    LeftFront.y  = phaseState.ySupport + STABLE_SWITCH * -StableHeightAdjust.pitch + STABLE_SWITCH * StableHeightAdjust.roll;
    LeftFront.y  = constrainValue(LeftFront.y,  70, 130);
    RightFront.y = phaseState.ySwing   + STABLE_SWITCH * -StableHeightAdjust.pitch - STABLE_SWITCH * StableHeightAdjust.roll;
    RightFront.y = constrainValue(RightFront.y, 70, 130);
    LeftBack.y   = phaseState.ySwing   - STABLE_SWITCH * -StableHeightAdjust.pitch + STABLE_SWITCH * StableHeightAdjust.roll;
    LeftBack.y   = constrainValue(LeftBack.y,   70, 130);
    RightBack.y  = phaseState.ySupport - STABLE_SWITCH * -StableHeightAdjust.pitch - STABLE_SWITCH * StableHeightAdjust.roll;
    RightBack.y  = constrainValue(RightBack.y,  70, 130);

    LeftFront.x  = phaseState.xSupport + X_ZERO;
    RightFront.x = phaseState.xSwing   + X_ZERO;
    LeftBack.x   = phaseState.xSwing   - X_ZERO;
    RightBack.x  = phaseState.xSupport - X_ZERO;
  }
}
void pace(float freq, int8_t height, int8_t stride)
{
  t = t + freq;
  if (t <= Ts * faai)
  {
    GaitPhasesState phaseState = gaitGenerator(0, height, stride);

    LeftFront.y  = phaseState.ySwing;
    LeftFront.y  = constrainValue(LeftFront.y,  70, 130);
    RightFront.y = phaseState.ySupport;
    RightFront.y = constrainValue(RightFront.y, 70, 130);
    LeftBack.y   = phaseState.ySwing;
    LeftBack.y   = constrainValue(LeftBack.y,   70, 130);
    RightBack.y  = phaseState.ySupport;
    RightBack.y  = constrainValue(RightBack.y,  70, 130);

    LeftFront.x  =  phaseState.xSwing   + X_ZERO;
    RightFront.x =  phaseState.xSupport + X_ZERO;
    LeftBack.x   =  phaseState.xSwing   - X_ZERO;
    RightBack.x  =  phaseState.xSupport - X_ZERO;
  }
  else if (t > Ts * faai && t <= Ts)
  {
    GaitPhasesState phaseState = gaitGenerator(1, height, stride);

    LeftFront.y  = phaseState.ySupport;
    LeftFront.y  = constrainValue(LeftFront.y,  70, 130);
    RightFront.y = phaseState.ySwing;
    RightFront.y = constrainValue(RightFront.y, 70, 130);
    LeftBack.y   = phaseState.ySupport; 
    LeftBack.y   = constrainValue(LeftBack.y,   70, 130);
    RightBack.y  = phaseState.ySwing;
    RightBack.y  = constrainValue(RightBack.y,  70, 130);

    LeftFront.x  =  phaseState.xSupport + X_ZERO;
    RightFront.x =  phaseState.xSwing   + X_ZERO;
    LeftBack.x   =  phaseState.xSupport - X_ZERO;
    RightBack.x  =  phaseState.xSwing   - X_ZERO;
  }
}
void bound(float freq, int8_t height, int8_t stride)
{
  t = t + freq;
  if (t <= Ts * faai)
  {
    GaitPhasesState phaseState = gaitGenerator(0, height, stride);

    LeftFront.y  = phaseState.ySwing;
    LeftFront.y  = constrainValue(LeftFront.y,  70, 130);
    RightFront.y = phaseState.ySwing;
    RightFront.y = constrainValue(RightFront.y, 70, 130);
    LeftBack.y   = phaseState.ySupport; 
    LeftBack.y   = constrainValue(LeftBack.y,   70, 130);
    RightBack.y  = phaseState.ySupport;
    RightBack.y  = constrainValue(RightBack.y,  70, 130);

    LeftFront.x  =  phaseState.xSwing   + X_ZERO;
    RightFront.x =  phaseState.xSwing   + X_ZERO;
    LeftBack.x   =  phaseState.xSupport - X_ZERO;
    RightBack.x  =  phaseState.xSupport - X_ZERO;
  }
  else if (t > Ts * faai && t <= Ts)
  {
    GaitPhasesState phaseState = gaitGenerator(1, height, stride);

    LeftFront.y  = phaseState.ySupport;
    LeftFront.y  = constrainValue(LeftFront.y,  70, 130);
    RightFront.y = phaseState.ySupport;
    RightFront.y = constrainValue(RightFront.y, 70, 130);
    LeftBack.y   = phaseState.ySwing; 
    LeftBack.y   = constrainValue(LeftBack.y,   70, 130);
    RightBack.y  = phaseState.ySwing;
    RightBack.y  = constrainValue(RightBack.y,  70, 130);

    LeftFront.x  =  phaseState.xSupport + X_ZERO;
    RightFront.x =  phaseState.xSupport + X_ZERO;
    LeftBack.x   =  phaseState.xSwing   - X_ZERO;
    RightBack.x  =  phaseState.xSwing   - X_ZERO;
  }
}
void stand(void)
{
  LeftFront.y  = BASE_HEIGHT + STABLE_SWITCH * -StableHeightAdjust.pitch + STABLE_SWITCH * StableHeightAdjust.roll;
  LeftFront.y  = constrainValue(LeftFront.y, 70, 150);
  RightFront.y = BASE_HEIGHT + STABLE_SWITCH * -StableHeightAdjust.pitch - STABLE_SWITCH * StableHeightAdjust.roll;
  RightFront.y = constrainValue(RightFront.y, 70, 130);
  LeftBack.y   = BASE_HEIGHT - STABLE_SWITCH * -StableHeightAdjust.pitch + STABLE_SWITCH * StableHeightAdjust.roll;
  LeftBack.y   = constrainValue(LeftBack.y, 70, 150);
  RightBack.y  = BASE_HEIGHT - STABLE_SWITCH * -StableHeightAdjust.pitch - STABLE_SWITCH * StableHeightAdjust.roll;
  RightBack.y  = constrainValue(RightBack.y, 70, 150);
  
  LeftFront.x = 20;
  RightFront.x = 20;
  LeftBack.x = -20;
  RightBack.x = -20;

}

void gaitMode(MODE_SET mode, float freq, uint8_t height, int8_t stride)
{
  switch (mode)
  {
    case TROT:
      trot(freq, height, stride);
      break;
    case PACE:
      pace(freq, height, stride);
      break;
    case BOUND:
      bound(freq, height, stride);
      break;
    case STAND:
      stand();
      break;
  }
}



