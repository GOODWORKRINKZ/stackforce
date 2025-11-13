#include <Arduino.h>
#include "SF_SERVO.h"
#include "gait.h"
#include "kinematics.h"
#include "config.h"
#include "SF_IMU.h"
#include "pid.h"
SF_Servo servos = SF_Servo(Wire);
extern Node LeftFront;
extern Node LeftBack;
extern Node RightFront;
extern Node RightBack;
extern uint16_t servoFrontLeftFront, servoFrontLeftRear, servoFrontRightFront, servoFrontRightRear;
extern uint16_t servoBackLeftFront, servoBackLeftRear, servoBackRightFront, servoBackRightRear;
extern float t;
SF_IMU mpu6050 = SF_IMU(Wire);
float roll, pitch, yaw, gyroX, gyroY, gyroZ;
float lpf_roll, lpf_pitch, lpf_gyroX, lpf_gyroY;
extern StableHeight StableHeightAdjust;
float lowPassFilter(float currentValue, float previousValue, float alpha)
{
  return alpha * currentValue + (1 - alpha) * previousValue;
}
PIDController pitchPid = PIDController(6.5,-0.0016,0,10000,50);
PIDController rollPid = PIDController(-1.5,0.0018,0,10000,50);
float targetGyroY, targetGyroX;
void IMUTask(void *pvParameters)// 陀螺仪数据读取
{
  while (1)
  {
    mpu6050.update();
    mpu6050.getMpu6050Value(&pitch, &roll, &yaw, &gyroX, &gyroY, &gyroZ, OFFSET_PITCH, OFFSET_ROLL);
    lpf_pitch = lowPassFilter(pitch,lpf_pitch,0.03);
    lpf_roll = lowPassFilter(roll,lpf_roll,0.03);
    lpf_gyroX = lowPassFilter(gyroX,lpf_gyroX,0.03);
    lpf_gyroY = lowPassFilter(gyroY,lpf_gyroY,0.03);
  }
}
void openThread(void) 
{
    // 陀螺仪读取任务进程
    xTaskCreatePinnedToCore(
                              IMUTask,   // 任务函数
                              "IMUTask", // 任务名称
                              4096,      // 堆栈大小
                              NULL,      // 传递的参数
                              1,         // 任务优先级
                              NULL,      // 任务句柄
                              0          // 运行在核心 0
                            );
}

void setup() 
{
    Serial.begin(115200);//debug
    Wire.begin(1, 2, 400000UL);
    servos.init();
    servos.setAngleRange(0, 300);
    servos.setPluseRange(500, 2500);   

    gaitMode(STAND, 0, 0, 0);
    inverseKinematics(LeftFront, RightFront,LeftBack, RightBack);
    servos.setEightServoAngle(  3, servoFrontLeftFront, SERVO_OFFSET_3, 
                                4, servoFrontLeftRear,  SERVO_OFFSET_4, 
                                2, servoFrontRightFront,SERVO_OFFSET_2, 
                                1, servoFrontRightRear, SERVO_OFFSET_1, 
                                7, servoBackLeftFront,  SERVO_OFFSET_7,
                                8, servoBackLeftRear,   SERVO_OFFSET_8,
                                6, servoBackRightFront, SERVO_OFFSET_6,
                                5, servoBackRightRear,  SERVO_OFFSET_5);
    delay(5000);

    mpu6050.init();
    mpu6050.calGyroOffsets();
    Serial.println("初始化完成");

    openThread();
    delay(5000);
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
    if(STABLE_SWITCH == 0)
    {
        StableHeightAdjust.pitch = 0;
        StableHeightAdjust.roll = 0;
    }
    else if(STABLE_SWITCH == 1)
    {
        targetGyroY = (0 - lpf_roll ) * rollPid.P;
        StableHeightAdjust.roll = StableHeightAdjust.roll - rollPid.I * (targetGyroY - gyroY);
        StableHeightAdjust.roll = constrainValue(StableHeightAdjust.roll, -90, 90);
        
        targetGyroX = (0 - lpf_pitch ) * pitchPid.P;
        StableHeightAdjust.pitch = StableHeightAdjust.pitch - pitchPid.I * (targetGyroX - gyroX);
        StableHeightAdjust.pitch = constrainValue(StableHeightAdjust.pitch, -90, 90);
    }
    gaitMode(TROT, 0.005, 50, 50);
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
