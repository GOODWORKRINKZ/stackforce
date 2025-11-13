#include <Arduino.h>
#include <Wire.h>
#include "SF_IMU.h"

//实例化SF_IMU类对象
SF_IMU mpu6050 = SF_IMU(Wire);
//姿态角
float roll,pitch,yaw;
//角速度
float gyroX,gyroY,gyroZ;

void setup(){
  Serial.begin(115200);
  Wire.begin(1, 2, 400000UL);
  mpu6050.init();//初始化
}


void loop() {

  mpu6050.update();//数据周期性更新
  roll = mpu6050.angle[0];
  pitch = mpu6050.angle[1];
  yaw = mpu6050.angle[2];
  gyroX = mpu6050.gyro[0];
  gyroY = mpu6050.gyro[1];
  gyroZ =mpu6050.gyro[2];
  Serial.printf("%f,%f,%f,%f,%f,%f\n",roll,pitch,yaw,gyroX,gyroY,gyroZ);
  delay(100);
}