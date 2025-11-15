#include "SF_IMU.h"

SF_IMU::SF_IMU(TwoWire &i2c)
    :_i2c(&i2c),_accCoef(0.03f),_gyroCoef(0.97f)  {}

void SF_IMU::init(){
    writeToIMU(MPU6050_SMPLRT_DIV, 0x00);
    writeToIMU(MPU6050_CONFIG, 0x00);
    writeToIMU(MPU6050_GYRO_CONFIG, 0x08);
    writeToIMU(MPU6050_ACCEL_CONFIG, 0x00);
    writeToIMU(MPU6050_PWR_MGMT_1, 0x01);
    calGyroOffsets();
    update();
    _preInterval = millis();
    angleGyro[0] = 0;
    angleGyro[1] = 0;
    angle[0] = angleAcc[0];
    angle[1] = angleAcc[1];
}

void SF_IMU::calGyroOffsets(){
    float x = 0, y = 0, z = 0;
    int16_t rx, ry, rz;

    delay(1000);
    Serial.println();
    Serial.println("========================================");
    Serial.println("Calculating gyro offsets");
    Serial.print("DO NOT MOVE MPU6050");

    for (int i = 0; i < 3000; i++) {
        if (i % 1000 == 0) Serial.print(".");
        
        _i2c->beginTransmission(MPU6050_ADDR);
        _i2c->write(0x43);
        _i2c->endTransmission(false);
        _i2c->requestFrom((int)MPU6050_ADDR, 6);

        rx = _i2c->read() << 8 | _i2c->read();
        ry = _i2c->read() << 8 | _i2c->read();
        rz = _i2c->read() << 8 | _i2c->read();

        x += ((float)rx) / 65.5;
        y += ((float)ry) / 65.5;
        z += ((float)rz) / 65.5;
    }

    _gyroOffset[0] = x / 3000;
    _gyroOffset[1] = y / 3000;
    _gyroOffset[2] = z / 3000;

    Serial.println("Done!");
    Serial.printf("%f,%f,%f\n",_gyroOffset[0],_gyroOffset[1],_gyroOffset[2]);
    Serial.print("========================================");

    delay(1000);
}

void SF_IMU::update(){
    _i2c->beginTransmission(MPU6050_ADDR);
    _i2c->write(0x3B);
    _i2c->endTransmission(false);
    _i2c->requestFrom((int)MPU6050_ADDR, 14);

    int16_t rawAccX = _i2c->read() << 8 | _i2c->read();
    int16_t rawAccY = _i2c->read() << 8 | _i2c->read();
    int16_t rawAccZ = _i2c->read() << 8 | _i2c->read();
    int16_t rawTemp = _i2c->read() << 8 | _i2c->read();
    int16_t rawGyroX = _i2c->read() << 8 | _i2c->read();
    int16_t rawGyroY = _i2c->read() << 8 | _i2c->read();
    int16_t rawGyroZ = _i2c->read() << 8 | _i2c->read();

    temp = (rawTemp + 12412.0) / 340.0;

    acc[0] = ((float)rawAccX) / 16384.0;
    acc[1] = ((float)rawAccY) / 16384.0;
    acc[2] = ((float)rawAccZ) / 16384.0;

    angleAcc[0] = atan2(acc[1], acc[2] + abs(acc[0])) * 360 / 2.0 / M_PI;
    angleAcc[1] = atan2(acc[0], acc[2] + abs(acc[1])) * 360 / -2.0 / M_PI;

    gyro[0] = ((float)rawGyroX) / 65.5 - _gyroOffset[0];
    gyro[1] = ((float)rawGyroY) / 65.5 - _gyroOffset[1];
    gyro[2] = ((float)rawGyroZ) / 65.5 - _gyroOffset[2];

    _interval = (millis() - _preInterval) * 0.001;

    angleGyro[0] += gyro[0] * _interval;
    angleGyro[1] += gyro[1] * _interval;
    angleGyro[2] += gyro[2] * _interval;

    angle[0] = (_gyroCoef * (angle[0] + gyro[0] * _interval)) + (_accCoef * angleAcc[0]);
    angle[1] = (_gyroCoef * (angle[1] + gyro[1] * _interval)) + (_accCoef * angleAcc[1]);
    angle[2] = angleGyro[2];

    _preInterval = millis();
}

void SF_IMU::writeToIMU(uint8_t addr, uint8_t data){
    _i2c->beginTransmission(MPU6050_ADDR);
    _i2c->write(addr);
    _i2c->write(data);
    _i2c->endTransmission();
}

uint8_t SF_IMU::readFromIMU(uint8_t addr){
    _i2c->beginTransmission(MPU6050_ADDR);
    _i2c->write(addr);
    _i2c->endTransmission();

    _i2c->requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)1);
    return _i2c->read();
}

void SF_IMU::setCoef(float accCoef, float gyroCoef){
    _accCoef = accCoef;
    _gyroCoef = gyroCoef;
}

void SF_IMU::setGyroOffsets(float x, float y, float z){
    _gyroOffset[0] = x;
    _gyroOffset[1] = y;
    _gyroOffset[2] = z;
}
