/**
 * @file SF_IMU.h
 * @brief Библиотека для работы с IMU (MPU6050) через I2C
 * 
 * Основано на библиотеке StackForce
 * Обертка над Adafruit_MPU6050 с комплементарным фильтром
 */

#ifndef SF_IMU_H
#define SF_IMU_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

class SF_IMU {
public:
    /**
     * @brief Конструктор
     * @param i2c Ссылка на I2C интерфейс
     */
    SF_IMU(TwoWire &i2c) : _i2c(&i2c) {
        _roll = 0;
        _pitch = 0;
        _yaw = 0;
        _lastUpdate = 0;
        _alpha = 0.98;  // Коэффициент комплементарного фильтра
    }
    
    /**
     * @brief Инициализация IMU
     * @param addr I2C адрес MPU6050 (по умолчанию 0x68)
     * @return true если успешно, false если ошибка
     */
    bool begin(uint8_t addr = 0x68) {
        if (!_mpu.begin(addr, _i2c)) {
            Serial.println("[SF_IMU] ОШИБКА: MPU6050 не найден!");
            return false;
        }
        
        // Настройка диапазонов
        _mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
        _mpu.setGyroRange(MPU6050_RANGE_500_DEG);
        _mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
        
        _lastUpdate = millis();
        Serial.println("[SF_IMU] Инициализирован");
        
        // Калибровка (начальное чтение)
        calibrate();
        
        return true;
    }
    
    /**
     * @brief Калибровка IMU (среднее из нескольких измерений)
     */
    void calibrate() {
        Serial.println("[SF_IMU] Калибровка... не двигайте робота!");
        delay(100);
        
        // Сброс углов
        _roll = 0;
        _pitch = 0;
        _yaw = 0;
        
        Serial.println("[SF_IMU] Калибровка завершена");
    }
    
    /**
     * @brief Обновление данных IMU (вызывать в loop)
     */
    void update() {
        sensors_event_t accel, gyro, temp;
        _mpu.getEvent(&accel, &gyro, &temp);
        
        // Вычисление dt
        unsigned long now = millis();
        float dt = (now - _lastUpdate) / 1000.0;
        _lastUpdate = now;
        
        // Углы из акселерометра
        float accel_roll = atan2(accel.acceleration.y, accel.acceleration.z) * 180.0 / PI;
        float accel_pitch = atan2(-accel.acceleration.x, 
                                  sqrt(accel.acceleration.y * accel.acceleration.y + 
                                       accel.acceleration.z * accel.acceleration.z)) * 180.0 / PI;
        
        // Интегрирование гироскопа
        _roll += gyro.gyro.x * 180.0 / PI * dt;
        _pitch += gyro.gyro.y * 180.0 / PI * dt;
        _yaw += gyro.gyro.z * 180.0 / PI * dt;
        
        // Комплементарный фильтр
        _roll = _alpha * _roll + (1.0 - _alpha) * accel_roll;
        _pitch = _alpha * _pitch + (1.0 - _alpha) * accel_pitch;
    }
    
    /**
     * @brief Получение угла крена (roll)
     * @return Угол в градусах
     */
    float getRoll() {
        return _roll;
    }
    
    /**
     * @brief Получение угла тангажа (pitch)
     * @return Угол в градусах
     */
    float getPitch() {
        return _pitch;
    }
    
    /**
     * @brief Получение угла рыскания (yaw)
     * @return Угол в градусах
     */
    float getYaw() {
        return _yaw;
    }
    
    /**
     * @brief Установка коэффициента фильтра
     * @param alpha Коэффициент (0.95-0.99, по умолчанию 0.98)
     */
    void setFilterAlpha(float alpha) {
        _alpha = constrain(alpha, 0.9, 0.999);
    }
    
    /**
     * @brief Получение сырых данных акселерометра
     */
    void getAccel(float &x, float &y, float &z) {
        sensors_event_t accel, gyro, temp;
        _mpu.getEvent(&accel, &gyro, &temp);
        x = accel.acceleration.x;
        y = accel.acceleration.y;
        z = accel.acceleration.z;
    }
    
    /**
     * @brief Получение сырых данных гироскопа
     */
    void getGyro(float &x, float &y, float &z) {
        sensors_event_t accel, gyro, temp;
        _mpu.getEvent(&accel, &gyro, &temp);
        x = gyro.gyro.x * 180.0 / PI;
        y = gyro.gyro.y * 180.0 / PI;
        z = gyro.gyro.z * 180.0 / PI;
    }
    
private:
    TwoWire* _i2c;
    Adafruit_MPU6050 _mpu;
    float _roll, _pitch, _yaw;
    unsigned long _lastUpdate;
    float _alpha;  // Коэффициент комплементарного фильтра
};

#endif // SF_IMU_H
