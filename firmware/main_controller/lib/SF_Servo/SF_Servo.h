/**
 * @file SF_Servo.h
 * @brief Библиотека для управления сервоприводами через I2C (PCA9685)
 * 
 * Основано на библиотеке StackForce
 * Обертка над Adafruit_PWMServoDriver
 */

#ifndef SF_SERVO_H
#define SF_SERVO_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

class SF_Servo {
public:
    /**
     * @brief Конструктор
     * @param i2c Ссылка на I2C интерфейс
     * @param addr I2C адрес PCA9685 (по умолчанию 0x40)
     */
    SF_Servo(TwoWire &i2c, uint8_t addr = 0x40) : _i2c(&i2c), _addr(addr) {
        _pwm = new Adafruit_PWMServoDriver(addr, i2c);
    }
    
    /**
     * @brief Деструктор
     */
    ~SF_Servo() {
        delete _pwm;
    }
    
    /**
     * @brief Инициализация PCA9685
     * @param freq Частота PWM (по умолчанию 50 Hz для серво)
     */
    void begin(uint16_t freq = 50) {
        _pwm->begin();
        _pwm->setPWMFreq(freq);
        Serial.printf("[SF_Servo] Инициализирован на адресе 0x%02X\n", _addr);
    }
    
    /**
     * @brief Установка угла сервопривода
     * @param channel Канал PCA9685 (0-15)
     * @param angle Угол в градусах (0-180)
     * @param minPulse Минимальная ширина импульса (по умолчанию 150)
     * @param maxPulse Максимальная ширина импульса (по умолчанию 600)
     */
    void setAngle(uint8_t channel, uint8_t angle, uint16_t minPulse = 150, uint16_t maxPulse = 600) {
        angle = constrain(angle, 0, 180);
        uint16_t pulse = map(angle, 0, 180, minPulse, maxPulse);
        _pwm->setPWM(channel, 0, pulse);
    }
    
    /**
     * @brief Установка нескольких углов одновременно
     * @param channels Массив номеров каналов
     * @param angles Массив углов
     * @param count Количество элементов
     */
    void setAngles(uint8_t* channels, uint8_t* angles, uint8_t count) {
        for (uint8_t i = 0; i < count; i++) {
            setAngle(channels[i], angles[i]);
        }
    }
    
    /**
     * @brief Отключение сервопривода (установка в 0)
     * @param channel Канал PCA9685
     */
    void disable(uint8_t channel) {
        _pwm->setPWM(channel, 0, 0);
    }
    
    /**
     * @brief Отключение всех сервоприводов
     */
    void disableAll() {
        for (uint8_t i = 0; i < 16; i++) {
            disable(i);
        }
    }
    
private:
    TwoWire* _i2c;
    uint8_t _addr;
    Adafruit_PWMServoDriver* _pwm;
};

#endif // SF_SERVO_H
