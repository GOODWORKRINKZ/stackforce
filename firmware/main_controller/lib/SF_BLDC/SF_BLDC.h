/**
 * @file SF_BLDC.h
 * @brief Библиотека для управления BLDC моторами через Serial (UART)
 * 
 * Основано на библиотеке StackForce
 * Управление BLDC драйверами через последовательный порт
 */

#ifndef SF_BLDC_H
#define SF_BLDC_H

#include <Arduino.h>

class SF_BLDC {
public:
    /**
     * @brief Конструктор
     * @param serial Ссылка на Serial порт (обычно Serial2)
     */
    SF_BLDC(HardwareSerial &serial) : _serial(&serial) {
        _speed_left = 0;
        _speed_right = 0;
    }
    
    /**
     * @brief Инициализация BLDC драйвера
     * @param baudrate Скорость UART (по умолчанию 115200)
     */
    void begin(uint32_t baudrate = 115200) {
        _serial->begin(baudrate);
        Serial.println("[SF_BLDC] Инициализирован");
    }
    
    /**
     * @brief Установка скорости левого мотора
     * @param speed Скорость -100 до 100
     */
    void setSpeedLeft(int16_t speed) {
        _speed_left = constrain(speed, -100, 100);
        sendCommand();
    }
    
    /**
     * @brief Установка скорости правого мотора
     * @param speed Скорость -100 до 100
     */
    void setSpeedRight(int16_t speed) {
        _speed_right = constrain(speed, -100, 100);
        sendCommand();
    }
    
    /**
     * @brief Установка скоростей обоих моторов
     * @param left Скорость левого мотора
     * @param right Скорость правого мотора
     */
    void setSpeeds(int16_t left, int16_t right) {
        _speed_left = constrain(left, -100, 100);
        _speed_right = constrain(right, -100, 100);
        sendCommand();
    }
    
    /**
     * @brief Остановка моторов
     */
    void stop() {
        _speed_left = 0;
        _speed_right = 0;
        sendCommand();
    }
    
    /**
     * @brief Получение текущей скорости левого мотора
     */
    int16_t getSpeedLeft() {
        return _speed_left;
    }
    
    /**
     * @brief Получение текущей скорости правого мотора
     */
    int16_t getSpeedRight() {
        return _speed_right;
    }
    
private:
    HardwareSerial* _serial;
    int16_t _speed_left;
    int16_t _speed_right;
    
    /**
     * @brief Отправка команды драйверу
     * Формат: <L:speed_left,R:speed_right>\n
     */
    void sendCommand() {
        char buffer[32];
        snprintf(buffer, sizeof(buffer), "L:%d,R:%d\n", _speed_left, _speed_right);
        _serial->print(buffer);
    }
};

#endif // SF_BLDC_H
