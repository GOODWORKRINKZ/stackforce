/**
 * @file config.h
 * @brief Конфигурация для калибровки сервоприводов
 * 
 * Эта прошивка используется для определения офсетов сервоприводов.
 * Позволяет:
 * - Читать текущие позиции сервоприводов
 * - Выставить ноги робота в нужное положение
 * - Получить офсеты для корректировки в основной прошивке
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ==================== I2C УСТРОЙСТВА ====================

#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22
#define I2C_FREQ 400000

// PCA9685 для сервоприводов
#define PCA9685_ADDR 0x40
#define SERVO_FREQ 50                 // 50 Hz для серво

// ==================== СЕРВОПРИВОДЫ ====================

// Всего 8 сервоприводов (4 ноги × 2 серво на ногу)
#define NUM_SERVOS 8

// Каналы PCA9685 (КАК В ГЛАВНОМ КОДЕ!)
// Передние ноги (Front)
#define SERVO_FL_FRONT 2  // Front Left Front
#define SERVO_FL_REAR  3  // Front Left Rear
#define SERVO_FR_FRONT 1  // Front Right Front
#define SERVO_FR_REAR  0  // Front Right Rear

// Задние ноги (Back)
#define SERVO_BL_FRONT 4  // Back Left Front
#define SERVO_BL_REAR  5  // Back Left Rear (ВНИМАНИЕ: канал 8 = индекс 7 в массиве!)
#define SERVO_BR_FRONT 7  // Back Right Front
#define SERVO_BR_REAR  6  // Back Right Rear

// Имена сервоприводов для вывода (В ПОРЯДКЕ КАНАЛОВ 0-7!)
const char* SERVO_NAMES[NUM_SERVOS] = {
    "CH0",          // 0
    "FR_REAR",      // 1
    "FR_FRONT",     // 2
    "FL_FRONT",     // 3
    "FL_REAR",      // 4
    "BR_REAR",      // 5
    "BR_FRONT",     // 6
    "BL_FRONT"      // 7
};

// ==================== КАЛИБРОВКА ====================

// Диапазон углов серво (градусы)
#define SERVO_ANGLE_MIN 0
#define SERVO_ANGLE_MAX 180

// Диапазон импульсов серво (микросекунды)
#define SERVO_PULSE_MIN 150           // Минимальная ширина импульса
#define SERVO_PULSE_MAX 600           // Максимальная ширина импульса

// Нейтральная позиция (90 градусов)
#define SERVO_NEUTRAL_ANGLE 90

// ==================== SERIAL КОМАНДЫ ====================

#define SERIAL_BAUD_RATE 115200

// Интервал обновления вывода (мс)
#define UPDATE_INTERVAL 100           // Вывод каждые 100 мс
#define CONTINUOUS_OUTPUT true        // Непрерывный вывод офсетов

// ==================== ИНДИКАЦИЯ ====================

#define LED_STATUS_PIN 2              // Статусный светодиод

#endif // CONFIG_H
