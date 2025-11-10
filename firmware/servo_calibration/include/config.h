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

// Каналы PCA9685 для всех сервоприводов
#define SERVO_FL_SHOULDER 0           // Передняя левая - плечо
#define SERVO_FL_HIP 1                // Передняя левая - бедро
#define SERVO_FR_SHOULDER 2           // Передняя правая - плечо
#define SERVO_FR_HIP 3                // Передняя правая - бедро
#define SERVO_BL_SHOULDER 4           // Задняя левая - плечо
#define SERVO_BL_HIP 5                // Задняя левая - бедро
#define SERVO_BR_SHOULDER 6           // Задняя правая - плечо
#define SERVO_BR_HIP 7                // Задняя правая - бедро

// Имена сервоприводов для вывода
const char* SERVO_NAMES[NUM_SERVOS] = {
    "FL_SHOULDER",  // 0
    "FL_HIP",       // 1
    "FR_SHOULDER",  // 2
    "FR_HIP",       // 3
    "BL_SHOULDER",  // 4
    "BL_HIP",       // 5
    "BR_SHOULDER",  // 6
    "BR_HIP"        // 7
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
