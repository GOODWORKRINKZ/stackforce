/**
 * @file config.h
 * @brief Конфигурация главного контроллера ESP32
 * 
 * Главный ESP32 управляет:
 * - BLDC моторами (4 шт) через CAN шину
 * - Сервоприводами ног (8 шт) через PWM
 * - Получает команды от второго ESP32 по CAN шине
 */

#ifndef CONFIG_H
#define CONFIG_H

// ==================== CAN ШИНА ====================

// CAN для связи с моторами и вторым ESP32
#define CAN_TX_PIN 5
#define CAN_RX_PIN 4
#define CAN_SPEED CAN_SPEED_1MBPS  // 1 Мбит/с для BLDC моторов

// CAN ID для BLDC моторов (StackForce протокол)
#define CAN_ID_MOTOR_FL 0x141  // Передний левый мотор
#define CAN_ID_MOTOR_FR 0x142  // Передний правый мотор
#define CAN_ID_MOTOR_BL 0x143  // Задний левый мотор
#define CAN_ID_MOTOR_BR 0x144  // Задний правый мотор

// CAN ID для команд от PPM контроллера
#define CAN_ID_CONTROL_CMD 0x100   // Команды управления
#define CAN_ID_IMU_DATA 0x101      // Данные IMU
#define CAN_ID_TELEMETRY 0x102     // Телеметрия главного контроллера

// ==================== СЕРВОПРИВОДЫ ====================

// I2C для PCA9685 (драйвер сервоприводов)
#define SERVO_SDA_PIN 21
#define SERVO_SCL_PIN 22
#define PCA9685_ADDRESS 0x40
#define SERVO_FREQUENCY 50  // 50 Hz для стандартных серво

// Каналы PCA9685 для сервоприводов
#define SERVO_FL_SHOULDER 0   // Передняя левая нога - плечо
#define SERVO_FL_HIP 1        // Передняя левая нога - бедро
#define SERVO_FR_SHOULDER 2   // Передняя правая нога - плечо
#define SERVO_FR_HIP 3        // Передняя правая нога - бедро
#define SERVO_BL_SHOULDER 4   // Задняя левая нога - плечо
#define SERVO_BL_HIP 5        // Задняя левая нога - бедро
#define SERVO_BR_SHOULDER 6   // Задняя правая нога - плечо
#define SERVO_BR_HIP 7        // Задняя правая нога - бедро

// Углы сервоприводов (в градусах)
#define SERVO_MIN_ANGLE 0
#define SERVO_MAX_ANGLE 180
#define SERVO_CENTER_ANGLE 90
#define SERVO_MIN_PULSE 150   // Минимальная ширина импульса (мкс)
#define SERVO_MAX_PULSE 600   // Максимальная ширина импульса (мкс)

// ==================== BLDC МОТОРЫ ====================

// Параметры управления BLDC моторами
#define MOTOR_MAX_SPEED 3000      // Максимальная скорость (RPM)
#define MOTOR_MIN_SPEED -3000     // Минимальная скорость (RPM)
#define MOTOR_ACCEL_LIMIT 1000    // Ускорение (RPM/s)

// Команды управления BLDC (StackForce протокол)
#define MOTOR_CMD_SET_SPEED 0x01
#define MOTOR_CMD_SET_CURRENT 0x02
#define MOTOR_CMD_SET_POSITION 0x03
#define MOTOR_CMD_STOP 0x04
#define MOTOR_CMD_GET_STATUS 0x05

// ==================== ИНДИКАЦИЯ ====================

#define LED_STATUS_PIN 2      // Светодиод статуса
#define LED_CAN_PIN 15        // Активность CAN

// ==================== ПАРАМЕТРЫ УПРАВЛЕНИЯ ====================

// Частота основного цикла управления
#define CONTROL_LOOP_FREQ 50  // 50 Hz

#endif // CONFIG_H
