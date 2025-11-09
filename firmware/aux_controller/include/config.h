/**
 * @file config.h
 * @brief Конфигурация вспомогательного (aux) контроллера
 * 
 * Aux контроллер отвечает за:
 * - Управление задними 2 BLDC моторами (колеса)
 * - Управление задними 4 сервоприводами (2 ноги)
 * - Прием команд от main контроллера по CAN
 * - Синхронизация движений с main контроллером
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <math.h>

// ==================== CAN ШИНА ====================

#define CAN_TX_PIN 5
#define CAN_RX_PIN 4
#define CAN_SPEED CAN_SPEED_1MBPS

// CAN ID сообщений
#define CAN_ID_MAIN_TO_AUX 0x110     // Команды от main к aux
#define CAN_ID_AUX_TO_MAIN 0x120     // Телеметрия от aux к main
#define CAN_ID_MOTOR_BL 0x143         // Задний левый BLDC
#define CAN_ID_MOTOR_BR 0x144         // Задний правый BLDC

// ==================== I2C УСТРОЙСТВА ====================

#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22
#define I2C_FREQ 400000

// PCA9685 для сервоприводов задних ног
#define PCA9685_ADDR 0x40
#define SERVO_FREQ 50

// Каналы PCA9685 для задних ног
#define SERVO_BL_SHOULDER 0           // Задняя левая - плечо
#define SERVO_BL_HIP 1                // Задняя левая - бедро
#define SERVO_BR_SHOULDER 2           // Задняя правая - плечо
#define SERVO_BR_HIP 3                // Задняя правая - бедро

// ==================== ПАРАМЕТРЫ УПРАВЛЕНИЯ ====================

#define MOTOR_MAX_SPEED 3000
#define MOTOR_MIN_SPEED -3000
#define SERVO_MIN_PULSE 150
#define SERVO_MAX_PULSE 600

#define CONTROL_LOOP_FREQ 50

// ==================== ИНДИКАЦИЯ ====================

#define LED_STATUS_PIN 2              // Статус aux
#define LED_CAN_PIN 15                // Активность CAN

// ==================== ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ ====================

inline float clamp(float value, float min_val, float max_val) {
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

#endif // CONFIG_H
