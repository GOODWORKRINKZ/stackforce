/**
 * @file config.h
 * @brief Конфигурация вспомогательного (aux) контроллера
 * 
 * Aux контроллер отвечает за:
 * - Управление ТОЛЬКО задними 2 BLDC моторами (M2, M3)
 * - Прием команд от main контроллера по CAN
 * - Все сервоприводы управляются main контроллером
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <math.h>

// ==================== CAN ШИНА ====================
// Используется CAN/RS485 модуль StackForce
// CANTX -> IO35, CANRX -> IO41

#define CAN_TX_PIN 35
#define CAN_RX_PIN 41
#define CAN_SPEED CAN_SPEED_1MBPS

// CAN ID сообщений
#define CAN_ID_MAIN_TO_AUX 0x100     // Команды от main к aux
#define CAN_ID_AUX_TO_MAIN 0x101     // Телеметрия от aux к main

// ==================== BLDC МОТОРЫ ====================
// Задние моторы M2, M3 подключены через Serial2

#define M2Dir 1                       // Направление мотора M2 (задний левый)
#define M3Dir -1                      // Направление мотора M3 (задний правый)

// ==================== ПАРАМЕТРЫ УПРАВЛЕНИЯ ====================

#define MOTOR_MAX_SPEED 3000
#define MOTOR_MIN_SPEED -3000
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
