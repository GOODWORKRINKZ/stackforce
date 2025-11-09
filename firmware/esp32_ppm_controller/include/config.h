/**
 * @file config.h
 * @brief Конфигурация для ESP32 PPM контроллера StackForce робота
 * 
 * ESP32 отвечает за:
 * - Прием PPM сигнала от RC приемника
 * - Чтение данных IMU (MPU6050)
 * - Фильтрация IMU (комплементарный фильтр)
 * - Отправку команд управления по CAN шине к StackForce STM32 контроллеру
 */

#ifndef CONFIG_H
#define CONFIG_H

// ==================== PPM КОНФИГУРАЦИЯ ====================

// Пин для приема PPM сигнала
#define PPM_INPUT_PIN 34

// PPM параметры
#define PPM_CHANNELS 8
#define PPM_FRAME_LENGTH 20000  // микросекунды
#define PPM_PULSE_LENGTH 300    // микросекунды
#define PPM_MIN_CHANNEL_VALUE 1000  // микросекунды
#define PPM_MAX_CHANNEL_VALUE 2000  // микросекунды

// Маппинг каналов PPM на функции
#define PPM_CHANNEL_THROTTLE 0    // Газ (движение вперед/назад)
#define PPM_CHANNEL_STEERING 1    // Рулевое управление (поворот)
#define PPM_CHANNEL_MODE 2        // Режим работы (стойка/присед)
#define PPM_CHANNEL_SPEED 3       // Регулировка скорости
#define PPM_CHANNEL_AUX1 4        // Дополнительный канал 1
#define PPM_CHANNEL_AUX2 5        // Дополнительный канал 2

// ==================== CAN ШИНА ====================

// CAN шина для связи с StackForce STM32 контроллером
#define CAN_TX_PIN 5
#define CAN_RX_PIN 4
#define CAN_SPEED CAN_SPEED_1MBPS  // StackForce использует 1Mbps

// CAN ID для сообщений (протокол StackForce)
#define CAN_ID_CONTROL_CMD 0x100   // Команды управления от ESP32
#define CAN_ID_IMU_DATA 0x101      // Данные IMU от ESP32
#define CAN_ID_STATUS 0x102        // Статус ESP32
#define CAN_ID_MOTOR_CMD 0x200     // Команды для моторов (от ESP32 к STM32)

// ==================== IMU (MPU6050) ====================

// I2C для IMU
#define IMU_SDA_PIN 21
#define IMU_SCL_PIN 22
#define IMU_I2C_FREQ 400000  // 400kHz

// Параметры комплементарного фильтра
#define IMU_FILTER_ALPHA 0.98  // Коэффициент фильтра (0.96-0.99)
#define IMU_UPDATE_RATE 100    // Частота обновления IMU (Hz)

// ==================== ИНДИКАЦИЯ ====================

// LED индикация
#define LED_STATUS_PIN 2      // Статус ESP32
#define LED_CAN_PIN 15        // Активность CAN
#define LED_PPM_PIN 16        // Прием PPM сигнала
#define LED_IMU_PIN 17        // Статус IMU

// ==================== БЕЗОПАСНОСТЬ ====================

// Таймаут потери PPM сигнала (мс)
#define PPM_TIMEOUT 500

// Дедзона для джойстиков
#define JOYSTICK_DEADZONE 5

#endif // CONFIG_H
