/**
 * @file config.h
 * @brief Конфигурация главного контроллера (main) четырехногого робота
 * 
 * Main контроллер отвечает за:
 * - Прием PPM сигнала от RC приемника
 * - Управление 4 BLDC моторами (колеса)
 * - Управление 8 сервоприводами (ноги: 2 серво на ногу)
 * - Чтение IMU (MPU6050)
 * - Кинематику ног (прямая и обратная)
 * - Генерацию походок (trot, walk, stand)
 * - Отправка команд aux контроллеру по CAN
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <math.h>

// ==================== PPM ПРИЕМНИК ====================

#define PPM_INPUT_PIN 34              // Вход PPM от RC приемника
#define PPM_CHANNELS 8                // Количество каналов
#define PPM_MIN_VALUE 1000            // Минимальное значение (мкс)
#define PPM_MAX_VALUE 2000            // Максимальное значение (мкс)
#define PPM_TIMEOUT 500               // Таймаут потери сигнала (мс)

// Маппинг каналов PPM
#define CH_THROTTLE 0                 // Канал газа (вперед/назад)
#define CH_STEERING 1                 // Канал рулевого управления
#define CH_MODE 2                     // Канал режима (высота стойки)
#define CH_SPEED 3                    // Канал скорости
#define CH_AUX1 4                     // Дополнительный канал 1
#define CH_AUX2 5                     // Дополнительный канал 2

#define JOYSTICK_DEADZONE 5           // Дедзона джойстика (%)

// ==================== CAN ШИНА ====================
// Используется CAN/RS485 модуль StackForce
// CANTX -> IO35, CANRX -> IO41

#define CAN_TX_PIN 35
#define CAN_RX_PIN 41
#define CAN_SPEED CAN_SPEED_1MBPS

// CAN ID сообщений
#define CAN_ID_MAIN_TO_AUX 0x100      // Команды от main к aux контроллеру
#define CAN_ID_AUX_TO_MAIN 0x101      // Телеметрия от aux к main

// ==================== I2C УСТРОЙСТВА ====================

#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22
#define I2C_FREQ 400000

// PCA9685 для сервоприводов передних ног
#define PCA9685_ADDR 0x40
#define SERVO_FREQ 50                 // 50 Hz для серво

// Каналы PCA9685 для передних ног
#define SERVO_FL_SHOULDER 0           // Передняя левая - плечо
#define SERVO_FL_HIP 1                // Передняя левая - бедро
#define SERVO_FR_SHOULDER 2           // Передняя правая - плечо
#define SERVO_FR_HIP 3                // Передняя правая - бедро

// MPU6050 IMU
#define IMU_ADDR 0x68
#define IMU_FILTER_ALPHA 0.98         // Коэффициент комплементарного фильтра
#define IMU_UPDATE_RATE 100           // Частота обновления (Hz)

// ==================== КИНЕМАТИКА НОГ ====================

// Длины звеньев ноги (мм)
#define LEG_SHOULDER_LENGTH 50.0      // Длина плеча
#define LEG_HIP_LENGTH 100.0          // Длина бедра

// Рабочая область ноги
#define LEG_MIN_X -50.0
#define LEG_MAX_X 150.0
#define LEG_MIN_Z -150.0
#define LEG_MAX_Z -30.0

// Углы серво (градусы)
#define SERVO_SHOULDER_MIN 0
#define SERVO_SHOULDER_MAX 180
#define SERVO_HIP_MIN 0
#define SERVO_HIP_MAX 180

// Смещение серво (калибровка)
#define SERVO_SHOULDER_OFFSET 0
#define SERVO_HIP_OFFSET 0

// ==================== ПОХОДКИ ====================

// Типы походок
enum GaitType {
    GAIT_STAND = 0,      // Стойка (все ноги на земле)
    GAIT_WALK = 1,       // Шаг (медленная, стабильная)
    GAIT_TROT = 2,       // Рысь (средняя скорость)
    GAIT_BOUND = 3       // Галоп (быстрая)
};

// Параметры походки
#define GAIT_STEP_HEIGHT 30.0         // Высота шага (мм)
#define GAIT_STEP_LENGTH 80.0         // Длина шага (мм)
#define GAIT_CYCLE_TIME 1000          // Время цикла походки (мс)
#define GAIT_BODY_HEIGHT 100.0        // Высота тела над землей (мм)

// Фазы походки для каждой ноги (0.0 - 1.0)
// FL, FR, BL, BR
const float GAIT_WALK_PHASE[4] = {0.0, 0.5, 0.75, 0.25};
const float GAIT_TROT_PHASE[4] = {0.0, 0.5, 0.5, 0.0};
const float GAIT_BOUND_PHASE[4] = {0.0, 0.0, 0.5, 0.5};

// ==================== ПАРАМЕТРЫ УПРАВЛЕНИЯ ====================

#define MOTOR_MAX_SPEED 3000          // Максимальная скорость BLDC (RPM)
#define MOTOR_MIN_SPEED -3000
#define SERVO_MIN_PULSE 150           // Минимальная ширина импульса серво (мкс)
#define SERVO_MAX_PULSE 600           // Максимальная ширина импульса серво (мкс)

#define CONTROL_LOOP_FREQ 50          // Частота основного цикла (Hz)

// ==================== ИНДИКАЦИЯ ====================

#define LED_STATUS_PIN 2              // Статус main
#define LED_PPM_PIN 16                // Прием PPM
#define LED_CAN_PIN 15                // Активность CAN
#define LED_IMU_PIN 17                // Статус IMU

// ==================== ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ ====================

// Преобразование градусов в радианы
inline float degToRad(float deg) { return deg * PI / 180.0; }

// Преобразование радиан в градусы
inline float radToDeg(float rad) { return rad * 180.0 / PI; }

// Ограничение значения
inline float clamp(float value, float min_val, float max_val) {
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

#endif // CONFIG_H
