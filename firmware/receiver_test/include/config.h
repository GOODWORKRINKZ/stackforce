/**
 * @file config.h
 * @brief Конфигурация для тестирования SBUS приемника
 */

#ifndef CONFIG_H
#define CONFIG_H

// ==================== SBUS ПРИЕМНИК ====================
// Подключение через плату распределения StackForce IO Connector:
// - SBUS Signal -> GPIO40 (пин 40 на плате распределения)
// - +5V -> 5V
// - GND -> GND
//
// ВНИМАНИЕ: Если не работает GPIO40, попробуйте другие пины:
// - GPIO41 (как в примере Денге lesson7_RollCtrl)
// - GPIO42, GPIO43, GPIO44 (альтернативные UART пины на ESP32-S3)

#define SBUS_RX_PIN 40  // RX пин для Serial1 (пин 40 на плате распределения)
#define SBUS_TX_PIN -1  // TX не используется

// ВАЖНО: Старая библиотека SBUS от Денге
// Использует Serial1 (100000 baud, 8E2, inverted)

// Частота обновления отладочного вывода (в миллисекундах)
#define DEBUG_PRINT_INTERVAL 100  // 10 Hz (100ms)

// Формат вывода
#define PRINT_RAW_VALUES    true   // Вывод сырых значений (172-1811)
#define PRINT_NORMALIZED    true   // Вывод нормализованных значений (-100% до +100%)
#define PRINT_MICROSECONDS  false  // Вывод в микросекундах PWM (1000-2000)

// Цветной вывод в терминал (ANSI escape codes)
#define USE_COLOR_OUTPUT    true

// Мёртвая зона джойстика (для определения центра)
#define STICK_DEADZONE 30  // ±30 из 2048 (~1.5%)

#endif // CONFIG_H
