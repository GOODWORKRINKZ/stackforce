/**
 * @file main.cpp
 * @brief Вспомогательный (aux) контроллер задней части робота
 * 
 * Функции:
 * - Управление ТОЛЬКО 2 задними BLDC моторами (M2, M3)
 * - Прием команд от main контроллера по CAN (TWAI)
 * - ВАЖНО: НЕТ СЕРВОПРИВОДОВ! Все 8 серво на main контроллере
 */

#include <Arduino.h>
#include "driver/twai.h"
#include "SF_BLDC.h"
#include "config.h"

// ==================== ГЛОБАЛЬНЫЕ ОБЪЕКТЫ ====================
SF_BLDC motors = SF_BLDC(Serial2);  // BLDC моторы M2, M3
static bool twai_installed = false;  // Флаг инициализации TWAI

// ==================== ДАННЫЕ ОТ MAIN КОНТРОЛЛЕРА ====================
struct CANCommand {
    int8_t throttle;    // -100..100
    int8_t turn;        // -100..100  
    uint8_t height;     // Высота (не используется в aux)
    uint8_t reserved;   // Резерв
    float roll;         // Крен от IMU
    float pitch;        // Тангаж от IMU
} cmd = {0, 0, 100, 0, 0.0, 0.0};

unsigned long lastCommandTime = 0;

/**
 * @brief Инициализация TWAI (CAN)
 */
void setupCAN() {
    // Конфигурация TWAI
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)CAN_TX_PIN,
        (gpio_num_t)CAN_RX_PIN,
        TWAI_MODE_NORMAL
    );
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();  // 1 Mbps
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    
    // Установка драйвера
    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        Serial.println("[AUX] TWAI driver установлен");
    } else {
        Serial.println("[AUX] ОШИБКА: Не удалось установить TWAI driver");
        return;
    }
    
    // Запуск драйвера
    if (twai_start() == ESP_OK) {
        Serial.println("[AUX] TWAI запущен (1 Mbps)");
        twai_installed = true;
    } else {
        Serial.println("[AUX] ОШИБКА: Не удалось запустить TWAI");
        return;
    }
}

/**
 * @brief Чтение команд от main контроллера по CAN
 */
void readCANCommands() {
    if (!twai_installed) return;
    
    twai_message_t rx_msg;
    while (twai_receive(&rx_msg, pdMS_TO_TICKS(1)) == ESP_OK) {
        if (rx_msg.identifier == CAN_ID_MAIN_TO_AUX && rx_msg.data_length_code >= 8) {
            // Распаковка команды
            cmd.throttle = (int8_t)(rx_msg.data[0]) - 100;  // 0..200 -> -100..100
            cmd.turn = (int8_t)(rx_msg.data[1]) - 100;
            cmd.height = rx_msg.data[2];
            cmd.reserved = rx_msg.data[3];
            
            int16_t roll_int = (rx_msg.data[4] << 8) | rx_msg.data[5];
            int16_t pitch_int = (rx_msg.data[6] << 8) | rx_msg.data[7];
            cmd.roll = roll_int / 10.0;
            cmd.pitch = pitch_int / 10.0;
            
            lastCommandTime = millis();
        }
    }
}

/**
 * @brief Управление задними моторами (M2, M3)
 * ВАЖНО: SF_BLDC всегда работает с M0/M1, но физически это M2/M3
 */
void controlRearMotors() {
    static unsigned long lastControl = 0;
    if (millis() - lastControl < 20) return;  // 50 Hz
    
    // Проверка таймаута команд (500 мс)
    if (millis() - lastCommandTime > 500) {
        // Аварийная остановка
        motors.setTargets(0, 0);
        lastControl = millis();
        return;
    }
    
    // Вычисление момента для задних моторов
    float baseThrottle = cmd.throttle;  // -100..100
    float turnAdjust = cmd.turn * 0.5;  // Меньше влияние поворота на задние
    
    float torqueM2 = (baseThrottle - turnAdjust) * M2Dir;  // Левый задний
    float torqueM3 = (baseThrottle + turnAdjust) * M3Dir;  // Правый задний
    
    // Ограничение
    torqueM2 = constrain(torqueM2, -100, 100);
    torqueM3 = constrain(torqueM3, -100, 100);
    
    // Отправка команд моторам (M0=M2 left, M1=M3 right)
    motors.setModes(4, 4);  // Режим 4 = velocity control
    motors.setTargets(torqueM2, torqueM3);
    
    lastControl = millis();
}

/**
 * @brief Вывод отладочной информации
 */
void printDebug() {
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint < 1000) return;  // 1 Hz
    
    Serial.printf("[AUX] Throttle=%d Turn=%d Height=%d | Roll=%.1f Pitch=%.1f | Timeout=%lums\n",
                  cmd.throttle, cmd.turn, cmd.height,
                  cmd.roll, cmd.pitch,
                  millis() - lastCommandTime);
    
    lastPrint = millis();
}

void setup() {
    Serial.begin(921600);
    delay(500);
    
    Serial.println("\n========================================");
    Serial.println("  AUX КОНТРОЛЛЕР (Задняя часть)");
    Serial.println("  ESP32-S3 DevKit C1");
    Serial.println("  Моторы M2, M3");
    Serial.println("========================================\n");
    
    // Инициализация Serial2 для BLDC моторов
    Serial2.begin(115200, SERIAL_8N1, 17, 18);  // RX=17, TX=18
    delay(100);
    
    // Инициализация моторов
    motors.init();
    delay(100);
    
    Serial.println("[AUX] BLDC моторы M2, M3 инициализированы");
    
    // Инициализация TWAI
    setupCAN();
    
    Serial.println("[AUX] Инициализация завершена\n");
    Serial.println("[AUX] Ожидание команд от main контроллера...\n");
    
    lastCommandTime = millis();
}

void loop() {
    readCANCommands();
    controlRearMotors();
    printDebug();
    
    delay(5);
}
