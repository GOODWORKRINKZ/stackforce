/**
 * @file main.cpp
 * @brief Вспомогательный (aux) контроллер задней части робота
 * 
 * Функции:
 * - Управление ТОЛЬКО 2 задними BLDC моторами (M2, M3)
 * - Прием команд от main контроллера по CAN (TWAI)
 * - Бузер на GPIO9 для звуковых эффектов (дуэт с main)
 * - ВАЖНО: НЕТ СЕРВОПРИВОДОВ! Все 8 серво на main контроллере
 */

#include <Arduino.h>
#include "driver/twai.h"
#include "SF_BLDC.h"
#include "Buzzer.h"
#include "config.h"

// ==================== GPIO ПИНЫ ====================
#define BUZZER_PIN 9
#define BUZZER_PWM_CHANNEL 0  // Такой же как на main

// ==================== ГЛОБАЛЬНЫЕ ОБЪЕКТЫ ====================
SF_BLDC motors = SF_BLDC(Serial2);  // BLDC моторы M2, M3
Buzzer buzzer(BUZZER_PIN, BUZZER_PWM_CHANNEL);  // Пассивный бузер
static bool twai_installed = false;  // Флаг инициализации TWAI

// ==================== ДАННЫЕ ОТ MAIN КОНТРОЛЛЕРА ====================
struct CANCommand {
    uint8_t mode;         // Режим работы робота
    int16_t motor2;       // Команда для M2
    int16_t motor3;       // Команда для M3
    bool motorsArmed;     // Флаг ARM моторов
} cmd = {0, 0, 0, false};

unsigned long lastCommandTime = 0;

/**
 * @brief Инициализация TWAI (CAN)
 */
void setupCAN() {
    Serial.printf("[AUX] Инициализация CAN интерфейса (TX=%d, RX=%d)...\n", CAN_TX_PIN, CAN_RX_PIN);
    
    // Конфигурация TWAI
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)CAN_TX_PIN,
        (gpio_num_t)CAN_RX_PIN,
        TWAI_MODE_NORMAL
    );
    
    // Увеличиваем размер очередей
    g_config.tx_queue_len = 10;
    g_config.rx_queue_len = 10;
    
    // 500 kbps - совпадает с main контроллером
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    
    // Установка драйвера
    esp_err_t install_result = twai_driver_install(&g_config, &t_config, &f_config);
    if (install_result == ESP_OK) {
        Serial.println("[AUX] ✓ TWAI driver установлен успешно");
    } else {
        Serial.printf("[AUX] ✗ ОШИБКА установки TWAI driver: 0x%X\n", install_result);
        return;
    }
    
    // Запуск драйвера
    esp_err_t start_result = twai_start();
    if (start_result == ESP_OK) {
        Serial.println("[AUX] ✓ TWAI запущен (500 kbps, SN65HVD230 transceiver)");
        twai_installed = true;
        
        // Диагностика
        twai_status_info_t status;
        if (twai_get_status_info(&status) == ESP_OK) {
            Serial.printf("[AUX] CAN статус: state=%d, tx_queue=%d, rx_queue=%d\n",
                          status.state, status.msgs_to_tx, status.msgs_to_rx);
        }
    } else {
        Serial.printf("[AUX] ✗ ОШИБКА запуска TWAI: 0x%X\n", start_result);
    }
}

/**
 * @brief Чтение команд от main контроллера по CAN
 */
void readCANCommands() {
    if (!twai_installed) return;
    
    static unsigned long lastDebug = 0;
    bool receivedMessage = false;
    
    twai_message_t rx_msg;
    while (twai_receive(&rx_msg, pdMS_TO_TICKS(1)) == ESP_OK) {
        if (rx_msg.identifier == CAN_ID_MAIN_TO_AUX) {
            // Проверка на команду бузера (маркер 0xFF)
            if (rx_msg.data_length_code >= 2 && rx_msg.data[0] == 0xFF) {
                // Команда бузера для дуэта
                BuzzerSound sound = (BuzzerSound)rx_msg.data[1];
                buzzer.play(sound);
                Serial.printf("[BUZZER] Играем дуэт: звук %d\n", sound);
                continue;
            }
            
            // Обычная команда моторов
            if (rx_msg.data_length_code >= 6) {
                // Распаковка команды по новому протоколу
                // [0]=mode, [1-2]=motor2_cmd, [3-4]=motor3_cmd, [5]=armed
                cmd.mode = rx_msg.data[0];
                cmd.motor2 = (int16_t)((rx_msg.data[1] << 8) | rx_msg.data[2]);
                cmd.motor3 = (int16_t)((rx_msg.data[3] << 8) | rx_msg.data[4]);
                cmd.motorsArmed = (rx_msg.data[5] == 1);
                
                lastCommandTime = millis();
                receivedMessage = true;
            }
        }
    }
    
    // Диагностика приема (каждые 5 секунд для уменьшения спама)
    if (millis() - lastDebug > 5000) {
        if (receivedMessage) {
            Serial.printf("[CAN] ✓ Прием от main работает (M2=%d, M3=%d, ARM=%d)\n",
                          cmd.motor2, cmd.motor3, cmd.motorsArmed);
        } else {
            Serial.println("[CAN] ✗ НЕТ данных от main контроллера!");
            
            // Диагностика состояния
            twai_status_info_t status;
            if (twai_get_status_info(&status) == ESP_OK) {
                Serial.printf("[CAN] Статус: state=%d, rx_queue=%d, rx_err=%d\n",
                              status.state, status.msgs_to_rx, status.rx_error_counter);
            }
        }
        lastDebug = millis();
    }
}

/**
 * @brief Управление задними моторами (M2, M3)
 * ВАЖНО: SF_BLDC всегда работает с M0/M1, но физически это M2/M3
 */
void controlRearMotors() {
    static unsigned long lastControl = 0;
    static unsigned long lastDebug = 0;
    
    if (millis() - lastControl < 20) return;  // 50 Hz
    
    // Проверка таймаута команд (500 мс)
    if (millis() - lastCommandTime > 500) {
        // Аварийная остановка при потере связи
        motors.setTargets(0, 0);
        
        if (millis() - lastDebug > 2000) {
            Serial.printf("[MOTOR] ⚠ TIMEOUT! Нет CAN данных %.1f сек\n",
                          (millis() - lastCommandTime) / 1000.0);
            lastDebug = millis();
        }
        
        lastControl = millis();
        return;
    }
    
    // Проверка флага ARM
    if (!cmd.motorsArmed) {
        motors.setTargets(0, 0);
        lastControl = millis();
        return;
    }
    
    // Получение команд напрямую из CAN протокола
    int16_t torqueM2 = cmd.motor2;  // Левый задний (M2 -> M0 физически)
    int16_t torqueM3 = cmd.motor3;  // Правый задний (M3 -> M1 физически)
    
    // Отправка команд моторам
    motors.setTargets(torqueM2, torqueM3);
    
    // Диагностика каждые 500мс
    if (millis() - lastDebug > 500) {
        SF_BLDC_DATA bldcData = motors.getBLDCData();
        Serial.printf("[MOTOR] M2=%d M3=%d | Vel: M2=%.1f M3=%.1f | ARM=%d\n",
                      torqueM2, torqueM3,
                      bldcData.M0_Vel, bldcData.M1_Vel,
                      cmd.motorsArmed);
        lastDebug = millis();
    }
    
    lastControl = millis();
}

/**
 * @brief Вывод отладочной информации
 */
void printDebug() {
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint < 2000) return;  // Каждые 2 сек
    
    Serial.printf("[AUX] Mode=%d | M2=%d M3=%d | ARM=%d | CAN age=%lums\n",
                  cmd.mode, cmd.motor2, cmd.motor3,
                  cmd.motorsArmed,
                  millis() - lastCommandTime);
    
    lastPrint = millis();
}

void setup() {
    Serial.begin(115200);
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
    
    motors.setModes(4, 4);  // Режим 4 = velocity control (КРИТИЧНО!)
    Serial.println("[AUX] BLDC моторы M2, M3 инициализированы (режим velocity)");
    
    // Инициализация бузера
    buzzer.begin();
    Serial.println("[AUX] Бузер инициализирован");
    
    // Инициализация TWAI
    setupCAN();
    
    Serial.println("[AUX] Инициализация завершена\n");
    Serial.println("[AUX] Ожидание команд от main контроллера...\n");
    
    // Приветственный звук отключен (проблема с LEDC на некоторых частотах)
    // delay(300);
    // buzzer.play(SOUND_BARK_SINGLE);
    
    lastCommandTime = millis();
}

void loop() {
    readCANCommands();
    controlRearMotors();
    buzzer.update();  // Обновление бузера
    printDebug();
    
    delay(5);
}
