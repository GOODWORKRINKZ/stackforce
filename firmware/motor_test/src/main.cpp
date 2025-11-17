/**
 * @file main.cpp
 * @brief Тест BLDC моторов через SF_BLDC библиотеку
 * 
 * Функции:
 * - Инициализация Serial2 для связи с Motor Main Controller
 * - Синусоидальное управление скоростью M0 и M1
 * - Вывод телеметрии в Serial Monitor
 */

#include <Arduino.h>
#include "SF_BLDC.h"

// Объект для управления моторами через Serial2
SF_BLDC motors = SF_BLDC(Serial2);

// Переменные для синусоиды
float time_seconds = 0.0;
const float FREQUENCY = 0.2;  // Частота синусоиды (период ~5 сек)
const float AMPLITUDE = 50.0; // Амплитуда ±50 (диапазон -50..+50)

// Направления моторов
int M0Dir = 1;   // M0 - левый мотор
int M1Dir = -1;  // M1 - правый мотор (инвертирован)

void setup() {
    Serial.begin(115200);
    delay(500);
    
    Serial.println("\n========================================");
    Serial.println("  ТЕСТ BLDC МОТОРОВ");
    Serial.println("  ESP32-S3 → Motor Main Controller");
    Serial.println("========================================\n");
    
    // Инициализация Serial2 для связи с Motor Main Controller
    // GPIO17 = RX, GPIO18 = TX, скорость 115200
    Serial.println("[INIT] Инициализация Serial2 (RX=17, TX=18, 115200)...");
    Serial2.begin(115200, SERIAL_8N1, 17, 18);
    delay(100);
    
    // Инициализация библиотеки SF_BLDC
    Serial.println("[INIT] Инициализация SF_BLDC...");
    motors.init();
    delay(100);
    
    // Установка режима управления (4 = velocity control)
    Serial.println("[INIT] Установка режима velocity control (mode=4)...");
    motors.setModes(4, 4);
    delay(100);
    
    Serial.println("[INIT] Инициализация завершена!\n");
    Serial.println("Запускаем синусоидальное управление...");
    Serial.println("Моторы будут крутиться вперёд-назад с периодом ~5 сек\n");
    
    delay(1000);
}

void loop() {
    // Вычисляем синусоиду для плавного изменения скорости
    float sineWave = sin(2.0 * PI * FREQUENCY * time_seconds);
    float targetSpeed = AMPLITUDE * sineWave;
    
    // Устанавливаем скорость для обоих моторов
    float M0_cmd = M0Dir * targetSpeed;
    float M1_cmd = M1Dir * targetSpeed;
    motors.setTargets(M0_cmd, M1_cmd);
    
    // Получаем телеметрию от моторов
    SF_BLDC_DATA telemetry = motors.getBLDCData();
    
    // Вывод данных каждые 200мс
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 200) {
        Serial.printf("[%6.1fs] CMD: M0=%6.1f M1=%6.1f | ", 
                      time_seconds, M0_cmd, M1_cmd);
        Serial.printf("VEL: M0=%6.1f M1=%6.1f | ", 
                      telemetry.M0_Vel, telemetry.M1_Vel);
        Serial.printf("ANGLE: M0=%6.1f M1=%6.1f | ", 
                      telemetry.M0_Angle, telemetry.M1_Angle);
        Serial.printf("Iq: M0=%5.1f M1=%5.1f\n", 
                      telemetry.M0_Iq, telemetry.M1_Iq);
        lastPrint = millis();
    }
    
    // Увеличиваем время
    time_seconds += 0.02;  // 20мс шаг (50 Hz)
    
    delay(20);  // 50 Hz loop rate
}
