/**
 * @file main.cpp
 * @brief Прошивка для калибровки офсетов сервоприводов
 * 
 * Функции:
 * - Чтение текущих углов всех 8 сервоприводов
 * - Непрерывный вывод офсетов в Serial консоль
 * - Управление сервоприводами через Serial команды
 * - Расчет офсетов относительно нейтральной позиции (90°)
 * 
 * Использование:
 * 1. Загрузите эту прошивку на ESP32
 * 2. Откройте Serial Monitor (115200 baud)
 * 3. Используйте команды для установки углов сервоприводов
 * 4. Выставьте ноги робота в нужное положение
 * 5. Запишите офсеты из вывода консоли
 * 
 * Команды Serial:
 * - s<номер> <угол>    : Установить угол серво (например: s0 90)
 * - a <угол>           : Установить все серво в один угол (например: a 90)
 * - n                  : Установить все серво в нейтраль (90°)
 * - r                  : Сбросить все офсеты
 * - h                  : Показать помощь
 */

#include <Arduino.h>
#include <Wire.h>
#include "SF_Servo.h"
#include "config.h"

// ==================== ГЛОБАЛЬНЫЕ ОБЪЕКТЫ ====================
SF_Servo servos = SF_Servo(Wire);           // PCA9685 драйвер серво

// ==================== ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ ====================

// Текущие углы сервоприводов (градусы)
uint16_t currentAngles[NUM_SERVOS];

// Целевые углы сервоприводов (градусы)
uint16_t targetAngles[NUM_SERVOS];

// Офсеты сервоприводов (градусы относительно нейтрали)
int16_t servoOffsets[NUM_SERVOS];

// Таймер для вывода
unsigned long lastUpdateTime = 0;

// Флаг инициализации
bool initialized = false;

// ==================== ФУНКЦИИ ====================

/**
 * @brief Инициализация сервоприводов
 */
void initServos() {
    Serial.println("=== Инициализация сервоприводов ===");
    
    // Инициализация I2C (для IMU и PCA9685)
    Wire.begin(1, 2, 400000UL);
    
    // Инициализация PCA9685
    servos.init();
    servos.setAngleRange(0, 300);
    servos.setPluseRange(500, 2500);
    
    // Установка всех серво в нейтральную позицию
    for (uint8_t i = 0; i < NUM_SERVOS; i++) {
        targetAngles[i] = SERVO_NEUTRAL_ANGLE;
        currentAngles[i] = SERVO_NEUTRAL_ANGLE;
        servoOffsets[i] = 0;
        servos.setAngle(i, SERVO_NEUTRAL_ANGLE);
    }
    
    delay(100);
    
    Serial.println("PCA9685 инициализирован");
    Serial.println("Все серво установлены в нейтральную позицию (90°)");
    initialized = true;
}

/**
 * @brief Расчет офсетов сервоприводов
 */
void calculateOffsets() {
    for (uint8_t i = 0; i < NUM_SERVOS; i++) {
        // Офсет = текущий угол - нейтральная позиция
        servoOffsets[i] = currentAngles[i] - SERVO_NEUTRAL_ANGLE;
    }
}

/**
 * @brief Вывод текущих углов и офсетов в Serial
 */
void printServoData() {
    Serial.println("\n========================================");
    Serial.println("         КАЛИБРОВКА СЕРВОПРИВОДОВ       ");
    Serial.println("========================================");
    
    Serial.println("\nТекущие углы и офсеты:");
    Serial.println("----------------------------------------");
    Serial.println("Серво          | Угол  | Офсет | Канал");
    Serial.println("----------------------------------------");
    
    for (uint8_t i = 0; i < NUM_SERVOS; i++) {
        char buffer[80];
        snprintf(buffer, sizeof(buffer), 
                 "%-14s | %3d°  | %+4d° | CH%d",
                SERVO_NAMES[i], 
                 currentAngles[i], 
                 servoOffsets[i],
                 i);
        Serial.println(buffer);
    }
    
    Serial.println("----------------------------------------");
    
    // Вывод офсетов в формате для config.h
    Serial.println("\nОфсеты для config.h:");
    Serial.println("----------------------------------------");
    for (uint8_t i = 0; i < NUM_SERVOS; i++) {
        char buffer[80];
        snprintf(buffer, sizeof(buffer), 
                 "#define SERVO_%s_OFFSET %+d",
                 SERVO_NAMES[i], 
                 servoOffsets[i]);
        Serial.println(buffer);
    }
    
    Serial.println("========================================\n");
}

/**
 * @brief Установка угла серво
 */
void setServoAngle(uint8_t servoNum, uint16_t angle) {
    if (servoNum >= NUM_SERVOS) {
        Serial.printf("Ошибка: номер серво должен быть 0-%d\n", NUM_SERVOS - 1);
        return;
    }
    
    if (angle < SERVO_ANGLE_MIN || angle > SERVO_ANGLE_MAX) {
        Serial.printf("Ошибка: угол должен быть %d-%d градусов\n", 
                      SERVO_ANGLE_MIN, SERVO_ANGLE_MAX);
        return;
    }
    
    targetAngles[servoNum] = angle;
    currentAngles[servoNum] = angle;
    servos.setAngle(servoNum, angle);
    
    Serial.printf("Серво %s (CH%d) установлено в %d°\n", 
                  SERVO_NAMES[servoNum], servoNum, angle);
}

/**
 * @brief Установка всех серво в один угол
 */
void setAllServos(uint16_t angle) {
    if (angle < SERVO_ANGLE_MIN || angle > SERVO_ANGLE_MAX) {
        Serial.printf("Ошибка: угол должен быть %d-%d градусов\n", 
                      SERVO_ANGLE_MIN, SERVO_ANGLE_MAX);
        return;
    }
    
    for (uint8_t i = 0; i < NUM_SERVOS; i++) {
        targetAngles[i] = angle;
        currentAngles[i] = angle;
        servos.setAngle(i, angle);
    }
    
    Serial.printf("Все серво установлены в %d°\n", angle);
}

/**
 * @brief Сброс в нейтральную позицию
 */
void resetToNeutral() {
    setAllServos(SERVO_NEUTRAL_ANGLE);
    Serial.println("Все серво сброшены в нейтральную позицию (90°)");
}

/**
 * @brief Сброс офсетов
 */
void resetOffsets() {
    for (uint8_t i = 0; i < NUM_SERVOS; i++) {
        servoOffsets[i] = 0;
    }
    Serial.println("Все офсеты сброшены");
}

/**
 * @brief Вывод справки
 */
void printHelp() {
    Serial.println("\n========================================");
    Serial.println("            КОМАНДЫ УПРАВЛЕНИЯ          ");
    Serial.println("========================================");
    Serial.println("\nДоступные команды:");
    Serial.println("  s<номер> <угол>  - Установить угол серво");
    Serial.println("                     Пример: s0 90");
    Serial.println("  a <угол>         - Установить все серво");
    Serial.println("                     Пример: a 90");
    Serial.println("  n                - Нейтральная позиция (90°)");
    Serial.println("  r                - Сбросить офсеты");
    Serial.println("  h                - Показать эту справку");
    Serial.println("\nСписок сервоприводов:");
    for (uint8_t i = 0; i < NUM_SERVOS; i++) {
        Serial.printf("  CH%d: %s\n", i, SERVO_NAMES[i]);
    }
    Serial.println("========================================\n");
}

/**
 * @brief Обработка команд из Serial
 */
void processSerialCommand() {
    if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        
        if (command.length() == 0) {
            return;
        }
        
        // Команда помощи
        if (command == "h" || command == "help") {
            printHelp();
        }
        // Команда нейтральной позиции
        else if (command == "n") {
            resetToNeutral();
        }
        // Команда сброса офсетов
        else if (command == "r") {
            resetOffsets();
        }
        // Команда установки одного серво
        else if (command.startsWith("s")) {
            int spaceIndex = command.indexOf(' ');
            if (spaceIndex > 0) {
                uint8_t servoNum = command.substring(1, spaceIndex).toInt();
                uint16_t angle = command.substring(spaceIndex + 1).toInt();
                setServoAngle(servoNum, angle);
            } else {
                Serial.println("Ошибка: неверный формат команды. Используйте: s<номер> <угол>");
            }
        }
        // Команда установки всех серво
        else if (command.startsWith("a")) {
            int spaceIndex = command.indexOf(' ');
            if (spaceIndex > 0) {
                uint16_t angle = command.substring(spaceIndex + 1).toInt();
                setAllServos(angle);
            } else {
                Serial.println("Ошибка: неверный формат команды. Используйте: a <угол>");
            }
        }
        else {
            Serial.println("Неизвестная команда. Введите 'h' для справки.");
        }
    }
}

/**
 * @brief Настройка
 */
void setup() {
    // Инициализация Serial
    Serial.begin(SERIAL_BAUD_RATE);
    delay(1000);
    
    Serial.println("\n\n");
    Serial.println("========================================");
    Serial.println("   КАЛИБРОВКА ОФСЕТОВ СЕРВОПРИВОДОВ    ");
    Serial.println("========================================");
    Serial.println("Версия: 1.0");
    Serial.println("ESP32-S3 + PCA9685");
    Serial.println("========================================\n");
    
    // Настройка светодиода статуса
    pinMode(LED_STATUS_PIN, OUTPUT);
    digitalWrite(LED_STATUS_PIN, LOW);
    
    // КРИТИЧЕСКИ ВАЖНО: Инициализация I2C ПЕРЕД инициализацией сервоприводов!
    Serial.println("DEBUG: Инициализация I2C...");
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQ);
    Serial.printf("DEBUG: I2C OK (SDA=%d, SCL=%d, FREQ=%d Hz)\n", I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQ);
    
    // Инициализация сервоприводов
    Serial.println("DEBUG: Запуск initServos()...");
    initServos();
    Serial.println("DEBUG: initServos() завершен!");
    
    // Вывод справки
    printHelp();
    
    // Первый вывод данных
    calculateOffsets();
    printServoData();
    
    digitalWrite(LED_STATUS_PIN, HIGH);
    Serial.println("Система готова к работе!");
    Serial.println("Введите команду или ждите автоматического обновления\n");
}

/**
 * @brief Основной цикл
 */
void loop() {
    // Обработка команд из Serial
    processSerialCommand();
    
    // Непрерывный вывод данных
    if (CONTINUOUS_OUTPUT) {
        unsigned long currentTime = millis();
        if (currentTime - lastUpdateTime >= UPDATE_INTERVAL) {
            lastUpdateTime = currentTime;
            
            // Расчет и вывод офсетов
            calculateOffsets();
            printServoData();
            
            // Мигание светодиодом статуса
            digitalWrite(LED_STATUS_PIN, !digitalRead(LED_STATUS_PIN));
        }
    }
    
    delay(10);
}
