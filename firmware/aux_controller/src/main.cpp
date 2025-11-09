/**/**

 * @file main.cpp * @file main.cpp

 * @brief Вспомогательный контроллер (Aux Controller) - управление задними моторами * @brief Aux контроллер (задняя часть - только управление задними моторами)

 * @version 2.0 * 

 * @date 2024-01 * Функции:

 *  * - Прием команд от main контроллера по CAN

 * Функционал: * - Управление 2 задними BLDC моторами (колеса)

 * - Прием команд от main контроллера по CAN шине * - Все 8 сервоприводов управляются main контроллером

 * - Управление задними BLDC моторами (M2, M3) */

 * - Все 8 серво управляются main контроллером

 * #include <Arduino.h>

 * Hardware:#include <Wire.h>

 * - ESP32-S3 DevKit#include <ESP32CAN.h>

 * - 2x BLDC мотора (задние колеса)#include <CAN_config.h>

 * - CAN трансивер#include "SF_BLDC.h"

 */#include "config.h"



#include <Arduino.h>// ==================== ГЛОБАЛЬНЫЕ ОБЪЕКТЫ ====================

#include <ESP32CAN.h>CAN_device_t CAN_cfg;

#include <CAN_config.h>SF_BLDC motors = SF_BLDC(Serial2);     // Задние BLDC моторы

#include "SF_BLDC.h"

#include "config.h"// Направления задних моторов

int M2Dir = 1;   // Задний левый

// ==================== БИБЛИОТЕКИ ====================int M3Dir = -1;  // Задний правый



SF_BLDC motors;// ==================== СТРУКТУРА КОМАНДЫ ОТ MAIN ====================

CAN_device_t CAN_cfg;struct RobotCommand {

    int8_t throttle;      // -100..100

// ==================== ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ ====================    int8_t turn;          // -100..100

    uint8_t height;       // Высота ног (не используется, только для информации)

struct RobotCommand {    uint8_t reserved;

    float throttle;   // -100...100    float roll;           // Крен от IMU main

    float turn;       // -100...100      float pitch;          // Тангаж от IMU main

    uint8_t height;   // 0...255} __attribute__((packed));

    uint8_t reserved;

    float roll;       // градусыRobotCommand robotCmd = {0, 0, 90, 0, 0.0, 0.0};

    float pitch;      // градусы

};// Текущие скорости задних моторов

int16_t motorSpeedBL = 0;

RobotCommand robotCmd = {0, 0, 120, 0, 0, 0};int16_t motorSpeedBR = 0;



float motorSpeedBL = 0;  // Back Left (M2)// Время последнего получения команды

float motorSpeedBR = 0;  // Back Right (M3)unsigned long lastCommandTime = 0;

unsigned long lastControlTime = 0;

unsigned long lastCommandTime = 0;

unsigned long lastControlTime = 0;uint8_t loopCnt = 0;

uint8_t loopCnt = 0;

// ==================== ФУНКЦИИ ====================

// ==================== ФУНКЦИИ ====================

/**

/** * @brief Инициализация CAN

 * @brief Инициализация CAN */

 */void setupCAN() {

void setupCAN() {    CAN_cfg.speed = CAN_SPEED_1MBPS;

    CAN_cfg.speed = CAN_SPEED_1MBPS;    CAN_cfg.tx_pin_id = (gpio_num_t)CAN_TX_PIN;

    CAN_cfg.tx_pin_id = (gpio_num_t)CAN_TX_PIN;    CAN_cfg.rx_pin_id = (gpio_num_t)CAN_RX_PIN;

    CAN_cfg.rx_pin_id = (gpio_num_t)CAN_RX_PIN;    CAN_cfg.rx_queue = xQueueCreate(10, sizeof(CAN_frame_t));

    CAN_cfg.rx_queue = xQueueCreate(10, sizeof(CAN_frame_t));    

        ESP32Can.CANInit();

    ESP32Can.CANInit();    Serial.println("[AUX] CAN инициализирован (1 Mbps)");

    Serial.println("[AUX] CAN инициализирован (1 Mbps)");}

}

/**

/** * @brief Чтение команд от main контроллера по CAN

 * @brief Чтение команд от main контроллера по CAN */

 */void readCANCommands() {

void readCANCommands() {    CAN_frame_t rx_frame;

    CAN_frame_t rx_frame;    

        while (CAN_OK == ESP32Can.CANReadFrame(&rx_frame)) {

    while (CAN_OK == ESP32Can.CANReadFrame(&rx_frame)) {        if (rx_frame.MsgID == CAN_ID_MAIN_TO_AUX && rx_frame.FIR.B.DLC >= 8) {

        if (rx_frame.MsgID == CAN_ID_MAIN_TO_AUX && rx_frame.FIR.B.DLC >= 8) {            // Распаковка команды

            // Распаковка команды            robotCmd.throttle = (int8_t)(rx_frame.data.u8[0]) - 100;

            robotCmd.throttle = (int8_t)(rx_frame.data.u8[0]) - 100;            robotCmd.turn = (int8_t)(rx_frame.data.u8[1]) - 100;

            robotCmd.turn = (int8_t)(rx_frame.data.u8[1]) - 100;            robotCmd.height = rx_frame.data.u8[2];

            robotCmd.height = rx_frame.data.u8[2];            robotCmd.reserved = rx_frame.data.u8[3];

            robotCmd.reserved = rx_frame.data.u8[3];            

                        int16_t roll_int = (rx_frame.data.u8[4] << 8) | rx_frame.data.u8[5];

            int16_t roll_int = (rx_frame.data.u8[4] << 8) | rx_frame.data.u8[5];            int16_t pitch_int = (rx_frame.data.u8[6] << 8) | rx_frame.data.u8[7];

            int16_t pitch_int = (rx_frame.data.u8[6] << 8) | rx_frame.data.u8[7];            robotCmd.roll = roll_int / 10.0;

            robotCmd.roll = roll_int / 10.0;            robotCmd.pitch = pitch_int / 10.0;

            robotCmd.pitch = pitch_int / 10.0;            

                        lastCommandTime = millis();

            lastCommandTime = millis();        }

        }    }

    }}

}

/**

/** * @brief Управление задними моторами

 * @brief Управление задними моторами */

 */void controlRearMotors() {

void controlRearMotors() {    if (millis() - lastControlTime < 20) return;  // 50 Hz

    if (millis() - lastControlTime < 20) return;  // 50 Hz    

        // Проверка таймаута команд

    // Проверка таймаута команд    if (millis() - lastCommandTime > 500) {

    if (millis() - lastCommandTime > 500) {        // Аварийная остановка при потере связи

        // Аварийная остановка при потере связи        motorSpeedBL = 0;

        motorSpeedBL = 0;        motorSpeedBR = 0;

        motorSpeedBR = 0;        motors.setTargets(0, 0);

        motors.setTargets(0, 0);        lastControlTime = millis();

        lastControlTime = millis();        return;

        return;    }

    }    

        // Вычисление крутящих моментов для задних моторов

    // Вычисление крутящих моментов для задних моторов    float torque = robotCmd.throttle * 0.1;

    float torque = robotCmd.throttle * 0.1;    float turnTorque = robotCmd.turn * 0.1;

    float turnTorque = robotCmd.turn * 0.1;    

        float torque1 = torque + turnTorque;

    float torque1 = torque + turnTorque;    float torque2 = torque - turnTorque;

    float torque2 = torque - turnTorque;    

        motors.setTargets(M2Dir * torque1, M3Dir * torque2);

    motors.setTargets(M2Dir * torque1, M3Dir * torque2);    

        lastControlTime = millis();

    lastControlTime = millis();}

}            robotCmd.throttle = (int8_t)(rx_frame.data.u8[0]) - 100;

            robotCmd.turn = (int8_t)(rx_frame.data.u8[1]) - 100;

// ==================== SETUP ====================            robotCmd.height = rx_frame.data.u8[2];

            robotCmd.reserved = rx_frame.data.u8[3];

void setup() {            

    Serial.begin(115200);            int16_t roll_int = (rx_frame.data.u8[4] << 8) | rx_frame.data.u8[5];

    delay(1000);            int16_t pitch_int = (rx_frame.data.u8[6] << 8) | rx_frame.data.u8[7];

                robotCmd.roll = roll_int / 10.0;

    Serial.println("\n\n[AUX] Запуск вспомогательного контроллера...");            robotCmd.pitch = pitch_int / 10.0;

    Serial.println("[AUX] Версия: 2.0");            

    Serial.println("[AUX] Функции: только задние моторы M2, M3");            lastCommandTime = millis();

            }

    // Инициализация BLDC моторов    }

    Serial2.begin(115200, SERIAL_8N1, 17, 18);}

    motors.init(&Serial2);

    delay(100);/**

    Serial.println("[AUX] BLDC моторы инициализированы"); * @brief Управление задними моторами

     */

    // Инициализация CANvoid controlRearMotors() {

    setupCAN();    if (millis() - lastControlTime < 20) return;  // 50 Hz

        

    Serial.println("[AUX] Инициализация завершена!");    // Проверка таймаута команд

    Serial.println("[AUX] Ожидание команд по CAN...");    if (millis() - lastCommandTime > 500) {

}        // Аварийная остановка при потере связи

        motorSpeedBL = 0;

// ==================== LOOP ====================        motorSpeedBR = 0;

        motors.setTargets(0, 0);

void loop() {        lastControlTime = millis();

    loopCnt++;        return;

        }

    // Чтение команд по CAN    

    readCANCommands();    // Вычисление крутящих моментов для задних моторов

        // Используем ту же логику что и main контроллер

    // Управление задними моторами    float torque = robotCmd.throttle * 0.1;  // Простая пропорция

    controlRearMotors();    float turnTorque = robotCmd.turn * 0.1;

        

    // Отладочная информация каждые 50 циклов    float torque1 = torque + turnTorque;

    if (loopCnt % 50 == 0) {    float torque2 = torque - turnTorque;

        Serial.printf("[AUX] Команды: throttle=%.1f turn=%.1f | Моторы: M2=%.1f M3=%.1f\n",    

            robotCmd.throttle, robotCmd.turn, motorSpeedBL, motorSpeedBR);    motors.setTargets(M2Dir * torque1, M3Dir * torque2);

    }    

        lastControlTime = millis();

    delay(5);  // ~200 Hz}

}

/**
 * @brief Управление задними сервоприводами
 */
/**
 * @brief Управление задними сервоприводами
 */
void controlRearServos() {
    // Синхронизация с main - те же координаты X, Y
    // В идеале main должен передавать X и Y, но пока используем высоту
    
    IKParam.XLeft = 0;    // Базовая позиция
    IKParam.XRight = 0;
    IKParam.YLeft = robotCmd.height;
    IKParam.YRight = robotCmd.height;
    
    inverseKinematics();
}

/**
 * @brief Настройка (инициализация)
 */
void setup() {
    Serial.begin(921600);
    delay(500);
    
    Serial.println("\n========================================");
    Serial.println("  AUX КОНТРОЛЛЕР (Задняя часть)");
    Serial.println("  ESP32 + BLDC + Servo (синхронизация)");
    Serial.println("========================================\n");
    
    // Инициализация I2C (для PCA9685)
    Wire.begin(1, 2, 400000UL);
    
    // Инициализация Serial2 для BLDC (RX=17, TX=18)
    Serial2.begin(115200, SERIAL_8N1, 17, 18);
    
    // Инициализация серво
    servos.init();
    servos.setAngleRange(0, 300);
    servos.setPluseRange(500, 2500);
    Serial.println("[AUX] Сервоприводы (PCA9685) инициализированы");
    
    // Инициализация BLDC моторов
    motors.init();
    motors.setModes(4, 4);  // Режим управления скоростью
    Serial.println("[AUX] BLDC моторы инициализированы");
    
    // Инициализация CAN
    setupCAN();
    
    // Установка начальной позиции задних ног (стоя на 90мм)
    IKParam.XLeft = 0;
    IKParam.XRight = 0;
    IKParam.YLeft = 90;
    IKParam.YRight = 90;
    inverseKinematics();
    
    Serial.println("[AUX] Инициализация завершена!\n");
    Serial.println("[AUX] Ожидание команд от main контроллера по CAN...\n");
    
    lastCommandTime = millis();
    delay(1000);
}

/**
 * @brief Основной цикл
 */
void loop() {
    // === 1. Чтение команд от main по CAN ===
    readCANCommands();
    
    // === 2. Управление задними моторами ===
    controlRearMotors();
    
    // === 3. Управление задними сервоприводами ===
    controlRearServos();
    
    // === 4. Отладочная информация ===
    loopCnt++;
    if (loopCnt >= 100) {
        Serial.printf("[AUX] Cmd: Thr=%d Turn=%d H=%d | Roll=%.1f Pitch=%.1f | Status: %s\n",
                     robotCmd.throttle, robotCmd.turn, robotCmd.height,
                     robotCmd.roll, robotCmd.pitch,
                     (millis() - lastCommandTime > 500) ? "LOST" : "OK");
        loopCnt = 0;
    }
    
    delay(5);  // ~200Hz цикл
}

