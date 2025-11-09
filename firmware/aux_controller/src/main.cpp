/**
 * @file main.cpp
 * @brief Вспомогательный (aux) контроллер задней части робота
 * 
 * Функции:
 * - Управление задними 2 BLDC моторами
 * - Управление задними 4 сервоприводами (2 ноги)
 * - Прием команд от main контроллера по CAN
 * - Синхронизация с main
 */

#include <Arduino.h>
#include <Wire.h>
#include <ESP32CAN.h>
#include <CAN_config.h>
#include <Adafruit_PWMServoDriver.h>
#include "config.h"

// Объекты
CAN_device_t CAN_cfg;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDR);

// Структура команды от main контроллера
struct RobotCommand {
    int8_t throttle;      // -100..100
    int8_t steering;      // -100..100
    uint8_t mode;         // 0..255
    uint8_t speed;        // 0..255
    float roll;           // IMU крен
    float pitch;          // IMU тангаж
} __attribute__((packed));

RobotCommand robotCmd = {0, 0, 128, 128, 0.0, 0.0};

// Текущие скорости задних моторов
int16_t motorSpeedBL = 0;
int16_t motorSpeedBR = 0;

// Углы задних сервоприводов
uint8_t servoAngles[4] = {90, 90, 90, 90};

// Время обновления
unsigned long lastCommandTime = 0;
unsigned long lastControlTime = 0;

/**
 * @brief Инициализация CAN
 */
void setupCAN() {
    CAN_cfg.speed = CAN_SPEED;
    CAN_cfg.tx_pin_id = (gpio_num_t)CAN_TX_PIN;
    CAN_cfg.rx_pin_id = (gpio_num_t)CAN_RX_PIN;
    CAN_cfg.rx_queue = xQueueCreate(10, sizeof(CAN_frame_t));
    
    ESP32Can.CANInit();
    Serial.println("[AUX] CAN инициализирован (1 Mbps)");
}

/**
 * @brief Инициализация PCA9685 и сервоприводов
 */
void setupServos() {
    pwm.begin();
    pwm.setPWMFreq(SERVO_FREQ);
    
    // Установка начальной позиции
    for (int i = 0; i < 4; i++) {
        setServoAngle(i, 90);
    }
    
    Serial.println("[AUX] Сервоприводы инициализированы");
}

/**
 * @brief Установка угла сервопривода
 */
void setServoAngle(uint8_t channel, uint8_t angle) {
    angle = constrain(angle, 0, 180);
    uint16_t pulse = map(angle, 0, 180, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
    pwm.setPWM(channel, 0, pulse);
    servoAngles[channel] = angle;
}

/**
 * @brief Отправка команды на BLDC мотор
 */
void setMotorSpeed(uint16_t motorId, int16_t speed) {
    speed = constrain(speed, MOTOR_MIN_SPEED, MOTOR_MAX_SPEED);
    
    CAN_frame_t tx_frame;
    tx_frame.MsgID = motorId;
    tx_frame.FIR.B.FF = CAN_frame_std;
    tx_frame.FIR.B.DLC = 8;
    
    tx_frame.data.u8[0] = 0x01;  // Команда: установить скорость
    tx_frame.data.u8[1] = (speed >> 8) & 0xFF;
    tx_frame.data.u8[2] = speed & 0xFF;
    tx_frame.data.u8[3] = 0x00;
    tx_frame.data.u8[4] = 0x00;
    tx_frame.data.u8[5] = 0x00;
    tx_frame.data.u8[6] = 0x00;
    tx_frame.data.u8[7] = 0x00;
    
    ESP32Can.CANWriteFrame(&tx_frame);
}

/**
 * @brief Чтение команд от main контроллера
 */
void readCANCommands() {
    CAN_frame_t rx_frame;
    
    while (CAN_OK == ESP32Can.CANReadFrame(&rx_frame)) {
        if (rx_frame.MsgID == CAN_ID_MAIN_TO_AUX && rx_frame.FIR.B.DLC >= 8) {
            // Распаковка команды
            robotCmd.throttle = (int8_t)(rx_frame.data.u8[0]) - 100;
            robotCmd.steering = (int8_t)(rx_frame.data.u8[1]) - 100;
            robotCmd.mode = rx_frame.data.u8[2];
            robotCmd.speed = rx_frame.data.u8[3];
            
            int16_t roll_int = (rx_frame.data.u8[4] << 8) | rx_frame.data.u8[5];
            int16_t pitch_int = (rx_frame.data.u8[6] << 8) | rx_frame.data.u8[7];
            robotCmd.roll = roll_int / 10.0;
            robotCmd.pitch = pitch_int / 10.0;
            
            lastCommandTime = millis();
            digitalWrite(LED_CAN_PIN, !digitalRead(LED_CAN_PIN));
        }
    }
}

/**
 * @brief Управление задними моторами и сервоприводами
 */
void controlRearLegs() {
    if (millis() - lastControlTime < (1000 / CONTROL_LOOP_FREQ)) return;
    
    // Проверка таймаута команд
    if (millis() - lastCommandTime > 1000) {
        // Аварийная остановка
        motorSpeedBL = 0;
        motorSpeedBR = 0;
        setMotorSpeed(CAN_ID_MOTOR_BL, 0);
        setMotorSpeed(CAN_ID_MOTOR_BR, 0);
        lastControlTime = millis();
        return;
    }
    
    // Вычисление скоростей моторов
    float speedMult = robotCmd.speed / 255.0;
    int16_t baseSpeed = map(robotCmd.throttle, -100, 100, MOTOR_MIN_SPEED, MOTOR_MAX_SPEED);
    int16_t turnOffset = map(robotCmd.steering, -100, 100, -1000, 1000);
    
    motorSpeedBL = (baseSpeed - turnOffset) * speedMult;
    motorSpeedBR = (baseSpeed + turnOffset) * speedMult;
    
    setMotorSpeed(CAN_ID_MOTOR_BL, motorSpeedBL);
    setMotorSpeed(CAN_ID_MOTOR_BR, motorSpeedBR);
    
    // Управление высотой стойки задних ног
    uint8_t legAngle = map(robotCmd.mode, 0, 255, 60, 120);
    setServoAngle(SERVO_BL_HIP, legAngle);
    setServoAngle(SERVO_BR_HIP, legAngle);
    
    lastControlTime = millis();
}

/**
 * @brief Отправка телеметрии main контроллеру
 */
void sendTelemetry() {
    static unsigned long lastSend = 0;
    if (millis() - lastSend < 100) return;  // 10 Hz
    
    CAN_frame_t tx_frame;
    tx_frame.MsgID = CAN_ID_AUX_TO_MAIN;
    tx_frame.FIR.B.FF = CAN_frame_std;
    tx_frame.FIR.B.DLC = 8;
    
    tx_frame.data.u8[0] = 0x01;  // Статус: OK
    tx_frame.data.u8[1] = (motorSpeedBL >> 8) & 0xFF;
    tx_frame.data.u8[2] = motorSpeedBL & 0xFF;
    tx_frame.data.u8[3] = (motorSpeedBR >> 8) & 0xFF;
    tx_frame.data.u8[4] = motorSpeedBR & 0xFF;
    tx_frame.data.u8[5] = servoAngles[SERVO_BL_HIP];
    tx_frame.data.u8[6] = servoAngles[SERVO_BR_HIP];
    tx_frame.data.u8[7] = 0x00;
    
    ESP32Can.CANWriteFrame(&tx_frame);
    lastSend = millis();
}

/**
 * @brief Вывод отладочной информации
 */
void printDebug() {
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint < 500) return;
    
    Serial.printf("[AUX] Cmd: T=%d S=%d M=%d | Motors: BL=%d BR=%d | IMU: R=%.1f P=%.1f\n",
                 robotCmd.throttle, robotCmd.steering, robotCmd.mode,
                 motorSpeedBL, motorSpeedBR, robotCmd.roll, robotCmd.pitch);
    
    lastPrint = millis();
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("\n========================================");
    Serial.println("  AUX КОНТРОЛЛЕР (Задняя часть)");
    Serial.println("  ESP32");
    Serial.println("========================================\n");
    
    // Инициализация пинов
    pinMode(LED_STATUS_PIN, OUTPUT);
    pinMode(LED_CAN_PIN, OUTPUT);
    
    digitalWrite(LED_STATUS_PIN, HIGH);
    
    // Инициализация I2C
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQ);
    
    // Инициализация подсистем
    setupCAN();
    setupServos();
    
    Serial.println("[AUX] Инициализация завершена\n");
    Serial.println("[AUX] Ожидание команд от main контроллера...\n");
    
    lastCommandTime = millis();
}

void loop() {
    readCANCommands();
    controlRearLegs();
    sendTelemetry();
    printDebug();
    
    // Мигание статуса
    static unsigned long lastBlink = 0;
    if (millis() - lastBlink > 500) {
        digitalWrite(LED_STATUS_PIN, !digitalRead(LED_STATUS_PIN));
        lastBlink = millis();
    }
    
    delay(5);
}
