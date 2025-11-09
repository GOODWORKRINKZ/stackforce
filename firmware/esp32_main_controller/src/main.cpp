/**
 * @file main.cpp
 * @brief Главный контроллер ESP32 для четырехногого робота StackForce
 * 
 * Управляет:
 * - 4 BLDC моторами через CAN шину
 * - 8 сервоприводами ног через PCA9685 (I2C)
 * - Получает команды управления от второго ESP32 по CAN
 */

#include <Arduino.h>
#include <Wire.h>
#include <ESP32CAN.h>
#include <CAN_config.h>
#include <Adafruit_PWMServoDriver.h>
#include "config.h"

// CAN конфигурация
CAN_device_t CAN_cfg;

// PCA9685 драйвер сервоприводов
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDRESS);

// Структура команды управления от PPM контроллера
struct ControlCommand {
    int8_t throttle;    // -100 до 100 (вперед/назад)
    int8_t steering;    // -100 до 100 (поворот)
    uint8_t mode;       // 0-255 (режим: высота стойки)
    uint8_t speed;      // 0-255 (множитель скорости)
} __attribute__((packed));

ControlCommand currentCommand = {0, 0, 128, 128};

// Данные IMU от PPM контроллера
struct IMUData {
    float roll;         // Крен
    float pitch;        // Тангаж
    float yaw;          // Рыскание
    int16_t accel_x;    // Ускорение X
    int16_t accel_y;    // Ускорение Y
    int16_t accel_z;    // Ускорение Z
} __attribute__((packed));

IMUData imuData = {0, 0, 0, 0, 0, 0};

// Текущие скорости моторов (RPM)
int16_t motorSpeeds[4] = {0, 0, 0, 0};

// Текущие углы сервоприводов
uint8_t servoAngles[8] = {90, 90, 90, 90, 90, 90, 90, 90};

// Время последнего обновления
unsigned long lastCommandTime = 0;
unsigned long lastTelemetryTime = 0;

/**
 * @brief Инициализация CAN шины
 */
void setupCAN() {
    CAN_cfg.speed = CAN_SPEED;
    CAN_cfg.tx_pin_id = (gpio_num_t)CAN_TX_PIN;
    CAN_cfg.rx_pin_id = (gpio_num_t)CAN_RX_PIN;
    CAN_cfg.rx_queue = xQueueCreate(20, sizeof(CAN_frame_t));
    
    ESP32Can.CANInit();
    
    Serial.println("[MAIN] CAN шина инициализирована");
    Serial.printf("[MAIN] CAN: TX=%d, RX=%d, Speed=1Mbps\n", CAN_TX_PIN, CAN_RX_PIN);
}

/**
 * @brief Инициализация PCA9685 для сервоприводов
 */
void setupServos() {
    Wire.begin(SERVO_SDA_PIN, SERVO_SCL_PIN);
    
    pwm.begin();
    pwm.setPWMFreq(SERVO_FREQUENCY);
    
    // Установка начальной позиции (стоя, все серво по центру)
    for (int i = 0; i < 8; i++) {
        setServoAngle(i, 90);
    }
    
    Serial.println("[MAIN] PCA9685 инициализирован, сервоприводы в центральной позиции");
}

/**
 * @brief Установка угла сервопривода
 * @param channel Канал PCA9685 (0-7)
 * @param angle Угол в градусах (0-180)
 */
void setServoAngle(uint8_t channel, uint8_t angle) {
    angle = constrain(angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
    
    // Преобразование угла в ширину импульса для PCA9685
    uint16_t pulse = map(angle, 0, 180, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
    pwm.setPWM(channel, 0, pulse);
    
    servoAngles[channel] = angle;
}

/**
 * @brief Отправка команды на BLDC мотор через CAN
 * @param motorId CAN ID мотора
 * @param speed Скорость в RPM (-3000 до 3000)
 */
void setMotorSpeed(uint16_t motorId, int16_t speed) {
    speed = constrain(speed, MOTOR_MIN_SPEED, MOTOR_MAX_SPEED);
    
    CAN_frame_t tx_frame;
    tx_frame.MsgID = motorId;
    tx_frame.FIR.B.FF = CAN_frame_std;
    tx_frame.FIR.B.DLC = 8;
    
    // Формат команды для BLDC мотора (StackForce протокол)
    tx_frame.data.u8[0] = MOTOR_CMD_SET_SPEED;
    tx_frame.data.u8[1] = (speed >> 8) & 0xFF;  // Старший байт скорости
    tx_frame.data.u8[2] = speed & 0xFF;         // Младший байт скорости
    tx_frame.data.u8[3] = 0x00;                 // Резерв
    tx_frame.data.u8[4] = 0x00;
    tx_frame.data.u8[5] = 0x00;
    tx_frame.data.u8[6] = 0x00;
    tx_frame.data.u8[7] = 0x00;
    
    ESP32Can.CANWriteFrame(&tx_frame);
}

/**
 * @brief Обработка команд управления
 */
void processControlCommand() {
    // Вычисление скоростей моторов на основе throttle и steering
    float speedMultiplier = currentCommand.speed / 255.0;
    
    // Базовая скорость из throttle (-100 до 100 -> -3000 до 3000 RPM)
    int16_t baseSpeed = map(currentCommand.throttle, -100, 100, MOTOR_MIN_SPEED, MOTOR_MAX_SPEED);
    
    // Смещение для поворота
    int16_t turnOffset = map(currentCommand.steering, -100, 100, -1000, 1000);
    
    // Применение дифференциального рулевого управления
    int16_t leftSpeed = (baseSpeed - turnOffset) * speedMultiplier;
    int16_t rightSpeed = (baseSpeed + turnOffset) * speedMultiplier;
    
    // Установка скоростей моторов
    motorSpeeds[0] = leftSpeed;   // Передний левый
    motorSpeeds[1] = rightSpeed;  // Передний правый
    motorSpeeds[2] = leftSpeed;   // Задний левый
    motorSpeeds[3] = rightSpeed;  // Задний правый
    
    // Отправка команд на моторы
    setMotorSpeed(CAN_ID_MOTOR_FL, motorSpeeds[0]);
    setMotorSpeed(CAN_ID_MOTOR_FR, motorSpeeds[1]);
    setMotorSpeed(CAN_ID_MOTOR_BL, motorSpeeds[2]);
    setMotorSpeed(CAN_ID_MOTOR_BR, motorSpeeds[3]);
    
    // Управление высотой стойки на основе mode
    uint8_t legAngle = map(currentCommand.mode, 0, 255, 60, 120);
    
    // Установка углов всех бедренных сервоприводов
    setServoAngle(SERVO_FL_HIP, legAngle);
    setServoAngle(SERVO_FR_HIP, legAngle);
    setServoAngle(SERVO_BL_HIP, legAngle);
    setServoAngle(SERVO_BR_HIP, legAngle);
}

/**
 * @brief Чтение команд от PPM контроллера через CAN
 */
void readCANCommands() {
    CAN_frame_t rx_frame;
    
    while (CAN_OK == ESP32Can.CANReadFrame(&rx_frame)) {
        // Команды управления
        if (rx_frame.MsgID == CAN_ID_CONTROL_CMD && rx_frame.FIR.B.DLC >= sizeof(ControlCommand)) {
            memcpy(&currentCommand, rx_frame.data.u8, sizeof(ControlCommand));
            lastCommandTime = millis();
            
            digitalWrite(LED_CAN_PIN, !digitalRead(LED_CAN_PIN));  // Мигание при приеме
            
            static unsigned long lastPrint = 0;
            if (millis() - lastPrint > 500) {
                Serial.printf("[MAIN] Команда: T=%d, S=%d, M=%d, Sp=%d\n",
                             currentCommand.throttle, currentCommand.steering,
                             currentCommand.mode, currentCommand.speed);
                lastPrint = millis();
            }
        }
        
        // Данные IMU
        else if (rx_frame.MsgID == CAN_ID_IMU_DATA && rx_frame.FIR.B.DLC >= sizeof(IMUData)) {
            memcpy(&imuData, rx_frame.data.u8, min((size_t)rx_frame.FIR.B.DLC, sizeof(IMUData)));
            
            static unsigned long lastIMUPrint = 0;
            if (millis() - lastIMUPrint > 1000) {
                Serial.printf("[MAIN] IMU: Roll=%.1f, Pitch=%.1f, Yaw=%.1f\n",
                             imuData.roll, imuData.pitch, imuData.yaw);
                lastIMUPrint = millis();
            }
        }
    }
    
    // Проверка таймаута команд (аварийная остановка)
    if (millis() - lastCommandTime > 1000) {
        // Остановка всех моторов
        for (int i = 0; i < 4; i++) {
            motorSpeeds[i] = 0;
        }
        setMotorSpeed(CAN_ID_MOTOR_FL, 0);
        setMotorSpeed(CAN_ID_MOTOR_FR, 0);
        setMotorSpeed(CAN_ID_MOTOR_BL, 0);
        setMotorSpeed(CAN_ID_MOTOR_BR, 0);
        
        static unsigned long lastWarning = 0;
        if (millis() - lastWarning > 2000) {
            Serial.println("[MAIN] ВНИМАНИЕ: Потеря связи с PPM контроллером!");
            lastWarning = millis();
        }
    }
}

/**
 * @brief Отправка телеметрии
 */
void sendTelemetry() {
    if (millis() - lastTelemetryTime < 100) return;  // 10 Hz
    
    CAN_frame_t tx_frame;
    tx_frame.MsgID = CAN_ID_TELEMETRY;
    tx_frame.FIR.B.FF = CAN_frame_std;
    tx_frame.FIR.B.DLC = 8;
    
    // Упаковка телеметрии
    tx_frame.data.u8[0] = 0x01;  // Статус: OK
    tx_frame.data.u8[1] = (motorSpeeds[0] >> 8) & 0xFF;
    tx_frame.data.u8[2] = motorSpeeds[0] & 0xFF;
    tx_frame.data.u8[3] = (motorSpeeds[1] >> 8) & 0xFF;
    tx_frame.data.u8[4] = motorSpeeds[1] & 0xFF;
    tx_frame.data.u8[5] = servoAngles[SERVO_FL_HIP];
    tx_frame.data.u8[6] = 0x00;
    tx_frame.data.u8[7] = 0x00;
    
    ESP32Can.CANWriteFrame(&tx_frame);
    lastTelemetryTime = millis();
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("\n[MAIN] =============================================");
    Serial.println("[MAIN] ESP32 Главный контроллер StackForce робота");
    Serial.println("[MAIN] =============================================");
    
    // Инициализация пинов индикации
    pinMode(LED_STATUS_PIN, OUTPUT);
    pinMode(LED_CAN_PIN, OUTPUT);
    digitalWrite(LED_STATUS_PIN, HIGH);
    
    // Инициализация подсистем
    setupCAN();
    setupServos();
    
    Serial.println("[MAIN] Инициализация завершена");
    Serial.println("[MAIN] Ожидание команд от PPM контроллера...");
}

void loop() {
    readCANCommands();
    processControlCommand();
    sendTelemetry();
    
    // Мигание светодиода статуса
    static unsigned long lastBlink = 0;
    if (millis() - lastBlink > 1000) {
        digitalWrite(LED_STATUS_PIN, !digitalRead(LED_STATUS_PIN));
        lastBlink = millis();
    }
    
    delay(20);  // 50 Hz основной цикл
}
