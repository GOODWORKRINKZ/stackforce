/**
 * @file main.cpp
 * @brief Голова робота (передняя часть) - ESP32 с PPM приемником
 * 
 * Функции:
 * - Прием PPM от RC приемника
 * - Управление передними 2 BLDC моторами
 * - Управление передними 4 сервоприводами (2 ноги)
 * - Чтение IMU (MPU6050)
 * - Отправка команд задней части по CAN
 */

#include <Arduino.h>
#include <Wire.h>
#include <ESP32CAN.h>
#include <CAN_config.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "config.h"

// Объекты
CAN_device_t CAN_cfg;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDR);
Adafruit_MPU6050 mpu;

// PPM данные
volatile uint16_t ppmChannels[PPM_CHANNELS];
volatile uint8_t ppmChannelIndex = 0;
volatile uint32_t ppmLastPulseTime = 0;
volatile bool ppmFrameReady = false;

// Структура команды управления для отправки задней части
struct RobotCommand {
    int8_t throttle;      // -100..100
    int8_t steering;      // -100..100
    uint8_t mode;         // 0..255
    uint8_t speed;        // 0..255
    float roll;           // IMU крен
    float pitch;          // IMU тангаж
} __attribute__((packed));

RobotCommand robotCmd = {0, 0, 128, 128, 0.0, 0.0};

// Текущие скорости передних моторов
int16_t motorSpeedFL = 0;
int16_t motorSpeedFR = 0;

// Углы передних сервоприводов
uint8_t servoAngles[4] = {90, 90, 90, 90};

// IMU данные
float imuRoll = 0, imuPitch = 0, imuYaw = 0;

// Время обновления
unsigned long lastPPMTime = 0;
unsigned long lastIMUTime = 0;
unsigned long lastControlTime = 0;

/**
 * @brief Обработчик прерывания PPM
 */
void IRAM_ATTR ppmInterrupt() {
    uint32_t currentTime = micros();
    uint32_t pulseDuration = currentTime - ppmLastPulseTime;
    ppmLastPulseTime = currentTime;

    if (pulseDuration > 3000) {
        ppmChannelIndex = 0;
        ppmFrameReady = true;
    } 
    else if (ppmChannelIndex < PPM_CHANNELS) {
        ppmChannels[ppmChannelIndex] = pulseDuration;
        ppmChannelIndex++;
    }
}

/**
 * @brief Инициализация CAN
 */
void setupCAN() {
    CAN_cfg.speed = CAN_SPEED;
    CAN_cfg.tx_pin_id = (gpio_num_t)CAN_TX_PIN;
    CAN_cfg.rx_pin_id = (gpio_num_t)CAN_RX_PIN;
    CAN_cfg.rx_queue = xQueueCreate(10, sizeof(CAN_frame_t));
    
    ESP32Can.CANInit();
    Serial.println("[HEAD] CAN инициализирован (1 Mbps)");
}

/**
 * @brief Инициализация PCA9685 и сервоприводов
 */
void setupServos() {
    pwm.begin();
    pwm.setPWMFreq(SERVO_FREQ);
    
    // Установка начальной позиции (стоя)
    for (int i = 0; i < 4; i++) {
        setServoAngle(i, 90);
    }
    
    Serial.println("[HEAD] Сервоприводы инициализированы");
}

/**
 * @brief Инициализация IMU
 */
void setupIMU() {
    if (!mpu.begin(IMU_ADDR, &Wire, 0)) {
        Serial.println("[HEAD] ОШИБКА: MPU6050 не найден!");
        digitalWrite(LED_IMU_PIN, LOW);
        return;
    }
    
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    
    digitalWrite(LED_IMU_PIN, HIGH);
    Serial.println("[HEAD] IMU инициализирован");
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
 * @brief Обработка PPM данных
 */
void processPPM() {
    if (!ppmFrameReady) return;
    ppmFrameReady = false;

    // Проверка валидности
    bool valid = true;
    for (int i = 0; i < 4; i++) {
        if (ppmChannels[i] < 800 || ppmChannels[i] > 2200) {
            valid = false;
            break;
        }
    }

    if (!valid) {
        robotCmd.throttle = 0;
        robotCmd.steering = 0;
        digitalWrite(LED_PPM_PIN, LOW);
        Serial.println("[HEAD] ПОТЕРЯ PPM СИГНАЛА!");
        return;
    }

    digitalWrite(LED_PPM_PIN, HIGH);
    lastPPMTime = millis();

    // Маппинг каналов
    int8_t throttle = map(ppmChannels[CH_THROTTLE], PPM_MIN_VALUE, PPM_MAX_VALUE, -100, 100);
    int8_t steering = map(ppmChannels[CH_STEERING], PPM_MIN_VALUE, PPM_MAX_VALUE, -100, 100);
    
    if (abs(throttle) < JOYSTICK_DEADZONE) throttle = 0;
    if (abs(steering) < JOYSTICK_DEADZONE) steering = 0;
    
    robotCmd.throttle = throttle;
    robotCmd.steering = steering;
    robotCmd.mode = map(ppmChannels[CH_MODE], PPM_MIN_VALUE, PPM_MAX_VALUE, 0, 255);
    robotCmd.speed = map(ppmChannels[CH_SPEED], PPM_MIN_VALUE, PPM_MAX_VALUE, 0, 255);
}

/**
 * @brief Обработка IMU
 */
void processIMU() {
    if (millis() - lastIMUTime < (1000 / IMU_UPDATE_RATE)) return;
    
    sensors_event_t accel, gyro, temp;
    mpu.getEvent(&accel, &gyro, &temp);
    
    // Углы из акселерометра
    float accelRoll = atan2(accel.acceleration.y, accel.acceleration.z) * 180.0 / PI;
    float accelPitch = atan2(-accel.acceleration.x, 
                            sqrt(accel.acceleration.y * accel.acceleration.y + 
                                 accel.acceleration.z * accel.acceleration.z)) * 180.0 / PI;
    
    // Интегрирование гироскопа
    float dt = (millis() - lastIMUTime) / 1000.0;
    static float gyroRoll = 0, gyroPitch = 0;
    
    gyroRoll += gyro.gyro.x * 180.0 / PI * dt;
    gyroPitch += gyro.gyro.y * 180.0 / PI * dt;
    imuYaw += gyro.gyro.z * 180.0 / PI * dt;
    
    // Комплементарный фильтр
    imuRoll = IMU_FILTER_ALPHA * gyroRoll + (1.0 - IMU_FILTER_ALPHA) * accelRoll;
    imuPitch = IMU_FILTER_ALPHA * gyroPitch + (1.0 - IMU_FILTER_ALPHA) * accelPitch;
    
    gyroRoll = imuRoll;
    gyroPitch = imuPitch;
    
    robotCmd.roll = imuRoll;
    robotCmd.pitch = imuPitch;
    
    lastIMUTime = millis();
}

/**
 * @brief Управление передними моторами и сервоприводами
 */
void controlFrontLegs() {
    if (millis() - lastControlTime < (1000 / CONTROL_LOOP_FREQ)) return;
    
    // Проверка таймаута PPM
    if (millis() - lastPPMTime > PPM_TIMEOUT) {
        motorSpeedFL = 0;
        motorSpeedFR = 0;
        setMotorSpeed(CAN_ID_MOTOR_FL, 0);
        setMotorSpeed(CAN_ID_MOTOR_FR, 0);
        lastControlTime = millis();
        return;
    }
    
    // Вычисление скоростей моторов
    float speedMult = robotCmd.speed / 255.0;
    int16_t baseSpeed = map(robotCmd.throttle, -100, 100, MOTOR_MIN_SPEED, MOTOR_MAX_SPEED);
    int16_t turnOffset = map(robotCmd.steering, -100, 100, -1000, 1000);
    
    motorSpeedFL = (baseSpeed - turnOffset) * speedMult;
    motorSpeedFR = (baseSpeed + turnOffset) * speedMult;
    
    setMotorSpeed(CAN_ID_MOTOR_FL, motorSpeedFL);
    setMotorSpeed(CAN_ID_MOTOR_FR, motorSpeedFR);
    
    // Управление высотой стойки
    uint8_t legAngle = map(robotCmd.mode, 0, 255, 60, 120);
    setServoAngle(SERVO_FL_HIP, legAngle);
    setServoAngle(SERVO_FR_HIP, legAngle);
    
    lastControlTime = millis();
}

/**
 * @brief Отправка команд задней части
 */
void sendToRear() {
    static unsigned long lastSend = 0;
    if (millis() - lastSend < 20) return;  // 50 Hz
    
    CAN_frame_t tx_frame;
    tx_frame.MsgID = CAN_ID_HEAD_TO_REAR;
    tx_frame.FIR.B.FF = CAN_frame_std;
    tx_frame.FIR.B.DLC = 8;
    
    tx_frame.data.u8[0] = robotCmd.throttle + 100;  // 0..200
    tx_frame.data.u8[1] = robotCmd.steering + 100;  // 0..200
    tx_frame.data.u8[2] = robotCmd.mode;
    tx_frame.data.u8[3] = robotCmd.speed;
    
    int16_t roll_int = (int16_t)(robotCmd.roll * 10);
    int16_t pitch_int = (int16_t)(robotCmd.pitch * 10);
    tx_frame.data.u8[4] = (roll_int >> 8) & 0xFF;
    tx_frame.data.u8[5] = roll_int & 0xFF;
    tx_frame.data.u8[6] = (pitch_int >> 8) & 0xFF;
    tx_frame.data.u8[7] = pitch_int & 0xFF;
    
    ESP32Can.CANWriteFrame(&tx_frame);
    
    digitalWrite(LED_CAN_PIN, !digitalRead(LED_CAN_PIN));
    lastSend = millis();
}

/**
 * @brief Вывод отладочной информации
 */
void printDebug() {
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint < 500) return;
    
    Serial.printf("[HEAD] PPM: T=%d S=%d M=%d Sp=%d | Motors: FL=%d FR=%d | IMU: R=%.1f P=%.1f\n",
                 robotCmd.throttle, robotCmd.steering, robotCmd.mode, robotCmd.speed,
                 motorSpeedFL, motorSpeedFR, imuRoll, imuPitch);
    
    lastPrint = millis();
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("\n========================================");
    Serial.println("  ГОЛОВА РОБОТА (Передняя часть)");
    Serial.println("  ESP32 с PPM приемником");
    Serial.println("========================================\n");
    
    // Инициализация пинов
    pinMode(PPM_INPUT_PIN, INPUT);
    pinMode(LED_STATUS_PIN, OUTPUT);
    pinMode(LED_PPM_PIN, OUTPUT);
    pinMode(LED_CAN_PIN, OUTPUT);
    pinMode(LED_IMU_PIN, OUTPUT);
    
    digitalWrite(LED_STATUS_PIN, HIGH);
    
    // Инициализация I2C
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQ);
    
    // Инициализация подсистем
    setupCAN();
    setupServos();
    setupIMU();
    
    // Настройка прерывания PPM
    attachInterrupt(digitalPinToInterrupt(PPM_INPUT_PIN), ppmInterrupt, RISING);
    Serial.printf("[HEAD] PPM приемник на GPIO %d\n", PPM_INPUT_PIN);
    
    Serial.println("[HEAD] Инициализация завершена\n");
    
    lastPPMTime = millis();
}

void loop() {
    processPPM();
    processIMU();
    controlFrontLegs();
    sendToRear();
    printDebug();
    
    // Мигание статуса
    static unsigned long lastBlink = 0;
    if (millis() - lastBlink > 1000) {
        digitalWrite(LED_STATUS_PIN, !digitalRead(LED_STATUS_PIN));
        lastBlink = millis();
    }
    
    delay(5);
}
