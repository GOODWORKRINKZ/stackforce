/**
 * @file main.cpp
 * @brief ESP32 PPM контроллер с IMU для робота StackForce
 * 
 * Функции:
 * - Прием и декодирование PPM сигнала от RC приемника
 * - Чтение данных IMU (MPU6050)
 * - Фильтрация IMU (комплементарный фильтр)
 * - Отправка команд управления по CAN шине главному ESP32
 */

#include <Arduino.h>
#include <Wire.h>
#include <ESP32CAN.h>
#include <CAN_config.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "config.h"

// CAN конфигурация
CAN_device_t CAN_cfg;

// IMU сенсор
Adafruit_MPU6050 mpu;

// Переменные для декодирования PPM
volatile uint16_t ppmChannels[PPM_CHANNELS];
volatile uint8_t ppmChannelIndex = 0;
volatile uint32_t ppmLastPulseTime = 0;
volatile bool ppmFrameReady = false;

// Структура команды управления
struct ControlCommand {
    int8_t throttle;    // -100 до 100
    int8_t steering;    // -100 до 100
    uint8_t mode;       // 0-255
    uint8_t speed;      // 0-255
} __attribute__((packed));

ControlCommand controlCommand = {0, 0, 128, 128};

// Структура данных IMU
struct IMUData {
    float roll;         // Крен
    float pitch;        // Тангаж
    float yaw;          // Рыскание
    int16_t accel_x;    // Ускорение X
    int16_t accel_y;    // Ускорение Y
    int16_t accel_z;    // Ускорение Z
} __attribute__((packed));

IMUData imuData = {0, 0, 0, 0, 0, 0};

// Время последнего обновления
unsigned long lastPPMTime = 0;
unsigned long lastIMUTime = 0;
unsigned long lastCANSendTime = 0;

/**
 * @brief Обработчик прерывания для PPM сигнала
 */
void IRAM_ATTR ppmInterrupt() {
    uint32_t currentTime = micros();
    uint32_t pulseDuration = currentTime - ppmLastPulseTime;
    ppmLastPulseTime = currentTime;

    // Начало нового фрейма (длинный импульс > 3ms)
    if (pulseDuration > 3000) {
        ppmChannelIndex = 0;
        ppmFrameReady = true;
    } 
    // Обычный канал
    else if (ppmChannelIndex < PPM_CHANNELS) {
        ppmChannels[ppmChannelIndex] = pulseDuration;
        ppmChannelIndex++;
    }
}

/**
 * @brief Преобразование PPM в диапазон -100 до 100
 */
int8_t mapPPMToRange(uint16_t ppmValue) {
    return map(constrain(ppmValue, 1000, 2000), 1000, 2000, -100, 100);
}

/**
 * @brief Преобразование PPM в диапазон 0 до 255
 */
uint8_t mapPPMToByte(uint16_t ppmValue) {
    return map(constrain(ppmValue, 1000, 2000), 1000, 2000, 0, 255);
}

/**
 * @brief Инициализация CAN шины
 */
void setupCAN() {
    CAN_cfg.speed = CAN_SPEED;
    CAN_cfg.tx_pin_id = (gpio_num_t)CAN_TX_PIN;
    CAN_cfg.rx_pin_id = (gpio_num_t)CAN_RX_PIN;
    CAN_cfg.rx_queue = xQueueCreate(10, sizeof(CAN_frame_t));
    
    ESP32Can.CANInit();
    
    Serial.println("[PPM] CAN шина инициализирована");
    Serial.printf("[PPM] CAN: TX=%d, RX=%d, Speed=1Mbps\n", CAN_TX_PIN, CAN_RX_PIN);
}

/**
 * @brief Инициализация IMU
 */
void setupIMU() {
    Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN);
    
    if (!mpu.begin(0x68, &Wire, 0)) {
        Serial.println("[PPM] ОШИБКА: MPU6050 не найден!");
        while (1) {
            digitalWrite(LED_IMU_PIN, !digitalRead(LED_IMU_PIN));
            delay(200);
        }
    }
    
    // Настройка диапазонов
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    
    Serial.println("[PPM] MPU6050 инициализирован");
    digitalWrite(LED_IMU_PIN, HIGH);
}

/**
 * @brief Обработка PPM данных
 */
void processPPMData() {
    if (!ppmFrameReady) return;
    
    ppmFrameReady = false;

    // Проверка валидности данных
    bool validData = true;
    for (int i = 0; i < 4; i++) {
        if (ppmChannels[i] < 800 || ppmChannels[i] > 2200) {
            validData = false;
            break;
        }
    }

    if (!validData) {
        // Аварийная остановка при потере сигнала
        controlCommand.throttle = 0;
        controlCommand.steering = 0;
        Serial.println("[PPM] ВНИМАНИЕ: Потеря PPM сигнала!");
        digitalWrite(LED_PPM_PIN, LOW);
        return;
    }

    digitalWrite(LED_PPM_PIN, HIGH);
    lastPPMTime = millis();

    // Маппинг каналов на команды
    int8_t throttle = mapPPMToRange(ppmChannels[PPM_CHANNEL_THROTTLE]);
    int8_t steering = mapPPMToRange(ppmChannels[PPM_CHANNEL_STEERING]);
    
    // Дедзона для джойстиков
    if (abs(throttle) < JOYSTICK_DEADZONE) throttle = 0;
    if (abs(steering) < JOYSTICK_DEADZONE) steering = 0;
    
    controlCommand.throttle = throttle;
    controlCommand.steering = steering;
    controlCommand.mode = mapPPMToByte(ppmChannels[PPM_CHANNEL_MODE]);
    controlCommand.speed = mapPPMToByte(ppmChannels[PPM_CHANNEL_SPEED]);

    // Отладочная информация
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 500) {
        Serial.printf("[PPM] T=%d S=%d M=%d Sp=%d | CH:[%d,%d,%d,%d]\n",
                     controlCommand.throttle, controlCommand.steering,
                     controlCommand.mode, controlCommand.speed,
                     ppmChannels[0], ppmChannels[1], ppmChannels[2], ppmChannels[3]);
        lastPrint = millis();
    }
}

/**
 * @brief Обработка данных IMU с комплементарным фильтром
 */
void processIMUData() {
    if (millis() - lastIMUTime < (1000 / IMU_UPDATE_RATE)) return;
    
    sensors_event_t accel, gyro, temp;
    mpu.getEvent(&accel, &gyro, &temp);
    
    // Сохранение сырых данных ускорения
    imuData.accel_x = (int16_t)(accel.acceleration.x * 100);
    imuData.accel_y = (int16_t)(accel.acceleration.y * 100);
    imuData.accel_z = (int16_t)(accel.acceleration.z * 100);
    
    // Вычисление углов из акселерометра
    float accelRoll = atan2(accel.acceleration.y, accel.acceleration.z) * 180.0 / PI;
    float accelPitch = atan2(-accel.acceleration.x, 
                            sqrt(accel.acceleration.y * accel.acceleration.y + 
                                 accel.acceleration.z * accel.acceleration.z)) * 180.0 / PI;
    
    // Интегрирование гироскопа
    float dt = (millis() - lastIMUTime) / 1000.0;
    static float gyroRoll = 0, gyroPitch = 0, gyroYaw = 0;
    
    gyroRoll += gyro.gyro.x * 180.0 / PI * dt;
    gyroPitch += gyro.gyro.y * 180.0 / PI * dt;
    gyroYaw += gyro.gyro.z * 180.0 / PI * dt;
    
    // Комплементарный фильтр
    imuData.roll = IMU_FILTER_ALPHA * gyroRoll + (1.0 - IMU_FILTER_ALPHA) * accelRoll;
    imuData.pitch = IMU_FILTER_ALPHA * gyroPitch + (1.0 - IMU_FILTER_ALPHA) * accelPitch;
    imuData.yaw = gyroYaw;  // Только гироскоп для yaw
    
    // Обновление интегрированных значений
    gyroRoll = imuData.roll;
    gyroPitch = imuData.pitch;
    
    lastIMUTime = millis();
    
    // Отладочная информация
    static unsigned long lastIMUPrint = 0;
    if (millis() - lastIMUPrint > 1000) {
        Serial.printf("[PPM] IMU: R=%.1f° P=%.1f° Y=%.1f° | Acc:[%d,%d,%d]\n",
                     imuData.roll, imuData.pitch, imuData.yaw,
                     imuData.accel_x, imuData.accel_y, imuData.accel_z);
        lastIMUPrint = millis();
    }
}

/**
 * @brief Отправка данных по CAN шине
 */
void sendCANData() {
    if (millis() - lastCANSendTime < 20) return;  // 50 Hz
    
    // Отправка команды управления
    CAN_frame_t tx_frame;
    tx_frame.MsgID = CAN_ID_CONTROL_CMD;
    tx_frame.FIR.B.FF = CAN_frame_std;
    tx_frame.FIR.B.DLC = sizeof(ControlCommand);
    
    memcpy(tx_frame.data.u8, &controlCommand, sizeof(ControlCommand));
    ESP32Can.CANWriteFrame(&tx_frame);
    
    // Отправка данных IMU (каждые 100мс)
    static unsigned long lastIMUSend = 0;
    if (millis() - lastIMUSend > 100) {
        tx_frame.MsgID = CAN_ID_IMU_DATA;
        tx_frame.FIR.B.DLC = 8;  // Отправим только первые 8 байт
        
        // Упаковка данных IMU
        int16_t roll_int = (int16_t)(imuData.roll * 10);
        int16_t pitch_int = (int16_t)(imuData.pitch * 10);
        int16_t yaw_int = (int16_t)(imuData.yaw * 10);
        
        tx_frame.data.u8[0] = (roll_int >> 8) & 0xFF;
        tx_frame.data.u8[1] = roll_int & 0xFF;
        tx_frame.data.u8[2] = (pitch_int >> 8) & 0xFF;
        tx_frame.data.u8[3] = pitch_int & 0xFF;
        tx_frame.data.u8[4] = (yaw_int >> 8) & 0xFF;
        tx_frame.data.u8[5] = yaw_int & 0xFF;
        tx_frame.data.u8[6] = 0x00;
        tx_frame.data.u8[7] = 0x00;
        
        ESP32Can.CANWriteFrame(&tx_frame);
        lastIMUSend = millis();
    }
    
    digitalWrite(LED_CAN_PIN, !digitalRead(LED_CAN_PIN));
    lastCANSendTime = millis();
}

/**
 * @brief Проверка таймаута PPM
 */
void checkPPMTimeout() {
    if (millis() - lastPPMTime > PPM_TIMEOUT) {
        // Остановка при потере сигнала
        controlCommand.throttle = 0;
        controlCommand.steering = 0;
        digitalWrite(LED_PPM_PIN, LOW);
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("\n[PPM] =============================================");
    Serial.println("[PPM] ESP32 PPM контроллер с IMU");
    Serial.println("[PPM] =============================================");
    
    // Инициализация пинов
    pinMode(PPM_INPUT_PIN, INPUT);
    pinMode(LED_STATUS_PIN, OUTPUT);
    pinMode(LED_CAN_PIN, OUTPUT);
    pinMode(LED_PPM_PIN, OUTPUT);
    pinMode(LED_IMU_PIN, OUTPUT);
    
    digitalWrite(LED_STATUS_PIN, HIGH);
    digitalWrite(LED_CAN_PIN, LOW);
    digitalWrite(LED_PPM_PIN, LOW);
    digitalWrite(LED_IMU_PIN, LOW);
    
    // Инициализация подсистем
    setupCAN();
    setupIMU();
    
    // Настройка прерывания для PPM
    attachInterrupt(digitalPinToInterrupt(PPM_INPUT_PIN), ppmInterrupt, RISING);
    Serial.printf("[PPM] PPM приемник на пине %d\n", PPM_INPUT_PIN);
    
    Serial.println("[PPM] Инициализация завершена");
    Serial.println("[PPM] Ожидание PPM сигнала...");
    
    lastPPMTime = millis();
    lastIMUTime = millis();
}

void loop() {
    processPPMData();
    processIMUData();
    sendCANData();
    checkPPMTimeout();
    
    // Мигание светодиода статуса
    static unsigned long lastBlink = 0;
    if (millis() - lastBlink > 500) {
        digitalWrite(LED_STATUS_PIN, !digitalRead(LED_STATUS_PIN));
        lastBlink = millis();
    }
    
    delay(10);  // 100 Hz основной цикл
}
