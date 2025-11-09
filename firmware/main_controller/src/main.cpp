/**
 * @file main.cpp
 * @brief Main контроллер четырехногого колесного робота
 * 
 * Функции:
 * - Прием SBUS от RC приемника
 * - Управление 2 передними BLDC моторами (M0, M1)
 * - Управление ВСЕХ 8 сервоприводов (4 ноги с 5-звенным механизмом)
 * - Чтение IMU (MPU6050)
 * - Обратная кинематика для управления всеми 4 ногами
 * - PID стабилизация (pitch, roll, скорость)
 * - Отправка команд aux контроллеру по CAN
 */

#include <Arduino.h>
#include <Wire.h>
#include "SF_Servo.h"
#include "SF_BLDC.h"
#include "SF_IMU.h"
#include "pid.h"
#include "sbus.h"
#include "quadrupedal_data.h"
#include "driver/twai.h"
#include "config.h"

// ==================== ГЛОБАЛЬНЫЕ ОБЪЕКТЫ ====================
SF_Servo servos = SF_Servo(Wire);           // PCA9685 драйвер серво
SF_IMU mpu6050 = SF_IMU(Wire);              // MPU6050 IMU
SF_BLDC motors = SF_BLDC(Serial2);          // BLDC моторы
bfs::SbusRx sbusRx(&Serial1);               // SBUS приемник

static bool twai_installed = false;         // Флаг инициализации TWAI

// ==================== PID КОНТРОЛЛЕРЫ ====================
PIDController PID_VEL{0.2, 0, 0, 1000, 50};     // PID для скорости
PIDController PID_PITCH{0.38, 0, 0, 1000, 50};  // PID для тангажа (pitch)
PIDController PID_ROLL{0.05, 0, 0, 1000, 50};   // PID для крена (roll)

// ==================== ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ ====================
int RCValue[6] = {1000, 1000, 1000, 1000, 1000, 1000};  // RC каналы
std::array<int16_t, bfs::SbusRx::NUM_CH()> sbusData;

robotposeparam robotPose;      // Текущая поза робота (IMU)
robotmotionparam robotMotion;  // Команды движения
IKparam IKParam;               // Параметры обратной кинематики
SF_BLDC_DATA BLDCData;         // Данные от BLDC моторов

// Направления моторов (-1 или 1)
int M0Dir = 1;
int M1Dir = -1;

// Параметры управления
float X = 0, Y = 90;              // Координаты конца ноги (X, Y в мм)
float Y_demand = 90;              // Желаемая высота ноги
float Kp_Y = 0.1;                 // Коэффициент для плавного изменения высоты
float Kp_X = 1.1;                 // Коэффициент для компенсации скорости по X
float Kp_roll = 0.05;             // Коэффициент для адаптации к неровностям
float turnKp = 0.1;               // Коэффициент поворота

float targetSpeed = 0;            // Целевая скорость
float stab_roll = 0;              // Накопленная коррекция крена
float rollLimit = 20;             // Максимальный крен (градусы)
float L = 100;                    // Расстояние между ногами (мм)

uint8_t lowest = ROBOT_LOWEST_FOR_MOT;
uint8_t highest = ROBOT_HIGHEST;

uint8_t loopCnt = 0;

// Углы серво (для всех 8 серво)
uint16_t servoFrontLeftFront = 90, servoFrontLeftRear = 90;
uint16_t servoFrontRightFront = 90, servoFrontRightRear = 90;
uint16_t servoBackLeftFront = 90, servoBackLeftRear = 90;
uint16_t servoBackRightFront = 90, servoBackRightRear = 90;


// ==================== ФУНКЦИИ ====================

/**
 * @brief Чтение значений с SBUS приемника
 */
void getRCValue() {
    if (sbusRx.Read()) {
        sbusData = sbusRx.ch();
        RCValue[0] = sbusData[0];
        RCValue[1] = sbusData[1];
        RCValue[2] = sbusData[2];
        RCValue[3] = sbusData[3];
        RCValue[4] = sbusData[4];
        RCValue[5] = sbusData[5];

        // Ограничение значений
        RCValue[0] = _constrain(RCValue[0], RCCHANNEL_MIN, RCCHANNEL_MAX);
        RCValue[1] = _constrain(RCValue[1], RCCHANNEL_MIN, RCCHANNEL_MAX);
        RCValue[2] = _constrain(RCValue[2], RCCHANNEL3_MIN, RCCHANNEL3_MAX);
        RCValue[3] = _constrain(RCValue[3], RCCHANNEL_MIN, RCCHANNEL_MAX);
    }
}

/**
 * @brief Получение данных MPU6050
 */
void getMPUValue() {
    mpu6050.update();
    // Внимание: из-за ориентации датчика оси могут быть инвертированы
    robotPose.pitch = -mpu6050.angle[0];
    robotPose.roll = mpu6050.angle[1];
    robotPose.yaw = mpu6050.angle[2];
    robotPose.GyroX = mpu6050.gyro[1];
    robotPose.GyroY = -mpu6050.gyro[0];
    robotPose.GyroZ = -mpu6050.gyro[2];
}

/**
 * @brief Обратная кинематика для 5-звенного механизма
 * Вычисляет углы для ВСЕХ 4 ног (передние и задние) и устанавливает серво
 */
void inverseKinematicsAll() {
    int16_t alphaLeftToAngle, betaLeftToAngle;
    int16_t alphaRightToAngle, betaRightToAngle;
    
    // ========== ПЕРЕДНИЕ НОГИ ==========
    float alpha1, alpha2, beta1, beta2;

    // === ЛЕВАЯ ПЕРЕДНЯЯ НОГА ===
    float aLeft = 2 * IKParam.XLeft * L1;
    float bLeft = 2 * IKParam.YLeft * L1;
    float cLeft = IKParam.XLeft * IKParam.XLeft + IKParam.YLeft * IKParam.YLeft + L1 * L1 - L2 * L2;
    float dLeft = 2 * L4 * (IKParam.XLeft - L5);
    float eLeft = 2 * L4 * IKParam.YLeft;
    float fLeft = ((IKParam.XLeft - L5) * (IKParam.XLeft - L5) + L4 * L4 + IKParam.YLeft * IKParam.YLeft - L3 * L3);

    alpha1 = 2 * atan((bLeft + sqrt((aLeft * aLeft) + (bLeft * bLeft) - (cLeft * cLeft))) / (aLeft + cLeft));
    alpha2 = 2 * atan((bLeft - sqrt((aLeft * aLeft) + (bLeft * bLeft) - (cLeft * cLeft))) / (aLeft + cLeft));
    beta1 = 2 * atan((eLeft + sqrt((dLeft * dLeft) + eLeft * eLeft - (fLeft * fLeft))) / (dLeft + fLeft));
    beta2 = 2 * atan((eLeft - sqrt((dLeft * dLeft) + eLeft * eLeft - (fLeft * fLeft))) / (dLeft + fLeft));

    alpha1 = (alpha1 >= 0) ? alpha1 : (alpha1 + 2 * PI);
    alpha2 = (alpha2 >= 0) ? alpha2 : (alpha2 + 2 * PI);

    if (alpha1 >= PI / 4) IKParam.alphaLeft = alpha1;
    else IKParam.alphaLeft = alpha2;
    
    if (beta1 >= 0 && beta1 <= PI / 4) IKParam.betaLeft = beta1;
    else IKParam.betaLeft = beta2;

    // === ПРАВАЯ ПЕРЕДНЯЯ НОГА ===
    float aRight = 2 * IKParam.XRight * L1;
    float bRight = 2 * IKParam.YRight * L1;
    float cRight = IKParam.XRight * IKParam.XRight + IKParam.YRight * IKParam.YRight + L1 * L1 - L2 * L2;
    float dRight = 2 * L4 * (IKParam.XRight - L5);
    float eRight = 2 * L4 * IKParam.YRight;
    float fRight = ((IKParam.XRight - L5) * (IKParam.XRight - L5) + L4 * L4 + IKParam.YRight * IKParam.YRight - L3 * L3);

    alpha1 = 2 * atan((aRight + sqrt((aRight * aRight) + (bRight * bRight) - (cRight * cRight))) / (aRight + cRight));
    alpha2 = 2 * atan((bRight - sqrt((aRight * aRight) + (bRight * bRight) - (cRight * cRight))) / (aRight + cRight));
    beta1 = 2 * atan((eRight + sqrt((dRight * dRight) + eRight * eRight - (fRight * fRight))) / (dRight + fRight));
    beta2 = 2 * atan((eRight - sqrt((dRight * dRight) + eRight * eRight - (fRight * fRight))) / (dRight + fRight));

    alpha1 = (alpha1 >= 0) ? alpha1 : (alpha1 + 2 * PI);
    alpha2 = (alpha2 >= 0) ? alpha2 : (alpha2 + 2 * PI);

    if (alpha1 >= PI / 4) IKParam.alphaRight = alpha1;
    else IKParam.alphaRight = alpha2;
    
    if (beta1 >= 0 && beta1 <= PI / 4) IKParam.betaRight = beta1;
    else IKParam.betaRight = beta2;

    // Конвертация в градусы для передних ног
    alphaLeftToAngle = (int)((IKParam.alphaLeft / 6.28) * 360);
    betaLeftToAngle = (int)((IKParam.betaLeft / 6.28) * 360);
    alphaRightToAngle = (int)((IKParam.alphaRight / 6.28) * 360);
    betaRightToAngle = (int)((IKParam.betaRight / 6.28) * 360);

    // Применение смещений для передних ног
    servoFrontLeftFront = 90 + betaLeftToAngle;
    servoFrontLeftRear = 90 + alphaLeftToAngle;
    servoFrontRightFront = 270 - betaRightToAngle;
    servoFrontRightRear = 270 - alphaRightToAngle;

    // Задние ноги - копируем те же углы (синхронизация)
    servoBackLeftFront = servoFrontLeftFront;
    servoBackLeftRear = servoFrontLeftRear;
    servoBackRightFront = servoFrontRightFront;
    servoBackRightRear = servoFrontRightRear;

    // Установка ВСЕХ 8 серво (по одному)
    servos.setAngle(SERVO_FL_FRONT, servoFrontLeftFront + SERVO_FL_FRONT_OFFSET);
    servos.setAngle(SERVO_FL_REAR,  servoFrontLeftRear  + SERVO_FL_REAR_OFFSET);
    servos.setAngle(SERVO_FR_FRONT, servoFrontRightFront + SERVO_FR_FRONT_OFFSET);
    servos.setAngle(SERVO_FR_REAR,  servoFrontRightRear  + SERVO_FR_REAR_OFFSET);
    servos.setAngle(SERVO_BL_FRONT, servoBackLeftFront   + SERVO_BL_FRONT_OFFSET);
    servos.setAngle(SERVO_BL_REAR,  servoBackLeftRear    + SERVO_BL_REAR_OFFSET);
    servos.setAngle(SERVO_BR_FRONT, servoBackRightFront  + SERVO_BR_FRONT_OFFSET);
    servos.setAngle(SERVO_BR_REAR,  servoBackRightRear   + SERVO_BR_REAR_OFFSET);
}


/**
 * @brief Инициализация CAN шины
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
        Serial.println("[MAIN] TWAI driver установлен");
    } else {
        Serial.println("[MAIN] ОШИБКА: Не удалось установить TWAI driver");
        return;
    }
    
    // Запуск драйвера
    if (twai_start() == ESP_OK) {
        Serial.println("[MAIN] TWAI запущен (1 Mbps)");
        twai_installed = true;
    } else {
        Serial.println("[MAIN] ОШИБКА: Не удалось запустить TWAI");
        return;
    }
}

/**
 * @brief Отправка команд aux контроллеру по CAN
 */
void sendToAux() {
    if (!twai_installed) return;
    
    static unsigned long lastSend = 0;
    if (millis() - lastSend < 20) return;  // 50 Hz
    
    // Создание TWAI сообщения
    twai_message_t tx_msg;
    tx_msg.identifier = CAN_ID_MAIN_TO_AUX;
    tx_msg.data_length_code = 8;
    tx_msg.flags = 0;  // Standard frame, data frame
    
    // Упаковка данных для aux контроллера
    int8_t throttle = (int8_t)constrain(targetSpeed, -100, 100);
    int8_t turn = (int8_t)constrain(robotMotion.turn * 10, -100, 100);
    
    tx_msg.data[0] = throttle + 100;  // 0..200
    tx_msg.data[1] = turn + 100;      // 0..200
    tx_msg.data[2] = (uint8_t)Y_demand;  // Высота
    tx_msg.data[3] = 0;  // Резерв
    
    int16_t roll_int = (int16_t)(robotPose.roll * 10);
    int16_t pitch_int = (int16_t)(robotPose.pitch * 10);
    tx_msg.data[4] = (roll_int >> 8) & 0xFF;
    tx_msg.data[5] = roll_int & 0xFF;
    tx_msg.data[6] = (pitch_int >> 8) & 0xFF;
    tx_msg.data[7] = pitch_int & 0xFF;
    
    // Отправка сообщения
    if (twai_transmit(&tx_msg, pdMS_TO_TICKS(10)) != ESP_OK) {
        Serial.println("[MAIN] TWAI: Ошибка отправки");
    }
    
    lastSend = millis();
}

/**
 * @brief Настройка (инициализация)
 */
void setup() {
    Serial.begin(921600);
    delay(500);
    
    Serial.println("\n========================================");
    Serial.println("  MAIN КОНТРОЛЛЕР (Двуногий робот)");
    Serial.println("  ESP32 с SBUS + IMU + BLDC + Servo");
    Serial.println("========================================\n");
    
    // Инициализация I2C (для IMU и PCA9685)
    Wire.begin(1, 2, 400000UL);
    
    // Инициализация Serial2 для BLDC (RX=17, TX=18)
    Serial2.begin(115200, SERIAL_8N1, 17, 18);
    
    // Инициализация SBUS (Serial1, RX=SBUSPIN)
    sbusRx.Begin(SBUSPIN, -1);
    Serial.printf("[MAIN] SBUS приемник на GPIO %d\n", SBUSPIN);
    
    // Инициализация IMU
    mpu6050.init();
    Serial.println("[MAIN] IMU (MPU6050) инициализирован");
    
    // Инициализация серво
    servos.init();
    servos.setAngleRange(0, 300);
    servos.setPluseRange(500, 2500);
    Serial.println("[MAIN] Сервоприводы (PCA9685) инициализированы");
    
    // Инициализация BLDC моторов
    motors.init();
    motors.setModes(4, 4);  // Режим управления скоростью
    Serial.println("[MAIN] BLDC моторы инициализированы");
    
    // Инициализация CAN
    setupCAN();
    
    // Установка начальной позиции ВСЕХ ног (стоя на высоте 90мм)
    IKParam.XLeft = 0;
    IKParam.XRight = 0;
    IKParam.YLeft = 90;
    IKParam.YRight = 90;
    inverseKinematicsAll();
    
    Serial.println("[MAIN] Инициализация завершена!\n");
    Serial.println("Управление:");
    Serial.println("  - Канал 1 (Forward): Вперед/назад");
    Serial.println("  - Канал 0 (Turn): Поворот");
    Serial.println("  - Канал 2 (Height): Высота ног");
    Serial.println("  - Канал 3 (Roll): Крен\n");
    
    delay(1000);
}

/**
 * @brief Основной цикл
 */
void loop() {
    // === 1. Чтение входных данных ===
    getRCValue();           // SBUS приемник
    getMPUValue();          // IMU датчик
    BLDCData = motors.getBLDCData();  // Данные от BLDC

    // === 2. Обработка команд управления ===
    
    // Поворот (от RC канала 0)
    robotMotion.turn = map(RCValue[RC_CH_TURN], RCCHANNEL_MIN, RCCHANNEL_MAX, -5, 5);
    
    // Целевая скорость (от RC канала 1)
    targetSpeed = map(RCValue[RC_CH_FORWARD], RCCHANNEL_MIN, RCCHANNEL_MAX, -20, 20);
    
    // Желаемая высота ног (от RC канала 2)
    Y_demand = (int)map(RCValue[RC_CH_HEIGHT], RCCHANNEL3_MIN, RCCHANNEL3_MAX, lowest, highest);
    
    // Желаемый крен (от RC канала 3) - для ручного управления балансом
    float Phi = map(RCValue[RC_CH_ROLL], RCCHANNEL_MIN, RCCHANNEL_MAX, -1 * rollLimit, rollLimit);

    // === 3. Управление BLDC моторами ===
    
    // Получение средней скорости колес
    float speedAvg = (M0Dir * BLDCData.M0_Vel + M1Dir * BLDCData.M1_Vel) / 2.0;
    robotPose.speedAvg = speedAvg;
    
    // PID для достижения целевой скорости -> целевой угол наклона
    float targetAngle = PID_VEL(targetSpeed - speedAvg);
    
    // PID для стабилизации тангажа -> крутящий момент моторов
    float torque = PID_PITCH(targetAngle - robotPose.pitch);
    
    // Применение поворота
    float turnTorque = robotMotion.turn * turnKp;
    motors.setTargets(M0Dir * (torque + turnTorque), M1Dir * (torque - turnTorque));
    
    // === 4. Обратная кинематика для управления ВСЕМИ ногами ===
    
    // Компенсация X по скорости (чем быстрее едем, тем больше наклон ноги)
    X = -Kp_X * (targetSpeed - speedAvg);
    
    // Плавное изменение высоты Y
    Y = Y + Kp_Y * (Y_demand - Y);
    
    // Адаптация к неровностям через PID по крену
    stab_roll = stab_roll + Kp_roll * (0 - robotPose.roll);
    
    // Вычисление высоты для левой и правой ноги
    float L_Height = Y + stab_roll;
    float R_Height = Y - stab_roll;
    
    // Установка координат для ВСЕХ ног (передние и задние одинаковые)
    IKParam.XLeft = X;
    IKParam.XRight = X;
    IKParam.YLeft = L_Height;
    IKParam.YRight = R_Height;
    
    // Вычисление и применение углов ВСЕХ 8 серво
    inverseKinematicsAll();

    // === 5. Отправка данных aux контроллеру ===
    sendToAux();

    // === 6. Отладочная информация ===
    loopCnt++;
    if (loopCnt >= 100) {
        Serial.printf("Status: Spd=%.1f Pitch=%.1f Roll=%.1f TgtAngle=%.1f Torque=%.1f Y=%.1f\n",
                     speedAvg, robotPose.pitch, robotPose.roll, targetAngle, torque, Y);
        loopCnt = 0;
    }
    
    delay(5);  // ~200Hz цикл
}
