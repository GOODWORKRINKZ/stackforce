/**
 * @file Robot.cpp
 * @brief Реализация класса управления роботом
 */

#include "Robot.h"
#include "quadrupedal_data.h"
#include "driver/twai.h"

/**
 * @brief Конструктор
 */
Robot::Robot() 
    : servos(Wire),
      imu(Wire),
      motors(Serial2),
      sbusRx(&Serial1),
      legFL(LegPosition::FRONT_LEFT, &servos),
      legFR(LegPosition::FRONT_RIGHT, &servos),
      legBL(LegPosition::BACK_LEFT, &servos),
      legBR(LegPosition::BACK_RIGHT, &servos),
      currentGait(&standGait),
      pidVelocity(0.2, 0, 0, 1000, 50),
      pidPitch(0.38, 0, 0, 1000, 50),
      pidRoll(0.05, 0, 0, 1000, 50),
      stabilizationEnabled(true),
      motorsEnabled(false),
      ikEnabled(true),
      twaiInstalled(false),
      m0Dir(1),
      m1Dir(-1),
      targetSpeed(0),
      stabRoll(0),
      stabPitch(0),
      kpY(0.1),
      kpX(1.1),
      kpRoll(0.05),
      turnKp(0.1),
      rollLimit(20),
      legDistance(100),
      lowestHeight(ROBOT_LOWEST_FOR_MOT),
      highestHeight(ROBOT_HIGHEST)
{
    // Инициализация массива RC значений
    for(int i = 0; i < 6; i++) {
        rcValues[i] = 1000;
    }
    
    // Инициализация структур данных
    memset(&pose, 0, sizeof(pose));
    memset(&motion, 0, sizeof(motion));
    memset(&bldcData, 0, sizeof(bldcData));
}

/**
 * @brief Инициализация робота
 */
void Robot::init() {
    Serial.begin(115200);
    Serial.println("[ROBOT] Инициализация...");
    
    // I2C
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(I2C_FREQ);
    Serial.println("[ROBOT] I2C инициализирован");
    
    // SBUS приемник (библиотека от Денге)
    sbusRx.Begin(SBUSPIN, -1);  // RX пин, TX не используется
    Serial.printf("[ROBOT] SBUS инициализирован на GPIO%d\n", SBUSPIN);
    
    // BLDC моторы
    Serial2.begin(115200, SERIAL_8N1, 17, 18);
    motors.init();
    Serial.println("[ROBOT] BLDC моторы инициализированы");
    
    // IMU
    imu.init();
    Serial.println("[ROBOT] IMU инициализирован");
    
    // Серво
    servos.init();
    servos.setAngleRange(0, 300);
    servos.setPluseRange(500, 2500);
    Serial.println("[ROBOT] Серво инициализированы");
    delay(100);
    
    // Инициализация ног
    legFL.init();
    legFR.init();
    legBL.init();
    legBR.init();
    Serial.println("[ROBOT] Ноги инициализированы");
    
    // CAN
    setupCAN();
    
    // Установка начального режима
    motion.mode = ROBOTMODE_MOTION;
    motion.highest = highestHeight;
    motion.lowest = lowestHeight;
    
    Serial.println("[ROBOT] Инициализация завершена");
}

/**
 * @brief Установить походку
 */
bool Robot::setGait(const String& gaitName) {
    String name = gaitName;
    name.toLowerCase();
    
    if (name == "stand") {
        currentGait = &standGait;
    } else if (name == "trot") {
        currentGait = &trotGait;
    } else if (name == "walk") {
        currentGait = &walkGait;
    } else if (name == "bound") {
        currentGait = &boundGait;
    } else if (name == "crawl") {
        currentGait = &crawlGait;
    } else {
        Serial.printf("[ROBOT] Неизвестная походка: %s\n", gaitName.c_str());
        return false;
    }
    
    currentGait->reset();
    Serial.printf("[ROBOT] Походка изменена на: %s\n", currentGait->getName().c_str());
    return true;
}

/**
 * @brief Чтение значений с SBUS приемника
 */
void Robot::readRC() {
    if (sbusRx.Read()) {
        sbusData = sbusRx.ch();
        
        for(int i = 0; i < 6; i++) {
            rcValues[i] = sbusData[i];
        }
        
        // Ограничение значений
        rcValues[0] = _constrain(rcValues[0], RCCHANNEL_MIN, RCCHANNEL_MAX);
        rcValues[1] = _constrain(rcValues[1], RCCHANNEL_MIN, RCCHANNEL_MAX);
        rcValues[2] = _constrain(rcValues[2], RCCHANNEL3_MIN, RCCHANNEL3_MAX);
        rcValues[3] = _constrain(rcValues[3], RCCHANNEL_MIN, RCCHANNEL_MAX);
    }
}

/**
 * @brief Получение данных MPU6050
 */
void Robot::readIMU() {
    imu.update();
    pose.pitch = -imu.angle[0];
    pose.roll = imu.angle[1];
    pose.yaw = imu.angle[2];
    pose.GyroX = imu.gyro[1];
    pose.GyroY = -imu.gyro[0];
    pose.GyroZ = -imu.gyro[2];
}

/**
 * @brief Обновление стабилизации
 */
void Robot::updateStabilization() {
    if (!stabilizationEnabled) {
        stabPitch = 0;
        stabRoll = 0;
        return;
    }
    
    // PID стабилизация по pitch и roll
    float pitchCorrection = pidPitch(0 - pose.pitch);
    float rollCorrection = pidRoll(0 - pose.roll);
    
    stabPitch += pitchCorrection * 0.01;  // Плавное накопление
    stabRoll += rollCorrection * 0.01;
    
    // Ограничение накопленной коррекции
    stabPitch = _constrain(stabPitch, -20, 20);
    stabRoll = _constrain(stabRoll, -rollLimit, rollLimit);
}

/**
 * @brief Обновление моторов
 */
void Robot::updateMotors() {
    if (!motorsEnabled) {
        motors.setTargets(0, 0);
        return;
    }
    
    // Получение команд из RC
    float forward = map(rcValues[RC_CH_FORWARD], RCCHANNEL_MIN, RCCHANNEL_MAX, -100, 100) / 100.0;
    float turn = map(rcValues[RC_CH_TURN], RCCHANNEL_MIN, RCCHANNEL_MAX, -100, 100) / 100.0;
    
    // Дедзона
    if (abs(forward) < 0.05) forward = 0;
    if (abs(turn) < 0.05) turn = 0;
    
    // Вычисление скоростей для дифференциального привода
    float leftSpeed = forward - turn;
    float rightSpeed = forward + turn;
    
    // Ограничение скорости
    leftSpeed = _constrain(leftSpeed, -1.0, 1.0);
    rightSpeed = _constrain(rightSpeed, -1.0, 1.0);
    
    // Отправка команд моторам
    motors.setTargets(m0Dir * leftSpeed * 100, m1Dir * rightSpeed * 100);
    
    // Получение телеметрии от моторов
    bldcData = motors.getBLDCData();
}

/**
 * @brief Инициализация CAN шины
 */
void Robot::setupCAN() {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)CAN_TX_PIN, 
        (gpio_num_t)CAN_RX_PIN, 
        TWAI_MODE_NORMAL
    );
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    
    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        Serial.println("[ROBOT] TWAI driver установлен");
    } else {
        Serial.println("[ROBOT] ОШИБКА: Не удалось установить TWAI driver");
        return;
    }
    
    if (twai_start() == ESP_OK) {
        Serial.println("[ROBOT] TWAI запущен (1 Mbps)");
        twaiInstalled = true;
    } else {
        Serial.println("[ROBOT] ОШИБКА: Не удалось запустить TWAI");
    }
}

/**
 * @brief Отправка команд aux контроллеру по CAN
 */
void Robot::sendToAux() {
    if (!twaiInstalled) return;
    
    static unsigned long lastSend = 0;
    if (millis() - lastSend < 20) return;  // 50 Hz
    
    twai_message_t tx_msg;
    tx_msg.identifier = CAN_ID_MAIN_TO_AUX;
    tx_msg.data_length_code = 8;
    tx_msg.flags = TWAI_MSG_FLAG_NONE;
    
    // Упаковка данных (пример: режим, команды моторов задних колес)
    tx_msg.data[0] = motion.mode;
    tx_msg.data[1] = 0;  // Резерв для будущих данных
    tx_msg.data[2] = 0;
    tx_msg.data[3] = 0;
    tx_msg.data[4] = 0;
    tx_msg.data[5] = 0;
    tx_msg.data[6] = 0;
    tx_msg.data[7] = 0;
    
    if (twai_transmit(&tx_msg, pdMS_TO_TICKS(10)) == ESP_OK) {
        // Успешно отправлено
    }
    
    lastSend = millis();
}

/**
 * @brief Основной цикл обновления
 */
void Robot::update() {
    static uint8_t loopCnt = 0;
    loopCnt++;
    
    // Чтение датчиков
    readRC();
    readIMU();
    
    // Обновление стабилизации
    updateStabilization();
    
    // Обновление моторов
    updateMotors();
    
    // Обновление позиций ног с использованием походки
    if (ikEnabled && currentGait) {
        // Подготовка параметров для походки
        GaitParams gaitParams;
        gaitParams.forward = map(rcValues[RC_CH_FORWARD], RCCHANNEL_MIN, RCCHANNEL_MAX, -100, 100) / 100.0;
        gaitParams.turn = map(rcValues[RC_CH_TURN], RCCHANNEL_MIN, RCCHANNEL_MAX, -100, 100) / 100.0;
        gaitParams.height = map(rcValues[RC_CH_HEIGHT], RCCHANNEL3_MIN, RCCHANNEL3_MAX, lowestHeight, highestHeight);
        gaitParams.pitch = pose.pitch;
        gaitParams.roll = pose.roll;
        gaitParams.speed = bldcData.M0_Vel;  // Скорость из телеметрии моторов
        
        // Дедзона для команд
        if (abs(gaitParams.forward) < 0.05) gaitParams.forward = 0;
        if (abs(gaitParams.turn) < 0.05) gaitParams.turn = 0;
        
        // Получение целевых позиций от походки
        GaitTargets targets = currentGait->update(gaitParams);
        
        // Применение к ногам
        legFL.moveTo(targets.frontLeft.x, targets.frontLeft.y);
        legFR.moveTo(targets.frontRight.x, targets.frontRight.y);
        legBL.moveTo(targets.backLeft.x, targets.backLeft.y);
        legBR.moveTo(targets.backRight.x, targets.backRight.y);
    }
    
    // Отправка данных на aux контроллер
    if (loopCnt % 10 == 0) {  // Каждые 10 циклов
        sendToAux();
    }
    
    // Вывод телеметрии
    if (loopCnt % 100 == 0) {  // Каждые 100 циклов
        Serial.printf("[ROBOT] Gait: %s | Pitch: %.1f Roll: %.1f | FL: %.1f FR: %.1f BL: %.1f BR: %.1f\n",
            currentGait->getName().c_str(),
            pose.pitch, pose.roll,
            legFL.getY(), legFR.getY(),
            legBL.getY(), legBR.getY());
    }
}

/**
 * @brief Установить высоту всех ног
 */
void Robot::setHeight(float height) {
    height = _constrain(height, lowestHeight, highestHeight);
    moveAllLegsTo(0, height);
}

/**
 * @brief Переместить все ноги в указанную позицию
 */
void Robot::moveAllLegsTo(float x, float y) {
    legFL.moveTo(x, y);
    legFR.moveTo(x, y);
    legBL.moveTo(x, y);
    legBR.moveTo(x, y);
}
