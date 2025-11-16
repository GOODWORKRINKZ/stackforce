/**
 * @file Robot.cpp
 * @brief Реализация класса управления роботом
 */

#include "Robot.h"
#include "quadrupedal_data.h"
#include "driver/twai.h"

/**
 * @brief Преобразование SBUS значения (172-1811) в проценты (-100..+100)
 * С учетом реального центра SBUS протокола (992) и мёртвой зоны
 */
static float sbusToPercent(int16_t sbusValue, int16_t deadzone = 20) {
    const int16_t SBUS_MIN = 172;
    const int16_t SBUS_MAX = 1811;
    const int16_t SBUS_CENTER = 992;
    
    // Мёртвая зона вокруг центра
    if (abs(sbusValue - SBUS_CENTER) < deadzone) {
        return 0.0f;
    }
    
    if (sbusValue < SBUS_CENTER) {
        // Отрицательная сторона: от SBUS_MIN до SBUS_CENTER → -100% to 0%
        return ((float)(sbusValue - SBUS_MIN) / (SBUS_CENTER - SBUS_MIN) - 1.0f) * 100.0f;
    } else {
        // Положительная сторона: от SBUS_CENTER до SBUS_MAX → 0% to +100%
        return ((float)(sbusValue - SBUS_CENTER) / (SBUS_MAX - SBUS_CENTER)) * 100.0f;
    }
}

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
    pidPitch(0.6, 0.01, 0, 2000, 60),
    pidRoll(0.25, 0.01, 0, 2000, 60),
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
    rollLimit(35),
    legDistance(100),
    lowestHeight(ROBOT_LOWEST_FOR_MOT),
    highestHeight(ROBOT_HIGHEST),
    imuFilterAlpha(0.03f),
    lpfPitch(0.0f),
    lpfRoll(0.0f),
    lpfGyroX(0.0f),
    lpfGyroY(0.0f),
    pitchZero(0.0f),
    rollZero(0.0f),
    baselineInitialized(false),
    baselinePitchAccum(0.0f),
    baselineRollAccum(0.0f),
    baselineSampleCount(0),
    imuTaskHandle(nullptr),
    controlTaskHandle(nullptr),
    poseMutex(nullptr)
{
    // Инициализация массива RC значений (центр SBUS)
    for(int i = 0; i < 8; i++) {
        rcValues[i] = RCCHANNEL_MID;
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
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQ);
   
    Serial.println("[ROBOT] I2C инициализирован");
    
    // Серво
    servos.init();
    servos.setAngleRange(0, 300);
    servos.setPluseRange(500, 2500);
    Serial.println("[ROBOT] Серво инициализированы");
    delay(100);
    // IMU
    imu.init();
    imu.calGyroOffsets();
    Serial.println("[ROBOT] IMU инициализирован");

    // SBUS приемник (библиотека от Денге)
    sbusRx.Begin(SBUSPIN, -1);  // RX пин, TX не используется
    Serial.printf("[ROBOT] SBUS инициализирован на GPIO%d\n", SBUSPIN);
    
    // BLDC моторы
    Serial2.begin(115200, SERIAL_8N1, 17, 18);
    motors.init();
    Serial.println("[ROBOT] BLDC моторы инициализированы");
    
    
    
    // Инициализация ног
    legFL.init();
    legFR.init();
    legBL.init();
    legBR.init();
    Serial.println("[ROBOT] Ноги инициализированы");
    
    // CAN
    setupCAN();
    
    startTasks();
    resetStabilizationBaseline();

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
        
        for(int i = 0; i < 8; i++) {  // Читаем 8 каналов (0-7)
            rcValues[i] = sbusData[i];
        }
        
        // Ограничение значений
        rcValues[0] = _constrain(rcValues[0], RCCHANNEL_MIN, RCCHANNEL_MAX);
        rcValues[1] = _constrain(rcValues[1], RCCHANNEL_MIN, RCCHANNEL_MAX);
        rcValues[2] = _constrain(rcValues[2], RCCHANNEL3_MIN, RCCHANNEL3_MAX);
        rcValues[3] = _constrain(rcValues[3], RCCHANNEL_MIN, RCCHANNEL_MAX);
        
        // ARM канал (CH5) - включение/выключение моторов
        // Переключатель в верхнем положении (>1500) = моторы включены
        bool armState = (rcValues[RC_CH_ARM] > 1500);
        if (armState != motorsEnabled) {
            motorsEnabled = armState;
            Serial.printf("[ROBOT] Моторы %s (ARM CH5 = %d)\n", 
                          motorsEnabled ? "ВКЛЮЧЕНЫ" : "ВЫКЛЮЧЕНЫ",
                          rcValues[RC_CH_ARM]);
        }
        
        // STAB канал (CH8) - включение/выключение стабилизации
        bool stabState = (rcValues[RC_CH_STAB] > 1500);
        if (stabState != stabilizationEnabled) {
            stabilizationEnabled = stabState;
            Serial.printf("[ROBOT] Стабилизация %s (STAB CH8 = %d)\n",
                          stabilizationEnabled ? "ВКЛЮЧЕНА" : "ВЫКЛЮЧЕНА",
                          rcValues[RC_CH_STAB]);
        }
    }
}

/**
 * @brief Получение данных MPU6050
 */
float Robot::lowPassFilter(float current, float previous) const {
    return imuFilterAlpha * current + (1.0f - imuFilterAlpha) * previous;
}

robotposeparam Robot::readIMU() {
    imu.update();
    robotposeparam newPose;
    // IMU is mounted with Y forward and X to the right, so copy Deng's mapping
    newPose.pitch = imu.angle[0];
    newPose.roll = imu.angle[1];
    newPose.yaw = imu.angle[2];
    newPose.GyroX = imu.gyro[1];
    newPose.GyroY = imu.gyro[0];
    newPose.GyroZ = imu.gyro[2];

    lpfPitch = lowPassFilter(newPose.pitch, lpfPitch);
    lpfRoll = lowPassFilter(newPose.roll, lpfRoll);
    lpfGyroX = lowPassFilter(newPose.GyroX, lpfGyroX);
    lpfGyroY = lowPassFilter(newPose.GyroY, lpfGyroY);


    if (!baselineInitialized) {
        baselinePitchAccum += lpfPitch;
        baselineRollAccum += lpfRoll;
        baselineSampleCount++;
        if (baselineSampleCount >= 200) {
            pitchZero = baselinePitchAccum / baselineSampleCount;
            rollZero = baselineRollAccum / baselineSampleCount;
            baselineInitialized = true;
            Serial.printf("[ROBOT] IMU baseline captured: pitchZero=%.2f rollZero=%.2f\n", pitchZero, rollZero);
        }
    }

    float correctedPitch = lpfPitch - pitchZero;
    float correctedRoll = lpfRoll - rollZero;
    newPose.pitch = correctedPitch;
    newPose.roll = correctedRoll;

    if (poseMutex) {
        if (xSemaphoreTake(poseMutex, portMAX_DELAY) == pdTRUE) {
            pose = newPose;
            xSemaphoreGive(poseMutex);
        }
    } else {
        pose = newPose;
    }

    return newPose;
}

/**
 * @brief Обновление стабилизации
 */
void Robot::updateStabilization(const robotposeparam& poseData) {
    float currentStabPitch = 0.0f;
    float currentStabRoll = 0.0f;

    if (poseMutex) {
        if (xSemaphoreTake(poseMutex, portMAX_DELAY) == pdTRUE) {
            currentStabPitch = stabPitch;
            currentStabRoll = stabRoll;
            xSemaphoreGive(poseMutex);
        } else {
            currentStabPitch = stabPitch;
            currentStabRoll = stabRoll;
        }
    } else {
        currentStabPitch = stabPitch;
        currentStabRoll = stabRoll;
    }

    float newStabPitch = 0.0f;
    float newStabRoll = 0.0f;

    if (stabilizationEnabled) {
        // Алгоритм Денге: targetGyro = (0 - lpf_angle) * P
        // StableHeightAdjust.X = StableHeightAdjust.X - I * (targetGyro - gyro)
        float targetGyroY = (0.0f - poseData.roll) * pidRoll.P;
        newStabRoll = currentStabRoll - pidRoll.I * (targetGyroY - lpfGyroY);
        newStabRoll = _constrain(newStabRoll, -rollLimit, rollLimit);

        float targetGyroX = (0.0f - poseData.pitch) * pidPitch.P;
        newStabPitch = currentStabPitch - pidPitch.I * (targetGyroX - lpfGyroX);
        newStabPitch = _constrain(newStabPitch, -25.0f, 25.0f);
    }

    if (poseMutex) {
        if (xSemaphoreTake(poseMutex, portMAX_DELAY) == pdTRUE) {
            stabPitch = newStabPitch;
            stabRoll = newStabRoll;
            xSemaphoreGive(poseMutex);
        }
    } else {
        stabPitch = newStabPitch;
        stabRoll = newStabRoll;
    }
}

/**
 * @brief Обновление моторов
 */
void Robot::updateMotors() {
    if (!motorsEnabled) {
        motors.setTargets(0, 0);
        return;
    }
    
    // Получение команд из RC (с правильным центром SBUS и дедзоной)
    float forward = sbusToPercent(rcValues[RC_CH_FORWARD]) / 100.0;
    float turn = sbusToPercent(rcValues[RC_CH_TURN]) / 100.0;
    
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

void Robot::copyPoseAndStab(robotposeparam& poseOut, float& stabPitchOut, float& stabRollOut) {
    if (poseMutex) {
        if (xSemaphoreTake(poseMutex, portMAX_DELAY) == pdTRUE) {
            poseOut = pose;
            stabPitchOut = stabPitch;
            stabRollOut = stabRoll;
            xSemaphoreGive(poseMutex);
            return;
        }
    }

    poseOut = pose;
    stabPitchOut = stabPitch;
    stabRollOut = stabRoll;
}

void Robot::resetStabilizationBaseline() {
    baselineInitialized = false;
    baselinePitchAccum = 0.0f;
    baselineRollAccum = 0.0f;
    baselineSampleCount = 0;
    Serial.println("[ROBOT] IMU baseline reset, hold robot level");
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

void Robot::startTasks() {
    if (!poseMutex) {
        poseMutex = xSemaphoreCreateMutex();
        if (!poseMutex) {
            Serial.println("[ROBOT] ОШИБКА: не удалось создать mutex позы");
            return;
        }
    }

    if (imuTaskHandle == nullptr) {
        BaseType_t res = xTaskCreatePinnedToCore(
            Robot::imuTaskEntry,
            "IMU",
            4096,
            this,
            3,
            &imuTaskHandle,
            0);
        if (res != pdPASS) {
            Serial.println("[ROBOT] ОШИБКА: не удалось запустить IMU таск");
            imuTaskHandle = nullptr;
        } else {
            Serial.println("[ROBOT] IMU таск запущен на ядре 0");
        }
    }

    if (controlTaskHandle == nullptr) {
        BaseType_t res = xTaskCreatePinnedToCore(
            Robot::controlTaskEntry,
            "Control",
            8192,
            this,
            2,
            &controlTaskHandle,
            1);
        if (res != pdPASS) {
            Serial.println("[ROBOT] ОШИБКА: не удалось запустить контрольный таск");
            controlTaskHandle = nullptr;
        } else {
            Serial.println("[ROBOT] Контрольный таск запущен на ядре 1");
        }
    }
}

void Robot::imuTaskEntry(void* arg) {
    if (arg) {
        static_cast<Robot*>(arg)->imuTaskLoop();
    }
    vTaskDelete(nullptr);
}

void Robot::controlTaskEntry(void* arg) {
    if (arg) {
        static_cast<Robot*>(arg)->controlTaskLoop();
    }
    vTaskDelete(nullptr);
}

void Robot::imuTaskLoop() {
    const TickType_t delayTicks = pdMS_TO_TICKS(1);
    while (true) {
        robotposeparam latestPose = readIMU();
        updateStabilization(latestPose);
        vTaskDelay(delayTicks);
    }
}

void Robot::controlTaskLoop() {
    const TickType_t delayTicks = pdMS_TO_TICKS(5);
    while (true) {
        update();
        vTaskDelay(delayTicks);
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

    robotposeparam poseSnapshot;
    float stabPitchSnapshot = 0.0f;
    float stabRollSnapshot = 0.0f;
    copyPoseAndStab(poseSnapshot, stabPitchSnapshot, stabRollSnapshot);
    
    // Чтение датчиков
    readRC();
    
    // Обновление моторов
    updateMotors();
    
    // Обновление позиций ног с использованием походки
    if (ikEnabled && currentGait) {
        // Подготовка параметров для походки
        GaitParams gaitParams;
        gaitParams.forward = sbusToPercent(rcValues[RC_CH_FORWARD]) / 100.0;
        gaitParams.turn = sbusToPercent(rcValues[RC_CH_TURN]) / 100.0;
        gaitParams.height = map(rcValues[RC_CH_HEIGHT], RCCHANNEL3_MIN, RCCHANNEL3_MAX, lowestHeight, highestHeight);
        gaitParams.pitch = stabilizationEnabled ? 0.0f : poseSnapshot.pitch;
        gaitParams.roll = stabilizationEnabled ? 0.0f : poseSnapshot.roll;
        gaitParams.stabPitch = stabilizationEnabled ? stabPitchSnapshot : 0.0f;
        gaitParams.stabRoll = stabilizationEnabled ? stabRollSnapshot : 0.0f;
        if (currentGait) {
            currentGait->setStabilization(stabilizationEnabled);
        }
        gaitParams.speed = bldcData.M0_Vel;  // Скорость из телеметрии моторов
        
        // Получение целевых позиций от походки
        GaitTargets targets = currentGait->update(gaitParams);
        auto clampLegY = [this](LegTarget& leg) {
            leg.y = _constrain(leg.y, lowestHeight, highestHeight);
        };
        clampLegY(targets.frontLeft);
        clampLegY(targets.frontRight);
        clampLegY(targets.backLeft);
        clampLegY(targets.backRight);
        
        // Применение к ногам
        legFL.moveTo(targets.frontLeft.x, targets.frontLeft.y);
        legFR.moveTo(targets.frontRight.x, targets.frontRight.y);
        legBL.moveTo(targets.backLeft.x, targets.backLeft.y);
        legBR.moveTo(targets.backRight.x, targets.backRight.y);
    }
    
    // Отправка данных на aux контроллер
    if (loopCnt % 100 == 0) {  // Каждые 100 циклов
        unsigned long now = millis();
        Serial.printf("[ROBOT][%lu ms] Gait: %s | Pitch: %.1f Roll: %.1f | FL: %.1f FR: %.1f BL: %.1f BR: %.1f\n",
                      now,
                      currentGait ? currentGait->getName().c_str() : "None",
                      poseSnapshot.pitch, poseSnapshot.roll,
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
