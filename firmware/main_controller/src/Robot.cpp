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
      buzzer(BUZZER_PIN, BUZZER_PWM_CHANNEL),
      legFL(LegPosition::FRONT_LEFT, &servos),
      legFR(LegPosition::FRONT_RIGHT, &servos),
      legBL(LegPosition::BACK_LEFT, &servos),
      legBR(LegPosition::BACK_RIGHT, &servos),
    currentGait(&standGait),
    pidVelocity(0.2, 0, 0, 1000, 50),
    pidPitch(10.0f, 5.08f, 3.4f, 100.0f, 35.0f),
    pidRoll(10.0f, 5.08f, 3.4f, 100.0f, 35.0f),
      stabilizationEnabled(true),
      motorsEnabled(false),
      ikEnabled(true),
      twaiInstalled(false),
      m0Dir(1),
      m1Dir(-1),
      targetSpeed(0),
      stabRoll(0),
      stabPitch(0),
      lpfStabRoll(0),
      lpfStabPitch(0),
      lastLeftCmd(0),
      lastRightCmd(0),
      kpY(0.1),
      kpX(1.1),
      kpRoll(0.05),
      turnKp(0.1),
    rollLimit(50),
    legDistance(100),
    lowestHeight(ROBOT_LOWEST_FOR_MOT),
    highestHeight(ROBOT_HIGHEST),
    imuFilterAlpha(0.15f),  // Увеличено с 0.03 из-за уменьшения частоты IMU до 200Hz
    lpfPitch(0.0f),
    lpfRoll(0.0f),
    lpfGyroX(0.0f),
    lpfGyroY(0.0f),
    kalmanPitch(0.005f, 0.15f, 1.0f),  // Q увеличен для 200Hz
    kalmanRoll(0.005f, 0.15f, 1.0f),
    lastKalmanUpdate(0),
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
    
    // Инициализация ног
    legFL.init();
    legFR.init();
    legBL.init();
    legBR.init();
    Serial.println("[ROBOT] Ноги инициализированы");
    
    // Установка ног в минимальную высоту перед калибровкой IMU
    Serial.println("[ROBOT] Установка ног в минимальную позицию для калибровки...");
    moveAllLegsTo(0, lowestHeight);
    delay(1000);  // Даем время сервоприводам установиться
    Serial.println("[ROBOT] Ноги установлены, робот стабилен");
    
    // IMU - калибровка после установки ног
    imu.init();
    Serial.println("[ROBOT] IMU инициализирован, начинается калибровка гироскопа...");
    Serial.println("[ROBOT] Не двигайте робота!");
    delay(500);
    imu.calGyroOffsets();
    Serial.println("[ROBOT] Калибровка гироскопа завершена");

    // SBUS приемник (библиотека от Денге)
    sbusRx.Begin(SBUSPIN, -1);  // RX пин, TX не используется
    Serial.printf("[ROBOT] SBUS инициализирован на GPIO%d\n", SBUSPIN);
    
    // BLDC моторы
    Serial2.begin(115200, SERIAL_8N1, 17, 18);
    Serial.println("[ROBOT] Serial2 UART инициализирован (RX=17, TX=18, 115200)");
    
    motors.init();
    Serial.println("[ROBOT] SF_BLDC motors.init() вызван");
    
    motors.setModes(4, 4);  // Режим 4 = velocity control (КРИТИЧНО!)
    Serial.println("[ROBOT] BLDC моторы установлены в режим velocity (4,4)");
    
    // Бузер
    buzzer.begin();
    Serial.println("[ROBOT] Бузер инициализирован");
    
    // CAN
    setupCAN();
    
    startTasks();
    resetStabilizationBaseline();

    // Установка начального режима
    motion.mode = ROBOTMODE_MOTION;
    motion.highest = highestHeight;
    motion.lowest = lowestHeight;
    
    Serial.println("[ROBOT] Инициализация завершена");
    
    // Приветственный звук
    buzzer.play(SOUND_STARTUP);
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

void Robot::setStabilization(bool enabled) {
    if (stabilizationEnabled == enabled) {
        return;
    }

    stabilizationEnabled = enabled;

    pidPitch.reset();
    pidRoll.reset();

    auto setCorrections = [&](float pitchValue, float rollValue) {
        if (poseMutex) {
            if (xSemaphoreTake(poseMutex, portMAX_DELAY) == pdTRUE) {
                stabPitch = pitchValue;
                stabRoll = rollValue;
                xSemaphoreGive(poseMutex);
                return;
            }
        }
        stabPitch = pitchValue;
        stabRoll = rollValue;
    };

    setCorrections(0.0f, 0.0f);
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
            
            // Звуковой сигнал при ARM
            if (motorsEnabled) {
                playSound(SOUND_BARK_DOUBLE, true);  // Двойное гавканье с дуэтом!
            } else {
                buzzer.play(SOUND_BEEP_SHORT);  // Короткий писк при выключении
            }
        }
        
        // STAB канал (CH8) - включение/выключение стабилизации
        bool stabState = (rcValues[RC_CH_STAB] > 1500);
        if (stabState != stabilizationEnabled) {
            setStabilization(stabState);
            Serial.printf("[ROBOT] Стабилизация %s (STAB CH8 = %d)\n",
                          stabilizationEnabled ? "ВКЛЮЧЕНА" : "ВЫКЛЮЧЕНА",
                          rcValues[RC_CH_STAB]);
            
            // Звук при переключении стабилизации
            buzzer.play(stabilizationEnabled ? SOUND_SUCCESS : SOUND_BEEP_SHORT);
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
    float rawPitch = imu.angle[0];
    float rawRoll = imu.angle[1];
    newPose.yaw = imu.angle[2];
    newPose.GyroX = imu.gyro[1];
    newPose.GyroY = imu.gyro[0];
    newPose.GyroZ = imu.gyro[2];

    // Применяем LPF к гироскопу для уменьшения шума
    lpfGyroX = lowPassFilter(newPose.GyroX, lpfGyroX);
    lpfGyroY = lowPassFilter(newPose.GyroY, lpfGyroY);

    // Вычисляем dt для фильтра Калмана
    unsigned long now = micros();
    float dt = lastKalmanUpdate > 0 ? (now - lastKalmanUpdate) / 1000000.0f : 0.001f;
    lastKalmanUpdate = now;
    
    // Ограничиваем dt на случай переполнения или первого запуска
    if (dt > 0.1f || dt <= 0) dt = 0.001f;

    // Применяем фильтр Калмана (объединяет акселерометр и гироскоп)
    float kalmanPitchValue = kalmanPitch.update(rawPitch, newPose.GyroX, dt);
    float kalmanRollValue = kalmanRoll.update(rawRoll, newPose.GyroY, dt);

    // Дополнительно применяем легкий LPF для сглаживания
    lpfPitch = lowPassFilter(kalmanPitchValue, lpfPitch);
    lpfRoll = lowPassFilter(kalmanRollValue, lpfRoll);

    // Калибровка нулевой точки
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

    // Применяем коррекцию нулевой точки
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
    if (!stabilizationEnabled) {
        return;
    }

    // Мёртвая зона для устранения дребезга (deadband)
    const float PITCH_DEADBAND = 3.5f;  // градусы
    const float ROLL_DEADBAND = 3.5f;   // градусы

    // Используем полноценный PID регулятор
    // Ошибка = текущий угол (цель = 0, удержание горизонта)
    float pitchError = -poseData.pitch/2.0;
    float rollError = -poseData.roll/2.0;

    // Применяем deadband - если отклонение меньше порога, считаем его нулевым
    if (abs(pitchError) < PITCH_DEADBAND) {
        pitchError = 0.0f;
    }
    if (abs(rollError) < ROLL_DEADBAND) {
        rollError = 0.0f;
    }

    // PID вычисляет коррекцию автоматически (P + I + D)
    float rawStabPitch = pidPitch(pitchError);
    float rawStabRoll = pidRoll(rollError);

    // Дополнительное ограничение
    rawStabPitch = _constrain(rawStabPitch, -rollLimit, rollLimit);
    rawStabRoll = _constrain(rawStabRoll, -rollLimit, rollLimit);

    // Применяем LPF для сглаживания выхода PID (убирает резкие рывки)
    const float PID_FILTER_ALPHA = 0.03f;  // Оптимальное значение
    lpfStabPitch = lpfStabPitch * (1.0f - PID_FILTER_ALPHA) + rawStabPitch * PID_FILTER_ALPHA;
    lpfStabRoll = lpfStabRoll * (1.0f - PID_FILTER_ALPHA) + rawStabRoll * PID_FILTER_ALPHA;

    if (poseMutex) {
        if (xSemaphoreTake(poseMutex, portMAX_DELAY) == pdTRUE) {
            stabPitch = lpfStabPitch;
            stabRoll = lpfStabRoll;
            xSemaphoreGive(poseMutex);
        }
    } else {
        stabPitch = lpfStabPitch;
        stabRoll = lpfStabRoll;
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
    
    // Вычисление команд для моторов (умножаем на 200 для более широкого диапазона)
    float leftCmd = m0Dir * leftSpeed * 200;
    float rightCmd = m1Dir * rightSpeed * 200;
    
    // Применяем deadband - минимальная команда для старта мотора
    const float MOTOR_DEADBAND = 250.0;  // Минимальная команда для FOC моторов
    if (abs(leftCmd) > 0.1 && abs(leftCmd) < MOTOR_DEADBAND) {
        leftCmd = (leftCmd > 0) ? MOTOR_DEADBAND : -MOTOR_DEADBAND;
    }
    if (abs(rightCmd) > 0.1 && abs(rightCmd) < MOTOR_DEADBAND) {
        rightCmd = (rightCmd > 0) ? MOTOR_DEADBAND : -MOTOR_DEADBAND;
    }
    
    // DEBUG: Вывод команд моторам каждые 500мс
    static unsigned long lastMotorDebug = 0;
    if (millis() - lastMotorDebug > 500) {
        Serial.printf("[MOTOR] CH1=%d CH3=%d | fwd=%.2f turn=%.2f | L=%.1f R=%.1f | M0cmd=%.1f M1cmd=%.1f | Vel: M0=%.1f M1=%.1f\n",
                      rcValues[RC_CH_FORWARD], rcValues[RC_CH_TURN],
                      forward, turn,
                      leftSpeed, rightSpeed,
                      leftCmd, rightCmd,
                      bldcData.M0_Vel, bldcData.M1_Vel);
        lastMotorDebug = millis();
    }
    
    // Сохраняем команды для отправки в aux через CAN
    lastLeftCmd = leftCmd;
    lastRightCmd = rightCmd;
    
    // Отправка команд моторам (без дополнительного множителя - уже *= 200)
    motors.setTargets(leftCmd, rightCmd);
    
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
    Serial.printf("[ROBOT] Инициализация CAN интерфейса (TX=%d, RX=%d)...\n", CAN_TX_PIN, CAN_RX_PIN);
    
    // Настройка TWAI (CAN) для ESP32
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)CAN_TX_PIN, 
        (gpio_num_t)CAN_RX_PIN, 
        TWAI_MODE_NORMAL
    );
    
    // Увеличиваем размер очередей для стабильности
    g_config.tx_queue_len = 10;
    g_config.rx_queue_len = 10;
    
    // 500 kbps - более надежная скорость для CAN (SN65HVD230 поддерживает до 1Mbps)
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    
    // Принимаем все CAN ID (без фильтрации)
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    
    // Установка драйвера
    esp_err_t install_result = twai_driver_install(&g_config, &t_config, &f_config);
    if (install_result == ESP_OK) {
        Serial.println("[ROBOT] ✓ TWAI driver установлен успешно");
    } else {
        Serial.printf("[ROBOT] ✗ ОШИБКА установки TWAI driver: 0x%X\n", install_result);
        return;
    }
    
    // Запуск CAN интерфейса
    esp_err_t start_result = twai_start();
    if (start_result == ESP_OK) {
        Serial.println("[ROBOT] ✓ TWAI запущен (500 kbps, SN65HVD230 transceiver)");
        twaiInstalled = true;
        
        // Проверка состояния CAN после старта
        twai_status_info_t status;
        if (twai_get_status_info(&status) == ESP_OK) {
            Serial.printf("[ROBOT] CAN статус: state=%d, tx_queue=%d, rx_queue=%d\n",
                          status.state, status.msgs_to_tx, status.msgs_to_rx);
        }
    } else {
        Serial.printf("[ROBOT] ✗ ОШИБКА запуска TWAI: 0x%X\n", start_result);
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

/**
 * @brief IMU Task Loop
 * Частота: 200 Hz (5ms период)
 * - Читает данные IMU
 * - Применяет фильтр Калмана
 * - Обновляет PID стабилизации
 * Частота выбрана по теореме Найквиста: 2x от частоты управления (100Hz)
 */
void Robot::imuTaskLoop() {
    const TickType_t delayTicks = pdMS_TO_TICKS(5);  // 200 Hz (был 1000 Hz)
    while (true) {
        robotposeparam latestPose = readIMU();
        updateStabilization(latestPose);
        vTaskDelay(delayTicks);
    }
}

/**
 * @brief Control Task Loop  
 * Частота: 100 Hz (10ms период)
 * - Обновляет походку
 * - Управляет моторами
 * - Отправляет команды сервоприводам
 * Частота оптимальна для сервоприводов (50-100 Hz типичный диапазон)
 */
void Robot::controlTaskLoop() {
    const TickType_t delayTicks = pdMS_TO_TICKS(10);  // 100 Hz - оптимально для серво
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
    static unsigned long lastDebug = 0;
    
    if (millis() - lastSend < 100) return;  // 10 Hz (каждые 100мс)
    
    // Подготовка CAN сообщения
    twai_message_t tx_msg;
    tx_msg.identifier = CAN_ID_MAIN_TO_AUX;
    tx_msg.data_length_code = 8;
    tx_msg.flags = TWAI_MSG_FLAG_NONE;  // Стандартный формат CAN 2.0B
    
    // Упаковка данных для aux контроллера (задние моторы M2, M3)
    // M2/M3 получают ТЕ ЖЕ команды что M0/M1 для 4WD режима
    int16_t motor2Cmd = (int16_t)lastRightCmd;   // M2 = копия M0 (левые)
    int16_t motor3Cmd = (int16_t)lastLeftCmd;  // M3 = копия M1 (правые)
    
    tx_msg.data[0] = motion.mode;
    tx_msg.data[1] = (motor2Cmd >> 8) & 0xFF;  // High byte
    tx_msg.data[2] = motor2Cmd & 0xFF;         // Low byte
    tx_msg.data[3] = (motor3Cmd >> 8) & 0xFF;
    tx_msg.data[4] = motor3Cmd & 0xFF;
    tx_msg.data[5] = motorsEnabled ? 1 : 0;    // Флаг ARM
    tx_msg.data[6] = 0;  // Резерв
    tx_msg.data[7] = 0;  // Резерв
    
    // Отправка с таймаутом 10мс
    esp_err_t result = twai_transmit(&tx_msg, pdMS_TO_TICKS(10));
    
    // Диагностика ошибок CAN каждые 2 секунды
    if (millis() - lastDebug > 2000) {
        if (result == ESP_OK) {
            Serial.println("[CAN] ✓ Передача в aux контроллер работает");
        } else if (result == ESP_ERR_TIMEOUT) {
            Serial.println("[CAN] ✗ TIMEOUT: TX очередь переполнена");
        } else if (result == ESP_FAIL) {
            Serial.println("[CAN] ✗ FAIL: TX ошибка (проверьте подключение CAN шины)");
            
            // Диагностика состояния CAN
            twai_status_info_t status;
            if (twai_get_status_info(&status) == ESP_OK) {
                Serial.printf("[CAN] Статус: state=%d, tx_err=%d, rx_err=%d, tx_failed=%d, arb_lost=%d\n",
                              status.state, status.tx_error_counter, status.rx_error_counter,
                              status.tx_failed_count, status.arb_lost_count);
                
                // Если CAN в BUS-OFF состоянии - переинициализируем
                if (status.state == TWAI_STATE_BUS_OFF) {
                    Serial.println("[CAN] ⚠ BUS-OFF detected! Попытка восстановления...");
                    twai_initiate_recovery();
                }
            }
        } else {
            Serial.printf("[CAN] ✗ Неизвестная ошибка: 0x%X\n", result);
        }
        
        lastDebug = millis();
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
    
    // Отправка команд aux контроллеру по CAN
    sendToAux();
    
    // Обновление бузера
    buzzer.update();
    
    // Обновление позиций ног с использованием походки
    if (ikEnabled && currentGait) {
        // Подготовка параметров для походки
        GaitParams gaitParams;
        gaitParams.forward = sbusToPercent(rcValues[RC_CH_FORWARD]) / 100.0;
        gaitParams.turn = sbusToPercent(rcValues[RC_CH_TURN]) / 100.0;
        gaitParams.height = map(rcValues[RC_CH_HEIGHT], RCCHANNEL3_MIN, RCCHANNEL3_MAX, lowestHeight, highestHeight);
        // Всегда передаем углы наклона для правильной геометрии
        gaitParams.pitch =stabilizationEnabled? poseSnapshot.pitch:0.0f;
        gaitParams.roll = stabilizationEnabled ? poseSnapshot.roll : 0.0f;
        // Коррекции стабилизации применяются дополнительно
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
        Serial.printf("[ROBOT][%lu ms] Gait: %s | Pitch: %.1f (stab: %.1f) Roll: %.1f (stab: %.1f) | FL: %.1f FR: %.1f BL: %.1f BR: %.1f\n",
                      now,
                      currentGait ? currentGait->getName().c_str() : "None",
                      poseSnapshot.pitch, stabPitchSnapshot,
                      poseSnapshot.roll, stabRollSnapshot,
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

/**
 * @brief Воспроизвести звук на бузере
 * @param sound Тип звука
 * @param sendToAux Отправить команду и на aux контроллер для дуэта
 */
void Robot::playSound(BuzzerSound sound, bool sendToAux) {
    // Проигрываем на main контроллере
    buzzer.play(sound);
    
    // Если нужен дуэт - отправляем команду через CAN
    if (sendToAux && twaiInstalled) {
        twai_message_t tx_msg;
        tx_msg.identifier = CAN_ID_MAIN_TO_AUX;
        tx_msg.data_length_code = 2;
        tx_msg.flags = TWAI_MSG_FLAG_NONE;
        
        // Формат: [команда=0xFF (спецкоманда), звук]
        tx_msg.data[0] = 0xFF;  // Маркер команды бузера
        tx_msg.data[1] = (uint8_t)sound;
        
        twai_transmit(&tx_msg, pdMS_TO_TICKS(5));
    }
}
