/**
 * @file calibration_realtime.cpp
 * @brief Калибровка офсетов сервоприводов в реальном времени + IK режим
 * 
 * Функции:
 * - РЕЖИМ 1: Прямая калибровка (90° + офсеты)
 * - РЕЖИМ 2: IK калибровка (X, Y координаты + офсеты)
 * - Переключение между режимами командой 'm'
 * - Автовыравнивание робота по IMU (команда 'l')
 * 
 * Протокол Serial (115200 baud):
 * - "o<канал> <офсет>" - установить офсет (например: o0 -15)
 * - "m" - переключить режим (калибровка ↔ IK)
 * - "x <значение>" - установить X координату (только в IK режиме)
 * - "y <значение>" - установить Y координату (только в IK режиме)
 * - "a 90" - установить все серво на 90° (калибровка режим)
 * - "l" - автовыравнивание (level) по IMU
 * - "p" - вывести текущие офсеты и параметры
 * - "r" - сбросить все офсеты в 0
 */

#include <Arduino.h>
#include <Wire.h>
#include "SF_Servo.h"
#include "SF_IMU.h"

// ==================== ГЕОМЕТРИЯ 5-ЗВЕННИКА ====================
#define L1 60   // Длина верхнего заднего звена
#define L2 100  // Длина среднего заднего звена
#define L3 100  // Длина среднего переднего звена
#define L4 60   // Длина верхнего переднего звена
#define L5 40   // Смещение

// ==================== КОНФИГУРАЦИЯ ====================
#define SERVO_ENABLE_PIN 42
#define PCA9685_ADDR 0x40
#define NUM_SERVOS 8

// Режим работы
enum CalibrationMode {
    MODE_DIRECT = 0,  // Прямая калибровка: 90° + офсет
    MODE_IK = 1       // IK калибровка: обратная кинематика + офсет
};

CalibrationMode currentMode = MODE_DIRECT;
float IK_X = 0.0;      // IK координата X
float IK_Y = 115.0;     // IK координата Y (начальное значение = как в примере lesson5_Stable)

// Каналы серво на PCA9685
// ВНИМАНИЕ: Робот повернут на 180° относительно гироскопа
#define SERVO_FL_FRONT 2   // Front Left Front (физически был BR)
#define SERVO_FL_REAR  3   // Front Left Rear (физически был BR)
#define SERVO_FR_FRONT 9   // Front Right Front (физически был BL)
#define SERVO_FR_REAR  8   // Front Right Rear (физически был BL)
#define SERVO_BL_FRONT 0   // Back Left Front (физически был FR)
#define SERVO_BL_REAR  1   // Back Left Rear (физически был FR)
#define SERVO_BR_FRONT 11  // Back Right Front (физически был FL)
#define SERVO_BR_REAR  10  // Back Right Rear (физически был FL)

// Имена для отладки (после поворота на 180°)
const char* SERVO_NAMES[16] = {
    "BL_FRONT", "BL_REAR", "FL_FRONT", "FL_REAR",
    "CH4", "CH5", "CH6", "CH7",
    "FR_REAR", "FR_FRONT", "BR_REAR", "BR_FRONT",
    "CH12", "CH13", "CH14", "CH15"
};

// ==================== ГЛОБАЛЬНЫЕ ОБЪЕКТЫ ====================
SF_Servo servos = SF_Servo(Wire);
SF_IMU mpu6050 = SF_IMU(Wire);

// Текущие офсеты (градусы) - откалиброваны в IK режиме при X=0, Y=115
// После поворота робота на 180° и исправления формулы IK (bRight вместо aRight)
int16_t servoOffsets[16] = {
    -74, -147, -35, -162,    // CH0-3: BL_FRONT, BL_REAR, FL_FRONT, FL_REAR
    0, 0, 0, 0,              // CH4-7: неиспользуемые
    -15, -150, -23, -108,    // CH8-11: FR_REAR, FR_FRONT, BR_REAR, BR_FRONT
    0, 0, 0, 0               // CH12-15: неиспользуемые
};

// Базовый угол (нейтраль)
const uint16_t BASE_ANGLE = 90;

// ==================== ФУНКЦИИ ====================

/**
 * @brief Применение офсета к серво
 */
void applyServo(uint8_t channel, int16_t offset) {
    int16_t finalAngle = BASE_ANGLE + offset;
    
    // Ограничение диапазона
    if (finalAngle < 0) finalAngle = 0;
    if (finalAngle > 300) finalAngle = 300;
    
    servos.setAngle(channel, finalAngle);
}

/**
 * @brief Применение всех офсетов
 */
void applyAllServos() {
    for (uint8_t i = 0; i < NUM_SERVOS; i++) {
        uint8_t channel;
        switch(i) {
            case 0: channel = SERVO_FL_FRONT; break;
            case 1: channel = SERVO_FL_REAR; break;
            case 2: channel = SERVO_FR_FRONT; break;
            case 3: channel = SERVO_FR_REAR; break;
            case 4: channel = SERVO_BL_FRONT; break;
            case 5: channel = SERVO_BL_REAR; break;
            case 6: channel = SERVO_BR_FRONT; break;
            case 7: channel = SERVO_BR_REAR; break;
            default: continue;
        }
        applyServo(channel, servoOffsets[channel]);
    }
}

/**
 * @brief Вывод текущих офсетов
 */
void printOffsets() {
    Serial.println("\n=== ТЕКУЩИЕ ОФСЕТЫ ===");
    Serial.printf("FL_FRONT (CH%d): %d\n", SERVO_FL_FRONT, servoOffsets[SERVO_FL_FRONT]);
    Serial.printf("FL_REAR  (CH%d): %d\n", SERVO_FL_REAR, servoOffsets[SERVO_FL_REAR]);
    Serial.printf("FR_FRONT (CH%d): %d\n", SERVO_FR_FRONT, servoOffsets[SERVO_FR_FRONT]);
    Serial.printf("FR_REAR  (CH%d): %d\n", SERVO_FR_REAR, servoOffsets[SERVO_FR_REAR]);
    Serial.printf("BL_FRONT (CH%d): %d\n", SERVO_BL_FRONT, servoOffsets[SERVO_BL_FRONT]);
    Serial.printf("BL_REAR  (CH%d): %d\n", SERVO_BL_REAR, servoOffsets[SERVO_BL_REAR]);
    Serial.printf("BR_FRONT (CH%d): %d\n", SERVO_BR_FRONT, servoOffsets[SERVO_BR_FRONT]);
    Serial.printf("BR_REAR  (CH%d): %d\n", SERVO_BR_REAR, servoOffsets[SERVO_BR_REAR]);
    Serial.println("======================\n");
}

/**
 * @brief Автовыравнивание робота по IMU
 * Корректирует офсеты так, чтобы робот был горизонтален
 */
void autoLevel() {
    Serial.println("\n========================================");
    Serial.println("АВТОВЫРАВНИВАНИЕ ПО IMU");
    Serial.println("========================================");
    Serial.println("[INFO] Робот должен стоять неподвижно!");
    Serial.println("[INFO] Начинаю измерение углов...\n");
    
    delay(500);
    
    // Усреднение показаний IMU (10 измерений)
    float avgPitch = 0, avgRoll = 0;
    for (int i = 0; i < 10; i++) {
        mpu6050.update();
        avgPitch += -mpu6050.angle[0];  // Инвертируем как в main_controller
        avgRoll += mpu6050.angle[1];
        delay(50);
    }
    avgPitch /= 10.0;
    avgRoll /= 10.0;
    
    Serial.printf("Текущие углы:\n");
    Serial.printf("  Pitch: %.2f°\n", avgPitch);
    Serial.printf("  Roll:  %.2f°\n", avgRoll);
    Serial.println();
    
    // Рассчитываем коррекцию офсетов
    // Pitch контролируется передними/задними ногами
    // Roll контролируется левыми/правыми ногами
    
    // Коэффициент: примерно 1° наклона = 1° серво
    int pitchCorrection = (int)(avgPitch * 1.0);
    int rollCorrection = (int)(avgRoll * 1.0);
    
    Serial.println("[INFO] Применяю коррекцию...");
    
    // Pitch коррекция: передние vs задние
    // Если Pitch > 0 (нос вверх), уменьшаем офсеты передних ног
    servoOffsets[SERVO_FL_FRONT] -= pitchCorrection;
    servoOffsets[SERVO_FL_REAR] -= pitchCorrection;
    servoOffsets[SERVO_FR_FRONT] -= pitchCorrection;
    servoOffsets[SERVO_FR_REAR] -= pitchCorrection;
    
    servoOffsets[SERVO_BL_FRONT] += pitchCorrection;
    servoOffsets[SERVO_BL_REAR] += pitchCorrection;
    servoOffsets[SERVO_BR_FRONT] += pitchCorrection;
    servoOffsets[SERVO_BR_REAR] += pitchCorrection;
    
    // Roll коррекция: левые vs правые
    // Если Roll > 0 (правый борт вниз), увеличиваем офсеты правых ног
    servoOffsets[SERVO_FR_FRONT] -= rollCorrection;
    servoOffsets[SERVO_FR_REAR] -= rollCorrection;
    servoOffsets[SERVO_BR_FRONT] -= rollCorrection;
    servoOffsets[SERVO_BR_REAR] -= rollCorrection;
    
    servoOffsets[SERVO_FL_FRONT] += rollCorrection;
    servoOffsets[SERVO_FL_REAR] += rollCorrection;
    servoOffsets[SERVO_BL_FRONT] += rollCorrection;
    servoOffsets[SERVO_BL_REAR] += rollCorrection;
    
    // Применить новые офсеты
    applyAllServos();
    delay(500);
    
    // Проверить результат
    avgPitch = 0; avgRoll = 0;
    for (int i = 0; i < 10; i++) {
        mpu6050.update();
        avgPitch += -mpu6050.angle[0];
        avgRoll += mpu6050.angle[1];
        delay(50);
    }
    avgPitch /= 10.0;
    avgRoll /= 10.0;
    
    Serial.println("\n[OK] Автовыравнивание завершено!");
    Serial.printf("Новые углы:\n");
    Serial.printf("  Pitch: %.2f°\n", avgPitch);
    Serial.printf("  Roll:  %.2f°\n", avgRoll);
    Serial.println("========================================\n");
    
    printOffsets();
}

/**
 * @brief Обратная кинематика (ИСПРАВЛЕННАЯ версия из примера!)
 */
void inverseKinematicsAll() {
    int16_t alphaLeftToAngle, betaLeftToAngle;
    int16_t alphaRightToAngle, betaRightToAngle;
    
    float alpha1, alpha2, beta1, beta2;

    // === ЛЕВАЯ НОГА ===
    float aLeft = 2 * IK_X * L1;
    float bLeft = 2 * IK_Y * L1;
    float cLeft = IK_X * IK_X + IK_Y * IK_Y + L1 * L1 - L2 * L2;
    float dLeft = 2 * L4 * (IK_X - L5);
    float eLeft = 2 * L4 * IK_Y;
    float fLeft = ((IK_X - L5) * (IK_X - L5) + L4 * L4 + IK_Y * IK_Y - L3 * L3);

    alpha1 = 2 * atan((bLeft + sqrt((aLeft * aLeft) + (bLeft * bLeft) - (cLeft * cLeft))) / (aLeft + cLeft));
    alpha2 = 2 * atan((bLeft - sqrt((aLeft * aLeft) + (bLeft * bLeft) - (cLeft * cLeft))) / (aLeft + cLeft));
    beta1 = 2 * atan((eLeft + sqrt((dLeft * dLeft) + eLeft * eLeft - (fLeft * fLeft))) / (dLeft + fLeft));
    beta2 = 2 * atan((eLeft - sqrt((dLeft * dLeft) + eLeft * eLeft - (fLeft * fLeft))) / (dLeft + fLeft));

    alpha1 = (alpha1 >= 0) ? alpha1 : (alpha1 + 2 * PI);
    alpha2 = (alpha2 >= 0) ? alpha2 : (alpha2 + 2 * PI);

    float alphaLeft, betaLeft;
    if (alpha1 >= PI / 4) alphaLeft = alpha1;
    else alphaLeft = alpha2;
    
    if (beta1 >= 0 && beta1 <= PI / 4) betaLeft = beta1;
    else betaLeft = beta2;

    // === ПРАВАЯ НОГА ===
    float aRight = 2 * IK_X * L1;
    float bRight = 2 * IK_Y * L1;
    float cRight = IK_X * IK_X + IK_Y * IK_Y + L1 * L1 - L2 * L2;
    float dRight = 2 * L4 * (IK_X - L5);
    float eRight = 2 * L4 * IK_Y;
    float fRight = ((IK_X - L5) * (IK_X - L5) + L4 * L4 + IK_Y * IK_Y - L3 * L3);

    alpha1 = 2 * atan((bRight + sqrt((aRight * aRight) + (bRight * bRight) - (cRight * cRight))) / (aRight + cRight));
    alpha2 = 2 * atan((bRight - sqrt((aRight * aRight) + (bRight * bRight) - (cRight * cRight))) / (aRight + cRight));
    beta1 = 2 * atan((eRight + sqrt((dRight * dRight) + eRight * eRight - (fRight * fRight))) / (dRight + fRight));
    beta2 = 2 * atan((eRight - sqrt((dRight * dRight) + eRight * eRight - (fRight * fRight))) / (dRight + fRight));

    alpha1 = (alpha1 >= 0) ? alpha1 : (alpha1 + 2 * PI);
    alpha2 = (alpha2 >= 0) ? alpha2 : (alpha2 + 2 * PI);

    float alphaRight, betaRight;
    if (alpha1 >= PI / 4) alphaRight = alpha1;
    else alphaRight = alpha2;
    
    if (beta1 >= 0 && beta1 <= PI / 4) betaRight = beta1;
    else betaRight = beta2;

    // Конвертация в градусы
    alphaLeftToAngle = (int)((alphaLeft / 6.28) * 360);
    betaLeftToAngle = (int)((betaLeft / 6.28) * 360);
    alphaRightToAngle = (int)((alphaRight / 6.28) * 360);
    betaRightToAngle = (int)((betaRight / 6.28) * 360);

    // Углы серво (БЕЗ офсетов - они добавятся в applyServo)
    int16_t servoFrontLeftFront = 90 + betaLeftToAngle;
    int16_t servoFrontLeftRear = 90 + alphaLeftToAngle;
    int16_t servoFrontRightFront = 270 - betaRightToAngle;
    int16_t servoFrontRightRear = 270 - alphaRightToAngle;

    // Установка всех 8 серво С офсетами
    int16_t final_FL_FRONT = servoFrontLeftFront + servoOffsets[SERVO_FL_FRONT];
    int16_t final_FL_REAR  = servoFrontLeftRear  + servoOffsets[SERVO_FL_REAR];
    int16_t final_FR_FRONT = servoFrontRightFront + servoOffsets[SERVO_FR_FRONT];
    int16_t final_FR_REAR  = servoFrontRightRear  + servoOffsets[SERVO_FR_REAR];
    int16_t final_BL_FRONT = servoFrontLeftFront  + servoOffsets[SERVO_BL_FRONT];
    int16_t final_BL_REAR  = servoFrontLeftRear   + servoOffsets[SERVO_BL_REAR];
    int16_t final_BR_FRONT = servoFrontRightFront + servoOffsets[SERVO_BR_FRONT];
    int16_t final_BR_REAR  = servoFrontRightRear  + servoOffsets[SERVO_BR_REAR];
    
    servos.setAngle(SERVO_FL_FRONT, final_FL_FRONT);
    servos.setAngle(SERVO_FL_REAR,  final_FL_REAR);
    servos.setAngle(SERVO_FR_FRONT, final_FR_FRONT);
    servos.setAngle(SERVO_FR_REAR,  final_FR_REAR);
    servos.setAngle(SERVO_BL_FRONT, final_BL_FRONT);
    servos.setAngle(SERVO_BL_REAR,  final_BL_REAR);
    servos.setAngle(SERVO_BR_FRONT, final_BR_FRONT);
    servos.setAngle(SERVO_BR_REAR,  final_BR_REAR);

    // Компактный вывод одной строкой
    Serial.printf("[IK] FL:%d/%d FR:%d/%d BL:%d/%d BR:%d/%d\n", 
        final_FL_FRONT, final_FL_REAR,
        final_FR_FRONT, final_FR_REAR,
        final_BL_FRONT, final_BL_REAR,
        final_BR_FRONT, final_BR_REAR);
    Serial.flush();
    delay(10);
}

/**
 * @brief Обработка команд Serial
 */
void handleSerial() {
    if (!Serial.available()) return;
    
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    
    if (cmd.length() == 0) return;
    
    // Команда: o<канал> <офсет>
    if (cmd.startsWith("o")) {
        int channel, offset;
        if (sscanf(cmd.c_str(), "o%d %d", &channel, &offset) == 2) {
            if (channel >= 0 && channel < 16) {
                servoOffsets[channel] = offset;
                
                // ВАЖНО: Применяем изменение в зависимости от режима!
                if (currentMode == MODE_DIRECT) {
                    // В DIRECT режиме - устанавливаем напрямую
                    applyServo(channel, offset);
                    Serial.printf("OK: CH%d=%d\n", channel, offset);
                } else {
                    // В IK режиме - пересчитываем всю IK с новыми офсетами!
                    Serial.printf("OK: CH%d=%d\n", channel, offset);
                    Serial.flush();
                    delay(5);
                    inverseKinematicsAll();
                }
            } else {
                Serial.println("ERROR: Канал должен быть 0-15");
            }
        } else {
            Serial.println("ERROR: Формат: o<канал> <офсет>");
        }
    }
    // Команда: a 90 (все серво на угол)
    else if (cmd.startsWith("a ")) {
        int angle;
        if (sscanf(cmd.c_str(), "a %d", &angle) == 1) {
            for (uint8_t i = 0; i < 16; i++) {
                servos.setAngle(i, angle);
            }
            Serial.printf("OK: Все серво установлены на %d°\n", angle);
        }
    }
    // Команда: p (вывести офсеты)
    else if (cmd == "p") {
        printOffsets();
        Serial.printf("\nРЕЖИМ: %s\n", currentMode == MODE_DIRECT ? "ПРЯМАЯ КАЛИБРОВКА" : "IK КАЛИБРОВКА");
        if (currentMode == MODE_IK) {
            Serial.printf("IK координаты: X=%.1f, Y=%.1f\n", IK_X, IK_Y);
        }
    }
    // Команда: l (автовыравнивание)
    else if (cmd == "l") {
        autoLevel();
    }
    // Команда: m (переключить режим)
    else if (cmd == "m") {
        currentMode = (currentMode == MODE_DIRECT) ? MODE_IK : MODE_DIRECT;
        
        if (currentMode == MODE_DIRECT) {
            Serial.println("MODE: DIRECT");
            applyAllServos();
        } else {
            Serial.printf("MODE: IK (X=%.1f Y=%.1f)\n", IK_X, IK_Y);
            inverseKinematicsAll();
        }
    }
    // Команда: x <значение> (установить X)
    else if (cmd.startsWith("x ")) {
        if (currentMode != MODE_IK) {
            Serial.println("ERROR: Команда 'x' работает только в IK режиме. Используй 'm' для переключения.");
        } else {
            float x_val;
            if (sscanf(cmd.c_str(), "x %f", &x_val) == 1) {
                IK_X = x_val;
                Serial.printf("OK: X=%.1f\n", IK_X);
                inverseKinematicsAll();
            } else {
                Serial.println("ERROR: Формат: x <значение>");
            }
        }
    }
    // Команда: y <значение> (установить Y)
    else if (cmd.startsWith("y ")) {
        if (currentMode != MODE_IK) {
            Serial.println("ERROR: Команда 'y' работает только в IK режиме. Используй 'm' для переключения.");
        } else {
            float y_val;
            if (sscanf(cmd.c_str(), "y %f", &y_val) == 1) {
                IK_Y = y_val;
                Serial.printf("OK: Y=%.1f\n", IK_Y);
                inverseKinematicsAll();
            } else {
                Serial.println("ERROR: Формат: y <значение>");
            }
        }
    }
    // Команда: r (сброс)
    else if (cmd == "r") {
        for (uint8_t i = 0; i < 16; i++) {
            servoOffsets[i] = 0;
        }
        applyAllServos();
        Serial.println("OK: Все офсеты сброшены в 0");
        printOffsets();
    }
    // Команда: h (помощь)
    else if (cmd == "h") {
        Serial.println("\n=== КОМАНДЫ ===");
        Serial.println("o<канал> <офсет>  - Установить офсет (например: o0 -15)");
        Serial.println("m                 - Переключить режим (калибровка ↔ IK)");
        Serial.println("x <значение>      - Установить X координату (IK режим)");
        Serial.println("y <значение>      - Установить Y координату (IK режим)");
        Serial.println("a <угол>          - Установить все серво на угол (например: a 90)");
        Serial.println("l                 - Автовыравнивание по IMU (level)");
        Serial.println("p                 - Вывести текущие офсеты и параметры");
        Serial.println("r                 - Сбросить все офсеты в 0");
        Serial.println("h                 - Показать эту помощь");
        Serial.println("===============\n");
    }
    else {
        Serial.println("ERROR: Неизвестная команда. Наберите 'h' для помощи");
    }
}

/**
 * @brief Инициализация
 */
void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("\n\n");
    Serial.println("========================================");
    Serial.println("  КАЛИБРОВКА + IK РЕЖИМ");
    Serial.println("========================================\n");
    
    // Инициализация I2C
    Wire.begin(1, 2, 400000UL);
    
    // Инициализация PCA9685
    servos.init();
    servos.setAngleRange(0, 300);
    servos.setPluseRange(500, 2500);
    delay(100);  // ВАЖНО: Задержка для полной инициализации PCA9685!
    
    Serial.println("[OK] PCA9685 инициализирован");
    
    // Инициализация IMU
    mpu6050.init();
    Serial.println("[OK] MPU6050 (IMU) инициализирован");
    
    // Установка всех серво с учетом офсетов
    Serial.println("[INFO] Применение офсетов...");
    
    // ВАЖНО: Применяем дважды с задержкой для надёжности!
    applyAllServos();
    delay(100);
    applyAllServos();  // Второй раз для гарантии
    delay(500);
    
    Serial.println("[OK] Все серво установлены с офсетами\n");
    Serial.println("========================================");
    Serial.println("ДВА РЕЖИМА РАБОТЫ:");
    Serial.println("========================================");
    Serial.println("1. ПРЯМАЯ КАЛИБРОВКА (текущий)");
    Serial.println("   - Серво: 90° + офсет");
    Serial.println("   - Команда 'o' для изменения офсетов");
    Serial.println("");
    Serial.println("2. IK КАЛИБРОВКА");
    Serial.println("   - Обратная кинематика + офсет");
    Serial.println("   - Команды 'x' и 'y' для координат");
    Serial.println("");
    Serial.println("Переключение: команда 'm'");
    Serial.println("========================================\n");
    Serial.println("Наберите 'h' для полного списка команд\n");
    
    printOffsets();
}

/**
 * @brief Основной цикл
 */
void loop() {
    handleSerial();
    delay(10);
}
