/**
 * @file imu_calibration.cpp
 * @brief Калибровка IMU (MPU6050) и определение переда робота
 * 
 * Функции:
 * 1. Калибровка гироскопа (офсеты для X, Y, Z)
 * 2. Калибровка акселерометра (офсеты для X, Y, Z при горизонтальном положении)
 * 3. Отображение текущих углов (pitch, roll, yaw)
 * 4. Определение переда робота
 * 
 * Команды Serial (115200 baud):
 * - "c" - калибровка гироскопа (робот должен быть неподвижен)
 * - "s" - показать текущие значения IMU
 * - "f" - установить текущее направление как "перед" робота
 * - "h" - помощь
 */

#include <Arduino.h>
#include <Wire.h>

// ==================== КОНФИГУРАЦИЯ ====================
#define MPU6050_ADDR 0x68
#define MPU6050_SMPLRT_DIV 0x19
#define MPU6050_CONFIG 0x1a
#define MPU6050_GYRO_CONFIG 0x1b
#define MPU6050_ACCEL_CONFIG 0x1c
#define MPU6050_PWR_MGMT_1 0x6b

// ==================== ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ ====================
float gyroOffset[3] = {0, 0, 0};
float accelOffset[3] = {0, 0, 0};
float yawOffset = 0;  // Офсет для определения "переда"

float temp, acc[3], gyro[3];
float angleGyro[3], angleAcc[2];
float angle[3];
float accCoef = 0.03f;
float gyroCoef = 0.97f;
unsigned long preInterval = 0;

// ==================== ФУНКЦИИ I2C ====================
void writeToIMU(uint8_t addr, uint8_t data) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(addr);
    Wire.write(data);
    Wire.endTransmission();
}

uint8_t readFromIMU(uint8_t addr) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(addr);
    Wire.endTransmission();
    Wire.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)1);
    return Wire.read();
}

// ==================== ИНИЦИАЛИЗАЦИЯ IMU ====================
void initIMU() {
    writeToIMU(MPU6050_SMPLRT_DIV, 0x00);
    writeToIMU(MPU6050_CONFIG, 0x00);
    writeToIMU(MPU6050_GYRO_CONFIG, 0x08);  // ±500°/s
    writeToIMU(MPU6050_ACCEL_CONFIG, 0x00); // ±2g
    writeToIMU(MPU6050_PWR_MGMT_1, 0x01);
    delay(100);
    Serial.println("[OK] MPU6050 инициализирован");
}

// ==================== КАЛИБРОВКА ГИРОСКОПА ====================
void calibrateGyro() {
    float x = 0, y = 0, z = 0;
    int16_t rx, ry, rz;

    Serial.println("\n========================================");
    Serial.println("КАЛИБРОВКА ГИРОСКОПА");
    Serial.println("НЕ ДВИГАЙТЕ РОБОТА!");
    Serial.println("========================================");
    Serial.print("Сбор данных");

    for (int i = 0; i < 3000; i++) {
        if (i % 300 == 0) Serial.print(".");
        
        Wire.beginTransmission(MPU6050_ADDR);
        Wire.write(0x43);
        Wire.endTransmission(false);
        Wire.requestFrom((int)MPU6050_ADDR, 6);

        rx = Wire.read() << 8 | Wire.read();
        ry = Wire.read() << 8 | Wire.read();
        rz = Wire.read() << 8 | Wire.read();

        x += ((float)rx) / 65.5;
        y += ((float)ry) / 65.5;
        z += ((float)rz) / 65.5;
        
        delay(1);
    }

    gyroOffset[0] = x / 3000;
    gyroOffset[1] = y / 3000;
    gyroOffset[2] = z / 3000;

    Serial.println(" Готово!\n");
    Serial.println("Офсеты гироскопа:");
    Serial.printf("  X: %.3f\n", gyroOffset[0]);
    Serial.printf("  Y: %.3f\n", gyroOffset[1]);
    Serial.printf("  Z: %.3f\n", gyroOffset[2]);
    Serial.println("========================================\n");
}

// ==================== ОБНОВЛЕНИЕ IMU ====================
void updateIMU() {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom((int)MPU6050_ADDR, 14);

    int16_t rawAccX = Wire.read() << 8 | Wire.read();
    int16_t rawAccY = Wire.read() << 8 | Wire.read();
    int16_t rawAccZ = Wire.read() << 8 | Wire.read();
    int16_t rawTemp = Wire.read() << 8 | Wire.read();
    int16_t rawGyroX = Wire.read() << 8 | Wire.read();
    int16_t rawGyroY = Wire.read() << 8 | Wire.read();
    int16_t rawGyroZ = Wire.read() << 8 | Wire.read();

    temp = (rawTemp + 12412.0) / 340.0;

    acc[0] = ((float)rawAccX) / 16384.0;
    acc[1] = ((float)rawAccY) / 16384.0;
    acc[2] = ((float)rawAccZ) / 16384.0;

    angleAcc[0] = atan2(acc[1], acc[2] + abs(acc[0])) * 360 / 2.0 / M_PI;
    angleAcc[1] = atan2(acc[0], acc[2] + abs(acc[1])) * 360 / -2.0 / M_PI;

    gyro[0] = ((float)rawGyroX) / 65.5 - gyroOffset[0];
    gyro[1] = ((float)rawGyroY) / 65.5 - gyroOffset[1];
    gyro[2] = ((float)rawGyroZ) / 65.5 - gyroOffset[2];

    float interval = (millis() - preInterval) * 0.001;

    angleGyro[0] += gyro[0] * interval;
    angleGyro[1] += gyro[1] * interval;
    angleGyro[2] += gyro[2] * interval;

    angle[0] = (gyroCoef * (angle[0] + gyro[0] * interval)) + (accCoef * angleAcc[0]);
    angle[1] = (gyroCoef * (angle[1] + gyro[1] * interval)) + (accCoef * angleAcc[1]);
    angle[2] = angleGyro[2] - yawOffset;

    preInterval = millis();
}

// ==================== ПОКАЗАТЬ ЗНАЧЕНИЯ ====================
void showValues() {
    Serial.println("\n========================================");
    Serial.println("ТЕКУЩИЕ ЗНАЧЕНИЯ IMU");
    Serial.println("========================================");
    Serial.printf("Акселерометр (g):\n");
    Serial.printf("  X: %7.3f  Y: %7.3f  Z: %7.3f\n", acc[0], acc[1], acc[2]);
    Serial.printf("\nГироскоп (°/s):\n");
    Serial.printf("  X: %7.3f  Y: %7.3f  Z: %7.3f\n", gyro[0], gyro[1], gyro[2]);
    Serial.printf("\nУглы (°):\n");
    Serial.printf("  Pitch: %7.2f\n", -angle[0]);
    Serial.printf("  Roll:  %7.2f\n", angle[1]);
    Serial.printf("  Yaw:   %7.2f\n", angle[2]);
    Serial.printf("\nТемпература: %.1f °C\n", temp);
    Serial.println("========================================\n");
}

// ==================== УСТАНОВИТЬ ПЕРЕД ====================
void setFront() {
    yawOffset = angleGyro[2];
    Serial.println("\n========================================");
    Serial.println("ПЕРЕД РОБОТА УСТАНОВЛЕН");
    Serial.println("========================================");
    Serial.printf("Yaw офсет: %.2f°\n", yawOffset);
    Serial.println("\nТеперь поворот робота будет измеряться");
    Serial.println("относительно этого направления.");
    Serial.println("========================================\n");
}

// ==================== ПОМОЩЬ ====================
void showHelp() {
    Serial.println("\n========================================");
    Serial.println("КОМАНДЫ КАЛИБРОВКИ IMU");
    Serial.println("========================================");
    Serial.println("c  - Калибровка гироскопа (робот неподвижен)");
    Serial.println("s  - Показать текущие значения IMU");
    Serial.println("f  - Установить текущее направление как ПЕРЕД");
    Serial.println("h  - Показать эту помощь");
    Serial.println("========================================\n");
}

// ==================== ОБРАБОТКА КОМАНД ====================
void handleSerial() {
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();

        if (cmd == "c") {
            calibrateGyro();
        }
        else if (cmd == "s") {
            showValues();
        }
        else if (cmd == "f") {
            setFront();
        }
        else if (cmd == "h") {
            showHelp();
        }
        else {
            Serial.println("Неизвестная команда. Наберите 'h' для помощи.");
        }
    }
}

// ==================== SETUP ====================
void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n\n========================================");
    Serial.println("  КАЛИБРОВКА IMU (MPU6050)");
    Serial.println("  StackForce Robot");
    Serial.println("========================================\n");

    // Инициализация I2C
    Wire.begin(1, 2, 400000UL);
    Serial.println("[OK] I2C инициализирован (SDA=1, SCL=2)");

    // Инициализация IMU
    initIMU();

    // Автоматическая калибровка гироскопа при старте
    Serial.println("\n[INFO] Автоматическая калибровка при старте...");
    calibrateGyro();

    updateIMU();
    preInterval = millis();
    angleGyro[0] = 0;
    angleGyro[1] = 0;
    angle[0] = angleAcc[0];
    angle[1] = angleAcc[1];

    showHelp();
}

// ==================== LOOP ====================
void loop() {
    updateIMU();
    handleSerial();

    // Периодический вывод углов (каждые 500мс)
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 500) {
        Serial.printf("Pitch: %6.2f°  Roll: %6.2f°  Yaw: %6.2f°\r", 
                      -angle[0], angle[1], angle[2]);
        lastPrint = millis();
    }

    delay(10);
}
