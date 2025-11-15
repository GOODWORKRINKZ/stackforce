/**
 * @file servo_test.ino
 * @brief Программа для тестирования и определения сервоприводов
 * 
 * Эта программа поможет определить:
 * 1. Какой канал PCA9685 управляет какой частью ноги
 * 2. Диапазон движения каждого серво
 * 3. Необходимые offset для калибровки
 * 
 * Использование через Serial Monitor (115200 baud):
 * - Введите номер канала (0-7) и угол (0-180)
 * - Например: "0 90" установит канал 0 в положение 90°
 * - "scan" - автоматически пройдется по всем каналам
 */

#include <Arduino.h>
#include <Wire.h>

// PCA9685 настройки
#define PCA9685_ADDR 0x40
#define PCA9685_MODE1 0x00
#define PCA9685_PRESCALE 0xFE
#define PCA9685_LED0_ON_L 0x06

#define SERVO_FREQ 50
#define I2C_SDA 8
#define I2C_SCL 9
#define SERVO_ENABLE_PIN 42

class ServoTester {
private:
    TwoWire* _i2c;
    
    void writeToPCA(uint8_t addr, uint8_t data) {
        _i2c->beginTransmission(PCA9685_ADDR);
        _i2c->write(addr);
        _i2c->write(data);
        _i2c->endTransmission();
    }
    
    void setPWM(uint8_t num, uint16_t on, uint16_t off) {
        _i2c->beginTransmission(PCA9685_ADDR);
        _i2c->write(PCA9685_LED0_ON_L + 4 * num);
        _i2c->write(on);
        _i2c->write(on >> 8);
        _i2c->write(off);
        _i2c->write(off >> 8);
        _i2c->endTransmission();
    }

public:
    ServoTester(TwoWire &i2c) : _i2c(&i2c) {}
    
    void init() {
        pinMode(SERVO_ENABLE_PIN, OUTPUT);
        digitalWrite(SERVO_ENABLE_PIN, HIGH);
        
        _i2c->begin(I2C_SDA, I2C_SCL);
        _i2c->setClock(400000);
        
        // Сброс PCA9685
        writeToPCA(PCA9685_MODE1, 0x00);
        
        // Установка частоты PWM
        writeToPCA(PCA9685_MODE1, 0x10); // Sleep
        uint8_t prescale = round(25000000.0 / (4096 * SERVO_FREQ)) - 1;
        writeToPCA(PCA9685_PRESCALE, prescale);
        writeToPCA(PCA9685_MODE1, 0x00);
        delay(5);
        writeToPCA(PCA9685_MODE1, 0xA0); // AutoIncrement
        
        Serial.println("[INIT] PCA9685 инициализирован");
    }
    
    void setAngle(uint8_t channel, uint16_t angle) {
        if (channel > 15) {
            Serial.println("ОШИБКА: Канал должен быть 0-15");
            return;
        }
        if (angle > 180) {
            Serial.println("ОШИБКА: Угол должен быть 0-180");
            return;
        }
        
        // Конвертация угла в PWM (500-2500 мкс)
        float pulse = 500.0 + (angle / 180.0) * 2000.0;
        uint16_t pwm = (uint16_t)((pulse * 4096) / (1000000.0 / SERVO_FREQ));
        
        setPWM(channel, 0, pwm);
        
        Serial.printf("[SET] Канал %d -> %d° (PWM: %d)\n", channel, angle, pwm);
    }
    
    void scanAll() {
        Serial.println("\n=== СКАНИРОВАНИЕ ВСЕХ КАНАЛОВ ===");
        Serial.println("Каждый канал будет двигаться: 90° -> 45° -> 135° -> 90°");
        Serial.println("Наблюдайте, какая часть робота движется\n");
        
        for (int ch = 0; ch < 8; ch++) {
            Serial.printf("\n--- КАНАЛ %d ---\n", ch);
            delay(1000);
            
            Serial.println("Позиция: 90° (центр)");
            setAngle(ch, 90);
            delay(2000);
            
            Serial.println("Позиция: 45° (влево/вниз)");
            setAngle(ch, 45);
            delay(2000);
            
            Serial.println("Позиция: 135° (вправо/вверх)");
            setAngle(ch, 135);
            delay(2000);
            
            Serial.println("Возврат в центр: 90°");
            setAngle(ch, 90);
            delay(1000);
        }
        
        Serial.println("\n=== СКАНИРОВАНИЕ ЗАВЕРШЕНО ===\n");
    }
    
    void centerAll() {
        Serial.println("[CENTER] Установка всех серво в центр (90°)");
        for (int ch = 0; ch < 8; ch++) {
            setAngle(ch, 90);
        }
    }
};

ServoTester tester(Wire);

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("\n========================================");
    Serial.println("  SERVO TESTER - Тестирование сервоприводов");
    Serial.println("========================================\n");
    
    tester.init();
    tester.centerAll();
    
    Serial.println("\nКоманды:");
    Serial.println("  <канал> <угол>  - установить серво (например: 0 90)");
    Serial.println("  scan            - автоматическое сканирование всех каналов");
    Serial.println("  center          - все серво в центр (90°)");
    Serial.println("\nГотов к работе!\n");
}

void loop() {
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        
        if (input == "scan") {
            tester.scanAll();
        }
        else if (input == "center") {
            tester.centerAll();
        }
        else {
            // Парсинг "канал угол"
            int spacePos = input.indexOf(' ');
            if (spacePos > 0) {
                int channel = input.substring(0, spacePos).toInt();
                int angle = input.substring(spacePos + 1).toInt();
                tester.setAngle(channel, angle);
            }
            else {
                Serial.println("ОШИБКА: Неверный формат. Используйте: <канал> <угол>");
            }
        }
    }
}
