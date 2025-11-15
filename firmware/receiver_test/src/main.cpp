/**
 * @file main.cpp
 * @brief Тестирование SBUS приемника - вывод всех каналов в Serial
 * 
 * ПОДКЛЮЧЕНИЕ ПРИЕМНИКА (согласно коду Денге lesson7_RollCtrl):
 * - SBUS Signal -> GPIO41 (RX1 на Serial1)
 * - +5V -> 5V
 * - GND -> GND
 * 
 * ВАЖНО: 
 * - Убедитесь что приемник в режиме SBUS (не PPM/iBUS)
 * - Включите передатчик перед загрузкой
 * - Скорость монитора: 115200
 */

#include <Arduino.h>
#include "sbus.h"
#include "config.h"

// ==================== ГЛОБАЛЬНЫЕ ОБЪЕКТЫ ====================
bfs::SbusRx sbus(&Serial1);  // Как в примере Денге

// Статистика
unsigned long totalPackets = 0;
unsigned long lostPackets = 0;
unsigned long failsafeEvents = 0;
unsigned long lastPacketTime = 0;

// ==================== ANSI COLOR CODES ====================
#if USE_COLOR_OUTPUT
  #define COLOR_RESET   "\033[0m"
  #define COLOR_RED     "\033[31m"
  #define COLOR_GREEN   "\033[32m"
  #define COLOR_YELLOW  "\033[33m"
  #define COLOR_BLUE    "\033[34m"
  #define COLOR_MAGENTA "\033[35m"
  #define COLOR_CYAN    "\033[36m"
  #define COLOR_WHITE   "\033[37m"
  #define COLOR_BOLD    "\033[1m"
#else
  #define COLOR_RESET   ""
  #define COLOR_RED     ""
  #define COLOR_GREEN   ""
  #define COLOR_YELLOW  ""
  #define COLOR_BLUE    ""
  #define COLOR_MAGENTA ""
  #define COLOR_CYAN    ""
  #define COLOR_WHITE   ""
  #define COLOR_BOLD    ""
#endif

// ==================== ФУНКЦИИ ====================

/**
 * @brief Преобразование SBUS значения (172-1811) в проценты (-100..+100)
 */
float sbusToPercent(int16_t sbusValue) {
    const int16_t SBUS_MIN = 172;
    const int16_t SBUS_MAX = 1811;
    const int16_t SBUS_CENTER = 992;  // (172 + 1811) / 2
    
    // Мёртвая зона
    if (abs(sbusValue - SBUS_CENTER) < STICK_DEADZONE) {
        return 0.0f;
    }
    
    if (sbusValue < SBUS_CENTER) {
        // Отрицательная сторона (-100% to 0%)
        return ((float)(sbusValue - SBUS_MIN) / (SBUS_CENTER - SBUS_MIN) - 1.0f) * 100.0f;
    } else {
        // Положительная сторона (0% to +100%)
        return ((float)(sbusValue - SBUS_CENTER) / (SBUS_MAX - SBUS_CENTER)) * 100.0f;
    }
}

/**
 * @brief Преобразование SBUS значения в микросекунды PWM (1000-2000)
 */
int16_t sbusToPWM(int16_t sbusValue) {
    const int16_t SBUS_MIN = 172;
    const int16_t SBUS_MAX = 1811;
    const int16_t PWM_MIN = 1000;
    const int16_t PWM_MAX = 2000;
    
    return map(sbusValue, SBUS_MIN, SBUS_MAX, PWM_MIN, PWM_MAX);
}

/**
 * @brief Отрисовка прогресс-бара
 */
void printProgressBar(float percent, int width = 20) {
    int filled = (int)((percent + 100.0f) / 200.0f * width);
    filled = constrain(filled, 0, width);
    
    Serial.print("[");
    for (int i = 0; i < width; i++) {
        if (i < filled) {
            Serial.print("=");
        } else if (i == filled) {
            Serial.print(">");
        } else {
            Serial.print(" ");
        }
    }
    Serial.print("]");
}

/**
 * @brief Вывод всех каналов в консоль
 */
void printChannels() {
    auto channels = sbus.ch();  // Старый API Денге
    
    Serial.println("\n" + String(COLOR_BOLD) + "╔════════════════════════════════════════════════════════════════════╗" + COLOR_RESET);
    Serial.println(String(COLOR_BOLD) + "║" + COLOR_CYAN + "                    SBUS RECEIVER TEST                          " + COLOR_BOLD + "║" + COLOR_RESET);
    Serial.println(String(COLOR_BOLD) + "╠════════════════════════════════════════════════════════════════════╣" + COLOR_RESET);
    
    // Вывод каналов
    for (int i = 0; i < 16; i++) {
        int16_t raw = channels[i];
        float percent = sbusToPercent(raw);
        int16_t pwm = sbusToPWM(raw);
        
        // Цвет в зависимости от значения
        String color = COLOR_WHITE;
        if (percent > 50.0f) color = COLOR_GREEN;
        else if (percent < -50.0f) color = COLOR_RED;
        else if (abs(percent) < 5.0f) color = COLOR_CYAN;
        else color = COLOR_YELLOW;
        
        Serial.printf("| %sCH%02d%s ", COLOR_BOLD, i+1, COLOR_RESET);
        
        #if PRINT_RAW_VALUES
        Serial.printf("Raw:%4d ", raw);
        #endif
        
        #if PRINT_NORMALIZED
        Serial.printf("%s%+6.1f%%%s ", color.c_str(), percent, COLOR_RESET);
        #endif
        
        #if PRINT_MICROSECONDS
        Serial.printf("PWM:%4dus ", pwm);
        #endif
        
        // Прогресс бар
        printProgressBar(percent, 15);
        
        Serial.println(" " + String(COLOR_BOLD) + "|" + COLOR_RESET);
    }
    
    // Дополнительные каналы и статус
    Serial.println(String(COLOR_BOLD) + "+====================================================================+" + COLOR_RESET);
    
    // CH17 и CH18 (дискретные)
    Serial.printf("| %sCH17:%s %s  %sCH18:%s %s                                        %s|%s\n",
                  COLOR_BOLD, COLOR_RESET, 
                  sbus.ch17() ? (String(COLOR_GREEN) + "ON " + COLOR_RESET).c_str() : (String(COLOR_RED) + "OFF" + COLOR_RESET).c_str(),
                  COLOR_BOLD, COLOR_RESET,
                  sbus.ch18() ? (String(COLOR_GREEN) + "ON " + COLOR_RESET).c_str() : (String(COLOR_RED) + "OFF" + COLOR_RESET).c_str(),
                  COLOR_BOLD, COLOR_RESET);
    
    // Статус связи
    String lostFrameStr = sbus.lost_frame() ? (String(COLOR_RED) + "YES" + COLOR_RESET) : (String(COLOR_GREEN) + "NO " + COLOR_RESET);
    String failsafeStr = sbus.failsafe() ? (String(COLOR_RED) + "YES" + COLOR_RESET) : (String(COLOR_GREEN) + "NO " + COLOR_RESET);
    
    Serial.printf("| %sLost Frame:%s %s  %sFailsafe:%s %s                                %s|%s\n",
                  COLOR_BOLD, COLOR_RESET, lostFrameStr.c_str(),
                  COLOR_BOLD, COLOR_RESET, failsafeStr.c_str(),
                  COLOR_BOLD, COLOR_RESET);
    
    // Статистика
    unsigned long uptime = millis() / 1000;
    float packetRate = totalPackets / (float)max(1UL, uptime);
    unsigned long timeSinceLastPacket = millis() - lastPacketTime;
    
    Serial.printf("| %sPackets:%s %lu  %sLost:%s %lu  %sFailsafe events:%s %lu             %s|%s\n",
                  COLOR_BOLD, COLOR_RESET, totalPackets,
                  COLOR_BOLD, COLOR_RESET, lostPackets,
                  COLOR_BOLD, COLOR_RESET, failsafeEvents,
                  COLOR_BOLD, COLOR_RESET);
    
    Serial.printf("| %sRate:%s %.1f pkt/s  %sUptime:%s %lus  %sLast packet:%s %lums ago       %s|%s\n",
                  COLOR_BOLD, COLOR_RESET, packetRate,
                  COLOR_BOLD, COLOR_RESET, uptime,
                  COLOR_BOLD, COLOR_RESET, timeSinceLastPacket,
                  COLOR_BOLD, COLOR_RESET);
    
    Serial.println(String(COLOR_BOLD) + "+====================================================================+" + COLOR_RESET);
}

/**
 * @brief Setup
 */
void setup() {
    // Инициализация Serial
    Serial.begin(250000);
    delay(1000);
    
    Serial.println("\n\n" + String(COLOR_BOLD) + COLOR_CYAN);
    Serial.println("+===========================================================+");
    Serial.println("|                                                           |");
    Serial.println("|           SBUS RECEIVER TEST & DEBUG TOOL                 |");
    Serial.println("|                  ESP32-S3 DevKit C1                       |");
    Serial.println("|                                                           |");
    Serial.println("+===========================================================+");
    Serial.println(COLOR_RESET);
    
    Serial.println(String(COLOR_YELLOW) + "Подключение приемника:" + COLOR_RESET);
    Serial.printf("  SBUS Signal → GPIO%d (RX1 Serial1)\n", SBUS_RX_PIN);
    Serial.println("  +5V → 5V");
    Serial.println("  GND → GND\n");
    
    Serial.println(String(COLOR_YELLOW) + "Настройки:" + COLOR_RESET);
    Serial.printf("  Debug interval: %dms (%dHz)\n", DEBUG_PRINT_INTERVAL, 1000/DEBUG_PRINT_INTERVAL);
    Serial.printf("  Stick deadzone: ±%d (%.1f%%)\n", STICK_DEADZONE, STICK_DEADZONE/2048.0*100);
    Serial.printf("  Raw values: %s\n", PRINT_RAW_VALUES ? "ON" : "OFF");
    Serial.printf("  Normalized: %s\n", PRINT_NORMALIZED ? "ON" : "OFF");
    Serial.printf("  PWM μs: %s\n\n", PRINT_MICROSECONDS ? "ON" : "OFF");
    
    // Инициализация SBUS (как в примере Денге)
    Serial.println(String(COLOR_GREEN) + "Инициализация SBUS..." + COLOR_RESET);
    Serial.printf("Пины: RX=%d, TX=%d\n", SBUS_RX_PIN, SBUS_TX_PIN);
    Serial.println("Используется Serial1 (100000 baud, 8E2, inverted)");
    
    sbus.Begin(SBUS_RX_PIN, SBUS_TX_PIN);
    
    Serial.println(String(COLOR_GREEN) + "✓ SBUS готов!" + COLOR_RESET);
    
    // Проверка что Serial1 работает
    Serial.printf("Проверка Serial1.available(): %d\n", Serial1.available());
    Serial.println(String(COLOR_YELLOW) + "\nОжидание данных от приемника...\n" + COLOR_RESET);
    
    delay(500);
}

/**
 * @brief Loop
 */
void loop() {
    static unsigned long lastPrint = 0;
    static unsigned long lastDebug = 0;
    
    // Периодическая отладка (1 раз в 5 секунд)
    if (millis() - lastDebug > 5000 && totalPackets == 0) {
        Serial.printf("[DEBUG] Serial1.available(): %d, Uptime: %lus\n", 
                      Serial1.available(), millis()/1000);
        lastDebug = millis();
    }
    
    // Чтение данных от SBUS
    if (sbus.Read()) {
        totalPackets++;
        lastPacketTime = millis();
        
        // Проверка статуса
        if (sbus.lost_frame()) {
            lostPackets++;
        }
        if (sbus.failsafe()) {
            failsafeEvents++;
        }
    }
    
    // Вывод данных с заданным интервалом
    if (millis() - lastPrint >= DEBUG_PRINT_INTERVAL) {
        // ОЧИСТКА ЭКРАНА И ВОЗВРАТ КУРСОРА В НАЧАЛО
        Serial.print("\033[2J");    // Очистить весь экран
        Serial.print("\033[H");     // Курсор в позицию (1,1)
        
        if (totalPackets > 0) {
            printChannels();
        } else {
            // Анимация ожидания
            static int dots = 0;
            Serial.print("\rОжидание данных");
            for (int i = 0; i < dots; i++) Serial.print(".");
            for (int i = dots; i < 3; i++) Serial.print(" ");
            dots = (dots + 1) % 4;
        }
        
        lastPrint = millis();
    }
    
    // Предупреждение о таймауте
    if (totalPackets > 0 && (millis() - lastPacketTime > 1000)) {
        static bool warningShown = false;
        if (!warningShown) {
            Serial.println("\n" + String(COLOR_RED) + COLOR_BOLD + "⚠ ВНИМАНИЕ: Потеря связи с приемником!" + COLOR_RESET);
            warningShown = true;
        }
    }
    
    delay(1);  // Небольшая задержка
}
