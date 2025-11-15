/**
 * @file main.cpp
 * @brief Упрощенная версия main контроллера с использованием классов Robot и Leg
 * 
 * ИСПОЛЬЗОВАНИЕ:
 * 1. Переименуйте текущий main.cpp в main_old.cpp
 * 2. Переименуйте этот файл в main.cpp
 * 3. Скомпилируйте проект
 */

#include <Arduino.h>
#include "Robot.h"

// ==================== ГЛОБАЛЬНЫЕ ОБЪЕКТЫ ====================
Robot robot;  // Главный объект робота

// ==================== SETUP ====================
void setup() {
    // Инициализация робота
    robot.init();
    
    Serial.println("\n========================================");
    Serial.println("  ЧЕТЫРЕХНОГИЙ РОБОТ (Новая архитектура)");
    Serial.println("  Классы: Robot + Leg + Gait");
    Serial.println("========================================\n");
    
    Serial.println("Доступные команды:");
    Serial.println("  'motors on'  - включить моторы");
    Serial.println("  'motors off' - выключить моторы");
    Serial.println("  'stab on'    - включить стабилизацию");
    Serial.println("  'stab off'   - выключить стабилизацию");
    Serial.println("  'ik on'      - включить обратную кинематику");
    Serial.println("  'ik off'     - выключить IK");
    Serial.println("  'h <высота>' - установить высоту (70-130 мм)");
    Serial.println("  'gait <тип>' - установить походку (stand/trot/walk/bound/crawl)");
    Serial.println("");
}

// ==================== LOOP ====================
void loop() {
    // Обработка Serial команд
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        cmd.toLowerCase();
        
        if (cmd == "motors on") {
            robot.setMotors(true);
            Serial.println("[CMD] Моторы ВКЛЮЧЕНЫ");
        }
        else if (cmd == "motors off") {
            robot.setMotors(false);
            Serial.println("[CMD] Моторы ВЫКЛЮЧЕНЫ");
        }
        else if (cmd == "stab on") {
            robot.setStabilization(true);
            Serial.println("[CMD] Стабилизация ВКЛЮЧЕНА");
        }
        else if (cmd == "stab off") {
            robot.setStabilization(false);
            Serial.println("[CMD] Стабилизация ВЫКЛЮЧЕНА");
        }
        else if (cmd == "ik on") {
            robot.setIK(true);
            Serial.println("[CMD] Обратная кинематика ВКЛЮЧЕНА");
        }
        else if (cmd == "ik off") {
            robot.setIK(false);
            Serial.println("[CMD] Обратная кинематика ВЫКЛЮЧЕНА");
        }
        else if (cmd.startsWith("h ")) {
            float height = cmd.substring(2).toFloat();
            if (height >= 70 && height <= 130) {
                robot.setHeight(height);
                Serial.printf("[CMD] Установлена высота: %.1f мм\n", height);
            } else {
                Serial.println("[CMD] Ошибка: высота должна быть в диапазоне 70-130 мм");
            }
        }
        else if (cmd.startsWith("gait ")) {
            String gaitName = cmd.substring(5);
            gaitName.trim();
            if (robot.setGait(gaitName)) {
                Serial.printf("[CMD] Походка изменена на: %s\n", gaitName.c_str());
            } else {
                Serial.println("[CMD] Доступные походки: stand, trot, walk, bound, crawl");
            }
        }
        else if (cmd.startsWith("move ")) {
            // Пример команды: "move fl 10 100" - переместить переднюю левую ногу в X=10, Y=100
            int spaceIdx1 = cmd.indexOf(' ', 5);
            int spaceIdx2 = cmd.indexOf(' ', spaceIdx1 + 1);
            
            if (spaceIdx1 > 0 && spaceIdx2 > 0) {
                String leg = cmd.substring(5, spaceIdx1);
                float x = cmd.substring(spaceIdx1 + 1, spaceIdx2).toFloat();
                float y = cmd.substring(spaceIdx2 + 1).toFloat();
                
                if (leg == "fl") {
                    if (robot.moveLegFL(x, y)) {
                        Serial.printf("[CMD] FL нога перемещена в X=%.1f Y=%.1f\n", x, y);
                    } else {
                        Serial.println("[CMD] Ошибка: координаты недостижимы");
                    }
                }
                else if (leg == "fr") {
                    if (robot.moveLegFR(x, y)) {
                        Serial.printf("[CMD] FR нога перемещена в X=%.1f Y=%.1f\n", x, y);
                    } else {
                        Serial.println("[CMD] Ошибка: координаты недостижимы");
                    }
                }
                else if (leg == "bl") {
                    if (robot.moveLegBL(x, y)) {
                        Serial.printf("[CMD] BL нога перемещена в X=%.1f Y=%.1f\n", x, y);
                    } else {
                        Serial.println("[CMD] Ошибка: координаты недостижимы");
                    }
                }
                else if (leg == "br") {
                    if (robot.moveLegBR(x, y)) {
                        Serial.printf("[CMD] BR нога перемещена в X=%.1f Y=%.1f\n", x, y);
                    } else {
                        Serial.println("[CMD] Ошибка: координаты недостижимы");
                    }
                }
                else if (leg == "all") {
                    robot.moveAllLegsTo(x, y);
                    Serial.printf("[CMD] Все ноги перемещены в X=%.1f Y=%.1f\n", x, y);
                }
            }
        }
        else {
            Serial.println("[CMD] Неизвестная команда");
        }
    }
    
    // Основной цикл обновления робота
    robot.update();
    
    delay(5);  // ~200Hz
}
