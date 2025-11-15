/**
 * @file Leg.h
 * @brief Класс для управления одной ногой робота
 */

#ifndef LEG_H
#define LEG_H

#include <Arduino.h>
#include "SF_Servo.h"
#include "quadrupedal_data.h"

/**
 * @brief Позиция ноги (идентификатор)
 */
enum class LegPosition {
    FRONT_LEFT,
    FRONT_RIGHT,
    BACK_LEFT,
    BACK_RIGHT
};

/**
 * @brief Класс управления одной ногой
 */
class Leg {
private:
    LegPosition position;           // Позиция ноги
    uint8_t servoFrontChannel;      // Канал переднего серво (плечо)
    uint8_t servoRearChannel;       // Канал заднего серво (бедро)
    int16_t servoFrontOffset;       // Калибровочное смещение переднего серво
    int16_t servoRearOffset;        // Калибровочное смещение заднего серво
    
    float currentX;                 // Текущая координата X (мм)
    float currentY;                 // Текущая координата Y (мм)
    
    SF_Servo* servos;               // Указатель на контроллер серво
    
    /**
     * @brief Обратная кинематика для 5-звенного механизма
     * @param x Координата X (мм)
     * @param y Координата Y (мм)
     * @param alphaOut Выходной угол alpha (радианы)
     * @param betaOut Выходной угол beta (радианы)
     * @return true если решение найдено
     */
    bool calculateIK(float x, float y, float& alphaOut, float& betaOut);
    
public:
    /**
     * @brief Конструктор
     * @param pos Позиция ноги
     * @param servoCtrl Указатель на контроллер серво
     */
    Leg(LegPosition pos, SF_Servo* servoCtrl);
    
    /**
     * @brief Инициализация ноги
     */
    void init();
    
    /**
     * @brief Переместить ногу в указанную позицию
     * @param x Координата X (мм)
     * @param y Координата Y (мм)
     * @return true если движение выполнено успешно
     */
    bool moveTo(float x, float y);
    
    /**
     * @brief Получить текущую позицию X
     */
    float getX() const { return currentX; }
    
    /**
     * @brief Получить текущую позицию Y
     */
    float getY() const { return currentY; }
    
    /**
     * @brief Получить позицию ноги
     */
    LegPosition getPosition() const { return position; }
};

#endif // LEG_H
