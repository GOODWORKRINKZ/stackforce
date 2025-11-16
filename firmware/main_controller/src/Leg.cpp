/**
 * @file Leg.cpp
 * @brief Реализация класса управления одной ногой робота
 */

#include "Leg.h"
#include <math.h>

/**
 * @brief Конструктор
 */
Leg::Leg(LegPosition pos, SF_Servo* servoCtrl) 
    : position(pos), 
      servos(servoCtrl),
      currentX(0),
      currentY(70)
{
    // Настройка каналов серво и смещений в зависимости от позиции ноги
    switch(position) {
        case LegPosition::FRONT_LEFT:
            servoFrontChannel = SERVO_FL_FRONT;
            servoRearChannel = SERVO_FL_REAR;
            servoFrontOffset = SERVO_FL_FRONT_OFFSET;
            servoRearOffset = SERVO_FL_REAR_OFFSET;
            break;
            
        case LegPosition::FRONT_RIGHT:
            servoFrontChannel = SERVO_FR_FRONT;
            servoRearChannel = SERVO_FR_REAR;
            servoFrontOffset = SERVO_FR_FRONT_OFFSET;
            servoRearOffset = SERVO_FR_REAR_OFFSET;
            break;
            
        case LegPosition::BACK_LEFT:
            servoFrontChannel = SERVO_BL_FRONT;
            servoRearChannel = SERVO_BL_REAR;
            servoFrontOffset = SERVO_BL_FRONT_OFFSET;
            servoRearOffset = SERVO_BL_REAR_OFFSET;
            break;
            
        case LegPosition::BACK_RIGHT:
            servoFrontChannel = SERVO_BR_FRONT;
            servoRearChannel = SERVO_BR_REAR;
            servoFrontOffset = SERVO_BR_FRONT_OFFSET;
            servoRearOffset = SERVO_BR_REAR_OFFSET;
            break;
    }
}

/**
 * @brief Инициализация ноги
 */
void Leg::init() {
    // Установить ногу в начальную позицию
    moveTo(0, 70);
}

/**
 * @brief Обратная кинематика для 5-звенного механизма
 */
bool Leg::calculateIK(float x, float y, float& alphaOut, float& betaOut) {
    float a = 2 * x * L1;
    float b = 2 * y * L1;
    float c = x * x + y * y + L1 * L1 - L2 * L2;
    float d = 2 * L4 * (x - L5);
    float e = 2 * L4 * y;
    float f = ((x - L5) * (x - L5) + L4 * L4 + y * y - L3 * L3);

    // Проверка на возможность решения
    float discriminant_alpha = (a * a) + (b * b) - (c * c);
    float discriminant_beta = (d * d) + (e * e) - (f * f);
    
    if (discriminant_alpha < 0 || discriminant_beta < 0) {
        return false;  // Решение невозможно
    }

    // Вычисление углов
    float alpha1 = 2 * atan((b + sqrt(discriminant_alpha)) / (a + c));
    float alpha2 = 2 * atan((b - sqrt(discriminant_alpha)) / (a + c));
    float beta1 = 2 * atan((e + sqrt(discriminant_beta)) / (d + f));
    float beta2 = 2 * atan((e - sqrt(discriminant_beta)) / (d + f));

    // Нормализация углов
    alpha1 = (alpha1 >= 0) ? alpha1 : (alpha1 + 2 * PI);
    alpha2 = (alpha2 >= 0) ? alpha2 : (alpha2 + 2 * PI);

    // Выбор правильного решения
    if (alpha1 >= PI / 4) alphaOut = alpha1;
    else alphaOut = alpha2;
    
    if (beta1 >= 0 && beta1 <= PI / 4) betaOut = beta1;
    else betaOut = beta2;

    return true;
}

/**
 * @brief Переместить ногу в указанную позицию
 */
bool Leg::moveTo(float x, float y) {
    constexpr float kLogThreshold = 0.5f;  // avoid spamming logs on tiny corrections
    const float prevX = currentX;
    const float prevY = currentY;
    const bool targetChanged = (fabsf(x - prevX) > kLogThreshold) || (fabsf(y - prevY) > kLogThreshold);

    float targetX = x;
    float targetY = y;
    const bool isFrontLeg = (position == LegPosition::FRONT_LEFT || position == LegPosition::FRONT_RIGHT);
    if (isFrontLeg) {
        targetY = (ROBOT_LOWEST_FOR_MOT + ROBOT_HIGHEST) - targetY;
    }
    targetY = _constrain(targetY, ROBOT_LOWEST_FOR_MOT, ROBOT_HIGHEST);

    if (targetChanged) {
        if (isFrontLeg) {
            Serial.printf("[LEG][%d] target -> X=%.1f Y=%.1f (adjY=%.1f) (prev X=%.1f Y=%.1f)\n",
                          static_cast<int>(position), x, y, targetY, prevX, prevY);
        } else {
            Serial.printf("[LEG][%d] target -> X=%.1f Y=%.1f (prev X=%.1f Y=%.1f)\n",
                          static_cast<int>(position), x, y, prevX, prevY);
        }
    }

    float alpha, beta;
    
    // Вычислить обратную кинематику
    if (!calculateIK(targetX, targetY, alpha, beta)) {
        Serial.printf("[LEG] Ошибка IK для ноги %d: X=%.1f Y=%.1f\n", (int)position, x, y);
        return false;
    }
    
    // Конвертация из радиан в градусы
    int16_t alphaAngle = (int)((alpha / (2 * PI)) * 360);
    int16_t betaAngle = (int)((beta / (2 * PI)) * 360);
    
    // Вычисление углов серво с учетом позиции ноги (левая/правая)
    int16_t servoFrontAngle, servoRearAngle;
    
    if (position == LegPosition::FRONT_LEFT || position == LegPosition::BACK_LEFT) {
        // Левые ноги
        servoFrontAngle = 90 + betaAngle;
        servoRearAngle = 90 + alphaAngle;
    } else {
        // Правые ноги
        servoFrontAngle = 270 - betaAngle;
        servoRearAngle = 270 - alphaAngle;
    }
    
    // Применение калибровочных смещений и установка серво
    servos->setAngle(servoFrontChannel, servoFrontAngle + servoFrontOffset);
    servos->setAngle(servoRearChannel, servoRearAngle + servoRearOffset);
    
    // Сохранение текущей позиции
    currentX = x;
    currentY = y;

    if (targetChanged) {
        Serial.printf("[LEG][%d] IK alpha=%.1fdeg beta=%.1fdeg | servoF(ch%d)=%d servoR(ch%d)=%d\n",
                      static_cast<int>(position),
                      (alpha / (2 * PI)) * 360.0f,
                      (beta / (2 * PI)) * 360.0f,
                      servoFrontChannel,
                      servoFrontAngle + servoFrontOffset,
                      servoRearChannel,
                      servoRearAngle + servoRearOffset);
    }
    
    return true;
}
