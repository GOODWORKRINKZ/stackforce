/**
 * @file kinematics.h
 * @brief Кинематика ног четырехногого робота
 * 
 * Содержит функции для:
 * - Прямой кинематики (углы серво -> положение ноги)
 * - Обратной кинематики (положение ноги -> углы серво)
 * - Генерации траекторий ноги
 */

#ifndef KINEMATICS_H
#define KINEMATICS_H

#include "config.h"

/**
 * @brief Структура для позиции точки в пространстве
 */
struct Point3D {
    float x;  // Вперед/назад (мм)
    float y;  // Влево/вправо (мм)
    float z;  // Вверх/вниз (мм)
    
    Point3D() : x(0), y(0), z(0) {}
    Point3D(float _x, float _y, float _z) : x(_x), y(_y), z(_z) {}
};

/**
 * @brief Структура для углов сустав
ов ноги
 */
struct LegAngles {
    float shoulder;  // Угол плеча (градусы)
    float hip;       // Угол бедра (градусы)
    
    LegAngles() : shoulder(90), hip(90) {}
    LegAngles(float s, float h) : shoulder(s), hip(h) {}
};

/**
 * @brief Класс для кинематики одной ноги
 */
class LegKinematics {
public:
    /**
     * @brief Прямая кинематика: углы серво -> положение конца ноги
     * @param angles Углы суставов
     * @return Положение конца ноги
     */
    static Point3D forwardKinematics(const LegAngles& angles) {
        float shoulderRad = degToRad(angles.shoulder);
        float hipRad = degToRad(angles.hip);
        
        // Простая 2D кинематика (игнорируем Y)
        float x = LEG_SHOULDER_LENGTH * cos(shoulderRad) + 
                  LEG_HIP_LENGTH * cos(shoulderRad + hipRad);
        float z = -LEG_SHOULDER_LENGTH * sin(shoulderRad) - 
                  LEG_HIP_LENGTH * sin(shoulderRad + hipRad);
        
        return Point3D(x, 0, z);
    }
    
    /**
     * @brief Обратная кинематика: положение конца ноги -> углы серво
     * @param target Целевое положение конца ноги
     * @param angles Выходные углы суставов
     * @return true если решение найдено, false если цель недостижима
     */
    static bool inverseKinematics(const Point3D& target, LegAngles& angles) {
        float x = target.x;
        float z = -target.z;  // Инвертируем Z
        
        // Проверка досягаемости
        float distance = sqrt(x*x + z*z);
        float maxReach = LEG_SHOULDER_LENGTH + LEG_HIP_LENGTH;
        float minReach = abs(LEG_SHOULDER_LENGTH - LEG_HIP_LENGTH);
        
        if (distance > maxReach || distance < minReach) {
            // Цель недостижима - вернем безопасную позицию
            angles.shoulder = 45;
            angles.hip = 90;
            return false;
        }
        
        // Косинусная теорема для угла в колене
        float cosHip = (LEG_SHOULDER_LENGTH*LEG_SHOULDER_LENGTH + 
                        LEG_HIP_LENGTH*LEG_HIP_LENGTH - 
                        distance*distance) / 
                       (2.0 * LEG_SHOULDER_LENGTH * LEG_HIP_LENGTH);
        
        cosHip = clamp(cosHip, -1.0, 1.0);
        float hipRad = acos(cosHip);
        
        // Угол плеча
        float alpha = atan2(z, x);
        float beta = acos((LEG_SHOULDER_LENGTH*LEG_SHOULDER_LENGTH + 
                          distance*distance - 
                          LEG_HIP_LENGTH*LEG_HIP_LENGTH) / 
                         (2.0 * LEG_SHOULDER_LENGTH * distance));
        
        float shoulderRad = alpha + beta;
        
        // Преобразование в градусы
        angles.shoulder = radToDeg(shoulderRad);
        angles.hip = radToDeg(hipRad);
        
        // Ограничение углов
        angles.shoulder = clamp(angles.shoulder, SERVO_SHOULDER_MIN, SERVO_SHOULDER_MAX);
        angles.hip = clamp(angles.hip, SERVO_HIP_MIN, SERVO_HIP_MAX);
        
        return true;
    }
    
    /**
     * @brief Генерация траектории качающейся ноги (swing phase)
     * @param phase Фаза движения (0.0 - 1.0)
     * @param stepLength Длина шага
     * @param stepHeight Высота шага
     * @return Положение конца ноги
     */
    static Point3D swingTrajectory(float phase, float stepLength, float stepHeight) {
        Point3D pos;
        
        // X: движение вперед
        pos.x = -stepLength/2.0 + stepLength * phase;
        
        // Z: параболическая траектория вверх и вниз
        // Максимум в середине фазы
        float heightPhase = 4.0 * phase * (1.0 - phase);  // 0 в начале и конце, 1 в середине
        pos.z = -GAIT_BODY_HEIGHT + stepHeight * heightPhase;
        
        pos.y = 0;
        
        return pos;
    }
    
    /**
     * @brief Генерация траектории опорной ноги (stance phase)
     * @param phase Фаза движения (0.0 - 1.0)
     * @param stepLength Длина шага
     * @return Положение конца ноги
     */
    static Point3D stanceTrajectory(float phase, float stepLength) {
        Point3D pos;
        
        // X: движение назад (тело движется вперед относительно ноги)
        pos.x = stepLength/2.0 - stepLength * phase;
        
        // Z: постоянная высота
        pos.z = -GAIT_BODY_HEIGHT;
        
        pos.y = 0;
        
        return pos;
    }
};

/**
 * @brief Класс для генерации походок
 */
class GaitGenerator {
public:
    GaitType currentGait;
    unsigned long cycleStartTime;
    float currentPhase;
    
    GaitGenerator() : currentGait(GAIT_STAND), cycleStartTime(0), currentPhase(0) {}
    
    /**
     * @brief Обновление фазы походки
     */
    void update() {
        if (currentGait == GAIT_STAND) {
            currentPhase = 0;
            return;
        }
        
        unsigned long elapsed = millis() - cycleStartTime;
        currentPhase = (float)(elapsed % GAIT_CYCLE_TIME) / GAIT_CYCLE_TIME;
    }
    
    /**
     * @brief Установка типа походки
     */
    void setGait(GaitType gait) {
        if (currentGait != gait) {
            currentGait = gait;
            cycleStartTime = millis();
            currentPhase = 0;
        }
    }
    
    /**
     * @brief Получение положения ноги в текущей фазе походки
     * @param legIndex Индекс ноги (0=FL, 1=FR, 2=BL, 3=BR)
     * @param velocity Скорость движения (0.0 - 1.0)
     * @return Положение конца ноги
     */
    Point3D getLegPosition(uint8_t legIndex, float velocity) {
        if (currentGait == GAIT_STAND) {
            // Стойка - все ноги на земле
            return Point3D(50, 0, -GAIT_BODY_HEIGHT);
        }
        
        // Получение фазового смещения для текущей походки
        float phaseOffset = 0;
        switch (currentGait) {
            case GAIT_WALK:
                phaseOffset = GAIT_WALK_PHASE[legIndex];
                break;
            case GAIT_TROT:
                phaseOffset = GAIT_TROT_PHASE[legIndex];
                break;
            case GAIT_BOUND:
                phaseOffset = GAIT_BOUND_PHASE[legIndex];
                break;
            default:
                phaseOffset = 0;
        }
        
        // Вычисление фазы для этой ноги
        float legPhase = fmod(currentPhase + phaseOffset, 1.0);
        
        // Масштабирование длины шага по скорости
        float stepLength = GAIT_STEP_LENGTH * velocity;
        float stepHeight = GAIT_STEP_HEIGHT;
        
        // Определение: нога в воздухе (swing) или на земле (stance)
        // Для walk и trot: 50% swing, 50% stance
        bool isSwing = (legPhase < 0.5);
        
        if (isSwing) {
            // Фаза качания (нога в воздухе)
            float swingPhase = legPhase * 2.0;  // 0-0.5 -> 0-1
            return LegKinematics::swingTrajectory(swingPhase, stepLength, stepHeight);
        } else {
            // Фаза опоры (нога на земле)
            float stancePhase = (legPhase - 0.5) * 2.0;  // 0.5-1.0 -> 0-1
            return LegKinematics::stanceTrajectory(stancePhase, stepLength);
        }
    }
};

#endif // KINEMATICS_H
