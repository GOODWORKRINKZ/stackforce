/**
 * @file Gait.cpp
 * @brief Реализация походок для четырехногого робота
 */

#include "Gait.h"
#include <math.h>
#define constrainValue(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt))) // 限幅函数
// ==================== ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ ====================

/**
 * @brief Интерполяция между двумя значениями
 */
float lerp(float a, float b, float t) {
    return a + (b - a) * constrain(t, 0.0f, 1.0f);
}

/**
 * @brief Плавная интерполяция (ease in-out)
 */
float smoothStep(float t) {
    t = constrain(t, 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

/**
 * @brief Траектория качания ноги (swing trajectory)
 * @param t Время в цикле (0.0 ... 1.0)
 * @param length Длина шага
 * @param height Высота подъема
 * @param baseX Базовая позиция X
 * @param baseY Базовая позиция Y
 * @return Позиция ноги
 */
LegTarget swingTrajectory(float t, float length, float height, float baseX, float baseY) {
    LegTarget target;
    
    // X: линейное движение от -length/2 до +length/2
    target.x = baseX + length * (t - 0.5f);
    
    // Y: параболическая траектория вверх и вниз
    float heightFactor = 4.0f * t * (1.0f - t);  // Парабола (макс в t=0.5)
    target.y = baseY + height * heightFactor;
    
    return target;
}

/**
 * @brief Траектория опорной ноги (stance trajectory)
 * @param t Время в цикле (0.0 ... 1.0)
 * @param length Длина шага
 * @param baseX Базовая позиция X
 * @param baseY Базовая позиция Y
 * @return Позиция ноги
 */
LegTarget stanceTrajectory(float t, float length, float baseX, float baseY) {
    LegTarget target;
    
    // X: движение назад (противоположно направлению движения)
    target.x = baseX + length * (0.5f - t);
    
    // Y: постоянная высота
    target.y = baseY;
    
    return target;
}

static void applyBodyCompensation(GaitTargets& targets, float pitchComp, float rollComp) {
    if (pitchComp == 0 && rollComp == 0) {
        return;
    }

    if (pitchComp != 0) {
        targets.frontLeft.y -= pitchComp;
        targets.frontRight.y -= pitchComp;
        targets.backLeft.y += pitchComp;
        targets.backRight.y += pitchComp;
    }

    if (rollComp != 0) {        // Apply opposing offsets per side so roll feedback stabilizes instead of amplifying
        // Roll should only differentiate left/right pairs; adjust both legs per side equally
        targets.frontLeft.y -= rollComp;
        targets.backLeft.y -= rollComp;
        targets.frontRight.y += rollComp;
        targets.backRight.y += rollComp;
    }
}

// ==================== СТОЙКА (STAND) ====================

GaitTargets StandGait::update(const GaitParams& params) {
    float baseHeight = constrainValue(params.height, 70.0f, 150.0f);
    GaitTargets targets(baseHeight);

    float rollComp = params.stabRoll + params.roll;
    float pitchComp = params.stabPitch + params.pitch;
    applyBodyCompensation(targets, pitchComp, rollComp);

    targets.frontLeft.y  = constrainValue(targets.frontLeft.y, 70, 150);
    targets.frontRight.y = constrainValue(targets.frontRight.y, 70, 150);
    targets.backLeft.y   = constrainValue(targets.backLeft.y, 70, 150);
    targets.backRight.y  = constrainValue(targets.backRight.y, 70, 150);

    return targets;
}

// ==================== РЫСЬ (TROT) ====================

GaitTargets TrotGait::update(const GaitParams& params) {
    // Обновление фазы
    uint32_t currentTime = millis();
    float dt = (currentTime - lastUpdateTime) / 1000.0f;
    lastUpdateTime = currentTime;
    
    phaseOffset += dt * frequency * params.forward;  // Скорость зависит от команды forward
    if (phaseOffset > 1.0f) phaseOffset -= 1.0f;
    if (phaseOffset < 0.0f) phaseOffset += 1.0f;
    
    // Адаптация параметров к скорости
    float actualStepLength = stepLength * abs(params.forward);
    float actualStepHeight = stepHeight;
    
    // Фазы для диагональных пар
    // Диагональ 1: FL + BR (начинают вместе)
    float phase1 = phaseOffset;
    // Диагональ 2: FR + BL (сдвиг на 0.5)
    float phase2 = fmod(phaseOffset + 0.5f, 1.0f);
    
    GaitTargets targets;
    
    // Передняя левая + задняя правая (диагональ 1)
    if (phase1 < dutyCycle) {
        // Swing фаза (нога в воздухе)
        float t = phase1 / dutyCycle;
        targets.frontLeft = swingTrajectory(t, actualStepLength, actualStepHeight, 0, params.height);
        targets.backRight = swingTrajectory(t, actualStepLength, actualStepHeight, 0, params.height);
    } else {
        // Stance фаза (нога на земле)
        float t = (phase1 - dutyCycle) / (1.0f - dutyCycle);
        targets.frontLeft = stanceTrajectory(t, actualStepLength, 0, params.height);
        targets.backRight = stanceTrajectory(t, actualStepLength, 0, params.height);
    }
    
    // Передняя правая + задняя левая (диагональ 2)
    if (phase2 < dutyCycle) {
        // Swing фаза
        float t = phase2 / dutyCycle;
        targets.frontRight = swingTrajectory(t, actualStepLength, actualStepHeight, 0, params.height);
        targets.backLeft = swingTrajectory(t, actualStepLength, actualStepHeight, 0, params.height);
    } else {
        // Stance фаза
        float t = (phase2 - dutyCycle) / (1.0f - dutyCycle);
        targets.frontRight = stanceTrajectory(t, actualStepLength, 0, params.height);
        targets.backLeft = stanceTrajectory(t, actualStepLength, 0, params.height);
    }
    
    float rollComp = params.stabRoll + params.roll * 0.2f;
    float pitchComp = params.stabPitch + params.pitch * 0.2f;
    applyBodyCompensation(targets, pitchComp, rollComp);
    
    return targets;
}

// ==================== ШАГ (WALK) ====================

GaitTargets WalkGait::update(const GaitParams& params) {
    // Обновление фазы
    uint32_t currentTime = millis();
    float dt = (currentTime - lastUpdateTime) / 1000.0f;
    lastUpdateTime = currentTime;
    
    phaseOffset += dt * frequency * abs(params.forward);
    
    // Переход на следующую фазу
    if (phaseOffset >= 1.0f) {
        phaseOffset -= 1.0f;
        phase = (phase + 1) % 4;
    }
    
    // Адаптация параметров
    float actualStepLength = stepLength * abs(params.forward);
    
    GaitTargets targets(params.height);
    
    // Последовательность: FR -> BL -> FL -> BR
    // В каждый момент только одна нога в воздухе
    
    float t = smoothStep(phaseOffset);
    
    switch(phase) {
        case 0:  // FR движется
            targets.frontRight = swingTrajectory(t, actualStepLength, stepHeight, 0, params.height);
            targets.frontLeft = stanceTrajectory(t, actualStepLength * 0.33f, 0, params.height);
            targets.backLeft = stanceTrajectory(t, actualStepLength * 0.33f, 0, params.height);
            targets.backRight = stanceTrajectory(t, actualStepLength * 0.33f, 0, params.height);
            break;
            
        case 1:  // BL движется
            targets.frontRight = stanceTrajectory(t, actualStepLength * 0.33f, 0, params.height);
            targets.backLeft = swingTrajectory(t, actualStepLength, stepHeight, 0, params.height);
            targets.frontLeft = stanceTrajectory(t, actualStepLength * 0.33f, 0, params.height);
            targets.backRight = stanceTrajectory(t, actualStepLength * 0.33f, 0, params.height);
            break;
            
        case 2:  // FL движется
            targets.frontRight = stanceTrajectory(t, actualStepLength * 0.33f, 0, params.height);
            targets.backLeft = stanceTrajectory(t, actualStepLength * 0.33f, 0, params.height);
            targets.frontLeft = swingTrajectory(t, actualStepLength, stepHeight, 0, params.height);
            targets.backRight = stanceTrajectory(t, actualStepLength * 0.33f, 0, params.height);
            break;
            
        case 3:  // BR движется
            targets.frontRight = stanceTrajectory(t, actualStepLength * 0.33f, 0, params.height);
            targets.backLeft = stanceTrajectory(t, actualStepLength * 0.33f, 0, params.height);
            targets.frontLeft = stanceTrajectory(t, actualStepLength * 0.33f, 0, params.height);
            targets.backRight = swingTrajectory(t, actualStepLength, stepHeight, 0, params.height);
            break;
    }
    
    float rollComp = params.stabRoll + params.roll * 0.25f;
    float pitchComp = params.stabPitch + params.pitch * 0.25f;
    applyBodyCompensation(targets, pitchComp, rollComp);
    
    return targets;
}

void WalkGait::reset() {
    Gait::reset();
    phase = 0;
}

// ==================== ПРЫЖОК (BOUND) ====================

GaitTargets BoundGait::update(const GaitParams& params) {
    // Обновление фазы
    uint32_t currentTime = millis();
    float dt = (currentTime - lastUpdateTime) / 1000.0f;
    lastUpdateTime = currentTime;
    
    phaseOffset += dt * frequency * abs(params.forward);
    if (phaseOffset > 1.0f) phaseOffset -= 1.0f;
    
    // Адаптация параметров
    float actualJumpLength = jumpLength * abs(params.forward);
    
    GaitTargets targets;
    
    // Фаза 0.0-0.5: Передние ноги прыгают, задние на земле
    // Фаза 0.5-1.0: Задние ноги прыгают, передние на земле
    
    if (phaseOffset < 0.5f) {
        // Передние ноги в воздухе
        float t = phaseOffset * 2.0f;  // Нормализуем к 0-1
        targets.frontLeft = swingTrajectory(t, actualJumpLength, jumpHeight, 0, params.height);
        targets.frontRight = swingTrajectory(t, actualJumpLength, jumpHeight, 0, params.height);
        
        // Задние ноги на земле (толкают)
        targets.backLeft = stanceTrajectory(t, actualJumpLength, 0, params.height);
        targets.backRight = stanceTrajectory(t, actualJumpLength, 0, params.height);
    } else {
        // Задние ноги в воздухе
        float t = (phaseOffset - 0.5f) * 2.0f;
        targets.backLeft = swingTrajectory(t, actualJumpLength, jumpHeight, 0, params.height);
        targets.backRight = swingTrajectory(t, actualJumpLength, jumpHeight, 0, params.height);
        
        // Передние ноги на земле
        targets.frontLeft = stanceTrajectory(t, actualJumpLength, 0, params.height);
        targets.frontRight = stanceTrajectory(t, actualJumpLength, 0, params.height);
    }
    
    return targets;
}

// ==================== ПОЛЗАНИЕ (CRAWL) ====================

GaitTargets CrawlGait::update(const GaitParams& params) {
    // Обновление фазы
    uint32_t currentTime = millis();
    float dt = (currentTime - lastUpdateTime) / 1000.0f;
    lastUpdateTime = currentTime;
    
    phaseOffset += dt * frequency * abs(params.forward);
    
    // Переход на следующую ногу
    if (phaseOffset >= 1.0f) {
        phaseOffset -= 1.0f;
        legSequence = (legSequence + 1) % 4;
    }
    
    // Адаптация параметров
    float actualStepLength = stepLength * abs(params.forward);
    
    GaitTargets targets(params.height);
    
    float t = smoothStep(phaseOffset);
    
    // Очень медленная последовательность: FR -> FL -> BR -> BL
    // Всегда 3 ноги на земле
    
    switch(legSequence) {
        case 0:  // FR движется
            targets.frontRight = swingTrajectory(t, actualStepLength, stepHeight, 0, params.height);
            targets.frontLeft = stanceTrajectory(t, actualStepLength * 0.25f, 0, params.height);
            targets.backRight = stanceTrajectory(t, actualStepLength * 0.25f, 0, params.height);
            targets.backLeft = stanceTrajectory(t, actualStepLength * 0.25f, 0, params.height);
            break;
            
        case 1:  // FL движется
            targets.frontRight = stanceTrajectory(t, actualStepLength * 0.25f, 0, params.height);
            targets.frontLeft = swingTrajectory(t, actualStepLength, stepHeight, 0, params.height);
            targets.backRight = stanceTrajectory(t, actualStepLength * 0.25f, 0, params.height);
            targets.backLeft = stanceTrajectory(t, actualStepLength * 0.25f, 0, params.height);
            break;
            
        case 2:  // BR движется
            targets.frontRight = stanceTrajectory(t, actualStepLength * 0.25f, 0, params.height);
            targets.frontLeft = stanceTrajectory(t, actualStepLength * 0.25f, 0, params.height);
            targets.backRight = swingTrajectory(t, actualStepLength, stepHeight, 0, params.height);
            targets.backLeft = stanceTrajectory(t, actualStepLength * 0.25f, 0, params.height);
            break;
            
        case 3:  // BL движется
            targets.frontRight = stanceTrajectory(t, actualStepLength * 0.25f, 0, params.height);
            targets.frontLeft = stanceTrajectory(t, actualStepLength * 0.25f, 0, params.height);
            targets.backRight = stanceTrajectory(t, actualStepLength * 0.25f, 0, params.height);
            targets.backLeft = swingTrajectory(t, actualStepLength, stepHeight, 0, params.height);
            break;
    }
    
    float rollComp = params.stabRoll + params.roll * 0.3f;
    float pitchComp = params.stabPitch + params.pitch * 0.3f;
    applyBodyCompensation(targets, pitchComp, rollComp);
    
    return targets;
}

void CrawlGait::reset() {
    Gait::reset();
    legSequence = 0;
}
