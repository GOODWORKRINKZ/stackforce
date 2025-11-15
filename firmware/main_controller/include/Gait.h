/**
 * @file Gait.h
 * @brief Базовый класс и реализации походок для четырехногого робота
 */

#ifndef GAIT_H
#define GAIT_H

#include <Arduino.h>

/**
 * @brief Структура для позиции одной ноги
 */
struct LegTarget {
    float x;  // Координата X (мм)
    float y;  // Координата Y (мм)
    
    LegTarget() : x(0), y(0) {}
    LegTarget(float _x, float _y) : x(_x), y(_y) {}
};

/**
 * @brief Структура для целевых позиций всех 4 ног
 */
struct GaitTargets {
    LegTarget frontLeft;
    LegTarget frontRight;
    LegTarget backLeft;
    LegTarget backRight;
    
    GaitTargets() {}
    GaitTargets(float y) : 
        frontLeft(0, y), frontRight(0, y), 
        backLeft(0, y), backRight(0, y) {}
};

/**
 * @brief Параметры управления походкой
 */
struct GaitParams {
    float forward;       // Вперед/назад (-1.0 ... 1.0)
    float turn;          // Поворот (-1.0 ... 1.0)
    float height;        // Базовая высота (мм)
    float pitch;         // Тангаж от IMU (градусы)
    float roll;          // Крен от IMU (градусы)
    float stabPitch;     // Накопленная коррекция стабилизации по pitch
    float stabRoll;      // Накопленная коррекция стабилизации по roll
    float speed;         // Текущая скорость робота
    
    GaitParams() : forward(0), turn(0), height(100), 
                   pitch(0), roll(0), stabPitch(0), stabRoll(0), speed(0) {}
};

/**
 * @brief Базовый абстрактный класс походки
 */
class Gait {
protected:
    String name;                // Название походки
    float phaseOffset;          // Смещение фазы (для анимации)
    uint32_t lastUpdateTime;    // Время последнего обновления
    
public:
    Gait(const String& gaitName) : 
        name(gaitName), phaseOffset(0), lastUpdateTime(0) {}
    
    virtual ~Gait() {}
    
    /**
     * @brief Обновить походку и вычислить целевые позиции ног
     * @param params Параметры управления
     * @return Целевые позиции для всех 4 ног
     */
    virtual GaitTargets update(const GaitParams& params) = 0;
    
    /**
     * @brief Сброс состояния походки
     */
    virtual void reset() {
        phaseOffset = 0;
        lastUpdateTime = millis();
    }

    /**
     * @brief Установить состояние стабилизации (по умолчанию игнорируется)
     */
    virtual void setStabilization(bool) {}
    
    /**
     * @brief Получить название походки
     */
    String getName() const { return name; }
};

/**
 * @brief Походка "Стойка" - робот стоит на месте
 */
class StandGait : public Gait {
private:
    bool stabilizationEnabled;
    
public:
    StandGait() : Gait("Stand"), stabilizationEnabled(true) {}
    
    void setStabilization(bool enabled) { stabilizationEnabled = enabled; }
    
    GaitTargets update(const GaitParams& params) override;
};

/**
 * @brief Походка "Рысь" (Trot) - диагональные пары ног движутся синхронно
 * Быстрая и стабильная походка для ровных поверхностей
 */
class TrotGait : public Gait {
private:
    float stepHeight;       // Высота шага (мм)
    float stepLength;       // Длина шага (мм)
    float frequency;        // Частота шагов (Гц)
    float dutyCycle;        // Соотношение опора/перенос (0.5 = 50%)
    
public:
    TrotGait() : Gait("Trot"), 
                 stepHeight(30), 
                 stepLength(50), 
                 frequency(1.0),
                 dutyCycle(0.5) {}
    
    void setStepHeight(float height) { stepHeight = height; }
    void setStepLength(float length) { stepLength = length; }
    void setFrequency(float freq) { frequency = freq; }
    
    GaitTargets update(const GaitParams& params) override;
};

/**
 * @brief Походка "Шаг" (Walk) - статически стабильная походка
 * Медленная, но очень стабильная - всегда 3 ноги на земле
 */
class WalkGait : public Gait {
private:
    float stepHeight;       // Высота шага (мм)
    float stepLength;       // Длина шага (мм)
    float frequency;        // Частота шагов (Гц)
    uint8_t phase;          // Текущая фаза (0-3)
    
public:
    WalkGait() : Gait("Walk"), 
                 stepHeight(25), 
                 stepLength(40), 
                 frequency(0.5),
                 phase(0) {}
    
    void setStepHeight(float height) { stepHeight = height; }
    void setStepLength(float length) { stepLength = length; }
    void setFrequency(float freq) { frequency = freq; }
    
    GaitTargets update(const GaitParams& params) override;
    void reset() override;
};

/**
 * @brief Походка "Прыжок" (Bound) - обе передние, затем обе задние
 * Быстрая походка для высоких скоростей
 */
class BoundGait : public Gait {
private:
    float jumpHeight;       // Высота прыжка (мм)
    float jumpLength;       // Длина прыжка (мм)
    float frequency;        // Частота прыжков (Гц)
    
public:
    BoundGait() : Gait("Bound"), 
                  jumpHeight(40), 
                  jumpLength(60), 
                  frequency(0.8) {}
    
    void setJumpHeight(float height) { jumpHeight = height; }
    void setJumpLength(float length) { jumpLength = length; }
    void setFrequency(float freq) { frequency = freq; }
    
    GaitTargets update(const GaitParams& params) override;
};

/**
 * @brief Походка "Ползание" (Crawl) - очень медленная и стабильная
 * Для сложного рельефа и препятствий
 */
class CrawlGait : public Gait {
private:
    float stepHeight;       // Высота шага (мм)
    float stepLength;       // Длина шага (мм)
    float frequency;        // Частота шагов (Гц)
    uint8_t legSequence;    // Последовательность ног (0-3)
    
public:
    CrawlGait() : Gait("Crawl"), 
                  stepHeight(20), 
                  stepLength(30), 
                  frequency(0.3),
                  legSequence(0) {}
    
    void setStepHeight(float height) { stepHeight = height; }
    void setStepLength(float length) { stepLength = length; }
    void setFrequency(float freq) { frequency = freq; }
    
    GaitTargets update(const GaitParams& params) override;
    void reset() override;
};

#endif // GAIT_H
