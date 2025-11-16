/**
 * @file Robot.h
 * @brief Класс для управления всем роботом
 */

#ifndef ROBOT_H
#define ROBOT_H

#include <Arduino.h>
#include "Leg.h"
#include "Gait.h"
#include "SF_Servo.h"
#include "SF_BLDC.h"
#include "SF_IMU.h"
#include "pid.h"
#include "sbus.h"
#include "quadrupedal_data.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

/**
 * @brief Класс управления роботом
 */
class Robot {
private:
    // Устройства
    SF_Servo servos;                    // PCA9685 драйвер серво
    SF_IMU imu;                         // MPU6050 IMU
    SF_BLDC motors;                     // BLDC моторы
    bfs::SbusRx sbusRx;                 // SBUS приемник
    
    // Ноги
    Leg legFL;                          // Передняя левая нога
    Leg legFR;                          // Передняя правая нога
    Leg legBL;                          // Задняя левая нога
    Leg legBR;                          // Задняя правая нога
    
    // Походки
    StandGait standGait;                // Стойка
    TrotGait trotGait;                  // Рысь
    WalkGait walkGait;                  // Шаг
    BoundGait boundGait;                // Прыжок
    CrawlGait crawlGait;                // Ползание
    Gait* currentGait;                  // Текущая походка
    
    // PID контроллеры
    PIDController pidVelocity;          // PID для скорости
    PIDController pidPitch;             // PID для тангажа
    PIDController pidRoll;              // PID для крена
    
    // Состояние
    robotposeparam pose;                // Текущая поза робота (IMU)
    robotmotionparam motion;            // Команды движения
    SF_BLDC_DATA bldcData;              // Данные от BLDC моторов
    
    int rcValues[8];                    // Значения каналов RC (CH1-CH8)
    std::array<int16_t, bfs::SbusRx::NUM_CH()> sbusData;
    
    // Флаги и параметры
    bool stabilizationEnabled;          // Стабилизация
    bool motorsEnabled;                 // Моторы
    bool ikEnabled;                     // Обратная кинематика
    bool twaiInstalled;                 // CAN инициализирован
    
    int m0Dir, m1Dir;                   // Направления моторов
    float targetSpeed;                  // Целевая скорость
    float stabRoll, stabPitch;          // Накопленная коррекция
    
    // Параметры управления
    float kpY;                          // Коэффициент для плавного изменения высоты
    float kpX;                          // Коэффициент для компенсации скорости по X
    float kpRoll;                       // Коэффициент для адаптации к неровностям
    float turnKp;                       // Коэффициент поворота
    float rollLimit;                    // Максимальный крен
    float legDistance;                  // Расстояние между ногами
    
    uint8_t lowestHeight;               // Минимальная высота
    uint8_t highestHeight;              // Максимальная высота

    // Фильтрация IMU
    float imuFilterAlpha;
    float lpfPitch;
    float lpfRoll;
    float lpfGyroX;
    float lpfGyroY;

    // Коэффициенты стабилизации (по примеру Денге)
    float stabPitchPGain;
    float stabPitchIGain;
    float stabRollPGain;
    float stabRollIGain;
    float pitchZero;
    float rollZero;
    
    // Приватные методы
    void readRC();                      // Чтение RC
    robotposeparam readIMU();           // Чтение IMU
    float lowPassFilter(float current, float previous) const;
    void updateStabilization(const robotposeparam& poseData); // Обновление стабилизации
    void updateMotors();                // Обновление моторов
    void setupCAN();                    // Инициализация CAN
    void sendToAux();                   // Отправка данных aux контроллеру
    void copyPoseAndStab(robotposeparam& poseOut, float& stabPitchOut, float& stabRollOut);
    void startTasks();
    static void imuTaskEntry(void* arg);
    static void controlTaskEntry(void* arg);
    void imuTaskLoop();
    void controlTaskLoop();
    
public:
    /**
     * @brief Конструктор
     */
    Robot();
    
    /**
     * @brief Инициализация робота
     */
    void init();
    
    /**
     * @brief Основной цикл обновления
     */
    void update();
    
    /**
     * @brief Включить/выключить стабилизацию
     */
    void setStabilization(bool enabled) { stabilizationEnabled = enabled; }
    
    /**
     * @brief Включить/выключить моторы
     */
    void setMotors(bool enabled) { motorsEnabled = enabled; }
    
    /**
     * @brief Включить/выключить обратную кинематику
     */
    void setIK(bool enabled) { ikEnabled = enabled; }
    
    /**
     * @brief Установить походку
     * @param gaitName Название походки: "stand", "trot", "walk", "bound", "crawl"
     * @return true если походка установлена успешно
     */
    bool setGait(const String& gaitName);
    
    /**
     * @brief Получить название текущей походки
     */
    String getCurrentGaitName() const { 
        return currentGait ? currentGait->getName() : "None"; 
    }
    
    /**
     * @brief Установить высоту всех ног
     * @param height Высота в мм
     */
    void setHeight(float height);
    
    /**
     * @brief Переместить все ноги в указанную позицию
     * @param x Координата X (мм)
     * @param y Координата Y (мм)
     */
    void moveAllLegsTo(float x, float y);
    
    /**
     * @brief Переместить переднюю левую ногу
     */
    bool moveLegFL(float x, float y) { return legFL.moveTo(x, y); }
    
    /**
     * @brief Переместить переднюю правую ногу
     */
    bool moveLegFR(float x, float y) { return legFR.moveTo(x, y); }
    
    /**
     * @brief Переместить заднюю левую ногу
     */
    bool moveLegBL(float x, float y) { return legBL.moveTo(x, y); }
    
    /**
     * @brief Переместить заднюю правую ногу
     */
    bool moveLegBR(float x, float y) { return legBR.moveTo(x, y); }
    
    /**
     * @brief Получить данные позы робота
     */
    const robotposeparam& getPose() const { return pose; }
    
    /**
     * @brief Получить данные движения
     */
    const robotmotionparam& getMotion() const { return motion; }

private:
    TaskHandle_t imuTaskHandle;
    TaskHandle_t controlTaskHandle;
    SemaphoreHandle_t poseMutex;
};

#endif // ROBOT_H
