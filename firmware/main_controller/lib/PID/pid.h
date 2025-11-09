/**
 * @file pid.h
 * @brief PID контроллер для стабилизации и управления
 * 
 * Основано на библиотеке StackForce (lesson9_enhance)
 * Используется для:
 * - Стабилизации баланса (угол крена/тангажа)
 * - Управления скоростью моторов
 * - Контроля позиции ног
 * - Поддержания заданной высоты
 */

#ifndef PID_H
#define PID_H

#include <Arduino.h>

/**
 * @brief Класс PID контроллера
 */
class PID {
public:
    /**
     * @brief Конструктор
     * @param Kp Пропорциональный коэффициент
     * @param Ki Интегральный коэффициент
     * @param Kd Дифференциальный коэффициент
     */
    PID(float Kp = 0.0, float Ki = 0.0, float Kd = 0.0) {
        _Kp = Kp;
        _Ki = Ki;
        _Kd = Kd;
        _prevError = 0.0;
        _integral = 0.0;
        _output = 0.0;
        _lastTime = 0;
        _outputMin = -1000.0;
        _outputMax = 1000.0;
        _integralMin = -100.0;
        _integralMax = 100.0;
    }
    
    /**
     * @brief Установка коэффициентов PID
     * @param Kp Пропорциональный коэффициент
     * @param Ki Интегральный коэффициент
     * @param Kd Дифференциальный коэффициент
     */
    void setGains(float Kp, float Ki, float Kd) {
        _Kp = Kp;
        _Ki = Ki;
        _Kd = Kd;
    }
    
    /**
     * @brief Установка лимитов выходного сигнала
     * @param min Минимальное значение
     * @param max Максимальное значение
     */
    void setOutputLimits(float min, float max) {
        _outputMin = min;
        _outputMax = max;
    }
    
    /**
     * @brief Установка лимитов интегральной составляющей (anti-windup)
     * @param min Минимальное значение
     * @param max Максимальное значение
     */
    void setIntegralLimits(float min, float max) {
        _integralMin = min;
        _integralMax = max;
    }
    
    /**
     * @brief Вычисление выходного сигнала PID
     * @param setpoint Заданное значение (целевая точка)
     * @param measured Измеренное значение (текущее значение)
     * @param dt Время с последнего вызова (секунды), если 0 - вычисляется автоматически
     * @return Выходной сигнал управления
     */
    float compute(float setpoint, float measured, float dt = 0.0) {
        // Вычисление dt если не задано
        if (dt == 0.0) {
            unsigned long now = millis();
            if (_lastTime == 0) {
                _lastTime = now;
                return 0.0;
            }
            dt = (now - _lastTime) / 1000.0;  // Перевод в секунды
            _lastTime = now;
        }
        
        // Вычисление ошибки
        float error = setpoint - measured;
        
        // Пропорциональная составляющая
        float P = _Kp * error;
        
        // Интегральная составляющая (с anti-windup)
        _integral += error * dt;
        _integral = constrain(_integral, _integralMin, _integralMax);
        float I = _Ki * _integral;
        
        // Дифференциальная составляющая
        float derivative = 0.0;
        if (dt > 0.0) {
            derivative = (error - _prevError) / dt;
        }
        float D = _Kd * derivative;
        
        // Сохранение предыдущей ошибки
        _prevError = error;
        
        // Вычисление выхода
        _output = P + I + D;
        _output = constrain(_output, _outputMin, _outputMax);
        
        return _output;
    }
    
    /**
     * @brief Сброс состояния PID контроллера
     */
    void reset() {
        _prevError = 0.0;
        _integral = 0.0;
        _output = 0.0;
        _lastTime = 0;
    }
    
    /**
     * @brief Получение текущего выходного значения
     */
    float getOutput() {
        return _output;
    }
    
    /**
     * @brief Получение значения P составляющей
     */
    float getP() {
        return _Kp * _prevError;
    }
    
    /**
     * @brief Получение значения I составляющей
     */
    float getI() {
        return _Ki * _integral;
    }
    
    /**
     * @brief Получение значения D составляющей
     */
    float getD(float dt) {
        if (dt > 0.0) {
            return _Kd * (_prevError / dt);
        }
        return 0.0;
    }
    
    /**
     * @brief Получение текущей интегральной суммы
     */
    float getIntegral() {
        return _integral;
    }
    
    /**
     * @brief Получение Kp
     */
    float getKp() {
        return _Kp;
    }
    
    /**
     * @brief Получение Ki
     */
    float getKi() {
        return _Ki;
    }
    
    /**
     * @brief Получение Kd
     */
    float getKd() {
        return _Kd;
    }
    
private:
    float _Kp, _Ki, _Kd;          // PID коэффициенты
    float _prevError;              // Предыдущая ошибка
    float _integral;               // Интегральная сумма
    float _output;                 // Выходное значение
    unsigned long _lastTime;       // Время последнего вызова
    float _outputMin, _outputMax;  // Лимиты выхода
    float _integralMin, _integralMax; // Лимиты интеграла (anti-windup)
};

#endif // PID_H
