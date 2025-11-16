/**
 * @file KalmanFilter.cpp
 * @brief Реализация фильтра Калмана
 */

#include "KalmanFilter.h"

KalmanFilter::KalmanFilter(float processNoise, float measurementNoise, float estimationError)
    : Q(processNoise)
    , R(measurementNoise)
    , P(estimationError)
    , X(0.0f)
{
}

float KalmanFilter::update(float measurement, float rate, float dt) {
    // Шаг 1: Предсказание (Prediction)
    // Используем гироскоп для предсказания следующего состояния
    // X(k|k-1) = X(k-1|k-1) + dt * rate
    X = X + dt * rate;
    
    // Обновляем ковариацию ошибки предсказания
    // P(k|k-1) = P(k-1|k-1) + Q
    P = P + Q;

    // Шаг 2: Коррекция (Update)
    // Вычисляем коэффициент Калмана
    // K = P(k|k-1) / (P(k|k-1) + R)
    float K = P / (P + R);
    
    // Корректируем оценку состояния с использованием измерения акселерометра
    // X(k|k) = X(k|k-1) + K * (measurement - X(k|k-1))
    X = X + K * (measurement - X);
    
    // Обновляем ковариацию ошибки оценки
    // P(k|k) = (1 - K) * P(k|k-1)
    P = (1.0f - K) * P;

    return X;
}

void KalmanFilter::reset(float value) {
    X = value;
    P = 1.0f;
}
