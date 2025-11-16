/**
 * @file KalmanFilter.h
 * @brief Простой одномерный фильтр Калмана для IMU
 */

#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

/**
 * @brief Одномерный фильтр Калмана для оценки углов IMU
 * 
 * Объединяет данные акселерометра (медленные, но точные) 
 * и гироскопа (быстрые, но дрифтуют) для получения оптимальной оценки угла
 */
class KalmanFilter {
public:
    /**
     * @brief Конструктор
     * @param processNoise Шум процесса (Q) - насколько мы доверяем модели
     * @param measurementNoise Шум измерения (R) - насколько мы доверяем сенсору
     * @param estimationError Начальная ошибка оценки (P)
     */
    KalmanFilter(float processNoise = 0.001f, 
                 float measurementNoise = 0.03f, 
                 float estimationError = 1.0f);

    /**
     * @brief Обновление фильтра
     * @param measurement Измерение с акселерометра (угол)
     * @param rate Скорость изменения с гироскопа (градусы/сек)
     * @param dt Временной интервал (секунды)
     * @return Отфильтрованный угол
     */
    float update(float measurement, float rate, float dt);

    /**
     * @brief Сброс фильтра
     * @param value Начальное значение состояния
     */
    void reset(float value = 0.0f);

    /**
     * @brief Получить текущее состояние
     */
    float getState() const { return X; }

    /**
     * @brief Установить шум процесса
     */
    void setProcessNoise(float noise) { Q = noise; }

    /**
     * @brief Установить шум измерения
     */
    void setMeasurementNoise(float noise) { R = noise; }

private:
    float Q;  // Ковариация шума процесса
    float R;  // Ковариация шума измерения
    float P;  // Ковариация ошибки оценки
    float X;  // Текущая оценка состояния (угол)
};

#endif // KALMAN_FILTER_H
