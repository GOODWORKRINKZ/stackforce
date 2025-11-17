/**
 * @file Buzzer.h
 * @brief Библиотека для управления пассивным бузером (без генератора)
 * 
 * Поддерживает:
 * - Разные мелодии и звуковые эффекты
 * - "Гавкание" робота-собаки 🐕
 * - Неблокирующее воспроизведение
 * - PWM генерация тона
 */

#ifndef BUZZER_H
#define BUZZER_H

#include <Arduino.h>

// Предопределенные звуковые эффекты
enum BuzzerSound {
    SOUND_NONE = 0,
    SOUND_STARTUP,       // Приветственная мелодия
    SOUND_BARK_SINGLE,   // Одиночное гавканье
    SOUND_BARK_DOUBLE,   // Двойное гавканье
    SOUND_BARK_ANGRY,    // Злое рычание
    SOUND_BEEP_SHORT,    // Короткий сигнал
    SOUND_BEEP_LONG,     // Длинный сигнал
    SOUND_SUCCESS,       // Успех (восходящая мелодия)
    SOUND_ERROR,         // Ошибка (нисходящая)
    SOUND_ALARM,         // Тревога (сирена)
    SOUND_MELODY_HAPPY,  // Радостная мелодия
};

class Buzzer {
public:
    /**
     * @brief Конструктор
     * @param pin GPIO пин бузера
     * @param channel PWM канал (0-15)
     */
    Buzzer(uint8_t pin, uint8_t channel = 0);
    
    /**
     * @brief Инициализация
     */
    void begin();
    
    /**
     * @brief Воспроизвести звук
     * @param sound Тип звука из enum BuzzerSound
     */
    void play(BuzzerSound sound);
    
    /**
     * @brief Воспроизвести тон заданной частоты
     * @param frequency Частота в Гц (50-20000)
     * @param duration Длительность в мс
     */
    void tone(uint16_t frequency, uint16_t duration);
    
    /**
     * @brief Остановить звук
     */
    void stop();
    
    /**
     * @brief Обновление (вызывать в loop)
     * @return true если звук проигрывается
     */
    bool update();
    
    /**
     * @brief Проверка активности
     * @return true если бузер играет
     */
    bool isPlaying() const { return playing; }
    
private:
    uint8_t pin;
    uint8_t pwmChannel;
    bool playing;
    
    // Структура ноты
    struct Note {
        uint16_t frequency;  // Частота в Гц (0 = пауза)
        uint16_t duration;   // Длительность в мс
    };
    
    // Буфер для мелодии
    static const uint8_t MAX_NOTES = 32;
    Note melody[MAX_NOTES];
    uint8_t noteCount;
    uint8_t currentNote;
    unsigned long noteStartTime;
    
    void playMelody(const Note* notes, uint8_t count);
    void playNextNote();
};

#endif // BUZZER_H
