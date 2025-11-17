/**
 * @file Buzzer.cpp
 * @brief Реализация библиотеки бузера
 */

#include "Buzzer.h"

// Музыкальные ноты (частоты в Гц)
#define NOTE_C4  262
#define NOTE_D4  294
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_G4  392
#define NOTE_A4  440
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_D5  587
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_G5  784
#define NOTE_A5  880
#define NOTE_B5  988
#define NOTE_C6  1047
#define NOTE_PAUSE 0

Buzzer::Buzzer(uint8_t pin, uint8_t channel) 
    : pin(pin), pwmChannel(channel), playing(false), 
      noteCount(0), currentNote(0), noteStartTime(0) {
}

void Buzzer::begin() {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
    
    // Настройка PWM: канал, частота 2000 Гц, разрешение 8 бит
    ledcSetup(pwmChannel, 2000, 8);
    ledcAttachPin(pin, pwmChannel);
    ledcWrite(pwmChannel, 0);
    
    Serial.printf("[BUZZER] Инициализирован на GPIO%d (PWM канал %d)\n", pin, pwmChannel);
}

void Buzzer::tone(uint16_t frequency, uint16_t duration) {
    if (frequency == 0) {
        ledcWrite(pwmChannel, 0);
        return;
    }
    
    // Ограничиваем частоту для ESP32-S3 (избегаем ошибок LEDC)
    if (frequency < 100) frequency = 100;
    if (frequency > 10000) frequency = 10000;
    
    // Настройка частоты PWM с проверкой
    uint32_t result = ledcSetup(pwmChannel, frequency, 8);
    if (result == 0) {
        // Не удалось установить частоту - используем базовую
        ledcSetup(pwmChannel, 2000, 8);
        ledcWrite(pwmChannel, 128);
    } else {
        ledcWrite(pwmChannel, 128);  // 50% duty cycle
    }
    
    // Сохранение времени для автоматической остановки
    noteStartTime = millis();
    noteCount = 1;
    currentNote = 0;
    melody[0] = {frequency, duration};
    playing = true;
}

void Buzzer::stop() {
    ledcWrite(pwmChannel, 0);
    playing = false;
    noteCount = 0;
    currentNote = 0;
}

void Buzzer::play(BuzzerSound sound) {
    stop();  // Останавливаем предыдущий звук
    
    switch (sound) {
        case SOUND_STARTUP: {
            // Приветственная мелодия (восходящие ноты)
            Note notes[] = {
                {NOTE_C4, 100},
                {NOTE_E4, 100},
                {NOTE_G4, 100},
                {NOTE_C5, 200}
            };
            playMelody(notes, 4);
            break;
        }
        
        case SOUND_BARK_SINGLE: {
            // Одиночное гавканье (быстрый низкий звук)
            Note notes[] = {
                {150, 80},   // Низкий "гав"
                {200, 60}
            };
            playMelody(notes, 2);
            break;
        }
        
        case SOUND_BARK_DOUBLE: {
            // Двойное гавканье "гав-гав"
            Note notes[] = {
                {150, 80},
                {200, 60},
                {NOTE_PAUSE, 50},
                {150, 80},
                {200, 60}
            };
            playMelody(notes, 5);
            break;
        }
        
        case SOUND_BARK_ANGRY: {
            // Злое рычание (низкие вибрирующие звуки)
            Note notes[] = {
                {100, 150},
                {120, 150},
                {100, 150},
                {80, 200}
            };
            playMelody(notes, 4);
            break;
        }
        
        case SOUND_BEEP_SHORT: {
            Note notes[] = {
                {NOTE_A5, 50}
            };
            playMelody(notes, 1);
            break;
        }
        
        case SOUND_BEEP_LONG: {
            Note notes[] = {
                {NOTE_A5, 300}
            };
            playMelody(notes, 1);
            break;
        }
        
        case SOUND_SUCCESS: {
            // Восходящая мелодия (успех)
            Note notes[] = {
                {NOTE_C5, 100},
                {NOTE_E5, 100},
                {NOTE_G5, 150}
            };
            playMelody(notes, 3);
            break;
        }
        
        case SOUND_ERROR: {
            // Нисходящая мелодия (ошибка)
            Note notes[] = {
                {NOTE_G5, 100},
                {NOTE_E5, 100},
                {NOTE_C5, 150}
            };
            playMelody(notes, 3);
            break;
        }
        
        case SOUND_ALARM: {
            // Сирена (чередование высокий-низкий)
            Note notes[] = {
                {800, 200},
                {600, 200},
                {800, 200},
                {600, 200}
            };
            playMelody(notes, 4);
            break;
        }
        
        case SOUND_MELODY_HAPPY: {
            // Радостная мелодия
            Note notes[] = {
                {NOTE_C5, 150},
                {NOTE_D5, 150},
                {NOTE_E5, 150},
                {NOTE_C5, 150},
                {NOTE_E5, 150},
                {NOTE_G5, 300}
            };
            playMelody(notes, 6);
            break;
        }
        
        default:
            break;
    }
}

void Buzzer::playMelody(const Note* notes, uint8_t count) {
    if (count > MAX_NOTES) count = MAX_NOTES;
    
    // Копируем ноты в буфер
    for (uint8_t i = 0; i < count; i++) {
        melody[i] = notes[i];
    }
    
    noteCount = count;
    currentNote = 0;
    playing = true;
    
    playNextNote();
}

void Buzzer::playNextNote() {
    if (currentNote >= noteCount) {
        stop();
        return;
    }
    
    Note& note = melody[currentNote];
    
    if (note.frequency == 0) {
        // Пауза
        ledcWrite(pwmChannel, 0);
    } else {
        // Звук
        ledcSetup(pwmChannel, note.frequency, 8);
        ledcWrite(pwmChannel, 128);  // 50% duty cycle
    }
    
    noteStartTime = millis();
}

bool Buzzer::update() {
    if (!playing) return false;
    
    // Проверка завершения текущей ноты
    if (millis() - noteStartTime >= melody[currentNote].duration) {
        currentNote++;
        
        if (currentNote >= noteCount) {
            stop();
            return false;
        }
        
        playNextNote();
    }
    
    return true;
}
