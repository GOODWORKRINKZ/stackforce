/**
 * @file sbus.h
 * @brief Библиотека для декодирования SBUS протокола от RC приемника
 * 
 * Основано на библиотеке StackForce
 * SBUS - цифровой протокол от Futaba/FrSky
 * - 16 каналов
 * - Инвертированный UART на 100000 baud
 * - 25 байт пакет каждые ~9-14мс
 */

#ifndef SBUS_H
#define SBUS_H

#include <Arduino.h>

#define SBUS_SIGNAL_OK          0x00
#define SBUS_SIGNAL_LOST        0x01
#define SBUS_SIGNAL_FAILSAFE    0x02

class SBUS {
public:
    /**
     * @brief Конструктор
     * @param serial Ссылка на Serial порт (обычно Serial2)
     */
    SBUS(HardwareSerial &serial) : _serial(&serial) {
        for (int i = 0; i < 16; i++) {
            _channels[i] = 1024; // Центральное значение
        }
        _failsafe = false;
        _lostFrame = false;
        _goodFrames = 0;
        _lostFrames = 0;
    }
    
    /**
     * @brief Инициализация SBUS приемника
     * @param rxPin Пин RX (например 16)
     * @param txPin Пин TX (не используется, но нужен для Serial, например 17)
     * @param inverse true для инвертированного сигнала (обычно true для SBUS)
     */
    void begin(int8_t rxPin = -1, int8_t txPin = -1, bool inverse = true) {
        // SBUS использует 100000 baud, 8E2 (8 бит данных, четность, 2 стоп-бита)
        if (rxPin >= 0 && txPin >= 0) {
            _serial->begin(100000, SERIAL_8E2, rxPin, txPin, inverse);
        } else {
            _serial->begin(100000, SERIAL_8E2);
        }
        Serial.println("[SBUS] Инициализирован (100000 baud, 8E2, inverted)");
    }
    
    /**
     * @brief Обновление данных (вызывать в loop)
     * @return true если новый пакет получен
     */
    bool update() {
        bool newData = false;
        
        while (_serial->available() > 0) {
            uint8_t byte = _serial->read();
            
            if (_bufferIndex == 0 && byte != 0x0F) {
                // Ждем стартовый байт
                continue;
            }
            
            _buffer[_bufferIndex++] = byte;
            
            if (_bufferIndex == 25) {
                // Полный пакет получен
                _bufferIndex = 0;
                
                if (_buffer[0] == 0x0F && _buffer[24] == 0x00) {
                    // Валидный пакет
                    parsePacket();
                    newData = true;
                    _goodFrames++;
                    _lastUpdate = millis();
                } else {
                    _lostFrames++;
                }
            }
        }
        
        // Проверка таймаута
        if (millis() - _lastUpdate > 100) {
            _lostFrame = true;
        } else {
            _lostFrame = false;
        }
        
        return newData;
    }
    
    /**
     * @brief Получение значения канала
     * @param channel Номер канала (0-15)
     * @return Значение канала (172-1811, центр ~992)
     */
    uint16_t getChannel(uint8_t channel) {
        if (channel < 16) {
            return _channels[channel];
        }
        return 1024;
    }
    
    /**
     * @brief Получение нормализованного значения канала
     * @param channel Номер канала (0-15)
     * @return Значение от -100 до 100
     */
    int16_t getChannelNormalized(uint8_t channel) {
        if (channel < 16) {
            return map(_channels[channel], 172, 1811, -100, 100);
        }
        return 0;
    }
    
    /**
     * @brief Проверка статуса failsafe
     * @return true если активен failsafe
     */
    bool isFailsafe() {
        return _failsafe;
    }
    
    /**
     * @brief Проверка потери сигнала
     * @return true если сигнал потерян
     */
    bool isLostFrame() {
        return _lostFrame;
    }
    
    /**
     * @brief Получение количества хороших пакетов
     */
    uint32_t getGoodFrames() {
        return _goodFrames;
    }
    
    /**
     * @brief Получение количества потерянных пакетов
     */
    uint32_t getLostFrames() {
        return _lostFrames;
    }
    
private:
    HardwareSerial* _serial;
    uint8_t _buffer[25];
    uint8_t _bufferIndex = 0;
    uint16_t _channels[16];
    bool _failsafe;
    bool _lostFrame;
    uint32_t _goodFrames;
    uint32_t _lostFrames;
    unsigned long _lastUpdate = 0;
    
    /**
     * @brief Парсинг SBUS пакета
     */
    void parsePacket() {
        // SBUS использует 11 бит на канал, упакованные в 22 байта
        _channels[0]  = ((_buffer[1]    | _buffer[2]<<8)                 & 0x07FF);
        _channels[1]  = ((_buffer[2]>>3 | _buffer[3]<<5)                 & 0x07FF);
        _channels[2]  = ((_buffer[3]>>6 | _buffer[4]<<2 | _buffer[5]<<10) & 0x07FF);
        _channels[3]  = ((_buffer[5]>>1 | _buffer[6]<<7)                 & 0x07FF);
        _channels[4]  = ((_buffer[6]>>4 | _buffer[7]<<4)                 & 0x07FF);
        _channels[5]  = ((_buffer[7]>>7 | _buffer[8]<<1 | _buffer[9]<<9)  & 0x07FF);
        _channels[6]  = ((_buffer[9]>>2 | _buffer[10]<<6)                & 0x07FF);
        _channels[7]  = ((_buffer[10]>>5| _buffer[11]<<3)                & 0x07FF);
        _channels[8]  = ((_buffer[12]   | _buffer[13]<<8)                & 0x07FF);
        _channels[9]  = ((_buffer[13]>>3| _buffer[14]<<5)                & 0x07FF);
        _channels[10] = ((_buffer[14]>>6| _buffer[15]<<2 | _buffer[16]<<10) & 0x07FF);
        _channels[11] = ((_buffer[16]>>1| _buffer[17]<<7)                & 0x07FF);
        _channels[12] = ((_buffer[17]>>4| _buffer[18]<<4)                & 0x07FF);
        _channels[13] = ((_buffer[18]>>7| _buffer[19]<<1 | _buffer[20]<<9) & 0x07FF);
        _channels[14] = ((_buffer[20]>>2| _buffer[21]<<6)                & 0x07FF);
        _channels[15] = ((_buffer[21]>>5| _buffer[22]<<3)                & 0x07FF);
        
        // Флаги в байте 23
        _failsafe = (_buffer[23] & 0x08) != 0;
        _lostFrame = (_buffer[23] & 0x04) != 0;
    }
};

#endif // SBUS_H
