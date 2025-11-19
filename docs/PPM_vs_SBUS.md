# Выбор протокола RC приемника: PPM vs SBUS

## Краткое сравнение

| Характеристика | PPM | SBUS |
|---------------|-----|------|
| **Тип протокола** | Аналоговый | Цифровой |
| **Количество каналов** | До 8-12 | До 16-18 |
| **Скорость обновления** | ~27ms | ~9ms |
| **Помехозащищенность** | Низкая | Высокая (CRC) |
| **Подключение** | 3 провода (+, -, сигнал) | 3 провода (+, -, сигнал) |
| **Популярность** | Старый стандарт | Современный |
| **Приемники** | FlySky FS-iA6B (PPM режим) | FrSky (X4R, R-XSR), FlySky новые |

## Подключение приемника (3 провода)

### Физическое подключение одинаковое:

```
RC Приемник → ESP32
    +5V     → 5V (или VIN)
    GND     → GND
   Signal   → GPIO (см. ниже)
```

### Различия в настройке:

**PPM:**
- Signal → GPIO 34 (цифровой вход с прерыванием)
- Режим приемника: PPM Output
- Код использует прерывание для измерения импульсов

**SBUS:**
- Signal → RX пин Serial2 (GPIO 16)
- Режим приемника: SBUS Output
- Сигнал **инвертированный** (ESP32 автоматически инвертирует)
- Код использует UART на 100000 baud, 8E2

## Как определить что у вас?

### 1. Посмотрите на приемник:

**FlySky (FS-iA6B, FS-iA10B):**
- Обычно поддерживает PPM и iBUS
- SBUS редко (только новые модели)
- Провод обычно называется "PPM/i-BUS"

**FrSky (X4R-SB, R-XSR, X8R):**
- Поддерживает SBUS из коробки
- Провод называется "SBUS"
- Может также выводить PPM (но SBUS лучше)

**Futaba:**
- Изобретатели SBUS
- Все современные поддерживают SBUS

### 2. Посмотрите документацию передатчика/приемника

### 3. Проверьте настройки приемника
- Некоторые приемники имеют переключатель PPM/SBUS
- Или настраивается через меню передатчика

## Конфигурация в коде

### Использование PPM (по умолчанию)

В `firmware/main_controller/include/config.h`:

```cpp
// Закомментируйте USE_SBUS, чтобы использовать PPM
// #define USE_SBUS

#define PPM_INPUT_PIN 34
#define PPM_CHANNELS 8
```

### Использование SBUS (рекомендуется)

В `firmware/main_controller/include/config.h`:

```cpp
// Раскомментируйте для использования SBUS
#define USE_SBUS

#ifdef USE_SBUS
    #define SBUS_RX_PIN 16
    #define SBUS_TX_PIN 17  // Не используется, но нужен для Serial2
    #define SBUS_CHANNELS 16
#endif
```

## Примеры подключения

### FlySky FS-iA6B в режиме PPM:

```
FS-iA6B (PPM порт)
├── +5V  → ESP32 5V
├── GND  → ESP32 GND
└── PPM  → ESP32 GPIO 34

Настройка: Переключить приемник в PPM режим
```

### FrSky X4R-SB в режиме SBUS:

```
X4R-SB (SBUS порт)
├── +    → ESP32 5V
├── -    → ESP32 GND
└── S    → ESP32 GPIO 16 (Serial2 RX)

TX пин 17 можно не подключать (только для прошивки Serial2)
```

## Что выбрать?

### Выбирайте **SBUS** если:
- ✅ У вас FrSky или Futaba приемник
- ✅ Нужно больше 8 каналов
- ✅ Хотите меньше задержки (~9ms vs ~27ms)
- ✅ Нужна лучшая помехозащищенность
- ✅ Строите новый робот

### Выбирайте **PPM** если:
- У вас старый FlySky приемник (FS-iA6B)
- Приемник не поддерживает SBUS
- 8 каналов достаточно
- Уже настроено PPM

## Проверка работы

### PPM диагностика:

```cpp
void setup() {
    Serial.begin(115200);
    pinMode(PPM_INPUT_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(PPM_INPUT_PIN), ppmInterrupt, RISING);
}

void loop() {
    if (ppmFrameReady) {
        Serial.printf("PPM CH1: %d, CH2: %d\n", ppmChannels[0], ppmChannels[1]);
    }
}
```

**Ожидаемые значения:** 1000-2000 мкс (центр ~1500)

### SBUS диагностика:

```cpp
SBUS sbus(Serial2);

void setup() {
    Serial.begin(115200);
    sbus.begin(16, 17, true);  // RX=16, TX=17, inverted=true
}

void loop() {
    if (sbus.update()) {
        Serial.printf("SBUS CH1: %d, CH2: %d\n", 
                     sbus.getChannel(0), sbus.getChannel(1));
        Serial.printf("Failsafe: %d, Lost: %d\n", 
                     sbus.isFailsafe(), sbus.isLostFrame());
    }
}
```

**Ожидаемые значения:** 172-1811 (центр ~992)

## Рекомендация для StackForce робота

Основываясь на оригинальном коде StackForce (lesson9), **рекомендуется использовать SBUS**:

1. Современнее и надежнее
2. 16 каналов (vs 8 у PPM)
3. Меньше задержка
4. Встроенная защита от ошибок
5. StackForce использует в продвинутых уроках

## Переключение между PPM и SBUS

Просто измените одну строку в `config.h`:

```cpp
// Для PPM (закомментировано)
// #define USE_SBUS

// Для SBUS (раскомментировано)
#define USE_SBUS
```

И перепрошейте контроллер!
