# Журнал изменений firmware

## Версия 2.0 - 2024-01 - Централизованное управление серво

### Основные изменения

**Архитектура:**
- ✅ Все 8 сервоприводов теперь управляются ТОЛЬКО main контроллером
- ✅ Aux контроллер упрощен до управления задними моторами M2, M3
- ✅ Использован паттерн `setEightServoAngle()` из примера quadrupedal robot
- ✅ **Четырехногий колесный робот** (4 ноги × 2 серво = 8 сервоприводов)

### Main Controller (`main_controller/`)

**Новый функционал:**
- Управление всеми 8 сервоприводами через PCA9685
- Обратная кинематика для всех 4 ног (front-left, front-right, back-left, back-right)
- PID стабилизация тангажа (pitch) и скорости
- Чтение SBUS приемника (throttle, turn, height, roll)
- Чтение IMU (MPU6050) для определения позы робота
- CAN master - отправка команд aux контроллеру
- Управление передними моторами M0, M1

**Ключевые файлы:**
- `include/quadrupedal_data.h` - конфигурация серво (каналы 1-8, смещения)
- `src/main.cpp` - основной цикл управления

**Mapping сервоприводов:**
```
SERVO_FL_FRONT = 3  (Front Left Front)
SERVO_FL_REAR  = 4  (Front Left Rear)
SERVO_FR_FRONT = 2  (Front Right Front)
SERVO_FR_REAR  = 1  (Front Right Rear)
SERVO_BL_FRONT = 7  (Back Left Front)
SERVO_BL_REAR  = 8  (Back Left Rear)
SERVO_BR_FRONT = 6  (Back Right Front)
SERVO_BR_REAR  = 5  (Back Right Rear)
```

### Aux Controller (`aux_controller/`)

**Упрощенный функционал:**
- ❌ Удалены все функции управления сервоприводами
- ❌ Удалена обратная кинематика
- ✅ CAN slave - прием команд от main
- ✅ Управление задними моторами M2, M3

**Протокол CAN:**
```cpp
// Message ID: CAN_ID_MAIN_TO_AUX (0x100)
// Формат: 8 байт
[0] = throttle + 100   // 0...200 (центр 100)
[1] = turn + 100       // 0...200 (центр 100)
[2] = height           // 0...255
[3] = reserved         // зарезервировано
[4] = roll_high        // roll * 10 (старший байт)
[5] = roll_low         // roll * 10 (младший байт)
[6] = pitch_high       // pitch * 10 (старший байт)
[7] = pitch_low        // pitch * 10 (младший байт)
```

### Библиотеки (копированы из примеров)

**SF_Servo** - Управление PCA9685:
- `setEightServoAngle(ch1, angle1, offset1, ..., ch8, angle8, offset8)` - установка 8 серво одновременно
- `setAngle(channel, angle)` - установка одного серво

**SF_BLDC** - Управление BLDC моторами:
- `init(&Serial2)` - инициализация через Serial2
- `setTargets(M0_torque, M1_torque)` - установка крутящих моментов

**SF_IMU** - Чтение MPU6050:
- `getMPUValue(robotPose)` - получение pitch, roll, yaw, gyro

**SBUS** - Прием FrSky:
- `begin(Serial1)` - инициализация на Serial1 (GPIO 16)
- `getChannel(n)` - чтение канала (172...1811)

**pid** - PID регулятор:
- `pid(P, I, D, windup_guard)` - конструктор
- `compute(setpoint, current_value, dt)` - вычисление

### Геометрия 5-звенной ноги

```
L1 = 60mm   (верхнее плечо)
L2 = 100mm  (верхнее бедро)
L3 = 100mm  (нижнее бедро)
L4 = 60mm   (нижнее плечо)
L5 = 40mm   (смещение по X)
```

### Частоты обновления

- Main loop: 200 Hz (5ms delay)
- Aux loop: 200 Hz (5ms delay)
- Motor control: 50 Hz (20ms интервал)
- Debug output: ~1 Hz (каждые 50 циклов)

### Безопасность

- Таймаут CAN команд: 500ms (аварийная остановка моторов)
- Проверка диапазонов углов серво: 0...300°
- Watchdog для потери связи с пультом

### Требования к сборке

- PlatformIO Core
- Framework: Arduino
- Platform: Espressif32
- Board: esp32-s3-devkitc-1

### Железо

**CAN коммуникация:**
- Модуль CAN/RS485 от StackForce
- CANTX -> GPIO35
- CANRX -> GPIO41
- Скорость: 1 Mbps

**BLDC моторы:**
- Управление через Serial2 (GPIO17 RX, GPIO18 TX)
- Main: M0, M1 (передние колеса)
- Aux: M2, M3 (задние колеса)

**Сервоприводы:**
- Все 8 серво на main контроллере
- PCA9685 @ I2C 0x40 (SDA=GPIO1, SCL=GPIO2)

**IMU:**
- MPU6050 @ I2C 0x68 (те же I2C пины)

**SBUS приемник:**
- GPIO16 (Serial1 RX)

### Примеры использования

Основаны на:
- `example/bipedal_wheeled_robot-master/` - lessons 3-9 (SBUS, IK, PID)
- `example/quadrupedal-wheeled-robot-master/` - lesson 5 (setEightServoAngle)

### Следующие шаги

1. Калибровка смещений сервоприводов (SERVO_*_OFFSET)
2. Настройка PID коэффициентов для стабилизации
3. Тестирование обратной кинематики на реальном роботе
4. Отладка CAN коммуникации между контроллерами
