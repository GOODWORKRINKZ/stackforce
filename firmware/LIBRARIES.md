# Библиотеки StackForce

Для корректной работы проекта нужно добавить оригинальные библиотеки из репозитория StackForce.

## Необходимые библиотеки

### 1. SF_BLDC - Управление BLDC моторами
**Расположение в оригинале:**
```
https://gitee.com/StackForce/bipedal_wheeled_robot/blob/master/课程代码/第五课_转弯控制_闭环走直线/lesson5_TurnCtrl/lib/SF_BLDC/
```

**Что нужно:**
- `SF_BLDC.h`
- `SF_BLDC.cpp`

**Использование:**
```cpp
SF_BLDC motors = SF_BLDC(Serial2);
motors.begin(115200);
motors.setSpeeds(50, 50);  // левый, правый
```

### 2. SF_Servo - Управление сервоприводами
**Расположение в оригинале:**
```
https://gitee.com/StackForce/quadrupedal-wheeled-robot/tree/master/课程代码/lesson4_Gait/lib/SF_Servo
```

**Что нужно:**
- `SF_Servo.h`
- `SF_Servo.cpp`

**Использование:**
```cpp
SF_Servo servos = SF_Servo(Wire);
servos.begin();
servos.setAngle(0, 90);  // канал 0, угол 90°
```

### 3. SF_IMU - Работа с IMU (MPU6050)
**Расположение в оригинале:**
```
https://gitee.com/StackForce/quadrupedal-wheeled-robot/blob/master/课程代码/lesson3_IMU/lib/SF_IMU/SF_IMU.cpp
```

**Что нужно:**
- `SF_IMU.h`
- `SF_IMU.cpp`

**Использование:**
```cpp
SF_IMU mpu6050 = SF_IMU(Wire);
mpu6050.begin();
mpu6050.update();
float roll = mpu6050.getRoll();
float pitch = mpu6050.getPitch();
```

### 4. PID - PID контроллер для стабилизации
**Расположение в оригинале:**
```
https://gitee.com/StackForce/bipedal_wheeled_robot/blob/master/课程代码/第九课_仿生腿实现高机动性/lesson9_enhance/lib/pid/pid.h
```

**Что нужно:**
- `pid.h` (или использовать нашу реализацию)

**Использование:**
```cpp
#include "pid.h"

PID balancePID(2.0, 0.1, 0.5);  // Kp, Ki, Kd
balancePID.setOutputLimits(-30, 30);

void loop() {
    float correction = balancePID.compute(targetValue, currentValue);
}
```

**Применение:**
- Стабилизация баланса (roll/pitch от IMU)
- Контроль скорости моторов
- Позиционирование ног
- Поддержание высоты тела

См. подробную документацию в `firmware/PID_CONTROLLER.md`

### 5. SBUS - Протокол RC приемника (опционально)
**Расположение в оригинале:**
```
https://gitee.com/StackForce/bipedal_wheeled_robot/blob/master/课程代码/第九课_仿生腿实现高机动性/lesson9_enhance/lib/SBUS/sbus.h
```

**Статус:** ✅ Уже включена в проект

**Использование:**
```cpp
SBUS sbus(Serial2);
sbus.begin(16, 17, true);  // RX, TX, inverted

void loop() {
    if (sbus.update()) {
        int ch1 = sbus.getChannel(0);  // 172-1811
    }
}
```

См. `firmware/PPM_vs_SBUS.md` для выбора между PPM и SBUS

## Инструкция по установке

### Вариант 1: Ручное копирование

1. Клонируйте оригинальные репозитории StackForce:
```bash
git clone https://gitee.com/StackForce/bipedal_wheeled_robot.git
git clone https://gitee.com/StackForce/quadrupedal-wheeled-robot.git
```

2. Скопируйте библиотеки:

**Для main контроллера:**
```bash
# SF_BLDC
cp -r bipedal_wheeled_robot/课程代码/第五课_转弯控制_闭环走直线/lesson5_TurnCtrl/lib/SF_BLDC firmware/main_controller/lib/

# SF_Servo
cp -r quadrupedal-wheeled-robot/课程代码/lesson4_Gait/lib/SF_Servo firmware/main_controller/lib/

# SF_IMU
cp -r quadrupedal-wheeled-robot/课程代码/lesson3_IMU/lib/SF_IMU firmware/main_controller/lib/

# PID (опционально, если хотите оригинал)
cp -r bipedal_wheeled_robot/课程代码/第九课_仿生腿实现高机动性/lesson9_enhance/lib/pid firmware/main_controller/lib/PID
```

**Для aux контроллера:**
```bash
# SF_BLDC
cp -r bipedal_wheeled_robot/课程代码/第五课_转弯控制_闭环走直线/lesson5_TurnCtrl/lib/SF_BLDC firmware/aux_controller/lib/

# SF_Servo
cp -r quadrupedal-wheeled-robot/课程代码/lesson4_Gait/lib/SF_Servo firmware/aux_controller/lib/
```

### Вариант 2: Прямые ссылки

Скачайте файлы напрямую с Gitee (требуется вход):

1. **SF_BLDC**:
   - https://gitee.com/StackForce/bipedal_wheeled_robot/raw/master/课程代码/第五课_转弯控制_闭环走直线/lesson5_TurnCtrl/lib/SF_BLDC/SF_BLDC.h
   - https://gitee.com/StackForce/bipedal_wheeled_robot/raw/master/课程代码/第五课_转弯控制_闭环走直线/lesson5_TurnCtrl/lib/SF_BLDC/SF_BLDC.cpp

2. **SF_Servo**:
   - https://gitee.com/StackForce/quadrupedal-wheeled-robot/raw/master/课程代码/lesson4_Gait/lib/SF_Servo/SF_Servo.h
   - https://gitee.com/StackForce/quadrupedal-wheeled-robot/raw/master/课程代码/lesson4_Gait/lib/SF_Servo/SF_Servo.cpp

3. **SF_IMU**:
   - https://gitee.com/StackForce/quadrupedal-wheeled-robot/raw/master/课程代码/lesson3_IMU/lib/SF_IMU/SF_IMU.h
   - https://gitee.com/StackForce/quadrupedal-wheeled-robot/raw/master/课程代码/lesson3_IMU/lib/SF_IMU/SF_IMU.cpp

4. **PID** (опционально):
   - https://gitee.com/StackForce/bipedal_wheeled_robot/raw/master/课程代码/第九课_仿生腿实现高机动性/lesson9_enhance/lib/pid/pid.h
   - Проект уже включает рабочую реализацию PID

## Структура после установки

```
firmware/
├── main_controller/
│   ├── lib/
│   │   ├── SF_BLDC/
│   │   │   ├── SF_BLDC.h
│   │   │   └── SF_BLDC.cpp
│   │   ├── SF_Servo/
│   │   │   ├── SF_Servo.h
│   │   │   └── SF_Servo.cpp
│   │   ├── SF_IMU/
│   │   │   ├── SF_IMU.h
│   │   │   └── SF_IMU.cpp
│   │   ├── PID/
│   │   │   └── pid.h                    # ✅ Уже включен
│   │   └── SBUS/
│   │       └── sbus.h                   # ✅ Уже включен
│   ├── src/main.cpp
│   └── platformio.ini
│
└── aux_controller/
    ├── lib/
    │   ├── SF_BLDC/
    │   │   ├── SF_BLDC.h
    │   │   └── SF_BLDC.cpp
    │   ├── SF_Servo/
    │   │   ├── SF_Servo.h
    │   │   └── SF_Servo.cpp
    │   └── PID/
    │       └── pid.h                    # ✅ Уже включен
    ├── src/main.cpp
    └── platformio.ini
```

## Примечания

- Текущие файлы в директориях `lib/` являются заглушками и должны быть заменены оригинальными
- Оригинальные библиотеки StackForce используют:
  - **SF_BLDC**: Serial2 (UART) для связи с BLDC драйвером
  - **SF_Servo**: Wire (I2C) для управления сервоприводами через PCA9685
  - **SF_IMU**: Wire (I2C) для работы с MPU6050
  - **PID**: ✅ Уже включен в проект (header-only библиотека)
  - **SBUS**: ✅ Уже включен в проект (header-only библиотека)
- Все библиотеки распространяются под лицензией Apache 2.0

## Дополнительная документация

- **PID контроллер**: См. `firmware/PID_CONTROLLER.md` - подробное руководство по использованию и настройке
- **PPM vs SBUS**: См. `firmware/PPM_vs_SBUS.md` - выбор протокола RC приемника

## Лицензия

Оригинальные библиотеки © StackForce, Apache 2.0 License
