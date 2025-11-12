#ifndef _QUADRUPEDAL_DATA_h
#define _QUADRUPEDAL_DATA_h

// ==================== SBUS НАСТРОЙКИ ====================
#define SBUSPIN 16  // RX пин для SBUS (Serial1 на ESP32)

// Калибровка каналов SBUS
#define RCCHANNEL_MIN 200
#define RCCHANNEL_MAX 1800
#define RCCHANNEL_MID 1000
#define RCCHANNEL3_MIN 200
#define RCCHANNEL3_MID 1000
#define RCCHANNEL3_MAX 1800

// Мэппинг каналов (4-ногий колесный робот)
#define RC_CH_FORWARD 1     // Канал вперед/назад
#define RC_CH_TURN 0        // Канал поворота
#define RC_CH_HEIGHT 2      // Канал высоты
#define RC_CH_ROLL 3        // Канал крена

// Каналы серво на PCA9685 (все 8 подключены к main контроллеру)
// ⚠️ КАЛИБРОВКА: Ищем переднюю правую ногу методом перебора!
// Передние ноги (Front)
#define SERVO_FL_FRONT 11  // Front Left Front
#define SERVO_FL_REAR  10  // Front Left Rear
#define SERVO_FR_FRONT 0  // Front Right Front
#define SERVO_FR_REAR  1  // Front Right Rear

// Задние ноги (Back)
#define SERVO_BL_FRONT 4  // Back Left Front
#define SERVO_BL_REAR  5  // Back Left Rear (ВНИМАНИЕ: канал 8 = индекс 7 в массиве!)
#define SERVO_BR_FRONT 6  // Back Right Front
#define SERVO_BR_REAR  7  // Back Right Rear

// Режимы работы робота
#define ROBOTMODE_DISABLE 0
#define ROBOTMODE_MOTION 1
#define ROBOTMODE_CALIBRATION 2

// Ограничения высоты (мм)
#define ROBOT_HIGHEST 130
#define ROBOT_LOWEST_FOR_MOT 70
#define ROBOT_LOWEST_FOR_CAL 70

// ==================== ГЕОМЕТРИЯ НОГИ ====================
// Длины звеньев в мм (5-звенный механизм)
#define L1  60   // Верхнее плечо
#define L2  100  // Верхнее бедро
#define L3  100  // Нижнее бедро
#define L4  60   // Нижнее плечо
#define L5  40   // Смещение по X
#define L6  0    // Дополнительное смещение

// Смещения серво для калибровки (градусы)
#define SERVO_FL_FRONT_OFFSET -200   // 90 - 117
#define SERVO_FL_REAR_OFFSET  -200   // 90 - 153
#define SERVO_FR_FRONT_OFFSET -200  // 90 - 243
#define SERVO_FR_REAR_OFFSET  -200  // 90 - 207

#define SERVO_BL_FRONT_OFFSET 0   // задние = передние
#define SERVO_BL_REAR_OFFSET  0
#define SERVO_BR_FRONT_OFFSET 0
#define SERVO_BR_REAR_OFFSET  0

// ==================== ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ ====================
extern int RCValue[6];  // Значения каналов RC

// ==================== СТРУКТУРЫ ДАННЫХ ====================

// Параметры позы робота (IMU данные)
typedef struct {
    float pitch;      // Тангаж (градусы)
    float roll;       // Крен (градусы)
    float yaw;        // Рыскание (градусы)
    float GyroX;      // Гироскоп X (град/с)
    float GyroY;      // Гироскоп Y (град/с)
    float GyroZ;      // Гироскоп Z (град/с)
    float height;     // Текущая высота
    float speedAvg;   // Средняя скорость колес
} robotposeparam;

// Параметры движения (команды управления)
typedef struct {
    uint8_t mode;          // Режим работы
    
    float forward;         // Вперед/назад (для колес и IK)
    float turn;            // Поворот (для IK)
    float updown;          // Вверх/вниз (для IK)
    float roll;            // Крен (для IK)
    
    float throttleLimit;   // Ограничение газа
    float turnLimit;       // Ограничение поворота
    float forwardLimit;    // Ограничение наклона
    float rollLimit;       // Ограничение крена
    uint8_t highest;       // Максимальная высота
    uint8_t lowest;        // Минимальная высота
} robotmotionparam;

// Режимы робота
typedef struct {
    uint8_t mode;
    bool printFlag;
    bool motorEnable;
    bool servoEnable;
} robotmode;

// Целевые значения для моторов и серво
typedef struct {
    uint16_t servoLeftFront;
    uint16_t servoLeftRear;
    uint16_t servoRightFront;
    uint16_t servoRightRear;
    float motorLeft;
    float motorRight;
} motorstarget;

// Статус моторов
typedef struct {
    float M0Speed;
    float M1Speed;
    int M0Dir, M1Dir;
    int M0SpdDir, M1SpdDir;
} motorstatus;

// Параметры управления
typedef struct {
    float height;
    float legRollLeft;
    float legRollRight;
    float legLeft;
    float legRight;
    float roll;
    float forward;
    float pitch;
    float velocity;
    float differVel;
    float centerAngleOffset;
} controlparam;

// Координаты
typedef struct {
    float x;
    float xLeft, xRight;
    float yLeft, yRight;
} coordinate;

// Параметры обратной кинематики
typedef struct {
    float alphaLeft, betaLeft;    // Углы левой ноги (радианы)
    float alphaRight, betaRight;  // Углы правой ноги (радианы)
    float XLeft, YLeft;           // Координаты конца левой ноги
    float XRight, YRight;         // Координаты конца правой ноги
} IKparam;

// ==================== ВСПОМОГАТЕЛЬНЫЕ МАКРОСЫ ====================
#define _constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

#endif // _QUADRUPEDAL_DATA_h

