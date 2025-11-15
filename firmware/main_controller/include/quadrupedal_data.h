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
// ВНИМАНИЕ: Робот повернут на 180° относительно гироскопа
// Физические каналы переназначены под правильную ориентацию

// Передние ноги (Front) - были задние
#define SERVO_FL_FRONT 2   // Front Left Front (физически был BR)
#define SERVO_FL_REAR  3   // Front Left Rear (физически был BR)
#define SERVO_FR_FRONT 9   // Front Right Front (физически был BL)
#define SERVO_FR_REAR  8   // Front Right Rear (физически был BL)

// Задние ноги (Back) - были передние
#define SERVO_BL_FRONT 0   // Back Left Front (физически был FR)
#define SERVO_BL_REAR  1   // Back Left Rear (физически был FR)
#define SERVO_BR_FRONT 11  // Back Right Front (физически был FL)
#define SERVO_BR_REAR  10  // Back Right Rear (физически был FL)

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
// Откалибровано в IK режиме при X=0, Y=115 (после исправления формулы IK!)
// После поворота робота на 180° офсеты переназначены
#define SERVO_FL_FRONT_OFFSET -35   // CH2 (физически был BR)
#define SERVO_FL_REAR_OFFSET  -162  // CH3 (физически был BR)
#define SERVO_FR_FRONT_OFFSET -150  // CH9 (физически был BL)
#define SERVO_FR_REAR_OFFSET  -15   // CH8 (физически был BL)

#define SERVO_BL_FRONT_OFFSET -74   // CH0 (физически был FR)
#define SERVO_BL_REAR_OFFSET  -147  // CH1 (физически был FR)
#define SERVO_BR_FRONT_OFFSET -108  // CH11 (физически был FL)
#define SERVO_BR_REAR_OFFSET  -23   // CH10 (физически был FL)

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

