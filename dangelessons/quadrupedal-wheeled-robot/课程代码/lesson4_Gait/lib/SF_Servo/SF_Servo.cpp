#include "SF_Servo.h"

SF_Servo::SF_Servo(TwoWire &i2c)
    :  _i2c(&i2c), freq(50){}

void SF_Servo::init(){
    _i2c->begin();
    reset();

    setPWMFreq(freq);

    enable();
}

void SF_Servo::setPWMFreq(float freq){
    if (freq < 1)
        freq = 1;
    if (freq > 3500)
        freq = 3500;  // 限制为3052=50MHz/(4*4096)
    
    float prescaleval = ((FREQUENCY_OSCILLATOR / (freq * 4096.0)) + 0.5) - 1;
    if (prescaleval < PCA9685_PRESCALE_MIN)
        prescaleval = PCA9685_PRESCALE_MIN;
    if (prescaleval > PCA9685_PRESCALE_MAX)
        prescaleval = PCA9685_PRESCALE_MAX;
    uint8_t prescale = (uint8_t)prescaleval;

    uint8_t oldmode = readFromPCA(PCA9685_MODE1);
    uint8_t newmode = (oldmode & ~MODE1_RESTART) | MODE1_SLEEP;  // sleep
    writeToPCA(PCA9685_MODE1, newmode);                              // go to sleep
    writeToPCA(PCA9685_PRESCALE, prescale);                          // 设置预分频器
    writeToPCA(PCA9685_MODE1, oldmode);
    delay(5);
    // 将MODE1寄存器设置为自动递增
    writeToPCA(PCA9685_MODE1, oldmode | MODE1_RESTART | MODE1_AI);
}

void SF_Servo::enable(){
    pinMode(SERVO_ENABLE_PIN, OUTPUT);
    digitalWrite(SERVO_ENABLE_PIN, 0);
}

void SF_Servo::disable(){
    pinMode(SERVO_ENABLE_PIN, OUTPUT);
    digitalWrite(SERVO_ENABLE_PIN, LOW);
}

void SF_Servo::reset(){
    writeToPCA(PCA9685_MODE1, MODE1_RESTART);
    delay(10);
}

void SF_Servo::sleep(){
    uint8_t awake = readFromPCA(PCA9685_MODE1);
    uint8_t sleep = awake | MODE1_SLEEP;  // 睡眠位调高
    writeToPCA(PCA9685_MODE1, sleep);
    delay(5);
}

void SF_Servo::wakeup(){
  uint8_t sleep = readFromPCA(PCA9685_MODE1);
  uint8_t wakeup = sleep & ~MODE1_SLEEP;  // 睡眠位调低
  writeToPCA(PCA9685_MODE1, wakeup);
}

void SF_Servo::setAngle(uint8_t num, uint16_t angle){
    if(angle < angleMin || angle > angleMax)
        return;
    uint16_t offTime = (int)(pluseMin + pluseRange * angle / angleRange);
    uint16_t off = (int)(offTime * 4096 / 20000);
    // Serial.printf("%d,%d,%d,%d,,%d,%d\n",angleRange,pluseRange,offTime,off,num,off);
    setPWM(num, 0, off);
}

void SF_Servo::setAngleRange(uint16_t min, uint16_t max){
    angleMin = (max > min) ? min : max;
    angleMax = (max > min) ? max : min;
    // angleMin = min;
    // angleMax = max;
    angleRange = angleMax - angleMin;
}

void SF_Servo::setPluseRange(uint16_t min, uint16_t max){
    pluseMin = (max > min) ? min : max;
    pluseMax = (max > min) ? max : min;
    // pluseMin = min;
    // pluseMax = max;
    pluseRange = pluseMax - pluseMin;

}

void SF_Servo::setPWM(uint8_t num, uint16_t on, uint16_t off){
    _i2c->beginTransmission(PCA9685_ADDR);
    _i2c->write(PCA9685_LED0_ON_L + 4 * num);
    _i2c->write(on);
    _i2c->write(on >> 8);
    _i2c->write(off);
    _i2c->write(off >> 8);
    _i2c->endTransmission();
}

void SF_Servo::setPin(uint8_t num, uint16_t val, bool invert){
  // 值在0~4095之间
    val = min(val, (uint16_t)4095);
    if (invert) {
        if (val == 0) {
        setPWM(num, 4096, 0);
        } 
        else if (val == 4095) {
        setPWM(num, 0, 4096);
        } else {
        setPWM(num, 0, 4095 - val);
        }
    } else {
        if (val == 4095) {

        setPWM(num, 4096, 0);
        } else if (val == 0) {

        setPWM(num, 0, 4096);
        } else {
        setPWM(num, 0, val);
        }
    }
}


void SF_Servo::writeToPCA(uint8_t addr, uint8_t data){
    _i2c->beginTransmission(PCA9685_ADDR);
    _i2c->write(addr);
    _i2c->write(data);
    _i2c->endTransmission();
}

uint8_t SF_Servo::readFromPCA(uint8_t addr){
    _i2c->beginTransmission(PCA9685_ADDR);
    _i2c->write(addr);
    _i2c->endTransmission();

    _i2c->requestFrom((uint8_t)PCA9685_ADDR, (uint8_t)1);
    return _i2c->read();
}
/**
 * @brief       设置8个舵机的角度
 * @param       1、舵机通道num1；   2、舵机角度angle1；  3、舵机角度angle1偏移offset1；
 *              4、舵机通道num2；   5、舵机角度angle2；  6、舵机角度angle2偏移offset2；
 *              7、舵机通道num3；   8、舵机角度angle3；  9、舵机角度angle3偏移offset3；
 *              10、舵机通道num4； 11、舵机角度angle4； 12、舵机角度angle4偏移offset4；
 *              13、舵机通道num5； 14、舵机角度angle5； 15、舵机角度angle5偏移offset5；
 *              16、舵机通道num6； 17、舵机角度angle6； 18、舵机角度angle6偏移offset6；
 *              19、舵机通道num7； 20、舵机角度angle7； 21、舵机角度angle7偏移offset7；
 *              22、舵机通道num8； 23、舵机角度angle8； 24、舵机角度angle8偏移offset8；
 * @retval      无
 */
void SF_Servo::setEightServoAngle(  uint8_t num1, uint16_t angle1, int16_t offset1,
                                    uint8_t num2, uint16_t angle2, int16_t offset2,
                                    uint8_t num3, uint16_t angle3, int16_t offset3,
                                    uint8_t num4, uint16_t angle4, int16_t offset4,
                                    uint8_t num5, uint16_t angle5, int16_t offset5,
                                    uint8_t num6, uint16_t angle6, int16_t offset6,
                                    uint8_t num7, uint16_t angle7, int16_t offset7,
                                    uint8_t num8, uint16_t angle8, int16_t offset8){
    setAngle(num1, angle1 + offset1);
    setAngle(num2, angle2 + offset2);
    setAngle(num3, angle3 + offset3);
    setAngle(num4, angle4 + offset4);
    setAngle(num5, angle5 + offset5);
    setAngle(num6, angle6 + offset6);
    setAngle(num7, angle7 + offset7);
    setAngle(num8, angle8 + offset8);
}
/* SF_SERVO V0.1  新增setEightServoAngle函数*/
