#include "kinematics.h"

uint16_t servoFrontLeftFront, servoFrontLeftRear, servoFrontRightFront, servoFrontRightRear;
uint16_t servoBackLeftFront, servoBackLeftRear, servoBackRightFront, servoBackRightRear;
void inverseKinematics(Node LeftFront, Node RightFront, Node LeftBack, Node RightBack)
{
    JointAngles jointAngles;

    float alpha1, alpha2, beta1, beta2;
    int16_t alphaLeftToAngle, betaLeftToAngle, alphaRightToAngle, betaRightToAngle;
    int16_t alphaBackRightToAngle, betaBackRightToAngle, alphaBackLefToAngle, betaBackLeftToAngle;

    LeftBack.x = -LeftBack.x;
    RightBack.x = -RightBack.x;

    float aRight = 2 * RightFront.x * L1;
    float bRight = 2 * RightFront.y * L1;
    float cRight = RightFront.x * RightFront.x + RightFront.y * RightFront.y + L1 * L1 - L2 * L2;
    float dRight = 2 * L4 * (RightFront.x - L5);
    float eRight = 2 * L4 * RightFront.y;
    float fRight = ((RightFront.x - L5) * (RightFront.x - L5) + L4 * L4 + RightFront.y * RightFront.y - L3 * L3);

    jointAngles.alphaRight = 2 * atan((bRight + sqrt((aRight * aRight) + (bRight * bRight) - (cRight * cRight))) / (aRight + cRight));
    jointAngles.betaRight = 2 * atan((eRight - sqrt((dRight * dRight) + eRight * eRight - (fRight * fRight))) / (dRight + fRight));

    alpha1 = 2 * atan((bRight + sqrt((aRight * aRight) + (bRight * bRight) - (cRight * cRight))) / (aRight + cRight));
    alpha2 = 2 * atan((bRight - sqrt((aRight * aRight) + (bRight * bRight) - (cRight * cRight))) / (aRight + cRight));
    beta1 = 2 * atan((eRight + sqrt((dRight * dRight) + eRight * eRight - (fRight * fRight))) / (dRight + fRight));
    beta2 = 2 * atan((eRight - sqrt((dRight * dRight) + eRight * eRight - (fRight * fRight))) / (dRight + fRight));

    alpha1 = (alpha1 >= 0) ? alpha1 : (alpha1 + 2 * PI);
    alpha2 = (alpha2 >= 0) ? alpha2 : (alpha2 + 2 * PI);

    if (alpha1 >= PI / 4)
    jointAngles.alphaRight = alpha1;
    else
    jointAngles.alphaRight = alpha2;
    if (beta1 >= 0 && beta1 <= PI / 4)
    jointAngles.betaRight = beta1;
    else
    jointAngles.betaRight = beta2;

    float aLeft = 2 * LeftFront.x * L1;
    float bLeft = 2 * LeftFront.y * L1;
    float cLeft = LeftFront.x * LeftFront.x + LeftFront.y * LeftFront.y + L1 * L1 - L2 * L2;
    float dLeft = 2 * L4 * (LeftFront.x - L5);
    float eLeft = 2 * L4 * LeftFront.y;
    float fLeft = ((LeftFront.x - L5) * (LeftFront.x - L5) + L4 * L4 + LeftFront.y * LeftFront.y - L3 * L3);

    alpha1 = 2 * atan((bLeft + sqrt((aLeft * aLeft) + (bLeft * bLeft) - (cLeft * cLeft))) / (aLeft + cLeft));
    alpha2 = 2 * atan((bLeft - sqrt((aLeft * aLeft) + (bLeft * bLeft) - (cLeft * cLeft))) / (aLeft + cLeft));
    beta1 = 2 * atan((eLeft + sqrt((dLeft * dLeft) + eLeft * eLeft - (fLeft * fLeft))) / (dLeft + fLeft));
    beta2 = 2 * atan((eLeft - sqrt((dLeft * dLeft) + eLeft * eLeft - (fLeft * fLeft))) / (dLeft + fLeft));

    alpha1 = (alpha1 >= 0) ? alpha1 : (alpha1 + 2 * PI);
    alpha2 = (alpha2 >= 0) ? alpha2 : (alpha2 + 2 * PI);

    if (alpha1 >= PI / 4)
    jointAngles.alphaLeft = alpha1;
    else
    jointAngles.alphaLeft = alpha2;
    if (beta1 >= 0 && beta1 <= PI / 4)
    jointAngles.betaLeft = beta1;
    else
    jointAngles.betaLeft = beta2;

    alphaLeftToAngle = (int)((jointAngles.alphaLeft / 6.28) * 360); // 弧度转角度
    betaLeftToAngle = (int)((jointAngles.betaLeft / 6.28) * 360);

    alphaRightToAngle = (int)((jointAngles.alphaRight / 6.28) * 360);
    betaRightToAngle = (int)((jointAngles.betaRight / 6.28) * 360);

    servoFrontLeftFront = 90 + betaLeftToAngle;
    servoFrontLeftRear = 90 + alphaLeftToAngle;
    servoFrontRightFront = 270 - betaRightToAngle;
    servoFrontRightRear = 270 - alphaRightToAngle;

    // 新增两条后腿的逆解计算逻辑 (以 LeftBack.x, LeftBack.y 和 RightBack.x, RightBack.y 为坐标)
    float aBackRight = 2 * LeftBack.x * L1;
    float bBackRight = 2 * LeftBack.y * L1;
    float cBackRight = LeftBack.x * LeftBack.x + LeftBack.y * LeftBack.y + L1 * L1 - L2 * L2;
    float dBackRight = 2 * L4 * (LeftBack.x - L5);
    float eBackRight = 2 * L4 * LeftBack.y;
    float fBackRight = ((LeftBack.x - L5) * (LeftBack.x - L5) + L4 * L4 + LeftBack.y * LeftBack.y - L3 * L3);

    jointAngles.alphaRight = 2 * atan((bRight + sqrt((aRight * aRight) + (bRight * bRight) - (cRight * cRight))) / (aRight + cRight));
    jointAngles.betaRight = 2 * atan((eRight - sqrt((dRight * dRight) + eRight * eRight - (fRight * fRight))) / (dRight + fRight));

    alpha1 = 2 * atan((bBackRight + sqrt((aBackRight * aBackRight) + (bBackRight * bBackRight) - (cBackRight * cBackRight))) / (aBackRight + cBackRight));
    alpha2 = 2 * atan((bBackRight - sqrt((aBackRight * aBackRight) + (bBackRight * bBackRight) - (cBackRight * cBackRight))) / (aBackRight + cBackRight));
    beta1 = 2 * atan((eBackRight + sqrt((dBackRight * dBackRight) + eBackRight * eBackRight - (fBackRight * fBackRight))) / (dBackRight + fBackRight));
    beta2 = 2 * atan((eBackRight - sqrt((dBackRight * dBackRight) + eBackRight * eBackRight - (fBackRight * fBackRight))) / (dBackRight + fBackRight));

    alpha1 = (alpha1 >= 0) ? alpha1 : (alpha1 + 2 * PI);
    alpha2 = (alpha2 >= 0) ? alpha2 : (alpha2 + 2 * PI);

    if (alpha1 >= PI / 4)
    jointAngles.alphaRight = alpha1;
    else
    jointAngles.alphaRight = alpha2;
    if (beta1 >= 0 && beta1 <= PI / 4)
    jointAngles.betaRight = beta1;
    else
    jointAngles.betaRight = beta2;

    float aBackLeft = 2 * RightBack.x * L1;
    float bBackLeft = 2 * RightBack.y * L1;
    float cBackLeft = RightBack.x * RightBack.x + RightBack.y * RightBack.y + L1 * L1 - L2 * L2;
    float dBackLeft = 2 * L4 * (RightBack.x - L5);
    float eBackLeft = 2 * L4 * RightBack.y;
    float fBackLeft = ((RightBack.x - L5) * (RightBack.x - L5) + L4 * L4 + RightBack.y * RightBack.y - L3 * L3);


    alpha1 = 2 * atan((bBackLeft + sqrt((aBackLeft * aBackLeft) + (bBackLeft * bBackLeft) - (cBackLeft * cBackLeft))) / (aBackLeft + cBackLeft));
    alpha2 = 2 * atan((bBackLeft - sqrt((aBackLeft * aBackLeft) + (bBackLeft * bBackLeft) - (cBackLeft * cBackLeft))) / (aBackLeft + cBackLeft));
    beta1 = 2 * atan((eBackLeft + sqrt((dBackLeft * dBackLeft) + eBackLeft * eBackLeft - (fBackLeft * fBackLeft))) / (dBackLeft + fBackLeft));
    beta2 = 2 * atan((eBackLeft - sqrt((dBackLeft * dBackLeft) + eBackLeft * eBackLeft - (fBackLeft * fBackLeft))) / (dBackLeft + fBackLeft));

    alpha1 = (alpha1 >= 0) ? alpha1 : (alpha1 + 2 * PI);
    alpha2 = (alpha2 >= 0) ? alpha2 : (alpha2 + 2 * PI);

    if (alpha1 >= PI / 4)
    jointAngles.alphaLeft = alpha1;
    else
    jointAngles.alphaLeft = alpha2;
    if (beta1 >= 0 && beta1 <= PI / 4)
    jointAngles.betaLeft = beta1;
    else
    jointAngles.betaLeft = beta2;

    alphaBackLefToAngle = (int)((jointAngles.alphaLeft / 6.28) * 360); // 弧度转角度
    betaBackLeftToAngle = (int)((jointAngles.betaLeft / 6.28) * 360);

    alphaBackRightToAngle = (int)((jointAngles.alphaRight / 6.28) * 360);
    betaBackRightToAngle = (int)((jointAngles.betaRight / 6.28) * 360);

    servoBackLeftFront = 90 + betaBackLeftToAngle;
    servoBackLeftRear = 90 + alphaBackLefToAngle;

    servoBackRightFront = 270 - betaBackRightToAngle;
    servoBackRightRear = 270 - alphaBackRightToAngle;
}