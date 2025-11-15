"""
Расчет офсетов для Y=70 (левая) и Y=130 (правая) с учетом stab_roll=-30
"""

import math

L1 = 60
L2 = 100
L3 = 100
L4 = 60
L5 = 40

print("=" * 60)
print("РАСЧЕТ ОФСЕТОВ С УЧЕТОМ stab_roll=-30")
print("=" * 60)

# Левая нога: Y=70
XLeft = 0
YLeft = 70

print("\n--- ЛЕВАЯ НОГА (Y=70) ---")
aLeft = 2 * XLeft * L1
bLeft = 2 * YLeft * L1
cLeft = XLeft * XLeft + YLeft * YLeft + L1 * L1 - L2 * L2
dLeft = 2 * L4 * (XLeft - L5)
eLeft = 2 * L4 * YLeft
fLeft = ((XLeft - L5) * (XLeft - L5) + L4 * L4 + YLeft * YLeft - L3 * L3)

alpha1 = 2 * math.atan((bLeft + math.sqrt((aLeft * aLeft) + (bLeft * bLeft) - (cLeft * cLeft))) / (aLeft + cLeft))
alpha2 = 2 * math.atan((bLeft - math.sqrt((aLeft * aLeft) + (bLeft * bLeft) - (cLeft * cLeft))) / (aLeft + cLeft))
beta1 = 2 * math.atan((eLeft + math.sqrt((dLeft * dLeft) + eLeft * eLeft - (fLeft * fLeft))) / (dLeft + fLeft))
beta2 = 2 * math.atan((eLeft - math.sqrt((dLeft * dLeft) + eLeft * eLeft - (fLeft * fLeft))) / (dLeft + fLeft))

alpha1 = alpha1 if alpha1 >= 0 else (alpha1 + 2 * math.pi)
alpha2 = alpha2 if alpha2 >= 0 else (alpha2 + 2 * math.pi)

alphaLeft = alpha1 if alpha1 >= math.pi / 4 else alpha2
betaLeft = beta1 if (beta1 >= 0 and beta1 <= math.pi / 4) else beta2

alphaLeftToAngle = int((alphaLeft / (2 * math.pi)) * 360)
betaLeftToAngle = int((betaLeft / (2 * math.pi)) * 360)

servoFrontLeftFront = 90 + betaLeftToAngle
servoFrontLeftRear = 90 + alphaLeftToAngle

print(f"servoFrontLeftFront = 90 + {betaLeftToAngle} = {servoFrontLeftFront}°")
print(f"servoFrontLeftRear = 90 + {alphaLeftToAngle} = {servoFrontLeftRear}°")

offsetFL_FRONT = 90 - servoFrontLeftFront
offsetFL_REAR = 90 - servoFrontLeftRear
print(f"\n✓ SERVO_FL_FRONT_OFFSET = {offsetFL_FRONT}")
print(f"✓ SERVO_FL_REAR_OFFSET  = {offsetFL_REAR}")

# Правая нога: Y=130
XRight = 0
YRight = 130

print("\n--- ПРАВАЯ НОГА (Y=130) ---")
aRight = 2 * XRight * L1
bRight = 2 * YRight * L1
cRight = XRight * XRight + YRight * YRight + L1 * L1 - L2 * L2
dRight = 2 * L4 * (XRight - L5)
eRight = 2 * L4 * YRight
fRight = ((XRight - L5) * (XRight - L5) + L4 * L4 + YRight * YRight - L3 * L3)

alpha1 = 2 * math.atan((bRight + math.sqrt((aRight * aRight) + (bRight * bRight) - (cRight * cRight))) / (aRight + cRight))
alpha2 = 2 * math.atan((bRight - math.sqrt((aRight * aRight) + (bRight * bRight) - (cRight * cRight))) / (aRight + cRight))
beta1 = 2 * math.atan((eRight + math.sqrt((dRight * dRight) + eRight * eRight - (fRight * fRight))) / (dRight + fRight))
beta2 = 2 * math.atan((eRight - math.sqrt((dRight * dRight) + eRight * eRight - (fRight * fRight))) / (dRight + fRight))

alpha1 = alpha1 if alpha1 >= 0 else (alpha1 + 2 * math.pi)
alpha2 = alpha2 if alpha2 >= 0 else (alpha2 + 2 * math.pi)

alphaRight = alpha1 if alpha1 >= math.pi / 4 else alpha2
betaRight = beta1 if (beta1 >= 0 and beta1 <= math.pi / 4) else beta2

alphaRightToAngle = int((alphaRight / (2 * math.pi)) * 360)
betaRightToAngle = int((betaRight / (2 * math.pi)) * 360)

servoFrontRightFront = 270 - betaRightToAngle
servoFrontRightRear = 270 - alphaRightToAngle

print(f"servoFrontRightFront = 270 - {betaRightToAngle} = {servoFrontRightFront}°")
print(f"servoFrontRightRear = 270 - {alphaRightToAngle} = {servoFrontRightRear}°")

offsetFR_FRONT = 90 - servoFrontRightFront
offsetFR_REAR = 90 - servoFrontRightRear
print(f"\n✓ SERVO_FR_FRONT_OFFSET = {offsetFR_FRONT}")
print(f"✓ SERVO_FR_REAR_OFFSET  = {offsetFR_REAR}")

# Сравнение с реальными значениями из лога
print("\n" + "=" * 60)
print("ПРОВЕРКА С ЛОГОМ:")
print("=" * 60)
print(f"Лог: FR_FRONT=206 → должно быть {servoFrontRightFront}° (разница: {206 - servoFrontRightFront}°)")
print(f"Лог: FR_REAR=175 → должно быть {servoFrontRightRear}° (разница: {175 - servoFrontRightRear}°)")

print("\n" + "=" * 60)
print("ИТОГОВЫЕ ОФСЕТЫ:")
print("=" * 60)
print(f"#define SERVO_FL_FRONT_OFFSET {offsetFL_FRONT}")
print(f"#define SERVO_FL_REAR_OFFSET  {offsetFL_REAR}")
print(f"#define SERVO_FR_FRONT_OFFSET {offsetFR_FRONT}")
print(f"#define SERVO_FR_REAR_OFFSET  {offsetFR_REAR}")
print(f"")
print(f"#define SERVO_BL_FRONT_OFFSET {offsetFL_FRONT}  // Задние = передние")
print(f"#define SERVO_BL_REAR_OFFSET  {offsetFL_REAR}")
print(f"#define SERVO_BR_FRONT_OFFSET {offsetFR_FRONT}")
print(f"#define SERVO_BR_REAR_OFFSET  {offsetFR_REAR}")
print("=" * 60)
