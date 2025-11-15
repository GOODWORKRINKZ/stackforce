"""
Расчет офсетов для сервоприводов
Вычисляет какие углы выдает обратная кинематика при Y=70, X=0
и какие офсеты нужны чтобы получить 90°
"""

import math

# Геометрия механизма (из main.cpp)
L1 = 60   # Верхнее плечо
L2 = 100  # Верхнее бедро
L3 = 100  # Нижнее бедро
L4 = 60   # Нижнее плечо
L5 = 40   # Смещение по X

# Нейтральная позиция (из main.cpp setup)
XLeft = 0
YLeft = 70
XRight = 0
YRight = 70

print("=" * 60)
print("РАСЧЕТ ОФСЕТОВ ДЛЯ Y=70mm, X=0mm")
print("=" * 60)

# === ЛЕВАЯ НОГА ===
print("\n--- ЛЕВАЯ НОГА (FL) ---")
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

# Конвертация в градусы
alphaLeftToAngle = int((alphaLeft / (2 * math.pi)) * 360)
betaLeftToAngle = int((betaLeft / (2 * math.pi)) * 360)

# Применение формул из main.cpp (строки 173-174)
servoFrontLeftFront = 90 + betaLeftToAngle
servoFrontLeftRear = 90 + alphaLeftToAngle

print(f"alphaLeft = {math.degrees(alphaLeft):.2f}° -> {alphaLeftToAngle}°")
print(f"betaLeft = {math.degrees(betaLeft):.2f}° -> {betaLeftToAngle}°")
print(f"servoFrontLeftFront = 90 + {betaLeftToAngle} = {servoFrontLeftFront}°")
print(f"servoFrontLeftRear = 90 + {alphaLeftToAngle} = {servoFrontLeftRear}°")

# Офсеты = 90 - вычисленный_угол
offsetFL_FRONT = 90 - servoFrontLeftFront
offsetFL_REAR = 90 - servoFrontLeftRear
print(f"\n✓ SERVO_FL_FRONT_OFFSET = {offsetFL_FRONT}  // {servoFrontLeftFront}° -> 90°")
print(f"✓ SERVO_FL_REAR_OFFSET  = {offsetFL_REAR}  // {servoFrontLeftRear}° -> 90°")

# === ПРАВАЯ НОГА ===
print("\n--- ПРАВАЯ НОГА (FR) ---")
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

# Конвертация в градусы
alphaRightToAngle = int((alphaRight / (2 * math.pi)) * 360)
betaRightToAngle = int((betaRight / (2 * math.pi)) * 360)

# Применение формул из main.cpp (строки 175-176)
servoFrontRightFront = 270 - betaRightToAngle
servoFrontRightRear = 270 - alphaRightToAngle

print(f"alphaRight = {math.degrees(alphaRight):.2f}° -> {alphaRightToAngle}°")
print(f"betaRight = {math.degrees(betaRight):.2f}° -> {betaRightToAngle}°")
print(f"servoFrontRightFront = 270 - {betaRightToAngle} = {servoFrontRightFront}°")
print(f"servoFrontRightRear = 270 - {alphaRightToAngle} = {servoFrontRightRear}°")

# Офсеты = 90 - вычисленный_угол
offsetFR_FRONT = 90 - servoFrontRightFront
offsetFR_REAR = 90 - servoFrontRightRear
print(f"\n✓ SERVO_FR_FRONT_OFFSET = {offsetFR_FRONT}  // {servoFrontRightFront}° -> 90°")
print(f"✓ SERVO_FR_REAR_OFFSET  = {offsetFR_REAR}  // {servoFrontRightRear}° -> 90°")

# === ИТОГОВЫЕ ОФСЕТЫ ===
print("\n" + "=" * 60)
print("ИТОГОВЫЕ ОФСЕТЫ ДЛЯ quadrupedal_data.h:")
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
