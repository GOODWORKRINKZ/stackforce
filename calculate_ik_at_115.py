"""
Расчет углов обратной кинематики при Y=115, X=0
Проверяем что должно получиться для вертикальных ног
"""
import math

# Геометрия (из quadrupedal_data.h)
L1 = 60   # Верхнее плечо
L2 = 100  # Верхнее бедро
L3 = 100  # Нижнее бедро
L4 = 60   # Нижнее плечо
L5 = 40   # Смещение по X

# Целевая позиция (вертикальные ноги при калибровке)
X = 0
Y = 115

print("="*60)
print(f"РАСЧЕТ УГЛОВ ДЛЯ X={X}, Y={Y}")
print("="*60)

# Левая нога
aLeft = 2 * X * L1
bLeft = 2 * Y * L1
cLeft = X*X + Y*Y + L1*L1 - L2*L2
dLeft = 2 * L4 * (X - L5)
eLeft = 2 * L4 * Y
fLeft = ((X - L5) * (X - L5) + L4*L4 + Y*Y - L3*L3)

print(f"\nЛевая нога:")
print(f"  a={aLeft:.2f}, b={bLeft:.2f}, c={cLeft:.2f}")
print(f"  d={dLeft:.2f}, e={eLeft:.2f}, f={fLeft:.2f}")

# Alpha (верхнее/заднее звено)
discriminant_alpha = (aLeft*aLeft) + (bLeft*bLeft) - (cLeft*cLeft)
print(f"  Discriminant alpha: {discriminant_alpha:.2f}")

if discriminant_alpha < 0:
    print(f"  ❌ ОШИБКА: Отрицательный дискриминант для alpha!")
else:
    alpha1 = 2 * math.atan((bLeft + math.sqrt(discriminant_alpha)) / (aLeft + cLeft))
    alpha2 = 2 * math.atan((bLeft - math.sqrt(discriminant_alpha)) / (aLeft + cLeft))
    
    alpha1 = alpha1 if alpha1 >= 0 else alpha1 + 2*math.pi
    alpha2 = alpha2 if alpha2 >= 0 else alpha2 + 2*math.pi
    
    if alpha1 >= math.pi/4:
        alphaLeft = alpha1
    else:
        alphaLeft = alpha2
    
    alphaLeftDeg = (alphaLeft / (2*math.pi)) * 360
    print(f"  Alpha: {alphaLeftDeg:.2f}° (радианы: {alphaLeft:.4f})")

# Beta (нижнее/переднее звено)
discriminant_beta = (dLeft*dLeft) + (eLeft*eLeft) - (fLeft*fLeft)
print(f"  Discriminant beta: {discriminant_beta:.2f}")

if discriminant_beta < 0:
    print(f"  ❌ ОШИБКА: Отрицательный дискриминант для beta!")
else:
    beta1 = 2 * math.atan((eLeft + math.sqrt(discriminant_beta)) / (dLeft + fLeft))
    beta2 = 2 * math.atan((eLeft - math.sqrt(discriminant_beta)) / (dLeft + fLeft))
    
    if beta1 >= 0 and beta1 <= math.pi/4:
        betaLeft = beta1
    else:
        betaLeft = beta2
    
    betaLeftDeg = (betaLeft / (2*math.pi)) * 360
    print(f"  Beta: {betaLeftDeg:.2f}° (радианы: {betaLeft:.4f})")

# Углы серво
servoLeftFront = 90 + int(betaLeftDeg)
servoLeftRear = 90 + int(alphaLeftDeg)

print(f"\n  Углы серво БЕЗ офсетов:")
print(f"    FL_FRONT: {servoLeftFront}°")
print(f"    FL_REAR:  {servoLeftRear}°")

# Офсеты (после поворота на 180°)
SERVO_FL_FRONT_OFFSET = 40
SERVO_FL_REAR_OFFSET = 27

print(f"\n  Офсеты:")
print(f"    FL_FRONT_OFFSET: {SERVO_FL_FRONT_OFFSET}°")
print(f"    FL_REAR_OFFSET:  {SERVO_FL_REAR_OFFSET}°")

print(f"\n  Углы серво С офсетами:")
print(f"    FL_FRONT: {servoLeftFront + SERVO_FL_FRONT_OFFSET}°")
print(f"    FL_REAR:  {servoLeftRear + SERVO_FL_REAR_OFFSET}°")

# Правая нога
aRight = 2 * X * L1
bRight = 2 * Y * L1
cRight = X*X + Y*Y + L1*L1 - L2*L2
dRight = 2 * L4 * (X - L5)
eRight = 2 * L4 * Y
fRight = ((X - L5) * (X - L5) + L4*L4 + Y*Y - L3*L3)

print(f"\n\nПравая нога:")
print(f"  a={aRight:.2f}, b={bRight:.2f}, c={cRight:.2f}")
print(f"  d={dRight:.2f}, e={eRight:.2f}, f={fRight:.2f}")

# Alpha
discriminant_alpha = (aRight*aRight) + (bRight*bRight) - (cRight*cRight)
if discriminant_alpha < 0:
    print(f"  ❌ ОШИБКА: Отрицательный дискриминант для alpha!")
else:
    alpha1 = 2 * math.atan((aRight + math.sqrt(discriminant_alpha)) / (aRight + cRight))
    alpha2 = 2 * math.atan((bRight - math.sqrt(discriminant_alpha)) / (aRight + cRight))
    
    alpha1 = alpha1 if alpha1 >= 0 else alpha1 + 2*math.pi
    alpha2 = alpha2 if alpha2 >= 0 else alpha2 + 2*math.pi
    
    if alpha1 >= math.pi/4:
        alphaRight = alpha1
    else:
        alphaRight = alpha2
    
    alphaRightDeg = (alphaRight / (2*math.pi)) * 360
    print(f"  Alpha: {alphaRightDeg:.2f}° (радианы: {alphaRight:.4f})")

# Beta
discriminant_beta = (dRight*dRight) + (eRight*eRight) - (fRight*fRight)
if discriminant_beta < 0:
    print(f"  ❌ ОШИБКА: Отрицательный дискриминант для beta!")
else:
    beta1 = 2 * math.atan((eRight + math.sqrt(discriminant_beta)) / (dRight + fRight))
    beta2 = 2 * math.atan((eRight - math.sqrt(discriminant_beta)) / (dRight + fRight))
    
    if beta1 >= 0 and beta1 <= math.pi/4:
        betaRight = beta1
    else:
        betaRight = beta2
    
    betaRightDeg = (betaRight / (2*math.pi)) * 360
    print(f"  Beta: {betaRightDeg:.2f}° (радианы: {betaRight:.4f})")

# Углы серво (правая нога инвертирована: 270 - угол)
servoRightFront = 270 - int(betaRightDeg)
servoRightRear = 270 - int(alphaRightDeg)

print(f"\n  Углы серво БЕЗ офсетов:")
print(f"    FR_FRONT: {servoRightFront}°")
print(f"    FR_REAR:  {servoRightRear}°")

# Офсеты
SERVO_FR_FRONT_OFFSET = -45
SERVO_FR_REAR_OFFSET = -16

print(f"\n  Офсеты:")
print(f"    FR_FRONT_OFFSET: {SERVO_FR_FRONT_OFFSET}°")
print(f"    FR_REAR_OFFSET:  {SERVO_FR_REAR_OFFSET}°")

print(f"\n  Углы серво С офсетами:")
print(f"    FR_FRONT: {servoRightFront + SERVO_FR_FRONT_OFFSET}°")
print(f"    FR_REAR:  {servoRightRear + SERVO_FR_REAR_OFFSET}°")

print("\n" + "="*60)
print("ИТОГ:")
print("="*60)
print(f"\nЕсли при калибровке сервоприводы стояли на 90° и ноги были вертикальны,")
print(f"то обратная кинематика при Y=115 должна выдать:")
print(f"  Левая нога:  FRONT={servoLeftFront}°, REAR={servoLeftRear}°")
print(f"  Правая нога: FRONT={servoRightFront}°, REAR={servoRightRear}°")
print(f"\nС офсетами (для компенсации механики):")
print(f"  FL: FRONT={servoLeftFront + SERVO_FL_FRONT_OFFSET}°, REAR={servoLeftRear + SERVO_FL_REAR_OFFSET}°")
print(f"  FR: FRONT={servoRightFront + SERVO_FR_FRONT_OFFSET}°, REAR={servoRightRear + SERVO_FR_REAR_OFFSET}°")
print(f"\n⚠️  Если углы сильно отличаются от 90°, значит проблема в геометрии!")
print("="*60)
