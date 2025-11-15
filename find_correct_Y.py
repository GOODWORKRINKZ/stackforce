"""
Обратный расчет: какой Y дает углы ~90° (вертикальные ноги)?
"""
import math

L1 = 60
L2 = 100
L3 = 100
L4 = 60
L5 = 40

X = 0  # Ноги вертикально

# Целевые углы серво (без офсетов)
# Для левой ноги: 90 + angle
# Для правой ноги: 270 - angle
# Если серво на 90°, то angle=0

# Целевые углы в радианах (alpha и beta должны быть ~0)
target_alpha_deg = 0  # Для FL_REAR = 90 + 0 = 90°
target_beta_deg = 0   # Для FL_FRONT = 90 + 0 = 90°

target_alpha = (target_alpha_deg / 360) * 2 * math.pi
target_beta = (target_beta_deg / 360) * 2 * math.pi

print("="*60)
print("ПОИСК Y ДЛЯ ВЕРТИКАЛЬНЫХ НОГ (углы серво = 90°)")
print("="*60)
print(f"\nЦелевые углы: alpha={target_alpha_deg}°, beta={target_beta_deg}°")

# Перебором ищем Y
for Y in range(50, 200, 1):
    aLeft = 2 * X * L1
    bLeft = 2 * Y * L1
    cLeft = X*X + Y*Y + L1*L1 - L2*L2
    dLeft = 2 * L4 * (X - L5)
    eLeft = 2 * L4 * Y
    fLeft = ((X - L5) * (X - L5) + L4*L4 + Y*Y - L3*L3)
    
    discriminant_alpha = (aLeft*aLeft) + (bLeft*bLeft) - (cLeft*cLeft)
    discriminant_beta = (dLeft*dLeft) + (eLeft*eLeft) - (fLeft*fLeft)
    
    if discriminant_alpha >= 0 and discriminant_beta >= 0:
        # Избегаем деления на ноль
        if abs(aLeft + cLeft) < 0.001:
            continue
        if abs(dLeft + fLeft) < 0.001:
            continue
            
        alpha1 = 2 * math.atan((bLeft + math.sqrt(discriminant_alpha)) / (aLeft + cLeft))
        alpha2 = 2 * math.atan((bLeft - math.sqrt(discriminant_alpha)) / (aLeft + cLeft))
        
        alpha1 = alpha1 if alpha1 >= 0 else alpha1 + 2*math.pi
        alpha2 = alpha2 if alpha2 >= 0 else alpha2 + 2*math.pi
        
        if alpha1 >= math.pi/4:
            alphaLeft = alpha1
        else:
            alphaLeft = alpha2
        
        beta1 = 2 * math.atan((eLeft + math.sqrt(discriminant_beta)) / (dLeft + fLeft))
        beta2 = 2 * math.atan((eLeft - math.sqrt(discriminant_beta)) / (dLeft + fLeft))
        
        if beta1 >= 0 and beta1 <= math.pi/4:
            betaLeft = beta1
        else:
            betaLeft = beta2
        
        alphaLeftDeg = (alphaLeft / (2*math.pi)) * 360
        betaLeftDeg = (betaLeft / (2*math.pi)) * 360
        
        servoLeftFront = 90 + int(betaLeftDeg)
        servoLeftRear = 90 + int(alphaLeftDeg)
        
        # Ищем Y где серво близки к 90°
        if abs(servoLeftFront - 90) < 5 and abs(servoLeftRear - 90) < 5:
            print(f"\n✓ Y={Y}mm:")
            print(f"  Alpha: {alphaLeftDeg:.2f}° → FL_REAR = {servoLeftRear}°")
            print(f"  Beta:  {betaLeftDeg:.2f}° → FL_FRONT = {servoLeftFront}°")
            print(f"  Отклонение от 90°: FRONT={servoLeftFront-90}°, REAR={servoLeftRear-90}°")

print("\n" + "="*60)
print("РЕКОМЕНДАЦИЯ:")
print("="*60)
print("\nИспользуй найденное значение Y как начальное в setup() и loop()!")
print("="*60)
