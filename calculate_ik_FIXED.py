#!/usr/bin/env python3
"""
Проверка ИСПРАВЛЕННОЙ обратной кинематики для X=0, Y=115
После обнаружения ошибки в формуле для правой ноги!

ИСПРАВЛЕНИЕ: alpha1 должно быть (bRight + sqrt(...)) а не (aRight + sqrt(...))
"""

import math

# Геометрия 5-звенного механизма
L1 = 60  # Длина верхнего заднего звена
L2 = 100 # Длина среднего заднего звена
L3 = 100 # Длина среднего переднего звена
L4 = 60  # Длина верхнего переднего звена
L5 = 40  # Смещение L5

# Калибровочные офсеты (из quadrupedal_data.h после ремаппинга)
SERVO_FL_FRONT_OFFSET = 40
SERVO_FL_REAR_OFFSET = 27
SERVO_FR_FRONT_OFFSET = -45
SERVO_FR_REAR_OFFSET = -16

def inverse_kinematics_left(X, Y):
    """Обратная кинематика для ЛЕВОЙ ноги"""
    a = 2 * X * L1
    b = 2 * Y * L1
    c = X*X + Y*Y + L1*L1 - L2*L2
    d = 2 * L4 * (X - L5)
    e = 2 * L4 * Y
    f = (X - L5)*(X - L5) + L4*L4 + Y*Y - L3*L3
    
    # Alpha
    alpha1 = 2 * math.atan((b + math.sqrt(a*a + b*b - c*c)) / (a + c))
    alpha2 = 2 * math.atan((b - math.sqrt(a*a + b*b - c*c)) / (a + c))
    
    # Beta
    beta1 = 2 * math.atan((e + math.sqrt(d*d + e*e - f*f)) / (d + f))
    beta2 = 2 * math.atan((e - math.sqrt(d*d + e*e - f*f)) / (d + f))
    
    # Нормализация
    if alpha1 < 0:
        alpha1 += 2 * math.pi
    if alpha2 < 0:
        alpha2 += 2 * math.pi
    
    # Выбор решения
    if alpha1 >= math.pi / 4:
        alpha = alpha1
    else:
        alpha = alpha2
    
    if beta1 >= 0 and beta1 <= math.pi / 4:
        beta = beta1
    else:
        beta = beta2
    
    # Градусы
    alpha_deg = int((alpha / (2 * math.pi)) * 360)
    beta_deg = int((beta / (2 * math.pi)) * 360)
    
    return alpha_deg, beta_deg

def inverse_kinematics_right(X, Y):
    """Обратная кинематика для ПРАВОЙ ноги - ИСПРАВЛЕННАЯ ВЕРСИЯ!"""
    a = 2 * X * L1
    b = 2 * Y * L1
    c = X*X + Y*Y + L1*L1 - L2*L2
    d = 2 * L4 * (X - L5)
    e = 2 * L4 * Y
    f = (X - L5)*(X - L5) + L4*L4 + Y*Y - L3*L3
    
    # Alpha - ИСПРАВЛЕНО: используем bRight, а не aRight!
    alpha1 = 2 * math.atan((b + math.sqrt(a*a + b*b - c*c)) / (a + c))
    alpha2 = 2 * math.atan((b - math.sqrt(a*a + b*b - c*c)) / (a + c))
    
    # Beta
    beta1 = 2 * math.atan((e + math.sqrt(d*d + e*e - f*f)) / (d + f))
    beta2 = 2 * math.atan((e - math.sqrt(d*d + e*e - f*f)) / (d + f))
    
    # Нормализация
    if alpha1 < 0:
        alpha1 += 2 * math.pi
    if alpha2 < 0:
        alpha2 += 2 * math.pi
    
    # Выбор решения
    if alpha1 >= math.pi / 4:
        alpha = alpha1
    else:
        alpha = alpha2
    
    if beta1 >= 0 and beta1 <= math.pi / 4:
        beta = beta1
    else:
        beta = beta2
    
    # Градусы
    alpha_deg = int((alpha / (2 * math.pi)) * 360)
    beta_deg = int((beta / (2 * math.pi)) * 360)
    
    return alpha_deg, beta_deg

# Проверяем для X=0, Y=115
X = 0
Y = 115

print("=" * 60)
print("ПРОВЕРКА ИСПРАВЛЕННОЙ ОБРАТНОЙ КИНЕМАТИКИ")
print("=" * 60)
print(f"Координаты: X={X}, Y={Y}")
print(f"Геометрия: L1={L1}, L2={L2}, L3={L3}, L4={L4}, L5={L5}")
print()

# Левая нога
alpha_left, beta_left = inverse_kinematics_left(X, Y)
servo_left_front = 90 + beta_left
servo_left_rear = 90 + alpha_left

print("ЛЕВАЯ НОГА (FL):")
print(f"  Alpha: {alpha_left}° -> Rear servo:  {servo_left_rear}°")
print(f"  Beta:  {beta_left}° -> Front servo: {servo_left_front}°")
print(f"  С офсетами:")
print(f"    FL_FRONT: {servo_left_front} + {SERVO_FL_FRONT_OFFSET} = {servo_left_front + SERVO_FL_FRONT_OFFSET}°")
print(f"    FL_REAR:  {servo_left_rear} + {SERVO_FL_REAR_OFFSET} = {servo_left_rear + SERVO_FL_REAR_OFFSET}°")
print()

# Правая нога
alpha_right, beta_right = inverse_kinematics_right(X, Y)
servo_right_front = 270 - beta_right
servo_right_rear = 270 - alpha_right

print("ПРАВАЯ НОГА (FR):")
print(f"  Alpha: {alpha_right}° -> Rear servo:  {servo_right_rear}°")
print(f"  Beta:  {beta_right}° -> Front servo: {servo_right_front}°")
print(f"  С офсетами:")
print(f"    FR_FRONT: {servo_right_front} + {SERVO_FR_FRONT_OFFSET} = {servo_right_front + SERVO_FR_FRONT_OFFSET}°")
print(f"    FR_REAR:  {servo_right_rear} + {SERVO_FR_REAR_OFFSET} = {servo_right_rear + SERVO_FR_REAR_OFFSET}°")
print()

# Проверка
print("=" * 60)
print("АНАЛИЗ:")
print("=" * 60)
if abs(servo_left_front - 90) < 10 and abs(servo_left_rear - 90) < 10:
    print("✅ Левая нога: углы близки к 90° - геометрия ПРАВИЛЬНАЯ!")
else:
    print(f"⚠️  Левая нога: углы {servo_left_front}°/{servo_left_rear}° отличаются от 90°")

if abs(servo_right_front - 90) < 10 and abs(servo_right_rear - 90) < 10:
    print("✅ Правая нога: углы близки к 90° - геометрия ПРАВИЛЬНАЯ!")
else:
    print(f"⚠️  Правая нога: углы {servo_right_front}°/{servo_right_rear}° отличаются от 90°")
