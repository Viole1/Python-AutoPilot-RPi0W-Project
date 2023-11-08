import cv2
import numpy as np
import RPi.GPIO as GPIO
import time

# Устанавливаем режим GPIO на Raspberry Pi
GPIO.setmode(GPIO.BCM)

# Определяем номера GPIO-портов для управления моторами
left_motor_forward = 23
left_motor_backward = 24
right_motor_forward = 22
right_motor_backward = 27
ena = 25  # ENA - включение левого мотора
enb = 17  # ENB - включение правого мотора

# Настраиваем порты как выходы
GPIO.setup(left_motor_forward, GPIO.OUT)
GPIO.setup(left_motor_backward, GPIO.OUT)
GPIO.setup(right_motor_forward, GPIO.OUT)
GPIO.setup(right_motor_backward, GPIO.OUT)
GPIO.setup(ena, GPIO.OUT)
GPIO.setup(enb, GPIO.OUT)

# Включаем моторы
GPIO.output(ena, GPIO.HIGH)
GPIO.output(enb, GPIO.HIGH)

# Функция для движения вперед
def move_forward():
    GPIO.output(left_motor_forward, GPIO.HIGH)
    GPIO.output(left_motor_backward, GPIO.LOW)
    GPIO.output(right_motor_forward, GPIO.HIGH)
    GPIO.output(right_motor_backward, GPIO.LOW)

# Функция для движения влево
def move_left():
    GPIO.output(left_motor_forward, GPIO.LOW)
    GPIO.output(left_motor_backward, GPIO.HIGH)
    GPIO.output(right_motor_forward, GPIO.HIGH)
    GPIO.output(right_motor_backward, GPIO.LOW)

# Функция для движения вправо
def move_right():
    GPIO.output(left_motor_forward, GPIO.HIGH)
    GPIO.output(left_motor_backward, GPIO.LOW)
    GPIO.output(right_motor_forward, GPIO.LOW)
    GPIO.output(right_motor_backward, GPIO.HIGH)

# Останавливаем движение
def stop():
    GPIO.output(left_motor_forward, GPIO.LOW)
    GPIO.output(left_motor_backward, GPIO.LOW)
    GPIO.output(right_motor_forward, GPIO.LOW)
    GPIO.output(right_motor_backward, GPIO.LOW)

# Захватываем видео с камеры (0 - индекс камеры)
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()

    if not ret:
        break

    # Конвертируем BGR изображение в HSV цветовое пространство
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Определение диапазона красного цвета в HSV
    lower_red = np.array([0, 100, 100])
    upper_red = np.array([10, 255, 255])

    # Создаем бинарное изображение, на котором красный цвет будет белым, а все остальное - черным
    mask = cv2.inRange(hsv, lower_red, upper_red)

    # Находим контуры объектов на бинарном изображении
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Определение центральной оси
    height, width, _ = frame.shape
    center_x = width // 2

    # Инициализируем переменные для определения положения красного цвета и движения
    red_position = None
    movement = "stop"

    # Отображаем контуры на исходном изображении и определяем положение красного цвета
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > 100:  # Ограничение по минимальной площади
            cv2.drawContours(frame, [contour], -1, (0, 255, 0), 2)
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                # вот тут настраиваем форвард и его границы меняем, где -50 и где +50
                if center_x - 75 <= cX <= center_x + 75:
                    red_position = "forward"
                    movement = "forward"  # Движение вперед
                elif cX < center_x:
                    red_position = "left"
                    movement = "left"  # Включаем левое движение
                else:
                    red_position = "right"
                    movement = "right"  # Включаем правое движение

    # Выводим положение красного цвета
    if red_position is not None:
        cv2.putText(frame, red_position, (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

    # Выполняем движение в соответствии с определенным направлением
    if movement == "forward":
        print("forward")
        move_forward()
    elif movement == "left":
        print("left")
        move_left()
    elif movement == "right":
        print("right")
        move_right()
    else:
        stop()
        print("stop")

    # Отображаем исходное изображение и маску
    cv2.imshow('Original', frame)
    cv2.imshow('Mask', mask)

    if cv2.waitKey(1) & 0xFF == 27:
        break

# Очистка GPIO-портов и завершение программы
GPIO.cleanup()
cap.release()
cv2.destroyAllWindows()

# git clone https://github.com/Viole1/Python-AutoPilot-RPi0W-Project