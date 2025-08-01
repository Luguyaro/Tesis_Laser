# d_servo_pitch.py
from gpiozero import Servo
from time import sleep

servo = Servo(17)

def ajustar_pitch(error_y):
    # Normalizar error (-1 a 1)
    valor = max(min(error_y / 100.0, 1), -1)
    servo.value = valor
    sleep(0.2)
