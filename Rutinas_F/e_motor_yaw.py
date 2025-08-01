# e_motor_yaw.py
import threading
import time
from gpiozero import Motor
from signal import pause
import pigpio

# Pines encoder (ejemplo GPIO 23 y 24)
A_PIN = 23
B_PIN = 24
motor = Motor(forward=27, backward=22)  # Pines IN1, IN2

# Posición absoluta del encoder
encoder_position = 0
pi = pigpio.pi()

def encoder_callback_A(gpio, level, tick):
    global encoder_position
    b = pi.read(B_PIN)
    encoder_position += 1 if b == 0 else -1

def iniciar_encoder():
    pi.set_mode(A_PIN, pigpio.INPUT)
    pi.set_mode(B_PIN, pigpio.INPUT)
    pi.callback(A_PIN, pigpio.RISING_EDGE, encoder_callback_A)

def ajustar_yaw(error_x):
    global encoder_position

    pasos_objetivo = int(error_x * 0.5)  # escalado al número de pasos
    pasos_iniciales = encoder_position
    limite_pasos = pasos_iniciales + pasos_objetivo

    if pasos_objetivo > 0:
        motor.forward()
        while encoder_position < limite_pasos:
            time.sleep(0.001)
    else:
        motor.backward()
        while encoder_position > limite_pasos:
            time.sleep(0.001)
    motor.stop()
