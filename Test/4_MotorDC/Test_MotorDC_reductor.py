from gpiozero.pins.lgpio import LGPIOFactory
from gpiozero import PWMOutputDevice
import time

# Establecer el backend lgpio (más preciso para RPi5)
factory = LGPIOFactory()

# Pines de control del motor (puente H)
IN1 = PWMOutputDevice(5, frequency=1000, pin_factory=factory)  # GPIO5
IN2 = PWMOutputDevice(6, frequency=1000, pin_factory=factory)  # GPIO6

duracion = 1  # segundos
PWM = 0.4  # Convertido a valor entre 0 y 1

def motor_adelante():
    IN1.value = PWM  # velocidad 60%
    IN2.value = 0

def motor_atras():
    IN1.value = 0
    IN2.value = PWM  # velocidad 60%

def motor_parar():
    IN1.value = 0
    IN2.value = 0

try:
    print("Motor adelante (60%)")
    motor_adelante()
    time.sleep(duracion)
    motor_atras()
    time.sleep(duracion)
    
    # print("Motor atrás (60%)")
    # motor_atras()
    # time.sleep(duracion)

    print("Motor detenido")
    motor_parar()
except KeyboardInterrupt:
    detener_todo()

finally:
    IN1.close()
    IN2.close()



