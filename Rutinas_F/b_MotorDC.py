# test/d_motor_dc.py
from gpiozero.pins.lgpio import LGPIOFactory
from gpiozero import PWMOutputDevice
import time

factory = LGPIOFactory()
IN1 = PWMOutputDevice(5, frequency=1000, pin_factory=factory)
IN2 = PWMOutputDevice(6, frequency=1000, pin_factory=factory)
PWM = 60 / 100  # Convertido a valor entre 0 y 1
def mover_motor_adelante(tiempo):
    print("🚗 Moviendo motor hacia adelante...")
    IN1.value = PWM
    IN2.value = 0
    time.sleep(tiempo)
    detener_motor()
def mover_motor_atras(tiempo):
    IN1.value = 0
    IN2.value = PWM  # velocidad 60%
    time.sleep(tiempo)
    detener_motor()
def detener_motor():
    IN1.value = 0
    IN2.value = 0
    print("🛑 Motor detenido")

def liberar_motor():
    IN1.close()
    IN2.close()

