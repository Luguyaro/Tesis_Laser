from threading import Thread
import time
from Clases import Pitch, Yaw, MotorDC, Camara
from Rutina import rutina

# Inicialización de clases
yaw = Yaw()
pitch = Pitch()
motor = MotorDC()
cam = Camara("/home/pi5/Downloads/Tesis_Laser/IA/Models/model_- 17 july 2025 10_04(2).tflite")

try:
    rutina(yaw, pitch, motor, cam)
except KeyboardInterrupt:
    print("Terminando ejecución")
finally:
    cam.liberar()
    yaw.liberar_gpio()
    motor.cerrar()
    pitch.cerrar()