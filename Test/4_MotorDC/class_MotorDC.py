from gpiozero.pins.lgpio import LGPIOFactory
from gpiozero import PWMOutputDevice
import time

class MotorDC:
    def __init__(self, pin_in1=5, pin_in2=6, frecuencia=1000, pwm=1.0):
        self.factory = LGPIOFactory()
        self.IN1 = PWMOutputDevice(pin_in1, frequency=frecuencia, pin_factory=self.factory)
        self.IN2 = PWMOutputDevice(pin_in2, frequency=frecuencia, pin_factory=self.factory)
        self.pwm = pwm  # Valor entre 0 y 1

    def adelante(self, duracion):
        print(f"🟢 Motor adelante por {duracion}s")
        self.IN1.value = self.pwm
        self.IN2.value = 0
        time.sleep(duracion)
        self.parar()

    def atras(self, duracion):
        print(f"🔴 Motor atrás por {duracion}s")
        self.IN1.value = 0
        self.IN2.value = self.pwm
        time.sleep(duracion)
        self.parar()

    def parar(self):
        self.IN1.value = 0
        self.IN2.value = 0
        print("⏹️ Motor detenido")

    def cerrar(self):
        self.IN1.close()
        self.IN2.close()
        print("🔌 Pines liberados")

# Ejemplo de uso
if __name__ == "__main__":
    motor = MotorDC(pwm=0.6)  # 60% de velocidad

    try:
        motor.adelante(1)  # Adelante por 1 segundo
        motor.atras(1)     # Atrás por 1 segundo
    except KeyboardInterrupt:
        motor.parar()
    finally:
        motor.cerrar()
