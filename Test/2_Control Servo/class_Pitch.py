# class_Pitch.py
import lgpio
import time

class Pitch:
    def __init__(self, gpio_pin=19, angulo_inicial=100):
        self.SERVO_GPIO = gpio_pin
        self.h = lgpio.gpiochip_open(0)
        self.MIN_ANGULO = 50
        self.MAX_ANGULO = 120
        self.pulso_actual = self.angulo_a_us(self._limitar_angulo(angulo_inicial))
        self.send_servo_pulse(self.pulso_actual)

    def _limitar_angulo(self, angulo):
        """
        Limita el ángulo entre MIN_ANGULO y MAX_ANGULO
        """
        return max(self.MIN_ANGULO, min(self.MAX_ANGULO, angulo))

    def angulo_a_us(self, angulo):
        """
        Convierte un ángulo (limitado entre 50° y 120°) a microsegundos
        """
        angulo = self._limitar_angulo(angulo)
        return int(500 + (angulo / 180.0) * 2000)

    def us_a_angulo(self, us):
        """
        Convierte microsegundos a ángulo aproximado
        """
        return 180.0 * (us - 500) / 2000.0

    def send_servo_pulse(self, us):
        """
        Envía un solo pulso PWM al servo
        """
        lgpio.gpio_write(self.h, self.SERVO_GPIO, 1)
        time.sleep(us / 1_000_000)
        lgpio.gpio_write(self.h, self.SERVO_GPIO, 0)
        time.sleep((20000 - us) / 1_000_000)

    def mover_a_angulo(self, angulo_destino):
        """
        Mueve el servo a un ángulo destino, limitado entre 50° y 120°
        """
        angulo_destino = self._limitar_angulo(angulo_destino)
        us_destino = self.angulo_a_us(angulo_destino)
        paso = 10 if us_destino > self.pulso_actual else -10

        for us in range(self.pulso_actual, us_destino + paso, paso):
            self.send_servo_pulse(us)
            time.sleep(0.015)
            self.pulso_actual = us

    def mover_por_pixeles(self, error_pixeles, sensibilidad=0.78):
        """
        Ajusta el ángulo del servo basado en error en píxeles
        """
        # Aproximación: 10µs ≈ 1° → convierte a microsegundos
        delta_angulo = -error_pixeles * sensibilidad
        angulo_actual = self._limitar_angulo(self.us_a_angulo(self.pulso_actual))
        nuevo_angulo = self._limitar_angulo(angulo_actual + delta_angulo)
        self.mover_a_angulo(nuevo_angulo)

    def cerrar(self):
        """
        Libera el controlador GPIO
        """
        lgpio.gpiochip_close(self.h)
        print("GPIO liberado correctamente.")

# 
# from class_Pitch import Pitch
# 
# pitch_servo = Pitch()
# 
# try:
#     while True:
#         ang = int(input("Ingresa ángulo (50–120): "))
#         if 50 <= ang <= 120:
#             pitch_servo.mover_a_angulo(ang)
#         else:
#             print("Ángulo fuera de rango.")
# except KeyboardInterrupt:
#     print("\nSaliendo...")
# finally:
#     pitch_servo.cerrar()
