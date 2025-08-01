import lgpio
import time

class Pitch:
    def __init__(self, gpio_servo=19):
        self.SERVO_GPIO = gpio_servo
        self.h = lgpio.gpiochip_open(0)
        self.pulso_actual = 1200  # microsegundos, centro inicial (≈90°)
        self.angulo_actual = 90   # En grados
        self.MIN_ANG = 50
        self.MAX_ANG = 120
        self.Kp = 0.05  # Ganancia proporcional: 0.05° por pixel

    def angulo_a_us(self, angulo):
        return int(500 + (angulo / 180.0) * 1900)  # 500–2400us

    def send_servo_pulse(self, us):
        lgpio.gpio_write(self.h, self.SERVO_GPIO, 1)
        time.sleep(us / 1_000_000)
        lgpio.gpio_write(self.h, self.SERVO_GPIO, 0)
        time.sleep((20000 - us) / 1_000_000)

    def mover_a(self, nuevo_angulo):
        nuevo_angulo = max(self.MIN_ANG, min(self.MAX_ANG, nuevo_angulo))
        us_destino = self.angulo_a_us(nuevo_angulo)

        paso = 10 if us_destino > self.pulso_actual else -10
        for us in range(self.pulso_actual, us_destino + paso, paso):
            self.send_servo_pulse(us)
            time.sleep(0.015)
        self.pulso_actual = us_destino
        self.angulo_actual = nuevo_angulo
        print(f"🎯 Servo Pitch movido a {nuevo_angulo:.1f}°")

    def mover_por_pixeles(self, error_pixeles):
        delta_ang = self.Kp * error_pixeles
        nuevo_angulo = self.angulo_actual + delta_ang
        self.mover_a(nuevo_angulo)

    def liberar_gpio(self):
        lgpio.gpiochip_close(self.h)
        print("🧹 GPIO de servo Pitch liberado.")
