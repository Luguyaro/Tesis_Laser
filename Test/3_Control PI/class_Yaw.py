
from gpiozero import DigitalInputDevice, PWMOutputDevice
from time import sleep

class Yaw:
    def __init__(self, pin_a=20, pin_b=13, pwm_a=27, pwm_b=22):
        self.encoder = DigitalInputDevice(pin_a, pull_up=True)
        self.encoder.when_activated = self._incrementar_paso

        self.motor_ena = PWMOutputDevice(pwm_a)
        self.motor_enb = PWMOutputDevice(pwm_b)

        self.step_count = 0
        self.PASOS_POR_GRADO = 2.32
        self.PASOS_POR_PIXEL = 0.36  # Valor calibrable
        self.posicion_actual = 0  # Acumulado en grados

    def _incrementar_paso(self):
        self.step_count += 1

    def set_pasos_por_grado(self, grados):
        if grados <= 60:
            self.PASOS_POR_GRADO = 2.32
        elif grados < 120:
            self.PASOS_POR_GRADO = 2.4
        else:
            self.PASOS_POR_GRADO = 2.5

    def pixeles_a_pasos(self,pixeles):
        return pixeles *self.PASOS_POR_PIXEL

    def _mover_motor(self, sentido, pwm):
        self.motor_ena.value = pwm if sentido > 0 else 0
        self.motor_enb.value = 0 if sentido > 0 else pwm

    def detener_motor(self):
        self.motor_ena.value = 0
        self.motor_enb.value = 0

    def mover(self, grados):
        if grados == 0:
            print("⏸️ Ángulo 0, sin movimiento.")
            return

        grados = float(grados)
        sentido = 1 if grados > 0 else -1
        grados = abs(grados)

        nueva_posicion = self.posicion_actual + (grados * sentido)
        if not -180 <= nueva_posicion <= 180:
            print(f"❌ Movimiento cancelado. Límite excedido: {nueva_posicion:.1f}° (actual: {self.posicion_actual:.1f}°)")
            return

        self.set_pasos_por_grado(grados)
        total_pasos = int(grados * self.PASOS_POR_GRADO)

        self.step_count = 0
        self._mover_motor(sentido, 0.5)

        try:
            while self.step_count < total_pasos:
                if total_pasos - self.step_count < 20:
                    self._mover_motor(sentido, 0.3)
                sleep(0.001)
        finally:
            self.detener_motor()
            self.posicion_actual += grados * sentido
            print(f"✅ Movimiento de {grados * sentido:+.0f}° completado. Posición actual: {self.posicion_actual:+.1f}°")

    def liberar_gpio(self):
        self.motor_ena.close()
        self.motor_enb.close()
        self.encoder.close()
        print("🧹 GPIO liberado.")

    def menu(self):
        while True:
            try:
                entrada = input("\n🎛️ Ingrese ángulo (ej. 90, -45) o 'q' para salir: ")
                if entrada.lower() == 'q':
                    break
                self.mover(float(entrada))
            except ValueError:
                print("❌ Entrada inválida.")
if __name__ == "__main__":
    yaw = Yaw()
    try:
        yaw.menu()
    finally:
        yaw.liberar_gpio()