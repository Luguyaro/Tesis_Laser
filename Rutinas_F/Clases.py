from gpiozero import DigitalInputDevice, PWMOutputDevice
from time import sleep
import lgpio
import time
from gpiozero.pins.lgpio import LGPIOFactory
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
        
#-------------------------------------------------------------------
#-------------------------------------------------------------------
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
    def esta_en_rango(self, grados_destino):
        nueva_posicion = self.posicion_actual + grados_destino
        return -90 <= nueva_posicion <= 90

    def mover(self, grados):
        if grados == 0:
            print(" angulo 0, sin movimiento.")
            return

        grados = float(grados)
        sentido = 1 if grados > 0 else -1
        grados = abs(grados)

        nueva_posicion = self.posicion_actual + (grados * sentido)
        if not -90 <= nueva_posicion <= 90:
            print(f"Movimiento cancelado. Límite ±90° excedido: {nueva_posicion:.1f}° (actual: {self.posicion_actual:.1f}°)")
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
            print(f"✅ Movimiento de {grados * sentido:+.0f}° completado. Posicion actual: {self.posicion_actual:+.1f}°")

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
#-------------------------------------------------------------------
#-------------------------------------------------------------------


class MotorDC:
    def __init__(self, pin_in1=5, pin_in2=6, frecuencia=1000, pwm=0.8):
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
#-------------------------------------------------------------------
#-------------------------------------------------------------------
    
import tflite_runtime.interpreter as tflite
import numpy as np
from PIL import Image
import cv2

class Camara:
    def __init__(self, model_path, input_size=640, conf_threshold=1600e-10, frame_width=640, frame_height=480):
        self.input_size = input_size
        self.conf_threshold = conf_threshold
        self.frame_width = frame_width
        self.frame_height = frame_height
        self.cx_frame = frame_width // 2
        self.cy_frame = frame_height // 2

        # Inicializar modelo TFLite
        self.interpreter = tflite.Interpreter(model_path=model_path)
        self.interpreter.allocate_tensors()
        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()

        # Inicializar cámara
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)

        if not self.cap.isOpened():
            raise RuntimeError("❌ No se pudo abrir la cámara.")

    def obtener_frame(self):
        for _ in range(5):  # Reintenta hasta 5 veces
            ret, frame = self.cap.read()
            if ret:
                frame = cv2.rotate(frame, cv2.ROTATE_180)
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                gray_bgr = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
                return gray, gray_bgr
            time.sleep(0.01)
        print("⚠️ No se pudo capturar frame válido.")
        return None, None


    def detectar_objeto(self, gray):
        pil_image = Image.fromarray(gray)
        original_width, original_height = pil_image.size

        image_resized = pil_image.resize((self.input_size, self.input_size))
        input_data = np.expand_dims(np.array(image_resized, dtype=np.float32), axis=0) / 255.0
        input_data = np.expand_dims(input_data, axis=-1)

        if self.input_details[0]['shape'][-1] == 3:
            input_data = np.repeat(input_data, 3, axis=-1)

        self.interpreter.set_tensor(self.input_details[0]['index'], input_data)
        self.interpreter.invoke()
        output_data = self.interpreter.get_tensor(self.output_details[0]['index'])
        output_data = np.squeeze(output_data).transpose()

        boxes = []
        for det in output_data:
            x, y, w, h = det[0:4]
            conf = det[4]
            class_probs = det[5:]
            class_id = np.argmax(class_probs)
            class_score = class_probs[class_id]
            total_conf = conf * class_score

            if total_conf > self.conf_threshold:
                cx, cy = x * self.input_size, y * self.input_size
                bw, bh = w * self.input_size, h * self.input_size
                x1 = int((cx - bw / 2) * (original_width / self.input_size))
                y1 = int((cy - bh / 2) * (original_height / self.input_size))
                x2 = int((cx + bw / 2) * (original_width / self.input_size))
                y2 = int((cy + bh / 2) * (original_height / self.input_size))
                cx_rescaled = int((x1 + x2) / 2)
                cy_rescaled = int((y1 + y2) / 2)
                boxes.append((x1, y1, x2, y2, class_id, total_conf, cx_rescaled, cy_rescaled))
        return boxes

    def liberar(self):
        self.cap.release()
        cv2.destroyAllWindows()

class ZonasExploradas:
    def __init__(self, tolerancia=3):
        self.rangos = []
        self.tolerancia = tolerancia

    def registrar(self, angulo):
        nuevo_rango = (angulo - self.tolerancia, angulo + self.tolerancia)
        self.rangos.append(nuevo_rango)
        self._fusionar_rangos()

    def _fusionar_rangos(self):
        if not self.rangos:
            return
        self.rangos.sort()
        fusionados = [self.rangos[0]]
        for inicio, fin in self.rangos[1:]:
            ultimo_inicio, ultimo_fin = fusionados[-1]
            if inicio <= ultimo_fin:
                fusionados[-1] = (ultimo_inicio, max(ultimo_fin, fin))
            else:
                fusionados.append((inicio, fin))
        self.rangos = fusionados

    def ya_explorado(self, angulo):
        for inicio, fin in self.rangos:
            if inicio <= angulo <= fin:
                return True
        return False

