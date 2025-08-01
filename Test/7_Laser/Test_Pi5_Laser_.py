from gpiozero.pins.lgpio import LGPIOFactory
from gpiozero import PWMOutputDevice
import time

# -------------------------------
# Configuración de Servo
# -------------------------------
factory = LGPIOFactory()
servo = PWMOutputDevice(26, frequency=50, pin_factory=factory)  # 50 Hz

# Parámetros servo MG996R
PERIODO_MS = 20
MIN_PULSE_US = 600     # 0° lógico (≈30° físico)
MAX_PULSE_US = 2400    # 180° lógico (≈150° físico)
DEAD_BAND_US = 20      # Ancho de zona muerta

# Convertidor de ángulo lógico (-60 a 60) a microsegundos de pulso
def angulo_logico_a_pulso_us(angulo):
    # Mapea -60° a 600us + 1/6*(1800us)
    angulo = max(-60, min(60, angulo))
    escala = (MAX_PULSE_US - MIN_PULSE_US) / 180.0  # us/° físico
    angulo_fisico = 90 + angulo  # Desplaza el rango a 30–150
    pulso = MIN_PULSE_US + angulo_fisico * escala
    return pulso

# Convierte ancho de pulso (us) a duty cycle (0.0 - 1.0)
def pulso_a_duty_cycle(pulso_us):
    return pulso_us / (PERIODO_MS * 1000.0)

# Mueve el servo suavemente desde el ángulo actual
def mover_servo_lento(angulo_objetivo, paso=2, retardo=0.05):
    global angulo_actual
    angulo_objetivo = max(-60, min(60, angulo_objetivo))

    while abs(angulo_objetivo - angulo_actual) >= 1:
        if angulo_objetivo > angulo_actual:
            angulo_actual += paso
        else:
            angulo_actual -= paso

        # Calcular pulso y duty cycle
        pulso = angulo_logico_a_pulso_us(angulo_actual)
        duty = pulso_a_duty_cycle(pulso)
        servo.value = duty
        time.sleep(retardo)

    # Apagar PWM para evitar zumbido
    time.sleep(0.2)
    servo.value = 0

# -------------------------------
# Ejecución interactiva
# -------------------------------
angulo_actual = 0
servo.value = 0
print("Ingresa un ángulo entre -60° y 60° (Ctrl+C para salir)")

try:
    while True:
        ang = float(input("Ángulo deseado: "))
        mover_servo_lento(ang)
except KeyboardInterrupt:
    print("Saliendo...")

finally:
    servo.close()
