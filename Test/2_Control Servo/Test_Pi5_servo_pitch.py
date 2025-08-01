import lgpio
import time

SERVO_GPIO = 19
h = lgpio.gpiochip_open(0)

# Estado actual del servo en µs
pulso_actual = 1200  # centro por defecto (90° aprox.)

# Envía un pulso manual de servo
def send_servo_pulse(us):
    lgpio.gpio_write(h, SERVO_GPIO, 1)
    time.sleep(us / 1_000_000)  # Duración del pulso en segundos
    lgpio.gpio_write(h, SERVO_GPIO, 0)
    time.sleep((20000 - us) / 1_000_000)  # Completa los 20 ms del ciclo

# Convierte ángulo a microsegundos (servo MG996R)
def angulo_a_us(angulo):
    # 0° = 500µs, 180° = 2500µs → mapa lineal
    return int(500 + (angulo / 180.0) * 2000)

# Interpola entre posición actual y nueva con pasos suaves
def mover_servo(angulo_destino):
    global pulso_actual

    us_destino = angulo_a_us(angulo_destino)
    paso = 10 if us_destino > pulso_actual else -10

    for us in range(pulso_actual, us_destino + paso, paso):
        for _ in range(1):  # Puedes subir a 2–3 si necesitas más persistencia
            send_servo_pulse(us)
        time.sleep(0.015)  # Delay entre microajustes
        pulso_actual = us

try:
    while True:
        ang = int(input("Ingresa ángulo (50–120): "))
        if 0 <= ang <= 180:
            mover_servo(ang)
        else:
            print("Ángulo fuera de rango.")
except KeyboardInterrupt:
    print("\nSaliendo...")
finally:
    lgpio.gpiochip_close(h)
    print("GPIO liberado.")
