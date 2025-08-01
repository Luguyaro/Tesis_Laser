from gpiozero.pins.lgpio import LGPIOFactory
from gpiozero import PWMOutputDevice, DigitalInputDevice
import time
import sys

# Usar backend lgpio
factory = LGPIOFactory()

# Pines motor L298N
IN1 = 27
IN2 = 22

# Pines encoder
ENCA = 20
ENCB = 13

# Inicialización de pines (usar factory)
motor_forward = PWMOutputDevice(IN1, frequency=5000, pin_factory=factory)
motor_backward = PWMOutputDevice(IN2, frequency=5000, pin_factory=factory)

enca = DigitalInputDevice(ENCA, pull_up=True, pin_factory=factory)
encb = DigitalInputDevice(ENCB, pull_up=True, pin_factory=factory)

# Variables globales
posi = 0
prevT = time.time()
eprev = 0
eintegral = 0
target = 0

# Filtro de rebote para encoder
last_encoder_time = time.time()
debounce_time = 0.001  # 1 ms

# Callback del encoder
def encoder_callback():
    global posi, last_encoder_time

    current_time = time.time()
    if current_time - last_encoder_time < debounce_time:
        return

    last_encoder_time = current_time

    movimiento = 0.17/90*270
    if enca.value == encb.value:
        posi += movimiento
    else:
        posi -= movimiento

    posi = max(-180, min(180, posi))

enca.when_activated = encoder_callback

# Controlador PID con mejoras
def pid_control(target_position):
    global posi, eprev, eintegral, prevT

    Kp = 10
    Ki = 0.01
    Kd = 0.005
    zona_muerta = 1.0
    MIN_PWM = 0.5

    e = target_position - posi
    current_time = time.time()
    deltaT = current_time - prevT
    prevT = current_time

    # Zona muerta: si el error es pequeño, no hacer nada
    if abs(e) < zona_muerta:
        eintegral = 0
        motor_forward.value = 0
        motor_backward.value = 0
        return

    eintegral += e * deltaT
    dedt = (e - eprev) / deltaT if deltaT > 0 else 0
    eprev = e

    output = Kp * e + Ki * eintegral + Kd * dedt
    pwm = 50/(270*90)
    output = max(-pwm, min(pwm, output))

    duty_cycle = abs(output) / 150.0

    # PWM mínimo para vencer fricción
    if duty_cycle > 0 and duty_cycle < MIN_PWM:
        duty_cycle = MIN_PWM

    if output < 0:
        motor_forward.value = duty_cycle
        motor_backward.value = 0
    elif output > 0:
        motor_forward.value = 0
        motor_backward.value = duty_cycle
    else:
        motor_forward.value = 0
        motor_backward.value = 0

def detener_todo():
    print("\nDeteniendo todos los dispositivos...")
    try:
        motor_forward.value = 0
        motor_backward.value = 0
        motor_forward.close()
        motor_backward.close()
        enca.close()
        encb.close()
    except Exception as e:
        print(f"Error al cerrar dispositivos: {e}")
    finally:
        sys.exit(0)

# Bucle principal
try:
    while True:
        input_angle = input("Ingrese el ángulo objetivo (-180 a 180): ")
        try:
            target = float(input_angle)
            if target < -180 or target > 180:
                print("Por favor, ingrese un valor dentro del rango permitido (-180 a 180).")
                continue
        except ValueError:
            print("Entrada no válida. Por favor, ingrese un número.")
            continue

        while abs(target - posi) > 1:
            pid_control(target)
            print(f"Posición actual: {posi:.2f}, Objetivo: {target}")
            time.sleep(0.01)

        motor_forward.value = 0
        motor_backward.value = 0
        print(f"Objetivo alcanzado: {posi:.2f}")

except KeyboardInterrupt:
    detener_todo()
