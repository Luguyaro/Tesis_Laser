from gpiozero import DigitalInputDevice, PWMOutputDevice
from time import sleep
import signal

# Pines del encoder (BCM)
hall_a = DigitalInputDevice(20, pull_up=True)  # Solo usamos hall_a
hall_b = DigitalInputDevice(13, pull_up=True)  # Eliminado porque no se usa para flanco

# Pines del puente H (PWM)
motor_ena = PWMOutputDevice(27)
motor_enb = PWMOutputDevice(22)

# Relación pasos por grado (basado en calibración)
PASOS_POR_GRADO = 0

def SetPasosxGrado(grado):
    global PASOS_POR_GRADO
    if grado <= 60:
        PASOS_POR_GRADO = 2.32
    elif grado >60 and grado < 120:
        PASOS_POR_GRADO = 5 -1.7
    else:
        PASOS_POR_GRADO = 5-2.5

# 🧠 Contador de pasos
step_count = 0

# 🧲 Interrupción en flanco de subida de hall_a
def detectar_paso():
    global step_count
    step_count += 1

hall_a.when_activated = detectar_paso  # Solo en flanco de subida

# 🛞 Control de motor
def mover_motor_derecha(pwm=0.5):
    motor_ena.value = pwm
    motor_enb.value = 0

def mover_motor_izquierda(pwm=0.5):
    motor_ena.value = 0
    motor_enb.value = pwm

def detener_motor():
    motor_ena.value = 0
    motor_enb.value = 0

# 🎯 Movimiento a un ángulo
def mover_a_grados(grados, sentido='1'):
    global step_count
    pasos_objetivo = int(grados * PASOS_POR_GRADO)
    step_count = 0

    print(f"🎯 Moviendo {grados}° hacia la {'derecha' if sentido == '1' else 'izquierda'} ({pasos_objetivo} pasos)...")

    if sentido == '1':
        mover_motor_derecha()
    else:
        mover_motor_izquierda()

    try:
        while step_count < pasos_objetivo:
            restante = pasos_objetivo - step_count
            if restante < 20:
                if sentido == '1':
                    mover_motor_derecha(pwm=0.3)
                else:
                    mover_motor_izquierda(pwm=0.3)
            sleep(0.001)
    finally:
        detener_motor()
        print(f"✅ Movimiento completado. Pasos detectados: {step_count}")

# 🧭 Menú de control
def menu():
    while True:
        try:
            entrada = input("\n🎛️ Ingresa ángulo a girar (ej. 90), o 'q' para salir: ")
            if entrada.lower() == 'q':
                break
            
            grados = float(entrada)
            SetPasosxGrado(grados)
            
            sentido = input("➡️ Dirección (derecha 1/izquierda 2): ").strip().lower()
            if sentido not in ['1', '2']:
                print("❌ Dirección inválida. Usa '1' o '2'.")
                continue

            mover_a_grados(grados, sentido)
        except ValueError:
            print("❌ Entrada inválida. Ingresa un número válido.")

# 🧹 Limpieza de recursos
def limpiar_gpio():
    motor_ena.close()
    motor_enb.close()
    hall_a.close()
    print("🧹 GPIO liberado correctamente.")

try:
    menu()
finally:
    limpiar_gpio()
