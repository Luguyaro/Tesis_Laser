from gpiozero import DigitalInputDevice, PWMOutputDevice
from time import sleep
from gpiozero.pins.native import NativeFactory
#Device.pin_factory = NativeFactory()

# Pines del encoder (BCM)
hall_a = DigitalInputDevice(20)
hall_b = DigitalInputDevice(13)

# Pines del puente H (PWM)
motor_ena = PWMOutputDevice(22)  # GPIO10
motor_enb = PWMOutputDevice(27)   # GPIO9

# Relación pasos por grado (basado en calibración
PASOS_POR_GRADO =0
def SetPasosxGrado(grado):
    #global grado
    if grado<=60:
        PASOS_POR_GRADO = 2.32-1
    else:
        PASOS_POR_GRADO = 2.32-2
    
# Variables de estado
last_a = hall_a.value
last_b = hall_b.value
step_count = 0

def detectar_paso():
    global last_a, last_b, step_count
    a = hall_a.value
    b = hall_b.value

    if last_a == 0 and a == 1:
        step_count += 1
    elif last_b == 0 and b == 1:
        step_count += 1

    last_a = a
    last_b = b

hall_a.when_activated = detectar_paso
hall_b.when_activated = detectar_paso

def mover_motor_derecha(pwm=0.5):
    motor_ena.value = pwm
    motor_enb.value = 0

def mover_motor_izquierda(pwm=0.5):
    motor_ena.value = 0
    motor_enb.value = pwm

def detener_motor():
    motor_ena.value = 0
    motor_enb.value = 0

def mover_a_grados(grados, sentido='1'):
    global step_count
    pasos_objetivo = int(grados * PASOS_POR_GRADO)
    step_count = 0

    print(f"🎯 Moviendo {grados}° hacia la {sentido} ({pasos_objetivo} pasos)...")

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
                print("❌ Dirección inválida. Usa 'derecha' o 'izquierda'.")
                continue
            mover_a_grados(grados, sentido)
        except ValueError:
            print("❌ Entrada inválida. Ingresa un número válido.")

# Al final del script
def limpiar_gpio():
    motor_ena.close()
    motor_enb.close()
    hall_a.close()
    hall_b.close()
    print("🧹 GPIO liberado correctamente.")

menu()
# Llamar esta función al salir del menú
#limpiar_gpio()