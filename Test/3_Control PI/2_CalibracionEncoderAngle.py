import time
import statistics
from gpiozero import DigitalInputDevice, PWMOutputDevice
from signal import pause

# Pines del encoder
hall_a = DigitalInputDevice(22)
hall_b = DigitalInputDevice(27)

# Pines PWM del puente H
pwm_ena = PWMOutputDevice(10)  # Dirección A
pwm_enb = PWMOutputDevice(9)   # Dirección B

# Variables globales
step_count = 0
last_a = hall_a.value
last_b = hall_b.value
direction = 1  # 1 = CW, -1 = CCW

# Tabla de pasos por ángulo
angle_targets = [90, 180, 270, 360]
calibration_data = {angle: [] for angle in angle_targets}

def encoder_callback():
    global step_count, last_a, last_b, direction
    a = hall_a.value
    b = hall_b.value

    if last_a == 0 and a == 1:
        if b == 0:
            direction = 1
        else:
            direction = -1
        step_count += direction
    elif last_b == 0 and b == 1:
        if a == 1:
            direction = 1
        else:
            direction = -1
        step_count += direction

    last_a = a
    last_b = b

def calibrar_motor():
    print("🔧 Iniciando rutina de calibración...")
    for angle in angle_targets:
        print(f"\n➡️ Girando hasta {angle}° (10 repeticiones)")
        for i in range(10):
            global step_count
            step_count = 0
            pwm_ena.value = 0.5
            pwm_enb.value = 0.0  # Dirección CW
            input(f"Presiona ENTER para iniciar giro #{i+1} hasta {angle}°...")
            input(f"Presiona ENTER cuando el motor alcance {angle}°...")
            pwm_ena.off()
            pasos = abs(step_count)
            calibration_data[angle].append(pasos)
            print(f"✅ Pasos registrados: {pasos}")
            time.sleep(1)

    # Guardar resultados
    with open("calibracion_resultados.txt", "w") as f:
        for angle in angle_targets:
            f.write(f"{angle}°: {calibration_data[angle]}\n")

    print("\n📁 Datos guardados en calibracion_resultados.txt")

def calcular_promedios():
    promedios = {}
    for angle, pasos in calibration_data.items():
        if pasos:
            promedios[angle] = statistics.mean(pasos)
    return promedios

def mover_a_grados(grados, promedios):
    if grados not in promedios:
        print("❌ Ángulo no calibrado.")
        return

    objetivo = int(promedios[grados])
    global step_count
    step_count = 0
    pwm_ena.value = 0.5
    pwm_enb.value = 0.0  # Dirección CW
    print(f"🎯 Moviendo motor hasta {grados}° ({objetivo} pasos)...")

    try:
        while abs(step_count) < objetivo:
            if abs(step_count) > objetivo * 0.95:
                pwm_ena.value = 0.3  # Reducir velocidad al acercarse
            time.sleep(0.01)
    finally:
        pwm_ena.off()
        print(f"✅ Movimiento completado. Pasos dados: {step_count}")

# Asignar callbacks
hall_a.when_activated = encoder_callback
hall_b.when_activated = encoder_callback

# Menú principal
def menu():
    while True:
        print("\n=== MENÚ DE CONTROL DE MOTOR ===")
        print("1. Calibrar motor (registrar pasos por ángulo)")
        print("2. Mostrar promedios de pasos por ángulo")
        print("3. Mover motor a un ángulo específico")
        print("4. Salir")
        opcion = input("Selecciona una opción: ")

        if opcion == '1':
            calibrar_motor()
        elif opcion == '2':
            promedios = calcular_promedios()
            for angle, avg in promedios.items():
                print(f"{angle}° → {avg:.2f} pasos")
        elif opcion == '3':
            promedios = calcular_promedios()
            grados = int(input("Ingresa el ángulo (90, 180, 270, 360): "))
            mover_a_grados(grados, promedios)
        elif opcion == '4':
            print("👋 Saliendo...")
            break
        else:
            print("❌ Opción inválida.")

menu()

