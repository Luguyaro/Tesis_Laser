from gpiozero import DigitalInputDevice
from time import sleep
import json

# Pines del encoder Hall (BCM)
hall_a = DigitalInputDevice(20)
hall_b = DigitalInputDevice(13)

# Variables de estado
last_a = hall_a.value
last_b = hall_b.value
step_count = 0

# Lista para guardar pasos por tramo
resultados = {
    "0→90": [],
    "90→180": [],
    "180→90": [],
    "90→0": []
}

def detectar_direccion():
    global last_a, last_b, step_count
    a = hall_a.value
    b = hall_b.value

    if last_a == 0 and a == 1:  # Flanco de subida en A
        step_count += 1
    elif last_b == 0 and b == 1:  # Flanco de subida en B
        step_count += 1

    last_a = a
    last_b = b

# Asignar funciones a eventos
hall_a.when_activated = detectar_direccion
hall_b.when_activated = detectar_direccion

# Rutina de calibración
tramos = ["0-90", "90-180", "180-90", "90-0"]

print("🔧 Iniciando rutina de calibración manual del encoder.")
print("🔁 Realiza el siguiente ciclo 10 veces:")

for ciclo in range(10):
    print(f"\n🔄 Ciclo {ciclo + 1}/10")
    for tramo in tramos:
        input(f"➡️ Gira manualmente el motor desde {tramo}. Presiona ENTER cuando termines...")
        pasos = step_count
        resultados[tramo].append(pasos)
        print(f"{tramo}: {pasos}")
        step_count = 0  # Reiniciar contador para el siguiente tramo

# Guardar resultados en archivo
with open("calibracion_resultados.txt", "w") as f:
    for tramo, pasos in resultados.items():
        f.write(f"{tramo}: {pasos}\n")

print("\n📁 Calibración completada. Resultados guardados en 'calibracion_resultados.txt'.")
