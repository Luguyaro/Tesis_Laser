from gpiozero import DigitalInputDevice
from signal import pause

# Pines según BCM
hall_a = DigitalInputDevice(20)
hall_b = DigitalInputDevice(13)

# Variables de estado
last_a = hall_a.value
last_b = hall_b.value

def detectar_direccion():
    global last_a, last_b
    a = hall_a.value
    b = hall_b.value

    if last_a == 0 and a == 1:  # Flanco de subida en A
        if b == 0:
            print("↻ Sentido horario (CW)")
        else:
            print("↺ Sentido antihorario (CCW)")
    elif last_b == 0 and b == 1:  # Flanco de subida en B
        if a == 1:
            print("↻ Sentido horario (CW)")
        else:
            print("↺ Sentido antihorario (CCW)")

    last_a = a
    last_b = b

# Asignar funciones a eventos
hall_a.when_activated = detectar_direccion
hall_b.when_activated = detectar_direccion

print("🧭 Detectando dirección de rotación... Presiona Ctrl+C para salir.")
pause()
