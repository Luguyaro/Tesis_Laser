import time
import VL53L1X

# Crear objeto del sensor
tof = VL53L1X.VL53L1X()

tof.open()  # Abre comunicación I2C
tof.start_ranging(2)  # 1 = SHORT, 2 = MEDIUM, 3 = LONG
#SHORT (1): más rápido, menos alcance.
#MEDIUM (2): balanceado.
#LONG (3): máximo alcance (~4 m).
print("📏 Sensor VL53L1X iniciado. Midiendo distancia...")

try:
    while True:
        distance = tof.get_distance()
        print(f"Distancia: {distance} mm")
        time.sleep(0.3)

except KeyboardInterrupt:
    print("✅ Lectura finalizada por el usuario")

finally:
    tof.stop_ranging()
    tof.close()