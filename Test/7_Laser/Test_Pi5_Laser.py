from gpiozero.pins.lgpio import LGPIOFactory
from gpiozero import PWMOutputDevice
import time

# ======================== CONFIGURACIÓN ========================
PWM_DUTY_CYCLE = .2    # Valor entre 0.0 y 1.0 (comienza bajo para pruebas)
PWM_FREQUENCY = 10000    # Frecuencia recomendada para láser (10 kHz)
GPIO_PIN = 12#25            # Pin GPIO conectado al control PWM del láser
# ===============================================================

# Configura el pin con PWM usando el backend lgpio (funciona en RPi 5)
factory = LGPIOFactory()
laser_pwm = PWMOutputDevice(GPIO_PIN, frequency=PWM_FREQUENCY, pin_factory=factory)

try:
    print(f"Láser encenderá con {PWM_DUTY_CYCLE*100:.1f}% de potencia cada 3 segundos.")
    while True:
        laser_pwm.value = PWM_DUTY_CYCLE  # Activa el láser con PWM parcial
        time.sleep(3)
        laser_pwm.value = 0.0             # Apagado total
        time.sleep(3)

except KeyboardInterrupt:
    print("Detenido por el usuario")

finally:
    laser_pwm.value = 0.0
    laser_pwm.close()
