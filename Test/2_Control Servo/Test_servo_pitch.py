import RPi.GPIO as GPIO
import time

# Configura el pin de salida PWM
SERVO_PIN = 16  # Puedes cambiarlo
GPIO.setmode(GPIO.BCM)
GPIO.setup(SERVO_PIN, GPIO.OUT)
 
# PWM a 50Hz
pwm = GPIO.PWM(SERVO_PIN, 50)
pwm.start(0)

def mover_servo(angulo):
    # Convierte ángulo (0-180) a ciclo útil (duty cycle)
    duty = 2.5 + (angulo / 180.0) * 10  # entre 2.5 y 12.5
    pwm.ChangeDutyCycle(duty)
    time.sleep(0.5)
    pwm.ChangeDutyCycle(0)  # Evita zumbido

try:
    while True:
        ang = int(input("Ingresa ángulo (50-180): "))
        if 50 <= ang <= 180:
            mover_servo(ang)
        else:
            print("Selecciona valores entre 50-180...")
except KeyboardInterrupt:
    print("Saliendo...")
finally:
    pwm.stop()
    GPIO.cleanup()