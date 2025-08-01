from simple_pid import PID
import RPi.GPIO as GPIO
import time

# Pines motor L298N
IN1 = 27
IN2 = 22
# Pines encoder
ENCODER_A = 23
ENCODER_B = 24

# Setup GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup([IN1, IN2], GPIO.OUT)
GPIO.setup([ENCODER_A, ENCODER_B], GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Posición global
position = 0

def encoder_callback(channel):
    global position
    if GPIO.input(ENCODER_B) == GPIO.HIGH:
        position -= 1
    else:
        position += 1

GPIO.add_event_detect(ENCODER_A, GPIO.RISING, callback=encoder_callback)

# PID Setup
setpoint = 90  # objetivo en "pulsos"
pid = PID(1, 0.2, 0.1, setpoint=setpoint)  # Ajusta PID según necesidad
pid.output_limits = (-1, 1)  # dirección: -1 izquierda, 1 derecha

def control_motor(output):
    if output > 0:
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.LOW)
    elif output < 0:
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.HIGH)
    else:
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.LOW)

try:
    while True:
        output = pid(position)
        control_motor(output)
        print(f"Posición: {position} | Salida PID: {output}")
        time.sleep(0.01)
except KeyboardInterrupt:
    GPIO.cleanup()