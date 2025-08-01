import RPi.GPIO as GPIO
import time


# Pines motor L298N
IN1 = 27   # Motor adelante
IN2 = 22   # Motor atrás
# Pines encoder
ENCA = 23  # Entrada A
ENCB = 24  # Entrada B

# Variables globales
posi = 0  # Posición actual
prevT = time.time()
eprev = 0
eintegral = 0
target = 0  # Posición objetivo inicial

# Configuración GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(ENCA, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(ENCB, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)

# Configuración de PWM
pwm_freq = 5000  # Frecuencia PWM en Hz
pwm1 = GPIO.PWM(IN1, pwm_freq)
pwm2 = GPIO.PWM(IN2, pwm_freq)
pwm1.start(0)
pwm2.start(0)

# Filtro para ruido del encoder
last_encoder_time = time.time()
debounce_time = 0.001  # 1 ms

# ISR para encoder con decodificación x2
def encoder_callback(channel):
    global posi, last_encoder_time
    current_time = time.time()
    
    # Evitar rebotes del encoder
    if current_time - last_encoder_time < debounce_time:
        return  # Ignorar rebotes

    last_encoder_time = current_time

    # Leer el estado de las entradas del encoder A y B
    a_state = GPIO.input(ENCA)
    b_state = GPIO.input(ENCB)

    # Decodificación x2 (aumento de la resolución)
    movimiento = 0.17
    if a_state == b_state:  # Cuando A y B tienen el mismo estado
        posi -= movimiento  # Movimiento en una dirección
    else:  # Cuando A y B tienen estado opuesto
        posi += movimiento # Movimiento en la otra dirección

    # Limitar la posición al rango permitido
    posi = max(-180, min(180, posi))

GPIO.add_event_detect(ENCA, GPIO.RISING, callback=encoder_callback)

# Control PID
def pid_control(target_position):
    global posi, eprev, eintegral, prevT

    # Constantes PID
    Kp = 6
    Ki = 0.01
    Kd = 0.05

    # Calcular error
    e = target_position - posi
    current_time = time.time()
    deltaT = current_time - prevT
    prevT = current_time

    # Integrador y derivador
    eintegral += e * deltaT
    dedt = (e - eprev) / deltaT if deltaT > 0 else 0
    eprev = e

    # Controlador PID
    output = Kp * e + Ki * eintegral + Kd * dedt

    # Limitar salida
    max_duty_cycle = 80
    output = max(-max_duty_cycle, min(max_duty_cycle, output))

    # Control de dirección y velocidad
    if output > 0:
        pwm1.ChangeDutyCycle(output)
        pwm2.ChangeDutyCycle(0)
    elif output < 0:
        pwm1.ChangeDutyCycle(0)
        pwm2.ChangeDutyCycle(-output)
    else:
        pwm1.ChangeDutyCycle(0)
        pwm2.ChangeDutyCycle(0)

# Bucle principal
try:
    while True:
        # Ingresar ángulo objetivo
        input_angle = input("Ingrese el ángulo objetivo (-180 a 180): ")
        try:
            target = float(input_angle)
            if target < -180 or target > 180:
                print("Por favor, ingrese un valor dentro del rango permitido (-180 a 180).")
                continue
        except ValueError:
            print("Entrada no válida. Por favor, ingrese un número.")
            continue
        target = target/1
        # Ejecutar PID hasta alcanzar el objetivo
        while abs(target - posi) > 1:  # Tolerancia de ±1
            pid_control(target)
            print(f"Posición actual: {posi}, Objetivo: {target}")
            time.sleep(0.01)

        # Detener motor al alcanzar el objetivo
        pwm1.ChangeDutyCycle(0)
        pwm2.ChangeDutyCycle(0)
        print(f"Objetivo alcanzado: {posi}")

except KeyboardInterrupt:
    print("\nDeteniendo...")
    pwm1.stop()
    pwm2.stop()
    GPIO.cleanup()