import cv2
import threading
import time
import lgpio
from gpiozero import DigitalInputDevice, PWMOutputDevice

# ----------------------------
# CONFIGURACIÓN GLOBAL
# ----------------------------
# Cámara
IMG_WIDTH = 640
IMG_HEIGHT = 480
DEADZONE_X = 20
DEADZONE_Y = 15

# Servo (Pitch)
SERVO_GPIO = 19
pulso_actual = 1300
moving_servo = threading.Event()

# Motor (Yaw)
HALL_A_PIN = 20
ENA_PIN = 27
ENB_PIN = 22
PASOS_POR_PIXEL_X = 0.18  # 🔧 Ajusta esto con calibración
moving_motor = threading.Event()
step_count = 0

# Objeto de la cámara
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

# GPIO setup
h = lgpio.gpiochip_open(0)
hall_a = DigitalInputDevice(HALL_A_PIN, pull_up=True)
motor_ena = PWMOutputDevice(ENA_PIN)
motor_enb = PWMOutputDevice(ENB_PIN)

# ----------------------------
# FUNCIONES DE CONTROL - SERVO
# ----------------------------
def send_servo_pulse(us):
    lgpio.gpio_write(h, SERVO_GPIO, 1)
    time.sleep(us / 1_000_000)
    lgpio.gpio_write(h, SERVO_GPIO, 0)
    time.sleep((20000 - us) / 1_000_000)

def mover_servo_incremento(delta):
    global pulso_actual
    if moving_servo.is_set():
        return
    moving_servo.set()
    nuevo_pulso = max(500, min(2500, pulso_actual + delta))
    for _ in range(5):
        send_servo_pulse(nuevo_pulso)
    pulso_actual = nuevo_pulso
    moving_servo.clear()

# ----------------------------
# FUNCIONES DE CONTROL - MOTOR YAW
# ----------------------------
def detectar_paso():
    global step_count
    step_count += 1

hall_a.when_activated = detectar_paso

def mover_motor_derecha(pwm=0.5):
    motor_ena.value = pwm
    motor_enb.value = 0

def mover_motor_izquierda(pwm=0.5):
    motor_ena.value = 0
    motor_enb.value = pwm

def detener_motor():
    motor_ena.value = 0
    motor_enb.value = 0

def mover_a_pasos(pasos_objetivo, sentido='1'):
    global step_count
    if moving_motor.is_set():
        return
    moving_motor.set()
    step_count = 0

    if sentido == '1':
        mover_motor_derecha()
    else:
        mover_motor_izquierda()

    try:
        while step_count < pasos_objetivo:
            restante = pasos_objetivo - step_count
            pwm = 0.3 if restante < 20 else 0.5
            if sentido == '1':
                mover_motor_derecha(pwm)
            else:
                mover_motor_izquierda(pwm)
            time.sleep(0.001)
    finally:
        detener_motor()
        moving_motor.clear()

def corregir_yaw(error_x):
    if abs(error_x) < DEADZONE_X or moving_motor.is_set():
        return
    pasos = int(abs(error_x) * PASOS_POR_PIXEL_X)
    sentido = '1' if error_x > 0 else '2'
    threading.Thread(target=mover_a_pasos, args=(pasos, sentido), daemon=True).start()

# ----------------------------
# DETECCIÓN + CONTROL
# ----------------------------
def detectar_objeto_y_mover():
    global pulso_actual
    mover_servo_incremento(0)  # Posición inicial

    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, IMG_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, IMG_HEIGHT)

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = face_cascade.detectMultiScale(gray, 1.3, 5)

        cx_frame = IMG_WIDTH // 2
        cy_frame = IMG_HEIGHT // 2

        if len(faces) > 0:
            (x, y, w, h) = faces[0]
            cx_obj = x + w // 2
            cy_obj = y + h // 2

            error_x = cx_obj - cx_frame
            error_y = cy_obj - cy_frame

            # Pitch (servo)
            if abs(error_y) > DEADZONE_Y and not moving_servo.is_set():
                delta_servo = int(error_y / 10) * -1
                threading.Thread(target=mover_servo_incremento, args=(delta_servo,), daemon=True).start()

            # Yaw (motor con encoder)
            corregir_yaw(error_x)

            # Dibujo en pantalla
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

            print(f"[Pitch] inputY: {cy_obj} | errorY: {error_y:+} px | PWM: {pulso_actual} µs")
            print(f"[Yaw] inputX: {cx_obj} | errorX: {error_x:+} px | Pasos: {step_count}")

        # Marcar centro de la cámara
        cv2.rectangle(frame, (cx_frame - 5, cy_frame - 5), (cx_frame + 5, cy_frame + 5), (255, 0, 0), 2)
        cv2.imshow("Tracking Cam", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

# ----------------------------
# LIMPIEZA
# ----------------------------
def limpiar():
    detener_motor()
    motor_ena.close()
    motor_enb.close()
    hall_a.close()
    lgpio.gpiochip_close(h)
    print("GPIO liberado.")

# ----------------------------
# EJECUCIÓN PRINCIPAL
# ----------------------------
try:
    detectar_objeto_y_mover()
finally:
    limpiar()
