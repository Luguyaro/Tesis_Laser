import cv2
import threading
import time
import lgpio

# ----------------------------
# Configuración inicial
# ----------------------------
SERVO_GPIO = 26
IMG_WIDTH = 640
IMG_HEIGHT = 480

# PI ajustado con valores en µs directamente
Kp_us = 1      # Ganancia proporcional (en µs / px)
Ki_us = 5      # Ganancia integral (en µs / px*s)
DEADZONE = 20     # Zona muerta en píxeles
INTEGRAL_MAX = 500  # Anti-windup para el acumulador
alpha = 0.2      # Suavizado exponencial
DELTA_US_MAX = 30   # Límite máximo de desplazamiento por ciclo en µs

# Estado global
moving_servo = threading.Event()
pulso_actual = 1500
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
h = lgpio.gpiochip_open(0)

# Variables PI
integralY = 0
last_time = time.time()

def send_servo_pulse(us):
    lgpio.gpio_write(h, SERVO_GPIO, 1)
    time.sleep(us / 1_000_000)
    lgpio.gpio_write(h, SERVO_GPIO, 0)
    time.sleep((20000 - us) / 1_000_000)

def mover_servo_a(us_destino):
    global pulso_actual
    moving_servo.set()
    paso = 2 if us_destino > pulso_actual else -2
    for us in range(pulso_actual, us_destino + paso, paso):
        send_servo_pulse(us)
        time.sleep(0.015)  # Espera entre cada micro-movimiento
        pulso_actual = us
    moving_servo.clear()

def servo_thread_func(target_us):
    if not moving_servo.is_set():
        mover_servo_a(target_us)

def detectar_objeto_y_mover():
    global integralY, last_time, pulso_actual

    # Mover a posición inicial
    mover_servo_a(1300)

    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, IMG_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, IMG_HEIGHT)

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        current_time = time.time()
        dt = current_time - last_time
        last_time = current_time

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = face_cascade.detectMultiScale(gray, 1.3, 5)

        cx_frame = IMG_WIDTH // 2
        cy_frame = IMG_HEIGHT // 2

        if len(faces) > 0:
            (x, y, w, h) = faces[0]
            cy_obj = y + h // 2

            error_y = cy_obj - cy_frame
            if abs(error_y) < DEADZONE:
                error_y = 0

            integralY += error_y * dt
            integralY = max(-INTEGRAL_MAX, min(INTEGRAL_MAX, integralY))

            delta_us = int(-Kp_us * error_y - Ki_us * integralY)
            delta_us = max(-DELTA_US_MAX, min(DELTA_US_MAX, delta_us))

            target_us = max(500, min(2500, pulso_actual + delta_us))

            # Suavizado
            target_us = int(pulso_actual + alpha * (target_us - pulso_actual))

            print(f"inputY: {cy_obj} | errorY: {error_y:+} px | PWM: {target_us} µs")

            if not moving_servo.is_set() and delta_us != 0:
                threading.Thread(target=servo_thread_func, args=(target_us,)).start()

            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)

        else:
            integralY = 0

        cv2.rectangle(frame, (cx_frame - 5, cy_frame - 5), (cx_frame + 5, cy_frame + 5), (255, 0, 0), 2)
        cv2.imshow("Seguimiento Vertical", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

try:
    detectar_objeto_y_mover()
finally:
    lgpio.gpiochip_close(h)
    print("GPIO liberado.")