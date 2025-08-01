import tflite_runtime.interpreter as tflite
import numpy as np
from PIL import Image
import cv2
import time
import threading
import lgpio
from gpiozero import DigitalInputDevice, PWMOutputDevice

# ---------------------------- CONFIGURACIÓN GLOBAL ----------------------------
IMG_WIDTH = 640
IMG_HEIGHT = 480
DEADZONE_X = 20
DEADZONE_Y = 15
TFLITE_MODEL = "/home/pi5/Downloads/Tesis_Laser/IA/Models/model_- 17 july 2025 10_04(2).tflite"
CONF_THRESHOLD = 1500e-10
PASOS_POR_PIXEL_X = 0.18  # Debe ser calibrado según tu cámara y motor
PASOS_POR_GRADO = 10      # ⚙️ Ajusta si tu motor tiene otra resolución
MAX_PASOS_YAW = 50 * PASOS_POR_GRADO  # ±50 grados
SERVO_GPIO = 19
pulso_actual = 1300
input_size = 640

moving_servo = threading.Event()
moving_motor = threading.Event()

# Estado global yaw
step_count = 0
step_total = 0  # 🚫 Control del rango de giro acumulado

# ---------------------------- HARDWARE SETUP ----------------------------
HALL_A_PIN = 20
ENA_PIN = 22
ENB_PIN = 27
h = lgpio.gpiochip_open(0)
hall_a = DigitalInputDevice(HALL_A_PIN, pull_up=True)
motor_ena = PWMOutputDevice(ENA_PIN)
motor_enb = PWMOutputDevice(ENB_PIN)

# ---------------------------- SERVO FUNCTIONS ----------------------------
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

# ---------------------------- MOTOR YAW FUNCTIONS ----------------------------
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
    global step_count, step_total
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
            if (sentido == '1' and step_total >= MAX_PASOS_YAW) or \
               (sentido == '2' and step_total <= -MAX_PASOS_YAW):
                print("⚠️ Límite de giro alcanzado")
                break
            restante = pasos_objetivo - step_count
            pwm = 0.3 if restante < 20 else 0.5
            if sentido == '1':
                mover_motor_derecha(pwm)
            else:
                mover_motor_izquierda(pwm)
            time.sleep(0.001)
        # Actualiza total de pasos
        if sentido == '1':
            step_total += step_count
        else:
            step_total -= step_count
    finally:
        detener_motor()
        moving_motor.clear()

def corregir_yaw(error_x):
    if abs(error_x) < DEADZONE_X or moving_motor.is_set():
        return
    pasos = int(abs(error_x) * PASOS_POR_PIXEL_X)
    sentido = '1' if error_x > 0 else '2'
    threading.Thread(target=mover_a_pasos, args=(pasos, sentido), daemon=True).start()

# ---------------------------- DETECCIÓN Y TRACKING ----------------------------
def detectar_objeto_y_mover():
    global pulso_actual
    mover_servo_incremento(0)

    interpreter = tflite.Interpreter(model_path=TFLITE_MODEL)
    interpreter.allocate_tensors()
    input_details = interpreter.get_input_details()
    output_details = interpreter.get_output_details()

    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, IMG_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, IMG_HEIGHT)

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Preprocesamiento
        input_img = cv2.resize(frame, (input_size, input_size))
        input_data = np.expand_dims(input_img.astype(np.float32) / 255.0, axis=0)

        interpreter.set_tensor(input_details[0]['index'], input_data)
        interpreter.invoke()
        output_data = interpreter.get_tensor(output_details[0]['index'])

        output_data = np.squeeze(output_data).transpose()
        best_box = None
        best_score = 0

        # Detectar solo un objeto (mayor score)
        for det in output_data:
            x, y, w, h = det[0:4]
            conf = det[4]
            class_probs = det[5:]
            class_id = np.argmax(class_probs)
            class_score = class_probs[class_id]
            total_conf = conf * class_score

            if total_conf > CONF_THRESHOLD and total_conf > best_score:
                cx, cy = x * input_size, y * input_size
                bw, bh = w * input_size, h * input_size
                x1 = int((cx - bw/2) * (IMG_WIDTH/input_size))
                y1 = int((cy - bh/2) * (IMG_HEIGHT/input_size))
                x2 = int((cx + bw/2) * (IMG_WIDTH/input_size))
                y2 = int((cy + bh/2) * (IMG_HEIGHT/input_size))
                best_box = (x1, y1, x2, y2, class_id, total_conf)
                best_score = total_conf

        if best_box:
            x1, y1, x2, y2, class_id, score = best_box
            cx_obj = (x1 + x2) // 2
            cy_obj = (y1 + y2) // 2
            cx_frame = IMG_WIDTH // 2
            cy_frame = IMG_HEIGHT // 2
            error_x = cx_obj - cx_frame
            error_y = cy_obj - cy_frame

            # Movimiento pitch
            if abs(error_y) > DEADZONE_Y and not moving_servo.is_set():
                delta_servo = int(error_y / 10) * -1
                threading.Thread(target=mover_servo_incremento, args=(delta_servo,), daemon=True).start()

            # Movimiento yaw
            corregir_yaw(error_x)

            # Dibujar
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            label = f"Clase {class_id} | Score: {score:.2f}"
            cv2.putText(frame, label, (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

            print(f"[Pitch] Y: {cy_obj} | errorY: {error_y:+} px | PWM: {pulso_actual} µs")
            print(f"[Yaw] X: {cx_obj} | errorX: {error_x:+} px | ΔSteps: {step_count} | Total: {step_total}")

        # Mostrar centro
        cv2.rectangle(frame, (IMG_WIDTH//2 - 5, IMG_HEIGHT//2 - 5), (IMG_WIDTH//2 + 5, IMG_HEIGHT//2 + 5), (255, 0, 0), 2)
        cv2.imshow("TFLite Tracking Cam", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

# ---------------------------- LIMPIEZA ----------------------------
def limpiar():
    detener_motor()
    motor_ena.close()
    motor_enb.close()
    hall_a.close()
    lgpio.gpiochip_close(h)
    print("GPIO liberado.")

# ---------------------------- EJECUCIÓN ----------------------------
try:
    detectar_objeto_y_mover()
finally:
    limpiar()
