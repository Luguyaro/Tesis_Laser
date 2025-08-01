import cv2
import time
import numpy as np
from gpiozero.pins.lgpio import LGPIOFactory
from gpiozero import Servo
from tflite_runtime.interpreter import Interpreter
import signal
import sys

# -------------------------------
# CONFIGURACION GENERAL
# -------------------------------
MODEL_PATH = "/home/pi5/Downloads/Tesis_Laser/IA/Models/model_- 14 july 2025 19_28.tflite"
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
SERVO_PIN = 26
servo_range = (-1, 1)  # Rango para gpiozero.Servo

# PID
Kp = 0.07
Ki = 0.015
Kd = 0.1
ERROR_THRESHOLD = 15
INTEGRAL_LIMIT = 1000
alpha = 0.8  # filtro pasa bajo
CONFIDENCE_THRESHOLD = 0.5
CLASS_FACE_ID = 0  # clase 'persona/rostro', según tu exportación

# -------------------------------
# INICIALIZACION DEL SERVO
# -------------------------------
factory = LGPIOFactory()
servo = Servo(SERVO_PIN, pin_factory=factory, min_pulse_width=0.0005, max_pulse_width=0.0025)
servo_angle = 0.0  # Inicial: centrado

def set_servo_from_angle(angle):
    angle = max(-90, min(90, angle))
    normalized = angle / 90  # de -90 a 90 → -1 a 1
    servo.value = normalized
    time.sleep(0.02)

set_servo_from_angle(servo_angle)

# -------------------------------
# MANEJO DE SALIDA SEGURA
# -------------------------------
def exit_gracefully(sig, frame):
    print("[INFO] Finalizando...")
    cap.release()
    cv2.destroyAllWindows()
    servo.detach()
    sys.exit()

signal.signal(signal.SIGINT, exit_gracefully)

# -------------------------------
# INICIALIZAR MODELO TFLITE
# -------------------------------
interpreter = Interpreter(model_path=MODEL_PATH)
interpreter.allocate_tensors()
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

input_shape = input_details[0]['shape']

# -------------------------------
# INICIALIZACION DE CAMARA
# -------------------------------
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)

if not cap.isOpened():
    print("No se pudo abrir la cámara.")
    sys.exit()

# -------------------------------
# VARIABLES DE CONTROL
# -------------------------------
error_anterior = 0
error_integral = 0
error_filtrado = 0
prev_time = time.time()
print("Presiona 'q' para salir")

# -------------------------------
# FUNCION DE DETECCION YOLO
# -------------------------------
def detect_faces(frame):
    img_resized = cv2.resize(frame, (640, 640))
    img_rgb = cv2.cvtColor(img_resized, cv2.COLOR_BGR2RGB)
    input_data = np.expand_dims(img_rgb, axis=0).astype(np.float32) / 255.0

    interpreter.set_tensor(input_details[0]['index'], input_data)
    interpreter.invoke()
    output_data = interpreter.get_tensor(output_details[0]['index'])[0]

    detections = []
    for i in range(0, len(output_data), 12):
        score = output_data[i + 4]
        class_id = int(output_data[i + 5])
        if score > CONFIDENCE_THRESHOLD and class_id == CLASS_FACE_ID:
            x_center = output_data[i + 0] * CAMERA_WIDTH
            y_center = output_data[i + 1] * CAMERA_HEIGHT
            w = output_data[i + 2] * CAMERA_WIDTH
            h = output_data[i + 3] * CAMERA_HEIGHT
            x1 = int(x_center - w / 2)
            y1 = int(y_center - h / 2)
            detections.append((x1, y1, int(w), int(h), y_center))
    return detections

# -------------------------------
# BUCLE PRINCIPAL
# -------------------------------
while True:
    ret, frame = cap.read()
    if not ret:
        print("Error al capturar frame")
        break

    frame_center_y = CAMERA_HEIGHT / 2
    detections = detect_faces(frame)

    if detections:
        # Elegir el rostro más centrado verticalmente
        detections.sort(key=lambda d: abs(frame_center_y - d[4]))
        x, y, w, h, obj_center_y = detections[0]

        # PID control vertical (pitch)
        current_time = time.time()
        delta_time = current_time - prev_time
        error_actual = frame_center_y - obj_center_y
        error_filtrado = alpha * error_filtrado + (1 - alpha) * error_actual
        delta_error = error_filtrado - error_anterior
        error_integral += error_filtrado * delta_time
        error_integral = max(-INTEGRAL_LIMIT, min(INTEGRAL_LIMIT, error_integral))

        output = Kp * error_filtrado + Ki * error_integral + (Kd * delta_error / delta_time if delta_time > 0 else 0)

        if abs(error_filtrado) > ERROR_THRESHOLD:
            servo_angle += output
            servo_angle = max(-90, min(90, servo_angle))
            set_servo_from_angle(servo_angle)

        error_anterior = error_filtrado
        prev_time = current_time

        # Dibujar resultados
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 128), 2)
        cv2.line(frame, (0, int(frame_center_y)), (CAMERA_WIDTH, int(frame_center_y)), (255, 255, 255), 1)
        cv2.line(frame, (0, int(obj_center_y)), (CAMERA_WIDTH, int(obj_center_y)), (0, 255, 255), 1)
        cv2.putText(frame, f"Error Y: {error_filtrado:.1f}px", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        cv2.putText(frame, f"Servo Angle: {servo_angle:.1f}", (10, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

    else:
        error_anterior = 0
        error_integral = 0
        error_filtrado = 0

    cv2.imshow("Tracking YOLOv5s-u Pitch", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# -------------------------------
# FINALIZACION
# -------------------------------
cap.release()
cv2.destroyAllWindows()
servo.detach()
