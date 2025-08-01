import cv2
import threading
import time
import lgpio
import tflite_runtime.interpreter as tflite
import numpy as np
from PIL import Image

# ----------------------------
# CONFIGURACIÓN
# ----------------------------
SERVO_GPIO = 19
IMG_WIDTH = 640
IMG_HEIGHT = 480
conf_threshold = 1500e-10
input_size = 640
DEADZONE = 15  # px
INCREMENTO_SERVO = 10  # µs por paso

# ----------------------------
# INICIALIZAR MODELO
# ----------------------------
model_path = "/home/pi5/Downloads/Tesis_Laser/IA/Models/model_- 17 july 2025 10_04(2).tflite"
interpreter = tflite.Interpreter(model_path=model_path)
interpreter.allocate_tensors()
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

# ----------------------------
# INICIALIZAR SERVO
# ----------------------------
h = lgpio.gpiochip_open(0)
pulso_actual = 2500  # 🟢 150° = 2500 µs
moving_servo = threading.Event()

def send_servo_pulse(us):
    lgpio.gpio_write(h, SERVO_GPIO, 1)
    time.sleep(us / 1_000_000)
    lgpio.gpio_write(h, SERVO_GPIO, 0)
    time.sleep((20_000 - us) / 1_000_000)

def ir_a_posicion_inicial():
    print("📌 Posicionando servo a 150° (2500 µs)")
    for _ in range(10):
        send_servo_pulse(2500)
        time.sleep(0.03)

def mover_servo_incremento(delta):
    global pulso_actual
    if moving_servo.is_set():
        return
    moving_servo.set()

    nuevo_pulso = max(500, min(2500, pulso_actual + delta))
    for _ in range(5):  # Reforzar el pulso para asegurar movimiento
        send_servo_pulse(nuevo_pulso)
        time.sleep(0.01)
    pulso_actual = nuevo_pulso
    moving_servo.clear()

# ----------------------------
# DETECCIÓN + SEGUIMIENTO
# ----------------------------
def detectar_objeto_y_mover():
    print("🧠 Iniciando detección + seguimiento vertical servo")
    ir_a_posicion_inicial()  # 🟢 Posición inicial antes de empezar

    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, IMG_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, IMG_HEIGHT)

    if not cap.isOpened():
        print("❌ Error: No se pudo abrir la cámara")
        return

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        rotated_frame = cv2.rotate(frame.copy(), cv2.ROTATE_180)
        pil_image = Image.fromarray(cv2.cvtColor(rotated_frame, cv2.COLOR_BGR2RGB))
        original_width, original_height = pil_image.size

        image_resized = pil_image.resize((input_size, input_size))
        input_data = np.expand_dims(np.array(image_resized, dtype=np.float32), axis=0) / 255.0

        interpreter.set_tensor(input_details[0]['index'], input_data)
        interpreter.invoke()
        output_data = interpreter.get_tensor(output_details[0]['index'])
        output_data = np.squeeze(output_data).transpose()

        boxes = []
        for det in output_data:
            x, y, w, h = det[0:4]
            conf = det[4]
            class_probs = det[5:]
            class_id = np.argmax(class_probs)
            class_score = class_probs[class_id]
            total_conf = conf * class_score

            if total_conf > conf_threshold:
                cx, cy = x * input_size, y * input_size
                bw, bh = w * input_size, h * input_size
                x1 = int((cx - bw/2) * (original_width / input_size))
                y1 = int((cy - bh/2) * (original_height / input_size))
                x2 = int((cx + bw/2) * (original_width / input_size))
                y2 = int((cy + bh/2) * (original_height / input_size))
                boxes.append((x1, y1, x2, y2, class_id, total_conf))

        cx_frame = IMG_WIDTH // 2
        cy_frame = IMG_HEIGHT // 2

        for (x1, y1, x2, y2, class_id, score) in boxes:
            cy_obj = (y1 + y2) // 2
            error_y = cy_obj - cy_frame

            # Si está fuera del DEADZONE, hacer ajuste mínimo
            if abs(error_y) > DEADZONE and not moving_servo.is_set():
                delta = -INCREMENTO_SERVO if error_y > 0 else INCREMENTO_SERVO
                threading.Thread(target=mover_servo_incremento, args=(delta,), daemon=True).start()
                print(f"🎯 Error Y: {error_y:+} px | PWM: {pulso_actual} µs")

            cv2.rectangle(rotated_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(rotated_frame, f"{score:.2f}", (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # Centro de referencia
        cv2.rectangle(rotated_frame, (cx_frame - 5, cy_frame - 5), (cx_frame + 5, cy_frame + 5), (255, 0, 0), 2)
        cv2.imshow("Tracking Pitch Servo", rotated_frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("⛔ Salida solicitada por el usuario.")
            break

    cap.release()
    cv2.destroyAllWindows()

# ----------------------------
# EJECUCIÓN
# ----------------------------
try:
    detectar_objeto_y_mover()
finally:
    lgpio.gpiochip_close(h)
    print("🧼 GPIO liberado correctamente.")
