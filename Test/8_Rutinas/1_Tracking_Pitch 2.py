import cv2
import threading
import time
import lgpio

# ----------------------------
# Configuración inicial
# ----------------------------
SERVO_GPIO = 19
IMG_WIDTH = 640
IMG_HEIGHT = 480

# Configuración
model_path = "/home/pi5/Downloads/Tesis_Laser/IA/Models/model_- 17 july 2025 10_04(2).tflite"
conf_threshold = 1500e-10
input_size = 640

# Cargar modelo
interpreter = tflite.Interpreter(model_path=model_path)
interpreter.allocate_tensors()
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

pulso_actual = 1000
moving_servo = threading.Event()
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
h = lgpio.gpiochip_open(0)

DEADZONE = 15  # Píxeles de tolerancia en vertical

# ----------------------------
# Funciones
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
    for _ in range(5):  # enviar múltiples pulsos para que el servo responda bien
        send_servo_pulse(nuevo_pulso)
    pulso_actual = nuevo_pulso
    moving_servo.clear()

# ----------------------------
# Hilo de procesamiento
# ----------------------------
def detectar_objeto_y_mover():
    global pulso_actual

    mover_servo_incremento(0)  # Ir a posición inicial 1300 µs

    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, IMG_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, IMG_HEIGHT)

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Rotar imagen 180 grados
        #frame = cv2.rotate(frame, cv2.ROTATE_180)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        faces = face_cascade.detectMultiScale(gray, 1.3, 5)

        cx_frame = IMG_WIDTH // 2
        cy_frame = IMG_HEIGHT // 2

        if len(faces) > 0:
            (x, y, w, h) = faces[0]
            cy_obj = y + h // 2

            error_y = cy_obj - cy_frame
            if abs(error_y) > DEADZONE and not moving_servo.is_set():
                delta = -5 if error_y > 0 else 5  # invertir si se mueve al revés
                threading.Thread(target=mover_servo_incremento, args=(delta,), daemon=True).start()

            print(f"inputY: {cy_obj} | errorY: {error_y:+} px | PWM: {pulso_actual} µs")

            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)

        cv2.rectangle(frame, (cx_frame - 5, cy_frame - 5), (cx_frame + 5, cy_frame + 5), (255, 0, 0), 2)
        cv2.imshow("Seguimiento Vertical Simple", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

# ----------------------------
# Ejecución principal
# ----------------------------
try:
    detectar_objeto_y_mover()
finally:
    lgpio.gpiochip_close(h)
    print("GPIO liberado.")
