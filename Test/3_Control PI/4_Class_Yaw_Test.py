import tflite_runtime.interpreter as tflite
import numpy as np
from PIL import Image
import cv2
import time
from class_Yaw import Yaw

# Ruta del modelo
MODEL_PATH = "/home/pi5/Downloads/Tesis_Laser/IA/Models/model_- 17 july 2025 10_04(2).tflite"
CONF_THRESHOLD = 1500e-10
INPUT_SIZE = 640

# Centro de la imagen
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
CX_FRAME = FRAME_WIDTH // 2
CY_FRAME = FRAME_HEIGHT // 2

# Inicializar modelo
interpreter = tflite.Interpreter(model_path=MODEL_PATH)
interpreter.allocate_tensors()
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

# Inicializar motor Yaw
yaw_motor = Yaw()

print("✅ Sistema en línea. Presiona 'q' para salir.")

texto_input = ""
font = cv2.FONT_HERSHEY_SIMPLEX

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

if not cap.isOpened():
    print("🚨 Error: No se pudo abrir la cámara.")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("❌ Error al capturar frame")
        break

    # Rotar 180° y convertir a escala de grises
    frame = cv2.rotate(frame, cv2.ROTATE_180)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray_bgr = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

    pil_image = Image.fromarray(gray)
    original_width, original_height = pil_image.size

    image_resized = pil_image.resize((INPUT_SIZE, INPUT_SIZE))
    input_data = np.expand_dims(np.array(image_resized, dtype=np.float32), axis=0) / 255.0
    input_data = np.expand_dims(input_data, axis=-1)

    if input_details[0]['shape'][-1] == 3:
        input_data = np.repeat(input_data, 3, axis=-1)

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

        if total_conf > CONF_THRESHOLD:
            cx, cy = x * INPUT_SIZE, y * INPUT_SIZE
            bw, bh = w * INPUT_SIZE, h * INPUT_SIZE
            x1 = int((cx - bw / 2) * (original_width / INPUT_SIZE))
            y1 = int((cy - bh / 2) * (original_height / INPUT_SIZE))
            x2 = int((cx + bw / 2) * (original_width / INPUT_SIZE))
            y2 = int((cy + bh / 2) * (original_height / INPUT_SIZE))
            cx_rescaled = int((x1 + x2) / 2)
            cy_rescaled = int((y1 + y2) / 2)

            boxes.append((x1, y1, x2, y2, class_id, total_conf, cx_rescaled, cy_rescaled))

    if boxes:
        best = min(boxes, key=lambda b: (b[6] - CX_FRAME) ** 2 + (b[7] - CY_FRAME) ** 2)
        x1, y1, x2, y2, class_id, score, cx_obj, cy_obj = best
        error_x = cx_obj - CX_FRAME
        error_y = cy_obj - CY_FRAME

        cv2.rectangle(gray_bgr, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(gray_bgr, f"Clase {class_id} ({score:.2f})", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(gray_bgr, f"ErrX:{error_x:+}  ErrY:{error_y:+}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

    else:
        error_x = 0
        error_y = 0
        print("🔎 Sin detecciones válidas")

    # Mostrar barra de entrada inferior
    cv2.rectangle(gray_bgr, (10, FRAME_HEIGHT - 40), (250, FRAME_HEIGHT - 10), (50, 50, 50), -1)
    cv2.putText(gray_bgr, f"Pixel input: {texto_input}", (15, FRAME_HEIGHT - 15), font, 0.6, (255, 255, 255), 1)

    # Mostrar centro
    cv2.rectangle(gray_bgr, (CX_FRAME - 5, CY_FRAME - 5), (CX_FRAME + 5, CY_FRAME + 5), (255, 0, 0), 2)
    cv2.imshow("Tracking Plaga", gray_bgr)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    elif key == 13:  # Enter
        try:
            pix = int(texto_input.strip() or "0")
            pasos = yaw_motor.pixeles_a_pasos(pix)
            grados = pasos / yaw_motor.PASOS_POR_GRADO
            print(f"🔁 Corrigiendo {pix} px -> {pasos:.2f} pasos -> {grados:.2f}°")
            yaw_motor.mover(grados)
            time.sleep(0.5)
        except:
            print("❌ Entrada inválida.")
        texto_input = ""
    elif key == 8:  # Backspace
        texto_input = texto_input[:-1]
    elif 48 <= key <= 57 or key == 45:  # números y signo -
        texto_input += chr(key)

cap.release()
yaw_motor.liberar_gpio()
cv2.destroyAllWindows()
