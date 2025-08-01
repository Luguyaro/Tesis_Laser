# test/c_Tracking.py
import tflite_runtime.interpreter as tflite
import numpy as np
from PIL import Image
import cv2
import time

# Ruta del modelo (puedes pasarla por parámetro si deseas)
MODEL_PATH = "/home/pi5/Downloads/Tesis_Laser/IA/Models/model_- 17 july 2025 10_04(2).tflite"
CONF_THRESHOLD = 1500e-10
INPUT_SIZE = 640

# Centro de la imagen (constante)
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
CX_FRAME = FRAME_WIDTH // 2
CY_FRAME = FRAME_HEIGHT // 2

# Inicializar modelo
interpreter = tflite.Interpreter(model_path=MODEL_PATH)
interpreter.allocate_tensors()
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()
# Variables globales
angulo_pitch_actual = 90  # Estado inicial

# Ajustar el ángulo suavemente según error en Y
def ajustar_pitch_por_error(error_y, factor_escala=0.05):
    global angulo_pitch_actual

    delta = int(error_y * factor_escala)  # Escala de corrección
    nuevo_angulo = angulo_pitch_actual + delta
    nuevo_angulo = max(50, min(120, nuevo_angulo))  # Limita rango

    if nuevo_angulo != angulo_pitch_actual:
        mover_servo(nuevo_angulo)
        angulo_pitch_actual = nuevo_angulo

def iniciar_tracking():
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

    if not cap.isOpened():
        print("🚨 Error: No se pudo abrir la cámara.")
        return

    print("🎯 Iniciando tracking - Presiona 'q' para salir")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("❌ Error al capturar frame")
            break

        # Convertir a escala de grises directamente
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray_bgr = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)  # Para mostrarlo con colores

        # PIL para el modelo
        pil_image = Image.fromarray(gray)
        original_width, original_height = pil_image.size

        # Preprocesar
        image_resized = pil_image.resize((INPUT_SIZE, INPUT_SIZE))
        input_data = np.expand_dims(np.array(image_resized, dtype=np.float32), axis=0) / 255.0
        input_data = np.expand_dims(input_data, axis=-1)  # [1, H, W, 1] para gray

        if input_details[0]['shape'][-1] == 3:
            input_data = np.repeat(input_data, 3, axis=-1)  # Convertimos [1, H, W, 1] → [1, H, W, 3] si hace falta

        # Inferencia
        interpreter.set_tensor(input_details[0]['index'], input_data)
        interpreter.invoke()
        output_data = interpreter.get_tensor(output_details[0]['index'])

        # Procesar detecciones
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

        # Elegir detección más cercana al centro
        error_x = None
        error_y = None
        if boxes:
            #centro_min = min(boxes, key=lambda b: (b[6] - CX_FRAME)**2 + (b[7] - CY_FRAME)**2)
            #x1, y1, x2, y2, class_id, score, cx_obj, cy_obj = centro_min
            closest_box = min(boxes, key=lambda b: ( ( (b[0]+b[2])//2 - img_center_x )**2 + ( (b[1]+b[3])//2 - img_center_y )**2 ))
            x1, y1, x2, y2, class_id, score = closest_box
            cx = (x1 + x2) // 2
            cy = (y1 + y2) // 2
            # Calcular error
            error_x = cx - CX_FRAME
            error_y = cy - CY_FRAME

            # Dibujo
            cv2.rectangle(gray_bgr, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(gray_bgr, f"Clase {class_id} ({score:.2f})", (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(gray_bgr, f"ErrX:{error_x:+}  ErrY:{error_y:+}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            ajustar_pitch_por_error(error_y)
            print(f"[DETECCION] Objeto centrado en ({cx_obj}, {cy_obj}) → Error X: {error_x:+}, Y: {error_y:+}")

        else:
            print("🔎 Sin detecciones válidas")

        # Mostrar imagen
        cv2.rectangle(gray_bgr, (CX_FRAME - 5, CY_FRAME - 5), (CX_FRAME + 5, CY_FRAME + 5), (255, 0, 0), 2)
        cv2.imshow("Tracking Plaga", gray_bgr)

        # Salida
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
