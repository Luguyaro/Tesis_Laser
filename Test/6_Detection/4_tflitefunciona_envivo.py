import tflite_runtime.interpreter as tflite
import numpy as np
from PIL import Image
import cv2
import time

# Configuración
model_path = "/home/pi5/Downloads/Tesis_Laser/IA/Models/model_- 17 july 2025 10_04(2).tflite"
conf_threshold = 1500e-10
input_size = 640

# Filtro a aplicar: "none", "gray", "gaussian", "canny", "bilateral"
selected_filter = "gray"

def apply_filter(image, filter_type):
    if filter_type == "gray":
        return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    elif filter_type == "gaussian":
        return cv2.GaussianBlur(image, (5, 5), 0)
    elif filter_type == "canny":
        return cv2.Canny(image, 100, 200)
    elif filter_type == "bilateral":
        return cv2.bilateralFilter(image, 9, 75, 75)
    else:
        return image

# Cargar modelo
interpreter = tflite.Interpreter(model_path=model_path)
interpreter.allocate_tensors()
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

# Inicializar cámara
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

if not cap.isOpened():
    print("Error: No se pudo abrir la cámara")
    exit()

# Variables para FPS
frame_count = 0
start_time = time.time()
fps = 0

print("Cámara lista - Detección en tiempo real")
print("Presiona 'q' para salir")

while True:
    ret, frame = cap.read()
    if not ret:
        print("Error al capturar frame")
        break

    # Rotación
    rotated_frame = cv2.rotate(frame.copy(), cv2.ROTATE_90_CLOCKWISE)
    rotated_frame = cv2.rotate(rotated_frame, cv2.ROTATE_90_CLOCKWISE)

    # Aplicar filtro
    filtered_frame = apply_filter(rotated_frame, selected_filter)

    # Si el filtro convierte a escala de grises o bordes, convertir a 3 canales
    if len(filtered_frame.shape) == 2:
        filtered_frame = cv2.cvtColor(filtered_frame, cv2.COLOR_GRAY2BGR)

    # Procesamiento
    pil_image = Image.fromarray(cv2.cvtColor(filtered_frame, cv2.COLOR_BGR2RGB))
    original_width, original_height = pil_image.size

    # Preprocesamiento
    image_resized = pil_image.resize((input_size, input_size))
    input_data = np.expand_dims(np.array(image_resized, dtype=np.float32), axis=0) / 255.0

    # Inferencia
    interpreter.set_tensor(input_details[0]['index'], input_data)
    interpreter.invoke()
    output_data = interpreter.get_tensor(output_details[0]['index'])

    # Procesamiento de salida
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
            x1 = int(cx - bw / 2)
            y1 = int(cy - bh / 2)
            x2 = int(cx + bw / 2)
            y2 = int(cy + bh / 2)

            # Reescalar
            x1 = int(x1 * (original_width / input_size))
            x2 = int(x2 * (original_width / input_size))
            y1 = int(y1 * (original_height / input_size))
            y2 = int(y2 * (original_height / input_size))

            boxes.append((x1, y1, x2, y2, class_id, total_conf))

    # Dibujar resultados
    for (x1, y1, x2, y2, class_id, score) in boxes:
        cv2.rectangle(rotated_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        label = f"Clase {class_id}: {score:.2f}"
        cv2.putText(rotated_frame, label, (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    # Calcular y mostrar FPS
    frame_count += 1
    if frame_count >= 10:
        fps = frame_count / (time.time() - start_time)
        frame_count = 0
        start_time = time.time()

    cv2.putText(rotated_frame, f"FPS: {fps:.1f}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    # Mostrar resultado
    cv2.imshow("Deteccion en Tiempo Real", rotated_frame)

    # Salir con 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        print("Programa finalizado")
        break

cap.release()
cv2.destroyAllWindows()

