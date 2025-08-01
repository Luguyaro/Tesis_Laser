import cv2
import numpy as np
from tflite_runtime.interpreter import Interpreter

# 📁 Ruta al modelo
TFLITE_MODEL_PATH = "/home/pi5/Downloads/Tesis_Laser/IA/Models/model_- 17 july 2025 10_04(1).tflite"
#TFLITE_MODEL_PATH = "/home/pi5/Downloads/Tesis_Laser/IA/Models/model_- 14 july 2025 19_28.tflite"
# 🧠 Inicializa el modelo TFLite
try:
    interpreter = Interpreter(model_path=TFLITE_MODEL_PATH)
    interpreter.allocate_tensors()
except Exception as e:
    print("❌ Error al cargar el modelo TFLite:", e)
    exit(1)

# 🔍 Input/output info
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()
input_shape = input_details[0]['shape']
print("🔎 Input shape:", input_details[0]['shape'])  # Por ejemplo: [1, 640, 640, 3]
print("📐 Input dtype:", input_details[0]['dtype'])
print("🔎 Output shape:", output_details[0]['shape'])  # Por ejemplo: [1, 25200, 14]
print("📐 Output dtype:", output_details[0]['dtype'])
output = interpreter.get_tensor(output_details[0]['index'])
print("🧪 Raw output shape:", output.shape)
output = output.T  # Transponer

# 🎛️ Configuración
CONF_THRESHOLD = 15000000e-10
LABELS = ["worm", "people"]  # Personaliza según tus clases
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

# ⚙️ Preprocesamiento
def preprocess(frame):
    resized = cv2.resize(frame, (input_shape[2], input_shape[1]))
    rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
    tensor = np.expand_dims(rgb, axis=0).astype(np.float32) / 255.0
    return tensor

# 🎥 Webcam
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("❌ No se detecta cámara conectada.")
    exit(1)

cv2.namedWindow("TFLite Detection", cv2.WINDOW_NORMAL)
cv2.resizeWindow("TFLite Detection", FRAME_WIDTH, FRAME_HEIGHT)

print("📡 Iniciando deteccion en tiempo real. Presiona 'q' para salir.")
while True:
    ret, frame = cap.read()
    if not ret:
        print("⚠️ Frame no recibido.")
        break

    input_tensor = preprocess(frame)
    interpreter.set_tensor(input_details[0]['index'], input_tensor)
    interpreter.invoke()
    output_data = interpreter.get_tensor(output_details[0]['index'])[0]

    # 🧠 Detección con bounding boxes
    boxes = []
    muestra_output= output_data[0:4]
    print(f"Paquete de datos: {muestra_output}")
    print(f"Los 4 primeros: {muestra_output[0:4]}")
    for det in output_data:
        x, y, w, h = det[0:4]
        conf = det[4]
        class_probs = det[5:]
        class_id = int(np.argmax(class_probs))
        class_score = class_probs[class_id]
        total_conf = conf * class_score

        if total_conf > CONF_THRESHOLD:
            cx, cy = x * FRAME_WIDTH*4, y * FRAME_WIDTH*4
            bw, bh = w * FRAME_WIDTH*4, h * FRAME_WIDTH*4
            x1 = int(cx - bw / 2)
            y1 = int(cy - bh / 2)
            x2 = int(cx + bw / 2)
            y2 = int(cy + bh / 2)
            boxes.append((x1, y1, x2, y2, class_id, total_conf))

    # 🖼️ Dibujar resultados
    for (x1, y1, x2, y2, class_id, score) in boxes:
        label = LABELS[class_id] if class_id < len(LABELS) else f"cls_{class_id}"
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(frame, f"{label} ({score:.2f})", (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    cv2.imshow("TFLite Detection", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        print("🛑 Detección finalizada por el usuario.")
        break
print(boxes)
cap.release()
cv2.destroyAllWindows()
