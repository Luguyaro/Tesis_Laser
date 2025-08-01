import cv2
import time
from yolo_tflite_detector import YOLOv8TFLiteDetector

# -----------------------------
# CONFIGURACION MANUAL
# -----------------------------
# Resolucion (elige segun tu camara y potencia)
CAMERA_WIDTH = 320 #640
CAMERA_HEIGHT = 240 #480

# Modelo TFLite
MODEL_PATH = "/home/pi5/Downloads/Tesis_Laser/IA/Models/model_- 28 june 2025 18_49.tflite"

# -----------------------------
# Inicializar detector
# -----------------------------
detector = YOLOv8TFLiteDetector(MODEL_PATH, conf_threshold=0.5)

# -----------------------------
# Inicializar camara
# -----------------------------
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)

if not cap.isOpened():
    print("No se pudo abrir la camara")
    exit()

# -----------------------------
# Bucle principal
# -----------------------------
print("Presiona 'q' para salir")
frame_counter = 0
start_time = time.time()

while True:
    ret, frame = cap.read()
    if not ret:
        print("Error al capturar frame")
        break

    detections = detector.detect(frame)

    for det in detections:
        x1, y1, x2, y2, score, cls_id = det
        label = f"{detector.class_names[int(cls_id)]} {score:.2f}"
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(frame, label, (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

    # Dibujar recuadro central 10x10
    h, w, _ = frame.shape
    cx, cy = w // 2, h // 2
    cv2.rectangle(frame, (cx - 5, cy - 5), (cx + 5, cy + 5), (255, 0, 0), 2)

    cv2.imshow("Deteccion en tiempo real", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    frame_counter += 1
    if frame_counter % 10 == 0:
        fps = frame_counter / (time.time() - start_time)
        print(f"FPS promedio: {fps:.2f}")

cap.release()
cv2.destroyAllWindows()
