import cv2
import time
import os

# -------------------------------
# CONFIGURACIÓN MANUAL
# -------------------------------

CASCADE_PATH = "/home/pi5/Downloads/Tesis_Laser/IA/Models/haarcascade_frontalface_default.xml"  # Asegúrate que este archivo existe y es válido

SCALE_FACTOR = 1.2      # Escala para detección (más bajo = más sensible)
MIN_NEIGHBORS = 2       # Umbral de decisión del Haar Cascade (más alto = más estricto)
MIN_SIZE = (30, 30)     # Tamaño mínimo del objeto detectado (ajustable)
MAX_SIZE = (200, 200)   # Tamaño máximo del objeto detectado (ajustable)

CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480

# -------------------------------
# VERIFICACIÓN DEL MODELO
# -------------------------------

if not os.path.exists(CASCADE_PATH):
    raise FileNotFoundError(f"❌ Archivo Haar Cascade no encontrado en: {CASCADE_PATH}")

face_cascade = cv2.CascadeClassifier(CASCADE_PATH)

if face_cascade.empty():
    raise IOError(f"❌ El archivo '{CASCADE_PATH}' no es un clasificador Haar válido o está dañado.")

# -------------------------------
# INICIALIZAR CÁMARA
# -------------------------------

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)

if not cap.isOpened():
    raise IOError("❌ No se pudo abrir la cámara.")

print("Cmara lista. Presiona 'q' para salir.")

# -------------------------------
# BUCLE PRINCIPAL
# -------------------------------

frame_counter = 0
start_time = time.time()

while True:
    ret, frame = cap.read()
    if not ret:
        print("⚠️ Error al capturar frame")
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    imagen_1 = cv2.rotate(frame.copy(), cv2.ROTATE_90_CLOCKWISE)
    imagen_1 = cv2.rotate(imagen_1, cv2.ROTATE_90_CLOCKWISE)
    # 🔍 Detección con parámetros ajustables
    faces = face_cascade.detectMultiScale(
        gray,
        scaleFactor=SCALE_FACTOR,
        minNeighbors=MIN_NEIGHBORS,
        minSize=MIN_SIZE,
        maxSize=MAX_SIZE
    )

    # 🔲 Dibujar resultados
    for (x, y, w, h) in faces:
        label = "worm"
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 128, 255), 2)
        cv2.putText(frame, label, (x, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 128, 255), 2)

    # 📊 FPS
    frame_counter += 1
    elapsed = time.time() - start_time
    fps = frame_counter / elapsed if elapsed > 0 else 0
    cv2.putText(frame, f"FPS: {fps:.2f}", (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    # 🖼️ Mostrar frame
    cv2.imshow("Deteccin Haar Cascade", imagen_1)

    # Salir con 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# -------------------------------
# LIMPIEZA
# -------------------------------

cap.release()
cv2.destroyAllWindows()
