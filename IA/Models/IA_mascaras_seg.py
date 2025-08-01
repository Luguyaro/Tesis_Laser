import cv2
import numpy as np
import joblib
import time

# --- Configuración del modelo y parámetros ---
MODEL_PATH = "hog_svm_model.pkl"
WIN_SIZE = (64, 128)

# Rango HSV (ajústalo según tu entorno/objeto)
LOWER_HSV = np.array([20, 30, 40])
UPPER_HSV = np.array([90, 255, 255])

# Inicializar el descriptor HOG
hog = cv2.HOGDescriptor(_winSize=WIN_SIZE,
                        _blockSize=(16, 16),
                        _blockStride=(8, 8),
                        _cellSize=(8, 8),
                        _nbins=9)

# Cargar modelo HOG + SVM entrenado
model = joblib.load(MODEL_PATH)

# Filtro HSV para eliminar fondo
def apply_hsv_mask(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, LOWER_HSV, UPPER_HSV)
    result = cv2.bitwise_and(img, img, mask=mask)
    return result

# Detección del objeto con mejor score
def detect_best_object(image, stride=8, scale_factor=1.25, threshold=1.5):
    best_score = -np.inf
    best_box = None
    h, w = image.shape[:2]
    scale = 1.0

    while True:
        resized = cv2.resize(image, (int(w / scale), int(h / scale)))
        if resized.shape[0] < WIN_SIZE[1] or resized.shape[1] < WIN_SIZE[0]:
            break

        for y in range(0, resized.shape[0] - WIN_SIZE[1], stride):
            for x in range(0, resized.shape[1] - WIN_SIZE[0], stride):
                patch = resized[y:y + WIN_SIZE[1], x:x + WIN_SIZE[0]]
                gray = cv2.cvtColor(patch, cv2.COLOR_BGR2GRAY)
                features = hog.compute(gray).reshape(1, -1)
                score = model.decision_function(features)[0]
                if score > threshold and score > best_score:
                    rx = int(x * scale)
                    ry = int(y * scale)
                    rw = int(WIN_SIZE[0] * scale)
                    rh = int(WIN_SIZE[1] * scale)
                    best_box = (rx, ry, rw, rh)
                    best_score = score
        scale *= scale_factor

    return best_box, best_score

# Captura de cámara
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("⚠️ No se pudo abrir la cámara.")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        break

    start_time = time.time()

    # Filtrar entorno con HSV
    filtered = apply_hsv_mask(frame)

    # Detección del objeto más confiable
    box, score = detect_best_object(filtered, stride=8, scale_factor=1.25, threshold=1.5)

    # Dibujar resultados si se encontró algo
    if box:
        x, y, w, h = box
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.putText(frame, f"Score: {score:.2f}", (x, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # Medir FPS
    elapsed = time.time() - start_time
    fps = 1 / elapsed if elapsed > 0 else 0
    cv2.putText(frame, f"FPS: {fps:.2f}", (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)

    cv2.imshow("HOG + SVM Real-Time Detection", frame)
    if cv2.waitKey(1) == 27:  # ESC para salir
        break

cap.release()
cv2.destroyAllWindows()