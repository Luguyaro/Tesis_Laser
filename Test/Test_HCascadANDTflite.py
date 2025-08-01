import cv2
import numpy as np
import os
from tflite_runtime.interpreter import Interpreter
from tkinter import Tk, filedialog

# ------------------------------
# CONFIGURACIONES
# ------------------------------
CASCADE_PATH = "/home/pi5/Downloads/Tesis_Laser/IA/Models/cascade.xml"
TFLITE_MODEL_PATH = "/home/pi5/Downloads/Tesis_Laser/IA/Models/model_- 17 july 2025 10_04.tflite"
LABEL = "gusano"
CONF_THRESHOLD = 0.7
INPUT_SIZE = (640, 480)

# ------------------------------
# FUNCIÓN PARA EXTRAER CARACTERÍSTICAS
# ------------------------------
def extraer_caracteristicas(img):
    try:
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        h, s, _ = cv2.split(img_hsv)

        dom_h = np.median(h) / 180.0
        dom_s = np.median(s) / 255.0

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return None

        c = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(c)
        perimeter = cv2.arcLength(c, True)
        circularity = 4 * np.pi * area / (perimeter ** 2 + 1e-6)
        orientation = cv2.fitEllipse(c)[2] if len(c) >= 5 else 0
        x, y, w, h = cv2.boundingRect(c)
        aspect_ratio = w / h if h != 0 else 0

        hist_h = cv2.calcHist([img_hsv], [0], None, [16], [0, 180])
        hist_s = cv2.calcHist([img_hsv], [1], None, [16], [0, 256])
        hist_h = cv2.normalize(hist_h, hist_h).flatten()
        hist_s = cv2.normalize(hist_s, hist_s).flatten()
        hist_hs = np.concatenate([hist_h, hist_s])

        return np.array([dom_h, dom_s, area, perimeter, circularity, orientation, aspect_ratio, *hist_hs], dtype=np.float32)
    except Exception as e:
        print(f"⚠️ Error extrayendo características: {e}")
        return None

# ------------------------------
# CARGAR MODELO TFLITE
# ------------------------------
try:
    interpreter = Interpreter(model_path=TFLITE_MODEL_PATH)
    interpreter.allocate_tensors()
    input_details = interpreter.get_input_details()
    output_details = interpreter.get_output_details()
except Exception as e:
    print(f"❌ Error cargando modelo TFLite: {e}")
    exit(1)

# ------------------------------
# SELECCIÓN DE IMAGEN
# ------------------------------
Tk().withdraw()
ruta_imagen = filedialog.askopenfilename(initialdir="/home/pi5/Downloads/Tesis_Laser/dataset/worm",
                                         title="Selecciona una imagen",
                                         filetypes=[("Imágenes", "*.jpg *.png *.jpeg")])
if not ruta_imagen:
    print("❌ Imagen no seleccionada.")
    exit(1)

imagen_original = cv2.imread(ruta_imagen)
if imagen_original is None:
    print("❌ No se pudo leer la imagen.")
    exit(1)

imagen = cv2.resize(imagen_original, INPUT_SIZE)

# ------------------------------
# CARGAR CLASIFICADOR HAAR CASCADE
# ------------------------------
detector = cv2.CascadeClassifier(CASCADE_PATH)
if detector.empty():
    print("❌ No se pudo cargar cascade.xml")
    exit(1)

# ------------------------------
# DETECCIÓN HAAR + FILTRO MLP
# ------------------------------
objetos = detector.detectMultiScale(
    imagen,
    scaleFactor=1.1,
    minNeighbors=1,
    minSize=(20, 20),
    maxSize=(150, 150)
)

imagen_out = imagen.copy()

for (x, y, w, h) in objetos:
    roi = imagen[y:y+h, x:x+w]
    features = extraer_caracteristicas(roi)

    if features is None:
        continue

    try:
        input_data = np.expand_dims(features.astype(np.float32), axis=0)
        interpreter.set_tensor(input_details[0]['index'], input_data)
        interpreter.invoke()
        output_data = interpreter.get_tensor(output_details[0]['index'])[0]

        score = float(np.max(output_data))
        label_index = int(np.argmax(output_data))

        if score >= CONF_THRESHOLD:
            cv2.rectangle(imagen_out, (x, y), (x + w, y + h), (0, 255, 0), 2)
            texto = f"{LABEL}: {score:.2f}"
            cv2.putText(imagen_out, texto, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    except Exception as e:
        print(f"⚠️ Error al ejecutar inferencia TFLite: {e}")

# ------------------------------
# MOSTRAR RESULTADOS
# ------------------------------
cv2.namedWindow("Detección Final", cv2.WINDOW_NORMAL)
cv2.imshow("Detección Final", imagen_out)
cv2.waitKey(0)
cv2.destroyAllWindows()
