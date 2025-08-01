import cv2
import numpy as np
import tflite_runtime.interpreter as tflite
from pathlib import Path
from tkinter import Tk
from tkinter.filedialog import askopenfilename

# Configuración
BASE_PATH = Path("/home/pi5/Downloads/Tesis_Laser/IA/")
MODEL_PATH = BASE_PATH / "Models/model_- 15 july 2025 13_10.tflite"
LABELS_PATH = BASE_PATH / "Models/labels.txt"
INPUT_SIZE = 640
CONF_THRESHOLD = 0.25

# Función para seleccionar imagen
def seleccionar_imagen():
    root = Tk()
    root.withdraw()
    path = askopenfilename(
        initialdir=str(BASE_PATH / "Dataset_test"),
        title="Selecciona una imagen",
        filetypes=[("Imagenes", "*.jpg *.jpeg *.png")]
    )
    return path

# Función para cargar labels
def cargar_labels(path):
    try:
        with open(path, "r") as f:
            return [line.strip() for line in f.readlines()]
    except Exception as e:
        print("Error cargando labels:", e)
        return []

# Carga modelo e imprime estructura
def diagnostico_modelo():
    img_path = seleccionar_imagen()
    if not img_path:
        print("Imagen no seleccionada")
        return

    imagen = cv2.imread(img_path)
    if imagen is None:
        print("Error al leer la imagen.")
        return

    print(f"\n📁 Imagen seleccionada: {img_path}")
    print(f"🖼️ Dimensiones originales: {imagen.shape}")

    # Cargar modelo
    interpreter = tflite.Interpreter(model_path=str(MODEL_PATH))
    interpreter.allocate_tensors()

    input_details = interpreter.get_input_details()
    output_details = interpreter.get_output_details()

    print("\n=== DETALLES DE ENTRADA ===")
    for inp in input_details:
        print(f"Nombre: {inp['name']}")
        print(f"Forma esperada: {inp['shape']}")
        print(f"Tipo: {inp['dtype']}")
    print("\n=== DETALLES DE SALIDA ===")
    for out in output_details:
        print(f"Nombre: {out['name']}")
        print(f"Forma: {out['shape']}")
        print(f"Tipo: {out['dtype']}")

    # Preprocesar imagen
    imagen_resized = cv2.resize(imagen, (INPUT_SIZE, INPUT_SIZE))
    imagen_rgb = cv2.cvtColor(imagen_resized, cv2.COLOR_BGR2RGB)
    input_tensor = np.expand_dims(imagen_rgb, axis=0).astype(np.float32) / 255.0

    # Inferencia
    interpreter.set_tensor(input_details[0]['index'], input_tensor)
    interpreter.invoke()
    output = interpreter.get_tensor(output_details[0]['index'])

    print(f"\nForma de salida del modelo: {output.shape}")
    if output.shape[1] == 14:
        output_data = output[0].T
    elif output.shape[2] == 14:
        output_data = output[0]
    else:
        print("⚠️ Formato no reconocido.")
        return

    # Diagnóstico de predicciones
    print("\n=== PREDICCIONES (Top 5 validas por confianza) ===")
    count_validas = 0
    labels = cargar_labels(LABELS_PATH)

    for i, pred in enumerate(output_data[:50]):  # Solo primeras 50 por tiempo
        x, y, w, h, obj_conf = pred[:5]
        class_scores = pred[5:]

        if np.max(class_scores) > 1.0 or np.sum(class_scores) > 1.0:
            class_scores = np.exp(class_scores)
            class_scores /= np.sum(class_scores)

        class_id = int(np.argmax(class_scores))
        class_score = class_scores[class_id]
        total_score = obj_conf * class_score

        if total_score > CONF_THRESHOLD:
            count_validas += 1
            label = labels[class_id] if class_id < len(labels) else f"ID {class_id}"
            print(f"\n🔹 Pred #{i}")
            print(f"Clase: {label}")
            print(f"Confianza objeto: {obj_conf:.4f}")
            print(f"Score clase: {class_score:.4f}")
            print(f"Score total: {total_score:.4f}")
            print(f"Bounding box normalizado (cx, cy, w, h): {x:.2f}, {y:.2f}, {w:.2f}, {h:.2f}")

    print(f"\n✅ Detecciones validas encontradas: {count_validas}")
    if count_validas == 0:
        print("⚠️ No se encontraron detecciones sobre el umbral.")

if __name__ == "__main__":
    diagnostico_modelo()
