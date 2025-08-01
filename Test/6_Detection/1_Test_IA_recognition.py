import cv2
import numpy as np
import tflite_runtime.interpreter as tflite
from pathlib import Path
from tkinter import Tk
from tkinter.filedialog import askopenfilename

# -------------------------
# CONFIGURACION
# -------------------------
BASE_PATH = Path("/home/pi5/Downloads/Tesis_Laser/IA/")
MODEL_PATH = BASE_PATH / "Models/model_- 14 july 2025 19_28.tflite"
LABELS_PATH = BASE_PATH / "Models/labels.txt"
INPUT_SIZE = 640
CONF_THRESHOLD = 0.25
IOU_THRESHOLD = 0.45

# -------------------------
# FUNCIONES
# -------------------------
def cargar_labels(path):
    try:
        with open(path, "r") as f:
            return [line.strip() for line in f.readlines()]
    except Exception as e:
        print("Error cargando labels:", e)
        return []

def cargar_interprete(path):
    interpreter = tflite.Interpreter(model_path=str(path))
    interpreter.allocate_tensors()
    return interpreter

def preprocesar(imagen, size):
    imagen_resized = cv2.resize(imagen, (size, size))
    imagen_rgb = cv2.cvtColor(imagen_resized, cv2.COLOR_BGR2RGB)
    return np.expand_dims(imagen_rgb, axis=0).astype(np.float32) / 255.0

def aplicar_nms(detecciones, iou_thresh):
    boxes = [d[:4] for d in detecciones]
    scores = [d[4] for d in detecciones]
    indices = cv2.dnn.NMSBoxes(boxes, scores, score_threshold=0.0, nms_threshold=iou_thresh)
    return [detecciones[i[0]] for i in indices] if len(indices) > 0 else []

def seleccionar_imagen():
    root = Tk()
    root.withdraw()
    path = askopenfilename(
        initialdir=str(BASE_PATH / "Dataset_test"),
        title="Selecciona una imagen",
        filetypes=[("Imagenes", "*.jpg *.jpeg *.png")]
    )
    return path

# -------------------------
# DETECCION
# -------------------------
def detectar_en_imagen():
    img_path = seleccionar_imagen()
    if not img_path:
        print("No se selecciono ninguna imagen.")
        return

    imagen = cv2.imread(img_path)
    if imagen is None:
        print("Error al cargar la imagen.")
        return

    ORIG_H, ORIG_W = imagen.shape[:2]
    labels = cargar_labels(LABELS_PATH)
    interpreter = cargar_interprete(MODEL_PATH)
    input_details = interpreter.get_input_details()
    output_details = interpreter.get_output_details()

    # Preprocesar imagen
    input_tensor = preprocesar(imagen, INPUT_SIZE)
    interpreter.set_tensor(input_details[0]['index'], input_tensor)
    interpreter.invoke()
    output = interpreter.get_tensor(output_details[0]['index'])

    # Detectar forma y corregir
    print("Forma salida modelo:", output.shape)

    if output.shape[1] == 14:
        output_data = output[0].T  # (8400, 14)
    elif output.shape[2] == 14:
        output_data = output[0]    # (8400, 14)
    else:
        raise ValueError("Formato de salida no reconocido: " + str(output.shape))

    detecciones = []
    for pred in output_data:
        x_center, y_center, width, height, obj_conf = pred[:5]
        class_scores = pred[5:]

        # Softmax manual si es necesario
        if np.max(class_scores) > 1.0 or np.sum(class_scores) > 1.0:
            class_scores = np.exp(class_scores)
            class_scores /= np.sum(class_scores)

        class_id = int(np.argmax(class_scores))
        class_score = class_scores[class_id]
        total_conf = obj_conf * class_score

        if total_conf < CONF_THRESHOLD:
            continue

        # Reescalar a tamaño original
        x_center *= ORIG_W
        y_center *= ORIG_H
        width *= ORIG_W
        height *= ORIG_H

        x1 = int(x_center - width / 2)
        y1 = int(y_center - height / 2)
        x2 = int(x_center + width / 2)
        y2 = int(y_center + height / 2)

        detecciones.append([x1, y1, x2, y2, total_conf, class_id])

    final_detecciones = aplicar_nms(detecciones, IOU_THRESHOLD)

    for det in final_detecciones:
        x1, y1, x2, y2, score, class_id = det
        label = labels[class_id] if class_id < len(labels) else f"ID {class_id}"
        texto = f"{label} ({score:.2f})"

        cv2.rectangle(imagen, (x1, y1), (x2, y2), (0, 255, 0), 2)
        (tw, th), _ = cv2.getTextSize(texto, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        cv2.rectangle(imagen, (x1, y1 - th - 4), (x1 + tw, y1), (0, 255, 0), -1)
        cv2.putText(imagen, texto, (x1, y1 - 2), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

    cv2.imshow("Resultado deteccion", imagen)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

# -------------------------
# MAIN
# -------------------------
if __name__ == "__main__":
    detectar_en_imagen()
