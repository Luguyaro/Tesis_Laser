import cv2
import numpy as np
import onnxruntime as ort

# Configuración
model_path = 'model_- 15 july 2025 13_10.onnx'  # Reemplaza con la ruta a tu modelo ONNX
conf_threshold = 0.5  # Umbral de confianza
iou_threshold = 0.5  # Umbral para NMS

# Tus clases
classes = [
    "bee", 
    "bird", 
    "cartoon snail", 
    "head_worm", 
    "person", 
    "polilla", 
    "snail", 
    "worm", 
    "worm_curved"
]

# Colores para cada clase (puedes personalizarlos)
colors = np.random.uniform(0, 255, size=(len(classes), 3))

# Cargar el modelo ONNX
session = ort.InferenceSession(model_path, providers=['CUDAExecutionProvider', 'CPUExecutionProvider'])

# Obtener información de entrada
input_name = session.get_inputs()[0].name
input_shape = session.get_inputs()[0].shape
input_height, input_width = input_shape[2:]

def preprocess(image):
    # Redimensionar y normalizar la imagen
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    image = cv2.resize(image, (input_width, input_height))
    image = image / 255.0
    image = image.transpose(2, 0, 1)
    image = np.expand_dims(image, axis=0).astype(np.float32)
    return image

def postprocess(outputs, image_shape):
    # Procesar las salidas del modelo
    boxes = []
    confidences = []
    class_ids = []
    
    # Asume que la salida es compatible con YOLOv8 (ajusta según tu modelo)
    outputs = np.squeeze(outputs[0]).T
    
    # Filtrar por confianza
    scores = np.max(outputs[:, 4:], axis=1)
    outputs = outputs[scores > conf_threshold]
    scores = scores[scores > conf_threshold]
    
    if len(outputs) == 0:
        return [], [], []
    
    # Obtener las clases con mayor probabilidad
    class_ids = np.argmax(outputs[:, 4:], axis=1)
    
    # Obtener cajas
    boxes = outputs[:, :4]
    
    # Escalar las cajas al tamaño original de la imagen
    img_height, img_width = image_shape
    x_scale = img_width / input_width
    y_scale = img_height / input_height
    
    boxes[:, 0] = (boxes[:, 0] - boxes[:, 2] / 2) * x_scale  # x min
    boxes[:, 1] = (boxes[:, 1] - boxes[:, 3] / 2) * y_scale  # y min
    boxes[:, 2] = boxes[:, 2] * x_scale  # width
    boxes[:, 3] = boxes[:, 3] * y_scale  # height
    
    # Convertir a enteros
    boxes = boxes.astype(np.int32)
    
    return boxes, scores, class_ids

def draw_detections(image, boxes, scores, class_ids):
    for box, score, class_id in zip(boxes, scores, class_ids):
        x, y, w, h = box
        color = colors[class_id]
        
        # Dibujar rectángulo
        cv2.rectangle(image, (x, y), (x + w, y + h), color, 2)
        
        # Crear etiqueta con nombre de clase y confianza
        label = f"{classes[class_id]}: {score:.2f}"
        
        # Calcular posición del texto
        (label_width, label_height), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        
        # Dibujar fondo para el texto
        cv2.rectangle(image, (x, y - label_height - 10), (x + label_width, y), color, -1)
        
        # Dibujar texto
        cv2.putText(image, label, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)

def detect(image_path):
    # Leer imagen
    image = cv2.imread(image_path)
    if image is None:
        print(f"No se pudo cargar la imagen: {image_path}")
        return
    
    original_shape = image.shape[:2]
    
    # Preprocesar
    input_tensor = preprocess(image)
    
    # Inferencia
    outputs = session.run(None, {input_name: input_tensor})
    
    # Postprocesar
    boxes, scores, class_ids = postprocess(outputs, original_shape)
    
    # Dibujar detecciones
    if len(boxes) > 0:
        draw_detections(image, boxes, scores, class_ids)
    
    # Mostrar resultado
    cv2.imshow("Detecciones", image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    
    # Guardar resultado (opcional)
    output_path = image_path.replace(".jpg", "_detected.jpg")
    cv2.imwrite(output_path, image)
    print(f"Resultado guardado en: {output_path}")

# Ejemplo de uso
detect("/home/pi5/Downloads/Tesis_Laser/IA/Dataset_test/3.jpeg")  # Reemplaza con la ruta a tu imagen```