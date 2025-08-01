import tflite_runtime.interpreter as tflite
import numpy as np
from PIL import Image, ImageDraw, ImageFont

# Rutas
model_path = "/home/pi5/Downloads/Tesis_Laser/IA/Models/model_- 17 july 2025 10_04.tflite"
image_path = "/home/pi5/Downloads/Tesis_Laser/IA/Dataset_test/3.jpeg"
output_image_path = "resultado_inferencia.jpg"

# Cargar y preprocesar la imagen
image = Image.open(image_path).convert("RGB")
original_width, original_height = image.size  # 640 x 480
input_width, input_height = 640, 640

scale_x = original_width / input_width
scale_y = original_height / input_height

image_resized = image.resize((640, 640))
input_data = np.expand_dims(np.array(image_resized, dtype=np.float32), axis=0) / 255.0

# Cargar el modelo
interpreter = tflite.Interpreter(model_path=model_path)
interpreter.allocate_tensors()

# Obtener detalles de entrada/salida
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

# Realizar inferencia
interpreter.set_tensor(input_details[0]['index'], input_data)
interpreter.invoke()
output_data = interpreter.get_tensor(output_details[0]['index'])

# Procesar la salida
output_data = np.squeeze(output_data).transpose()  # (8400, 14)
output_data = output_data
# Filtrar por confianza
conf_threshold = 1500e-10
boxes = []
print(output_data)
for det in output_data:
    x, y, w, h = det[0:4]
    conf = det[4]
    class_probs = det[5:]
    class_id = np.argmax(class_probs)
    class_score = class_probs[class_id]
    #print(conf , class_score)
    if conf * class_score > conf_threshold:
        cx, cy = x * 640, y * 640
        bw, bh = w * 640, h * 640
        x1 = int(cx - bw / 2)
        y1 = int(cy - bh / 2)
        x2 = int(cx + bw / 2)
        y2 = int(cy + bh / 2)
        
        boxes.append((x1, y1, x2, y2, class_id, conf * class_score))

# Dibujar resultados
draw = ImageDraw.Draw(image)
font = ImageFont.load_default()
for box in boxes:
    x1, y1, x2, y2, class_id, score = box
    # 🔁 Reescalar a dimensiones originales
    x1 = int(x1 * scale_x)
    x2 = int(x2 * scale_x)
    y1 = int(y1 * scale_y)
    y2 = int(y2 * scale_y)

    draw.rectangle([x1, y1, x2, y2], outline="red", width=2)
    label = f"Clase {class_id}: {score:.2f}"
    draw.text((x1, y1 - 10), label, fill="red", font=font)
    #print(score)
# Guardar y mostrar
image.save(output_image_path)
image.show()
print(f"Se guardó la imagen con resultados en: {output_image_path}")
print(boxes)