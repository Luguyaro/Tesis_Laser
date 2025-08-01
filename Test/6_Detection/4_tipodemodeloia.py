import tflite_runtime.interpreter as tflite
import numpy as np
from PIL import Image

# Cargar y preprocesar la imagen
image = Image.open("/home/pi5/Downloads/Tesis_Laser/IA/Dataset_test/1.jpg").convert("RGB")
image = image.resize((640, 640))  # Ajusta al tamaño que espera tu modelo
input_data = np.expand_dims(np.array(image, dtype=np.float32), axis=0)  # Añade dimensión batch

# Normaliza si es necesario (dependiendo del modelo)
input_data = input_data / 255.0

# Cargar el modelo
interpreter = tflite.Interpreter(model_path="/home/pi5/Downloads/Tesis_Laser/IA/Models/model_- 14 july 2025 19_28.tflite")
interpreter.allocate_tensors()

# Obtener detalles de entrada/salida
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

# Realizar inferencia
# (preprocesar tu imagen según lo que espera el modelo)
interpreter.set_tensor(input_details[0]['index'], input_data)
interpreter.invoke()
output_data = interpreter.get_tensor(output_details[0]['index'])

print("Output shape:", output_data.shape)
print("Output data:", output_data)

