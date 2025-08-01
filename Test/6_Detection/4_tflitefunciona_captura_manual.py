import tflite_runtime.interpreter as tflite
import numpy as np
from PIL import Image, ImageDraw, ImageFont
import cv2
# Configuración (igual que tu código original)
model_path = "/home/pi5/Downloads/Tesis_Laser/IA/Models/model_- 17 july 2025 10_04.tflite"
conf_threshold = 1500e-10
input_size = 640  # Tamaño de entrada del modelo
# Cargar modelo (igual que tu código)
interpreter = tflite.Interpreter(model_path=model_path)
interpreter.allocate_tensors()
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()
# Inicializar cámara
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
if not cap.isOpened():
    print("❌ No se pudo abrir la cámara")
    exit()

# Configurar resolución (ajustar según tu cámara)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

print("🟢 Cámara lista")
print("Presiona:")
print("- ESPACIO para capturar un frame")
print("- 'q' para salir")

while True:
    # Capturar frame
    ret, frame = cap.read()
    if not ret:
        print("⚠️ Error al capturar frame")
        break
    frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
    frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
    # Mostrar vista previa
    cv2.imshow("Vista Previa - Presiona ESPACIO para capturar", frame)
    
    # Esperar tecla
    key = cv2.waitKey(1)
    
    if key == ord('q'):
        print("🛑 Programa finalizado")
        break
    elif key == 32:  # Tecla ESPACIO
        # Guardar frame capturado
        captured_frame = frame.copy()
        print("📸 Frame capturado!")
        print("💾 Frame guardado como 'frame_capturado.jpg'")
        pil_image = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        original_width, original_height = pil_image.size
        
        # Calcular factores de escala (igual que tu código)
        scale_x = original_width / input_size
        scale_y = original_height / input_size
        
        # Preprocesamiento (idéntico a tu código)
        image_resized = pil_image.resize((input_size, input_size))
        input_data = np.expand_dims(np.array(image_resized, dtype=np.float32), axis=0) / 255.0
        
        # Inferencia (igual que tu código)
        interpreter.set_tensor(input_details[0]['index'], input_data)
        interpreter.invoke()
        output_data = interpreter.get_tensor(output_details[0]['index'])
        # Procesamiento de salida (idéntico a tu código)
        output_data = np.squeeze(output_data).transpose()  # (8400, 14)
        boxes = []
        
        for det in output_data:
            x, y, w, h = det[0:4]
            conf = det[4]
            class_probs = det[5:]
            class_id = np.argmax(class_probs)
            class_score = class_probs[class_id]
            
            if conf * class_score > conf_threshold:
                cx, cy = x * input_size, y * input_size
                bw, bh = w * input_size, h * input_size
                x1 = int(cx - bw / 2)
                y1 = int(cy - bh / 2)
                x2 = int(cx + bw / 2)
                y2 = int(cy + bh / 2)
                
                # Reescalar (igual que tu código)
                x1 = int(x1 * scale_x)
                x2 = int(x2 * scale_x)
                y1 = int(y1 * scale_y)
                y2 = int(y2 * scale_y)
                
                boxes.append((x1, y1, x2, y2, class_id, conf * class_score))
        #Dibujar los boxes
        for (x1, y1, x2, y2, class_id, score) in boxes:
            # Dibujar rectángulo
            cv2.rectangle(captured_frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
            # Mostrar etiqueta
            label = f"Clase {class_id}: {score:.2f}"
            cv2.putText(captured_frame, label, (x1, y1-10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2)
        # Mostrar frame capturado en nueva ventana
        cv2.imshow("Frame Capturado", captured_frame)
        # Guardar a archivo para verificación
        cv2.imwrite("frame_capturado.jpg", captured_frame)
        print("Coordenadas:", boxes)
# Liberar recursos
cap.release()
cv2.destroyAllWindows()