import tflite_runtime.interpreter as tflite
import numpy as np
from PIL import Image
import cv2
import time

# Configuración
model_path = "/home/pi5/Downloads/Tesis_Laser/IA/Models/model_- 17 july 2025 10_04.tflite"
conf_threshold = 1500e-10
input_size = 640
capture_interval = 1.0  # Segundos entre capturas

# Cargar modelo
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

print("🟢 Cámara lista - Captura automática cada 1 segundo")
print("Presiona 'q' para salir")

last_capture_time = time.time()
result_window_open = False

while True:
    ret, frame = cap.read()
    if not ret:
        print("⚠️ Error al capturar frame")
        break
    
    # Mostrar vista previa (sin rotación)
    cv2.imshow("Vista Previa - Captura automatica", frame)
    
    # Captura automática cada segundo
    current_time = time.time()
    if current_time - last_capture_time >= capture_interval:
        # Rotación CORREGIDA (solo para procesamiento)
        rotated_frame = cv2.rotate(frame.copy(), cv2.ROTATE_90_CLOCKWISE)
        rotated_frame = cv2.rotate(rotated_frame, cv2.ROTATE_90_CLOCKWISE)
        
        # Procesamiento
        pil_image = Image.fromarray(cv2.cvtColor(rotated_frame, cv2.COLOR_BGR2RGB))
        original_width, original_height = pil_image.size
        
        # Preprocesamiento
        image_resized = pil_image.resize((input_size, input_size))
        input_data = np.expand_dims(np.array(image_resized, dtype=np.float32), axis=0) / 255.0
        
        # Inferencia
        interpreter.set_tensor(input_details[0]['index'], input_data)
        interpreter.invoke()
        output_data = interpreter.get_tensor(output_details[0]['index'])
        
        # Procesamiento de salida
        output_data = np.squeeze(output_data).transpose()
        boxes = []
        
        for det in output_data:
            x, y, w, h = det[0:4]
            conf = det[4]
            class_probs = det[5:]
            class_id = np.argmax(class_probs)
            class_score = class_probs[class_id]
            total_conf = conf * class_score
            
            if total_conf > conf_threshold:
                cx, cy = x * input_size, y * input_size
                bw, bh = w * input_size, h * input_size
                x1 = int(cx - bw/2)
                y1 = int(cy - bh/2)
                x2 = int(cx + bw/2)
                y2 = int(cy + bh/2)
                
                # Reescalar
                x1 = int(x1 * (original_width/input_size))
                x2 = int(x2 * (original_width/input_size))
                y1 = int(y1 * (original_height/input_size))
                y2 = int(y2 * (original_height/input_size))
                
                boxes.append((x1, y1, x2, y2, class_id, total_conf))
        
        # Dibujar resultados en el frame ROTADO
        result_frame = rotated_frame.copy()
        for (x1, y1, x2, y2, class_id, score) in boxes:
            cv2.rectangle(result_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            label = f"Clase {class_id}: {score:.2f}"
            cv2.putText(result_frame, label, (x1, y1-10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
        
        # Rotar de vuelta para visualización
        display_frame = cv2.rotate(result_frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
        display_frame = cv2.rotate(display_frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
        
        # Mostrar y guardar resultados
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        output_filename = f"detection_{timestamp}.jpg"
        cv2.imwrite(output_filename, display_frame)
        
        if result_window_open:
            cv2.destroyWindow("Resultado de Detección")
        cv2.imshow("Resultado de Deteccion", display_frame)
        result_window_open = True
        
        print(f"📸 Captura procesada - {len(boxes)} detecciones - Guardado como {output_filename}")
        last_capture_time = current_time
    
    # Salir con 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        print("🛑 Programa finalizado")
        break

cap.release()
cv2.destroyAllWindows()