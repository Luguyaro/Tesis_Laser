import tflite_runtime.interpreter as tflite
import numpy as np
from PIL import Image
import cv2

class Camara:
    def __init__(self, model_path, input_size=640, conf_threshold=1500e-10, frame_width=640, frame_height=480):
        self.input_size = input_size
        self.conf_threshold = conf_threshold
        self.frame_width = frame_width
        self.frame_height = frame_height
        self.cx_frame = frame_width // 2
        self.cy_frame = frame_height // 2

        # Inicializar modelo TFLite
        self.interpreter = tflite.Interpreter(model_path=model_path)
        self.interpreter.allocate_tensors()
        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()

        # Inicializar cámara
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)

        if not self.cap.isOpened():
            raise RuntimeError("❌ No se pudo abrir la cámara.")

    def obtener_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            return None, None

        frame = cv2.rotate(frame, cv2.ROTATE_180)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray_bgr = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        return gray, gray_bgr

    def detectar_objeto(self, gray):
        pil_image = Image.fromarray(gray)
        original_width, original_height = pil_image.size

        image_resized = pil_image.resize((self.input_size, self.input_size))
        input_data = np.expand_dims(np.array(image_resized, dtype=np.float32), axis=0) / 255.0
        input_data = np.expand_dims(input_data, axis=-1)

        if self.input_details[0]['shape'][-1] == 3:
            input_data = np.repeat(input_data, 3, axis=-1)

        self.interpreter.set_tensor(self.input_details[0]['index'], input_data)
        self.interpreter.invoke()
        output_data = self.interpreter.get_tensor(self.output_details[0]['index'])
        output_data = np.squeeze(output_data).transpose()

        boxes = []
        for det in output_data:
            x, y, w, h = det[0:4]
            conf = det[4]
            class_probs = det[5:]
            class_id = np.argmax(class_probs)
            class_score = class_probs[class_id]
            total_conf = conf * class_score

            if total_conf > self.conf_threshold:
                cx, cy = x * self.input_size, y * self.input_size
                bw, bh = w * self.input_size, h * self.input_size
                x1 = int((cx - bw / 2) * (original_width / self.input_size))
                y1 = int((cy - bh / 2) * (original_height / self.input_size))
                x2 = int((cx + bw / 2) * (original_width / self.input_size))
                y2 = int((cy + bh / 2) * (original_height / self.input_size))
                cx_rescaled = int((x1 + x2) / 2)
                cy_rescaled = int((y1 + y2) / 2)
                boxes.append((x1, y1, x2, y2, class_id, total_conf, cx_rescaled, cy_rescaled))
        return boxes

    def liberar(self):
        self.cap.release()
        cv2.destroyAllWindows()
