#source /home/pi5/Downloads/Tesis_Laser/venv_tesis/bin/activate
#pip install tflite-runtime opencv-python numpy# yolo_tflite_detector.pyimport numpy as npimport cv2
import numpy as np
import tflite_runtime.interpreter as tflite
import cv2

class YOLOv8TFLiteDetector:
    def __init__(self, model_path, conf_threshold=0.5):
        self.interpreter = tflite.Interpreter(model_path=model_path)
        self.interpreter.allocate_tensors()
        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()
        self.input_size = self.input_details[0]['shape'][1]
        self.conf_threshold = conf_threshold
        self.class_names = [
            'bee', 'bird', 'cartoon snail', 'person',
            'polilla', 'snail', 'worm', 'worm_curved'
        ]

    def preprocess(self, image):
        resized = cv2.resize(image, (self.input_size, self.input_size))
        input_data = resized.astype('float32') / 255.0
        input_data = input_data.reshape(1, self.input_size, self.input_size, 3)
        return input_data

    def detect(self, image):
        input_data = self.preprocess(image)
        self.interpreter.set_tensor(self.input_details[0]['index'], input_data)
        self.interpreter.invoke()

        output_data = self.interpreter.get_tensor(self.output_details[0]['index'])[0]  # shape: (12, 8400)

        boxes = []
        image_h, image_w = image.shape[:2]

        for i in range(output_data.shape[1]):
            cls_scores = output_data[4:, i]
            cls_id = cls_scores.argmax()
            score = cls_scores[cls_id]

            if score < self.conf_threshold:
                continue

            x_center, y_center, w, h = output_data[0, i], output_data[1, i], output_data[2, i], output_data[3, i]
            x1 = int((x_center - w / 2) * image_w)
            y1 = int((y_center - h / 2) * image_h)
            x2 = int((x_center + w / 2) * image_w)
            y2 = int((y_center + h / 2) * image_h)

            boxes.append([x1, y1, x2, y2, float(score), int(cls_id)])

        return boxes
