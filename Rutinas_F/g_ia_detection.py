# g_ia_detection.py
from tflite_runtime.interpreter import Interpreter
import cv2
import numpy as np

interpreter = Interpreter(model_path="/home/pi5/Downloads/Tesis_Laser/IA/Models/model_- 17 july 2025 10_04(2).tflite")
interpreter.allocate_tensors()
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()
input_shape = input_details[0]['shape']

def get_detection(frame):
    img = cv2.resize(frame, (input_shape[2], input_shape[1]))
    input_data = np.expand_dims(img, axis=0).astype(np.uint8)

    interpreter.set_tensor(input_details[0]['index'], input_data)
    interpreter.invoke()
    output = interpreter.get_tensor(output_details[0]['index'])

    detections = []
    for obj in output[0]:
        confidence = obj[2]
        if confidence > 0.6:
            x_center = int((obj[0] + obj[2]) / 2 * frame.shape[1])
            y_center = int((obj[1] + obj[3]) / 2 * frame.shape[0])
            detections.append((x_center, y_center, confidence))
    return detections
