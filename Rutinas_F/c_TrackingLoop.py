# c_Tracking.py
import cv2
import numpy as np
from g_ia_detection import get_detection  # detección IA
from d_servo_pitch import ajustar_pitch
from e_motor_yaw import ajustar_yaw, iniciar_encoder

# Umbral para decir que el objeto está centrado
ERROR_UMBRAL = 10

def tracking_loop(camera):
    iniciar_encoder()  # Inicia lectura del encoder en background

    while True:
        ret, frame = camera.read()
        if not ret:
            print("Error al capturar fotograma")
            continue

        detections = get_detection(frame)
        if detections:
            for detection in detections:
                x_center, y_center, conf = detection
                error_x = x_center - frame.shape[1] // 2
                error_y = y_center - frame.shape[0] // 2

                print(f"Error X: {error_x}, Error Y: {error_y}, Confianza: {conf}")

                if abs(error_x) > ERROR_UMBRAL:
                    ajustar_yaw(error_x)  # mueve el motor DC
                if abs(error_y) > ERROR_UMBRAL:
                    ajustar_pitch(error_y)  # mueve el servo

        # Opcional: Espera o FPS control
        cv2.imshow("Frame", frame)
        if cv2.waitKey(1) == 27:  # ESC
            break

    camera.release()
    cv2.destroyAllWindows()
