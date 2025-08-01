# main_prueba_tracking.py

from Clases import Pitch, Yaw, Camara
import time
import cv2

TFLITE_MODEL_PATH = "/home/pi5/Downloads/Tesis_Laser/IA/Models/model_- 17 july 2025 10_04(2).tflite"

yaw_motor = Yaw()
pitch_servo = Pitch()
camara = Camara(model_path=TFLITE_MODEL_PATH)

import threading
def paso_3_tracking(tiempo_estable=0.8, tolerancia_px=15):
    print("Paso 3: Tracking del objetivo detectado")

    # Centrar yaw y pitch antes de empezar
    yaw_motor.mover(-yaw_motor.posicion_actual)
    pitch_servo.mover_a_angulo(100)

    tiempo_inicio = time.time()
    timeout_inicial = 5

    # Esperar objeto inicial
    while time.time() - tiempo_inicio < timeout_inicial:
        gray, frame_mostrar = camara.obtener_frame()
        if gray is None:
            continue
        objetos = camara.detectar_objeto(gray)
        if objetos:
            print("Objeto detectado. Iniciando tracking...")
            break
        cv2.imshow("Tracking", frame_mostrar)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            return False
    else:
        print("Timeout sin detectar objeto.")
        return False

    # Seguimiento y verificación de estabilidad
    tiempo_centrado = None

    while True:
        gray, frame_mostrar = camara.obtener_frame()
        if gray is None:
            continue

        objetos = camara.detectar_objeto(gray)
        if not objetos:
            print("Objeto perdido. Volviendo a paso 2.")
            return False

        x1, y1, x2, y2, _, score, cx, cy = max(objetos, key=lambda obj: obj[5])
        error_x = cx - camara.cx_frame
        error_y = cy - camara.cy_frame

        print(f"[Tracking] Error X: {error_x} | Error Y: {error_y}")

        # Mostrar cuadro
        cv2.rectangle(frame_mostrar, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(frame_mostrar, f"Score: {score:.2f}", (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.imshow("Tracking", frame_mostrar)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("Tracking interrumpido manualmente.")
            return False

        # Verificar si esta centrado
        if abs(error_x) <= tolerancia_px and abs(error_y) <= tolerancia_px:
            if tiempo_centrado is None:
                tiempo_centrado = time.time()
            elif time.time() - tiempo_centrado >= tiempo_estable:
                print("Objeto centrado de forma estable. Paso 3 completado.")
                return True
        else:
            tiempo_centrado = None  # Reiniciar temporizador

        # Correcciones
        if abs(error_x) > tolerancia_px:
            pasos = yaw_motor.pixeles_a_pasos(error_x)
            yaw_motor.mover(pasos)

        if abs(error_y) > tolerancia_px:
            pitch_servo.mover_por_pixeles(error_y)

        time.sleep(0.05)  # Evitar saturar la CPU


    camara.liberar()
    pitch_servo.cerrar()
    yaw_motor.liberar_gpio()

paso_3_tracking()
