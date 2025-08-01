# main.py
from Clases import Pitch, Yaw, MotorDC, Camara, ZonasExploradas
import time
import cv2
# =========================
# CONFIGURACIÓN Y VARIABLES
# =========================
TFLITE_MODEL_PATH = "/home/pi5/Downloads/Tesis_Laser/IA/Models/model_- 17 july 2025 10_04(2).tflite"
POSICION_INICIAL_YAW = 0  # en grados
SONDEO_RANGO = 90  # ±90°
MARGEN_PIXELES = 10

# =========================
# INSTANCIAS
# =========================
yaw_motor = Yaw()
pitch_servo = Pitch()
motor_avance = MotorDC()
camara = Camara(model_path=TFLITE_MODEL_PATH)

# =========================
# PARTE 1: IR A UBICACIÓN
# =========================
def paso_1_ir_a_ubicacion():
    print("Paso 1: Avanzando 1 segundo al objetivo...")
    motor_avance.adelante(2.5)
# =========================
# PARTE 2: SONDEO
# =========================
def paso_2_sondeo():
    print("Paso 2: Iniciando sondeo horizontal con camara activa")

    pitch_servo.mover_a_angulo(120)
    print("Servo en angulo inicial para deteccion (120 grados)")

    paso_grados = 5
    zona_muerta = 10
    sentido = 1 if yaw_motor.posicion_actual < 90 else -1

    zona_derecha_visitada = False
    zona_izquierda_visitada = False

    while True:
        # Verificar si ya termino de recorrer ambos extremos
        if zona_derecha_visitada and zona_izquierda_visitada:
            print("Ambas zonas exploradas sin exito. Pasando a paso 6")
            return False

        # Verificar limites y forzar cambio de direccion
        if yaw_motor.posicion_actual >= 90:
            print("Limite derecho alcanzado. Cambiando direccion")
            zona_derecha_visitada = True
            sentido = -1
            yaw_motor.mover(-paso_grados)  # Forzar movimiento hacia el otro lado
            continue

        elif yaw_motor.posicion_actual <= -90:
            print("Limite izquierdo alcanzado. Cambiando direccion")
            zona_izquierda_visitada = True
            sentido = 1
            yaw_motor.mover(paso_grados)
            continue

        # Obtener frame
        gray, frame_mostrar = camara.obtener_frame()
        if gray is None:
            print("Frame invalido")
            continue

        objetos = camara.detectar_objeto(gray)

        # Mostrar ventana
        for (x1, y1, x2, y2, class_id, score, cx, cy) in objetos:
            cv2.rectangle(frame_mostrar, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame_mostrar, f"Objeto {score:.2f}", (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        cv2.imshow("Sondeo", frame_mostrar)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            print("Sondeo interrumpido manualmente")
            return False

        # Determinar si esta en una zona ya visitada
        en_zona_ya_visitada = (
            (sentido == -1 and zona_derecha_visitada and yaw_motor.posicion_actual > 0) or
            (sentido == 1 and zona_izquierda_visitada and yaw_motor.posicion_actual < 0)
        )

        # Validar deteccion si es permitido
        if objetos and not en_zona_ya_visitada:
            print("Objeto detectado. Pasando al paso 3")
            return True
        elif objetos and en_zona_ya_visitada:
            print("Deteccion ignorada en zona ya explorada")

        # Mover el yaw en la direccion actual
        yaw_motor.mover(sentido * paso_grados)



# =========================
# PARTE 3: TRACKING (YAW → PITCH)
# =========================
def paso_3_tracking():
    print("Paso 3: Tracking del objetivo detectado")

    while True:
        gray, _ = camara.obtener_frame()
        if gray is None:
            print("Frame invalido. Saltando...")
            continue

        objetos = camara.detectar_objeto(gray)

        if not objetos:
            print("Objeto perdido durante tracking. Volviendo a paso 2")
            return False

        # Solo usar el objeto con mayor confianza
        objeto_principal = max(objetos, key=lambda obj: obj[5])  # obj[5] = score
        _, _, _, _, _, _, cx, cy = objeto_principal

        error_x = cx - camara.cx_frame
        error_y = cy - camara.cy_frame

        print(f"Error X: {error_x} px | Error Y: {error_y} px")

        if abs(error_x) <= 15 and abs(error_y) <= 15:
            print("Objeto centrado. Pasando a paso 4")
            return True

        # Primero corregir Yaw (horizontal)
        if abs(error_x) > 15:
            pasos = yaw_motor.pixeles_a_pasos(error_x)
            yaw_motor.mover(pasos)

        # Luego corregir Pitch (vertical)
        if abs(error_y) > 15:
            pitch_servo.mover_por_pixeles(error_y)

        time.sleep(0.1)

# =========================
# PARTE 4: Laser
# =========================

def paso_4_laser():
    print("Paso 4: Preparando disparo del laser...")

    # Esperar medio segundo antes de disparar
    time.sleep(0.5)
    # Activar el laser (PWM 0.01 durante 1 segundo)
    laser = PWMOutputDevice(12)  # Asegúrate que GPIO17 es el correcto
    laser.value = 0.01
    print("Laser activado")
    time.sleep(1)
    laser.value = 0
    print("Laser desactivado")
    laser.close()



#paso_1_ir_a_ubicacion(2.5)
while True:
    if paso_2_sondeo():
         if paso_3_tracking():
             break
# Paso 1: Ir a la ubicacion inicial
#motor_avance.adelante(3.5)
#paso_2_sondeo()
# Esperar confirmacion manual para pasar al paso 3
# input("Presiona Enter para continuar al paso 3 (tracking)...")
# 
# # Paso 3: Tracking
#paso_3_tracking()
# if paso_3_tracking():
#     paso_4_laser()
# else:
#     print("Fallo en el tracking. Finalizando.")
# 
