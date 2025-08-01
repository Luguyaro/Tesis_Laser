#Proyecto Tesis: Eliminador de Plaga con Laser
# Bruno Vallejos
# Luis Yampufe
import threading
#from serial_listener import iniciar_escucha_uart
import time
# main.py
from Rutinas_F import a_Verif_cam, b_MotorDC, c_Tracking, c_TrackingLoop

def rutina_completa():
    print("\n🌟 Iniciando rutina principal")
    # 1. Verificar cámara
    camaras = a_Verif_cam.verificar_camaras()
    if not camaras:
        print("❌ No se puede iniciar sin cámara.")
        return
    # 2. Mover motor DC inicial
    val_time=2
    print(val_time)
    #b_MotorDC.mover_motor_adelante(tiempo=val_time)
    # 3. Apagar motor para iniciar sondeo de plaga
    #b_MotorDC.liberar_motor()
    # 4. Iniciar tracking (servo + motor yaw)
    #c_TrackingLoop.iniciar_tracking()
    camera = cv2.VideoCapture(0)
    c_TrackingLoop.tracking_loop(camera)
    print("✅ Rutina completada\n")

# Diccionario de estado compartido
estado_sistema = {
    "modo": "esperando",
    "larvas_detectadas": 0,
    "errores": [],
    "timestamp": time.time()
}

# Iniciar hilo UART (escucha comandos del ESP32)
#iniciar_escucha_uart(estado_sistema)

# Menú para el usuario en terminal
def menu():
    while True:
        print("\n---- MENÚ DE OPERACIÓN ----")
        print("1. Ejecutar detección IA (rutina final)")
        print("2. Test de cada rutina")
        print("3. Salir")
        opcion = input("Selecciona una opción: ")
        #Path de cada parte
        path_ia = "/home/pi/Downloads/Tesis_Laser/IA/Models/"
        path_rutina = "/home/pi/Downloads/Tesis_Laser/Test/"
        
        if opcion == "1":
            #Elegir cual de los 3 modelo usar en el proyecto
            modelo_tflite = "model_- 14_july_2025_19_28.tflite"
            modelo_onnx = "model_- 14 july 2025 19_28.onnx"
            modelo_haar = "haarcascade_frontalface_default.xml"
            #from h_todo import todo
            rutina_completa()
        elif opcion == "2":
            print("1. Prueba de Camara")
            print("2. Prueba de servomotor (Pitch-Y)")
            print("3. Prueba de motor encoder (Yaw-Z)")
            print("4. Prueba de motor traslacion(X-axis)")
            print("5. Prueba de sensor distancia(VL53L1X)")
            print("6. Prueba de IA (YOLO+CNN)")
            print("7. Prueba de Laser (PWM)")
            print("8. Prueba de Final de carrera(Switch)")
            print("9. Salir")
            opcion2 = input("Selecciona una opción: ")
            if opcion2 == "1":
                # 1. Verificar cámara
                camaras = a_Verif_cam.verificar_camaras()
                if not camaras:
                    print("❌ No se puede iniciar sin cámara.")
                    return
                continue
            elif opcion2 == "2":
                continue
            elif opcion2 == "3":
                continue
            elif opcion2 == "4":
                # 2. Mover motor DC inicial
                b_MotorDC.mover_motor_adelante(tiempo=2)
                b_MotorDC.mover_motor_atras(tiempo=2)
            elif opcion2 == "5":
                continue
            elif  opcion2 == "6":
                continue
            elif opcion2 == "7":
                continue
            elif opcion2 == "8":
                continue
            elif opcion2 == "9":
                break
        elif opcion == "3":
            print("Saliendo...")
            break
        else:
            print("Opción inválida. Intenta nuevamente.")

if __name__ == "__main__":
    time.sleep(1)
    menu()