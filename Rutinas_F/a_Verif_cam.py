# test/a_camera.py
import cv2

def verificar_camaras(max_index=5):
    camaras_disponibles = []
    print("🔍 Buscando cámaras conectadas...")

    for i in range(max_index):
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            print(f"✅ Cámara detectada en /dev/video{i}")
            camaras_disponibles.append(i)
            cap.release()
        else:
            print(f"❌ No hay cámara en /dev/video{i}")
    
    if not camaras_disponibles:
        print("🚨 No se encontraron cámaras.")
    return camaras_disponibles
