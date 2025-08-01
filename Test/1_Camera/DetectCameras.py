import cv2

def detectar_camaras(max_index=10):
    camaras_disponibles = []

    print("Buscando cámaras conectadas...\n")

    for i in range(max_index):
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            print(f"✅ Cámara detectada en /dev/video{i}")
            camaras_disponibles.append(i)
            cap.release()
        else:
            print(f"❌ No hay cámara en /dev/video{i}")
    
    if not camaras_disponibles:
        print("\n🚨 No se encontraron cámaras.")
    else:
        print(f"\n🎥 Cámaras disponibles: {camaras_disponibles}")
    
    return camaras_disponibles

if __name__ == "__main__":
    detectar_camaras()
