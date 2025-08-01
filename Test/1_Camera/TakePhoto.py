import cv2
import os

# Ruta de guardado
SAVE_DIR = "/home/pi5/Downloads/Tesis_Laser/IA/Dataset_test"
os.makedirs(SAVE_DIR, exist_ok=True)

# Inicializar cámara
cap = cv2.VideoCapture(0)
cap.set(3, 640)  # Ancho
cap.set(4, 480)  # Alto

if not cap.isOpened():
    print("Error: no se pudo acceder a la cámara.")
    exit()

for i in range(2):
    input(f"\nPresiona ENTER para capturar la imagen {i+1}...")

    ret, frame = cap.read()
    if not ret:
        print("Error al capturar la imagen.")
        break

    # Rotar la imagen 180 grados
    rotated = cv2.rotate(frame, cv2.ROTATE_180)

    # Mostrar imagen rotada
    cv2.imshow(f"Imagen {i+1} (rotada)", rotated)
    cv2.waitKey(1000)  # Mostrar durante 1 segundo
    cv2.destroyAllWindows()

    # Pedir nombre para guardar
    nombre = input("Ingresa el nombre para esta imagen: ").strip()
    if not nombre:
        nombre = f"captura_{i+1}"

    path = os.path.join(SAVE_DIR, nombre + ".jpg")
    cv2.imwrite(path, rotated)
    print(f"Imagen {i+1} guardada como: {path}")

# Cerrar cámara
cap.release()
cv2.destroyAllWindows()
