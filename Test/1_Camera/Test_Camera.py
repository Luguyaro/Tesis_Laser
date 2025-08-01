import cv2

# Fuerza a usar V4L2 como backend
cap = cv2.VideoCapture(0, cv2.CAP_V4L2)

# Intenta MJPEG
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 160)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 120)

ret, frame = cap.read()
print("Frame capturado:", ret)

if ret:
    cv2.imshow("USB Cam", frame)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
else:
    print("❌ No se pudo capturar imagen.")

cap.release()
