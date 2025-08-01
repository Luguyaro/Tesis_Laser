import gpiozero as GPIO
import time

ENCA = 22
ENCB = 27
pos = 0

def callback(channel):
    global pos
    a = GPIO.input(ENCA)
    b = GPIO.input(ENCB)
    if a == b:
        pos -= 1
    else:
        pos += 1
    print(f"Contador: {pos}")

GPIO.setmode(GPIO.BCM)
GPIO.setup(ENCA, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(ENCB, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.add_event_detect(ENCA, GPIO.RISING, callback=callback)

try:
    while True:
        time.sleep(0.1)
except KeyboardInterrupt:
    GPIO.cleanup()