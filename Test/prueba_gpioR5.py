from gpiozero import DigitalInputDevice, DigitalOutputDevice, PWMOutputDevice
from time import sleep

def leer_entrada(pin):
    try:
        entrada = DigitalInputDevice(pin)
        estado = entrada.value
        print(f"📥 GPIO{pin} está en estado: {'ALTO (1)' if estado else 'BAJO (0)'}")
    except Exception as e:
        print(f"❌ Error al leer GPIO{pin}: {e}")

def escribir_salida(pin):
    try:
        salida = DigitalOutputDevice(pin)
        valor = input("¿Qué valor deseas escribir? (1 = ALTO, 0 = BAJO): ")
        if valor == '1':
            salida.on()
            print(f"✅ GPIO{pin} establecido en ALTO (3.3V)")
        elif valor == '0':
            salida.off()
            print(f"✅ GPIO{pin} establecido en BAJO (0V)")
        else:
            print("❌ Valor inválido. Usa 1 o 0.")
    except Exception as e:
        print(f"❌ Error al escribir en GPIO{pin}: {e}")

def generar_pwm(pin):
    try:
        frecuencia = float(input("📈 Ingresa la frecuencia en Hz (ej. 1000): "))
        duty = float(input("🔁 Ingresa el ciclo de trabajo (0 a 100): ")) / 100.0
        pwm = PWMOutputDevice(pin, frequency=frecuencia)
        pwm.value = duty
        print(f"🎵 Generando PWM en GPIO{pin} a {frecuencia}Hz con {duty*100:.1f}% duty cycle.")
        input("Presiona ENTER para detener PWM...")
        pwm.close()
        print("🛑 PWM detenido.")
    except Exception as e:
        print(f"❌ Error al generar PWM en GPIO{pin}: {e}")

def menu():
    while True:
        print("\n=== 🧪 MENÚ DE PRUEBAS GPIO (gpiozero) ===")
        print("1. Leer entrada digital")
        print("2. Escribir salida digital")
        print("3. Generar señal PWM")
        print("4. Salir")
        opcion = input("Selecciona una opción (1-4): ")

        if opcion in ['1', '2', '3']:
            try:
                pin = int(input("🔌 Ingresa el número de pin GPIO (BCM): "))
                if opcion == '1':
                    leer_entrada(pin)
                elif opcion == '2':
                    escribir_salida(pin)
                elif opcion == '3':
                    generar_pwm(pin)
            except ValueError:
                print("❌ Número de pin inválido.")
        elif opcion == '4':
            print("👋 Saliendo del programa.")
            break
        else:
            print("❌ Opción inválida.")

menu()
