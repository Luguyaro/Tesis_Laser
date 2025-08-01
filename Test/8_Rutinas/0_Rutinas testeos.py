import time
import sys

# =====================================
# PRUEBA 1: Control de SERVO con lgpio
# =====================================
def test_servo_lgpio():
    import lgpio

    SERVO_GPIO = 26
    h = lgpio.gpiochip_open(0)
    pulso_actual = 1500

    def send_servo_pulse(us):
        lgpio.gpio_write(h, SERVO_GPIO, 1)
        time.sleep(us / 1_000_000)
        lgpio.gpio_write(h, SERVO_GPIO, 0)
        time.sleep((20000 - us) / 1_000_000)

    def angulo_a_us(angulo):
        return int(500 + (angulo / 180.0) * 2000)

    def mover_servo(angulo_destino):
        nonlocal pulso_actual
        us_destino = angulo_a_us(angulo_destino)
        paso = 10 if us_destino > pulso_actual else -10

        for us in range(pulso_actual, us_destino + paso, paso):
            send_servo_pulse(us)
            time.sleep(0.015)
            pulso_actual = us

    try:
        while True:
            ang = input("Ángulo (0–180) o 'q' para salir: ").strip()
            if ang.lower() == 'q':
                break
            if ang.isdigit() and 0 <= int(ang) <= 180:
                mover_servo(int(ang))
            else:
                print("Entrada inválida o fuera de rango.")
    finally:
        lgpio.gpiochip_close(h)
        print("GPIO liberado.")

# =================================================
# PRUEBA 2: Motor con Encoder + PID (gpiozero + lgpio)
# =================================================
def test_pid_encoder_motor():
    from gpiozero.pins.lgpio import LGPIOFactory
    from gpiozero import PWMOutputDevice, DigitalInputDevice

    factory = LGPIOFactory()

    IN1, IN2 = 22, 27
    ENCA, ENCB = 20, 19

    motor_forward = PWMOutputDevice(IN1, frequency=5000, pin_factory=factory)
    motor_backward = PWMOutputDevice(IN2, frequency=5000, pin_factory=factory)

    enca = DigitalInputDevice(ENCA, pull_up=True, pin_factory=factory)
    encb = DigitalInputDevice(ENCB, pull_up=True, pin_factory=factory)

    posi = 0
    prevT = time.time()
    eprev = 0
    eintegral = 0
    target = 0
    last_encoder_time = time.time()
    debounce_time = 0.001

    def encoder_callback():
        nonlocal posi, last_encoder_time
        current_time = time.time()
        if current_time - last_encoder_time < debounce_time:
            return
        last_encoder_time = current_time
        movimiento = 0.17/90*270
        if enca.value == encb.value:
            posi += movimiento
        else:
            posi -= movimiento
        posi = max(-180, min(180, posi))

    enca.when_activated = encoder_callback

    def pid_control(target_position):
        nonlocal posi, eprev, eintegral, prevT

        Kp = 10
        Ki = 0.01
        Kd = 0.005
        zona_muerta = 1.0
        MIN_PWM = 0.5

        e = target_position - posi
        current_time = time.time()
        deltaT = current_time - prevT
        prevT = current_time

        if abs(e) < zona_muerta:
            eintegral = 0
            motor_forward.value = 0
            motor_backward.value = 0
            return

        eintegral += e * deltaT
        dedt = (e - eprev) / deltaT if deltaT > 0 else 0
        eprev = e

        output = Kp * e + Ki * eintegral + Kd * dedt
        pwm = 50/270*90
        output = max(-pwm, min(pwm, output))

        duty_cycle = abs(output) / 150.0
        if 0 < duty_cycle < MIN_PWM:
            duty_cycle = MIN_PWM

        if output < 0:
            motor_forward.value = duty_cycle
            motor_backward.value = 0
        elif output > 0:
            motor_forward.value = 0
            motor_backward.value = duty_cycle
        else:
            motor_forward.value = 0
            motor_backward.value = 0

    try:
        while True:
            entrada = input("Ángulo objetivo (-180 a 180) o 'q' para salir: ").strip()
            if entrada.lower() == 'q':
                break
            try:
                target = float(entrada)
                if not -180 <= target <= 180:
                    print("Fuera de rango.")
                    continue
            except ValueError:
                print("Entrada inválida.")
                continue

            while abs(target - posi) > 1:
                pid_control(target)
                print(f"Posición actual: {posi:.2f}° | Objetivo: {target}°")
                time.sleep(0.01)

            print(f"✅ Objetivo alcanzado: {posi:.2f}°")

    finally:
        print("Liberando GPIO...")
        motor_forward.close()
        motor_backward.close()
        enca.close()
        encb.close()

# ========================================
# PRUEBA 3: Motor DC simple
# ========================================
def test_motor_simple():
    from gpiozero.pins.lgpio import LGPIOFactory
    from gpiozero import PWMOutputDevice

    factory = LGPIOFactory()

    IN1 = PWMOutputDevice(5, frequency=1000, pin_factory=factory)
    IN2 = PWMOutputDevice(6, frequency=1000, pin_factory=factory)

    duracion = 2
    PWM = 15 * 4 / 100

    try:
        print("→ Motor adelante")
        IN1.value = PWM
        IN2.value = 0
        time.sleep(duracion)

        print("→ Motor atrás")
        IN1.value = 0
        IN2.value = PWM
        time.sleep(duracion)

        print("→ Motor detenido")
        IN1.value = 0
        IN2.value = 0

        input("Presiona Enter o 'q' para continuar...")  # pausa simple

    finally:
        IN1.close()
        IN2.close()
        print("GPIO cerrado.")

# ====================================
# MENÚ SECUENCIAL
# ====================================
if __name__ == "__main__":
    funciones = [
        ("Servo con lgpio", test_servo_lgpio),
        ("Motor con PID y Encoder", test_pid_encoder_motor),
        ("Motor DC Simple", test_motor_simple)
    ]

    for nombre, funcion in funciones:
        while True:
            eleccion = input(f"\n→ Ejecutar {nombre}? [E]jecutar / [S]altar / [T]erminar: ").lower().strip()
            if eleccion == 'e':
                funcion()
                break
            elif eleccion == 's':
                print("⏭ Saltando prueba.")
                break
            elif eleccion == 't':
                print("🛑 Terminando pruebas.")
                sys.exit(0)
            else:
                print("❓ Opción inválida. Usa E, S o T.")
