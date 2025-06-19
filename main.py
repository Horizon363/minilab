from control_stepper import StepperDRV8825
from control_servo import set_servo, stop_servo
from control_centrifuga import CentrifugaPID
from vision_streaming import CameraStream, start_flask_stream
from api_user import start_api_server, user_settings
import Adafruit_BBIO.GPIO as GPIO
import time

# Pines (elige según tus conexiones)
STEPPER_Y_STEP = "P8_7"
STEPPER_Y_DIR = "P8_8"
STEPPER_Z_STEP = "P8_9"
STEPPER_Z_DIR = "P8_10"
SERVO_PIN = "P9_22"


def main_logic():
    paso_y = StepperDRV8825(STEPPER_Y_STEP, STEPPER_Y_DIR)
    paso_z = StepperDRV8825(STEPPER_Z_STEP, STEPPER_Z_DIR)
    centrifuga = CentrifugaPID(kp=0.10, ki=0.0, kd=0.02)
    # iniciar stream
    camera = CameraStream()
    start_flask_stream(camera)
    start_api_server()

    while True:
        if user_settings["running"]:
            print("Iniciando ciclo automático...")

            # a) Mueve robot YZ para tomar tubo según user_settings["tubos"]
            # ejemplo mover a 2do tubo:
            # paso_y.move_steps(200, True)
            # paso_z.move_steps(150, False)
            # set_servo(SERVO_PIN, 0)   # abrir gripper
            # set_servo(SERVO_PIN, 90)  # cerrar gripper para tomar tubo

            # b) Lleva a posición sobre centrifuga y suelta
            # c) Si necesita compensar, va por tubo de control y repite
            # d) Centrifuga: posición de rotor (usar motor/encoder)
            # e) Inicia rampa de velocidad:
            import threading
            pid_thread = threading.Thread(target=centrifuga.pid_control, args=(
                user_settings["velocidad"],), daemon=True)
            pid_thread.start()
            # f) Espera tiempo user_settings["tiempo"] (segundos)
            time.sleep(user_settings["tiempo"])
            # g) Detén centrifuga
            centrifuga.stop()
            # h) Vuelve a home y suelta tubos

            user_settings["running"] = False
            print("Ciclo completado.")
        else:
            time.sleep(0.5)


if __name__ == "__main__":
    try:
        main_logic()
    except KeyboardInterrupt:
        print("Apagándose...")
        GPIO.cleanup()
