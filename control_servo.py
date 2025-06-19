import Adafruit_BBIO.PWM as PWM


def set_servo(pin, angle):
    duty = 5 + (angle / 160.0) * 5
    PWM.start(pin, duty, 50)


def stop_servo(pin):
    PWM.stop(pin)
    PWM.cleanup()
