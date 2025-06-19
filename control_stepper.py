import Adafruit_BBIO.GPIO as GPIO
import time


class StepperDRV8825:
    def __init__(self, step_pin, dir_pin, enable_pin=None):
        self.step_pin = step_pin
        self.dir_pin = dir_pin
        self.enable_pin = enable_pin

        GPIO.setup(step_pin, GPIO.OUT)
        GPIO.setup(dir_pin, GPIO.OUT)
        if enable_pin:
            GPIO.setup(enable_pin, GPIO.OUT)
            GPIO.output(enable_pin, GPIO.LOW)

    def move_steps(self, steps, direction, step_time=0.001):
        GPIO.output(self.dir_pin, direction)
        if self.enable_pin:
            GPIO.output(self.enable_pin, GPIO.LOW)

        for _ in range(abs(steps)):
            GPIO.output(self.step_pin, GPIO.HIGH)
            time.sleep(step_time)
            GPIO.output(self.step_pin, GPIO.LOW)
            time.sleep(step_time)

        if self.enable_pin:
            GPIO.output(self.enable_pin, GPIO.HIGH)
