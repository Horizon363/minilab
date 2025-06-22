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
        self.target = 0

    def move_steps(self, steps, direction, min_dt=0.0009, max_dt=0.004):
        GPIO.output(self.dir_pin, direction)
        if self.enable_pin:
            GPIO.output(self.enable_pin, GPIO.LOW)
        ramp = max_dt
        for i in range(abs(steps)):
            GPIO.output(self.step_pin, GPIO.HIGH)
            time.sleep(ramp)
            GPIO.output(self.step_pin, GPIO.LOW)
            time.sleep(ramp)
            ramp = max(min_dt, ramp - (max_dt-min_dt)/abs(steps)) if abs(steps)>10 else min_dt
        if self.enable_pin:
            GPIO.output(self.enable_pin, GPIO.HIGH)

    def pid_move(self, setpoint, kp, ki=0, kd=0, windup_guard=100, max_iter=700, min_step=1, max_step=16, base_dt=0.002):
        last_error = setpoint - self.target
        integral = 0.0
        for i in range(max_iter):
            error = setpoint - self.target
            integral += error
            if abs(integral) > windup_guard:
                integral = windup_guard if integral > 0 else -windup_guard
            derivative = error - last_error
            output = int(kp*error + ki*integral + kd*derivative)
            n_steps = min(max(abs(output), min_step), max_step)
            if abs(error) <= min_step:
                break
            dir_ = GPIO.HIGH if output > 0 else GPIO.LOW
            self.move_steps(n_steps, dir_, min_dt=base_dt/2, max_dt=base_dt)
            self.target += n_steps if dir_ == GPIO.HIGH else -n_steps
            last_error = error
            time.sleep(0.001)
        return self.target

if __name__ == "__main__":
    stepper = StepperDRV8825("P8_7", "P8_8", "P8_9")
    destino = 400
    final = stepper.pid_move(destino, kp=0.5, ki=0.03, kd=0.01)
    print("Final:", final)
