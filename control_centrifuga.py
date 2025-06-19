import Adafruit_BBIO.GPIO as GPIO
import Adafruit_BBIO.PWM as PWM
import time
import threading

ENC_A = "P8_15"
PWM_MOTOR = "P9_14"
AIN1 = "P8_17"
AIN2 = "P8_18"


class CentrifugaPID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.int_err = 0.0
        self.prev_err = 0.0
        self.setpoint = 0.0
        self.encoder_ticks = 0
        self._running = False

        GPIO.setup(AIN1, GPIO.OUT)
        GPIO.setup(AIN2, GPIO.OUT)
        GPIO.setup(ENC_A, GPIO.IN)
        PWM.start(PWM_MOTOR, 0, 1000)  # 1kHz

        GPIO.add_event_detect(ENC_A, GPIO.RISING,
                              callback=self._encoder_callback)

    def _encoder_callback(self, channel):
        self.encoder_ticks += 1

    def reset_encoder(self):
        self.encoder_ticks = 0

    def get_rpm(self, pulses_per_rev=20, interval=1.0):
        # Cuenta durante interval
        start_count = self.encoder_ticks
        time.sleep(interval)
        end_count = self.encoder_ticks
        pulses = end_count - start_count
        rpm = (pulses * 60) / (pulses_per_rev * interval)
        return rpm

    def set_motor(self, val):
        # val +-100 (direccion)
        if val >= 0:
            GPIO.output(AIN1, GPIO.HIGH)
            GPIO.output(AIN2, GPIO.LOW)
        else:
            GPIO.output(AIN1, GPIO.LOW)
            GPIO.output(AIN2, GPIO.HIGH)
        PWM.set_duty_cycle(PWM_MOTOR, min(100, abs(val)))

    def pid_control(self, target_rpm, pulses_per_rev=20):
        """ Ejecuta bucle PID en hilo propio. """
        self.setpoint = target_rpm
        self._running = True
        last_time = time.time()
        while self._running:
            rpm = self.get_rpm(pulses_per_rev, 0.1)
            err = self.setpoint - rpm
            now = time.time()
            dt = now - last_time
            self.int_err += err * dt
            d_err = (err - self.prev_err)/dt if dt > 0 else 0
            output = self.kp * err + self.ki * self.int_err + self.kd * d_err
            output = max(min(output, 100), -100)
            self.set_motor(output)
            self.prev_err = err
            last_time = now
        self.set_motor(0)

    def stop(self):
        self._running = False
        self.set_motor(0)
