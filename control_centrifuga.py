import Adafruit_BBIO.GPIO as GPIO
import Adafruit_BBIO.PWM as PWM
import time
import threading
import numpy as np

ENC_A = "P8_15"
PWM_MOTOR = "P9_14"
AIN1 = "P8_17"
AIN2 = "P8_18"

class ObservadorEstado:
    def __init__(self, J, b, K, Ts, L):
        self.J = J
        self.b = b
        self.K = K
        self.Ts = Ts
        self.L = L
        self.state = 0.0
        self.u = 0.0
        self.y = 0.0

    def update(self, u, y): 
        omega_est = self.state
        omega_est += self.Ts/self.J * (-self.b * omega_est + self.K * u)
        omega_est += self.L * (y - omega_est)
        self.state = omega_est
        return self.state

    def reset(self, start_state=0.0):
        self.state = start_state

class CentrifugaPIDObs:
    def __init__(self, kp, ki, kd, observador: ObservadorEstado, pulses_per_rev=20):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.int_err = 0.0
        self.prev_err = 0.0
        self.setpoint = 0.0
        self._running = False
        self.encoder_ticks = 0
        self.lock = threading.Lock()
        self.pulses_per_rev = pulses_per_rev
        self.Ts = observador.Ts
        self.observador = observador
        self.log = []

        GPIO.setup(AIN1, GPIO.OUT)
        GPIO.setup(AIN2, GPIO.OUT)
        GPIO.setup(ENC_A, GPIO.IN)
        PWM.start(PWM_MOTOR, 0, 1000)
        GPIO.add_event_detect(ENC_A, GPIO.RISING, callback=self._encoder_callback, bouncetime=1)

        self._enc_last_time = time.time()
        self._enc_last_ticks = 0
        self._rpm_measured = 0.0

    def _encoder_callback(self, channel):
        with self.lock:
            self.encoder_ticks += 1

    def reset_encoder(self):
        with self.lock:
            self.encoder_ticks = 0

    def read_encoder_rpm(self, interval=0.1):
        with self.lock:
            start = self.encoder_ticks
        time.sleep(interval)
        with self.lock:
            end = self.encoder_ticks
        pulses = end - start
        rpm = (pulses * 60) / (self.pulses_per_rev * interval)
        if rpm < 0: rpm = 0
        return rpm

    def est_velocidad_rpm(self, raw_rpm=None):
        n = np.random.normal(0, 1)
        bias = 2 * np.sin(time.time()/8)
        return (raw_rpm if raw_rpm is not None else self._rpm_measured) + n + bias

    def set_motor(self, val):
        val = max(min(val, 100), -100)
        if val >= 0:
            GPIO.output(AIN1, GPIO.HIGH)
            GPIO.output(AIN2, GPIO.LOW)
        else:
            GPIO.output(AIN1, GPIO.LOW)
            GPIO.output(AIN2, GPIO.HIGH)
        PWM.set_duty_cycle(PWM_MOTOR, abs(val))

    def pid_obs_control(self, target_rpm, total_runtime=10.0):
        self.setpoint = target_rpm
        self.int_err = 0
        self.prev_err = 0
        self._running = True
        t0 = time.time()
        last = t0
        rpm_est = 0.0
        history = []
        self.observador.reset()

        while self._running and (time.time()-t0) < total_runtime:
            now = time.time()
            rpm_enc = self.read_encoder_rpm(self.Ts)
            self._rpm_measured = rpm_enc
            u_norm = getattr(self, "_last_u", 0.0)
            rpm_est = self.observador.update(u=u_norm, y=self.est_velocidad_rpm(rpm_enc))
            err = self.setpoint - rpm_est
            dt = now-last if last is not None else self.Ts
            self.int_err += err * dt
            d_err = (err - self.prev_err) / dt if dt > 0 else 0
            output = self.kp * err + self.ki * self.int_err + self.kd * d_err
            output = max(min(output, 100), -100)
            self.set_motor(output)
            self._last_u = output
            self.prev_err = err
            last = now
            self.log.append({
                "time": now-t0,
                "rpm_set": self.setpoint,
                "rpm_encoder": rpm_enc,
                "rpm_est": rpm_est,
                "u": output,
                "err": err
            })
            time.sleep(self.Ts * (0.96 + 0.08*np.random.random()))
        self.set_motor(0)
        print("[CONTROL] Fin de control PID+Observador.")

    def stop(self):
        self._running = False
        self.set_motor(0)

if __name__ == "__main__":
    J = 5e-5
    b = 3e-3
    K = 0.09
    Ts = 0.055
    L = 0.23

    obs = ObservadorEstado(J, b, K, Ts, L)
    pidobs = CentrifugaPIDObs(kp=0.12, ki=0.3, kd=0.05, observador=obs)

    try:
        t = threading.Thread(target=pidobs.pid_obs_control, args=(1000, 18))
        t.start()
        t.join()
    finally:
        pidobs.stop()
        print("[MAIN] Control detenido. Log generado:", len(pidobs.log))
