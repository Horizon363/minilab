import cv2
from flask import Flask, Response
import threading
import time


class CameraStream:
    def __init__(self):
        self.cap = cv2.VideoCapture(0)
        self.frame = None
        self.running = True
        threading.Thread(target=self._update, daemon=True).start()

    def _update(self):
        while self.running:
            ret, frame = self.cap.read()
            if ret:
                self.frame = frame.copy()
            time.sleep(0.03)

    def stop(self):
        self.running = False
        self.cap.release()

    def get_jpg_bytes(self):
        if self.frame is None:
            return b''
        _, buf = cv2.imencode('.jpg', self.frame)
        return buf.tobytes()


def start_flask_stream(camera, port=5000):
    app = Flask(__name__)

    @app.route('/')
    def index():
        return '''
        <html><body><h2>Video en Vivo</h2>
        <img src="/video_feed"></body></html>'''

    def gen():
        while True:
            jpg = camera.get_jpg_bytes()
            yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + jpg + b'\r\n')
            time.sleep(0.04)

    @app.route('/video_feed')
    def video_feed():
        return Response(gen(), mimetype='multipart/x-mixed-replace; boundary=frame')

    threading.Thread(target=app.run, kwargs={
                     'host': '0.0.0.0', 'port': port, 'debug': False}, daemon=True).start()
