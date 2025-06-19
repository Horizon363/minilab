import threading
import time
from flask import Flask, jsonify, request

user_settings = {
    # 1: ocupado, 0: libre, api post
    "tubos": [1, 0, 0, 0, 0, 0],
    "velocidad": 3000,        # RPM objetivo
    "tiempo": 180,            # segundos
    "running": False
}


def start_api_server(port=5100):
    app = Flask(__name__)

    @app.route('/status')
    def status():
        return jsonify(user_settings)

    @app.route('/set', methods=['POST'])
    def set_user():
        data = request.json
        for k in data:
            if k in user_settings:
                user_settings[k] = data[k]
        return jsonify({"result": "OK", "new_settings": user_settings})

    @app.route('/start', methods=['POST'])
    def start_proc():
        user_settings["running"] = True
        return jsonify({"result": "started"})

    @app.route('/stop', methods=['POST'])
    def stop_proc():
        user_settings["running"] = False
        return jsonify({"result": "stopped"})

    threading.Thread(target=app.run, kwargs={
                     'host': '0.0.0.0', 'port': port, 'debug': False}, daemon=True).start()
