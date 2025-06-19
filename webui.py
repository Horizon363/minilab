from flask import Flask, render_template, request, redirect, jsonify
import requests

BACKEND_API = "http://127.0.0.1:5100"
STREAM_URL = "http://127.0.0.1:5000/video_feed"

app = Flask(__name__)


@app.route("/", methods=["GET"])
def index():
    status = requests.get(f"{BACKEND_API}/status").json()
    return render_template("index.html",
                           status=status,
                           stream_url=STREAM_URL)


@app.route("/api/update", methods=["POST"])
def update_config():
    val = request.json
    r = requests.post(f"{BACKEND_API}/set", json=val)
    return jsonify(r.json())


@app.route("/api/start", methods=["POST"])
def do_start():
    r = requests.post(f"{BACKEND_API}/start")
    return jsonify(r.json())


@app.route("/api/stop", methods=["POST"])
def do_stop():
    r = requests.post(f"{BACKEND_API}/stop")
    return jsonify(r.json())


@app.route("/api/status", methods=["GET"])
def get_status():
    r = requests.get(f"{BACKEND_API}/status")
    return jsonify(r.json())


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5200, debug=True)
