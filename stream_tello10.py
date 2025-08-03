import os
import socket
import threading
import time
import asyncio
import json
from collections import deque
from queue import Queue
from threading import Lock

import cv2
import numpy as np
import torch
from ultralytics import YOLO
from bleak import BleakScanner, BleakClient
from flask import Flask, render_template, Response, request, jsonify

# ================== CONFIG ==================
TARGET_NAME = "ESP-D"  # your ESP BLE name
UART_RX_UUID = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"

TELLO_IP = '192.168.10.1'
TELLO_PORT = 8889

EVENT_WINDOW_SEC = 5 * 60
POLL_INTERVAL_SEC = 1.0
DETECTION_CONFIRM_SEC = 2
DISEASE_LOG_INTERVAL = 3
FRESH_LOG_INTERVAL = 3

ENABLE_TEST_EVENTS = False

# ================== Globals ==================
app = Flask(__name__)

ble_client = None
ble_queue: Queue[str] = Queue()

os.system("fuser -k 9000/udp || true")
tello_address = (TELLO_IP, TELLO_PORT)
tello_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
tello_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
try:
    tello_socket.bind(('', 9000))
    print("✅ Bound to port 9000 for Tello communication.")
except OSError as e:
    print(f"⚠️ Port 9000 busy, binding to a random free port. Error: {e}")
    tello_socket.bind(('', 0))
    print(f"✅ Bound to fallback port: {tello_socket.getsockname()[1]}")

device = 'cuda' if torch.cuda.is_available() else 'cpu'
print(f"Using device: {device}")
try:
    model = YOLO("yolo_model/OLD.pt").to(device)
    print("✅ YOLO model loaded.")
except Exception as e:
    print("❌ Failed to load YOLO model:", e)
    model = None

video_running = threading.Event()
video_thread = None
flight_thread = None
ble_thread = None
test_thread = None
keep_alive_thread = None
latest_jpeg = None

last_class = None
start_time = None
sent_flag = False
last_disease_log_time = 0
last_fresh_log_time = 0

event_lock = Lock()
events = {
    "disease": deque(maxlen=5000),
    "fresh":   deque(maxlen=5000),
    "spray":   deque(maxlen=5000),
}

def now_ms() -> int:
    return int(time.time() * 1000)

def trim_old_events():
    cutoff = now_ms() - EVENT_WINDOW_SEC * 1000
    for key in events:
        dq = events[key]
        while dq and dq[0] < cutoff:
            dq.popleft()

def push_event(kind: str):
    with event_lock:
        events[kind].append(now_ms())
        trim_old_events()

def send_tello_command(command):
    try:
        tello_socket.sendto(command.encode(), tello_address)
        print(f"✈️ Sent to Tello: {command}")
    except Exception as e:
        print(f"❌ Tello send failed: {e}")

def enhance_image(img, gamma=1.2):
    invGamma = 1.0 / gamma
    table = np.array([(i / 255.0) ** invGamma * 255 for i in np.arange(256)]).astype("uint8")
    return cv2.LUT(img, table)

async def ble_loop():
    global ble_client
    print("🔍 Scanning for ESP...")
    devices = await BleakScanner.discover()
    for d in devices:
        if d.name == TARGET_NAME:
            ble_client = BleakClient(d)
            await ble_client.connect()
            print(f"✅ Connected to {d.name}")
            break
    else:
        print("❌ ESP not found")
        return

    while True:
        message = await asyncio.to_thread(ble_queue.get)
        if ble_client and ble_client.is_connected:
            try:
                await ble_client.write_gatt_char(UART_RX_UUID, message.encode())
                print(f"📤 Sent BLE message: {message}")
            except Exception as e:
                print(f"❌ BLE send failed: {e}")

def start_ble_background_loop():
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(ble_loop())

def test_event_generator():
    if not ENABLE_TEST_EVENTS:
        return
    while True:
        push_event("disease")
        push_event("fresh")
        push_event("spray")
        print("🧪 Test events pushed")
        time.sleep(5)

def receive_video():
    global last_class, start_time, sent_flag
    global last_disease_log_time, last_fresh_log_time
    global latest_jpeg

    cap = cv2.VideoCapture("udp://@0.0.0.0:11111")
    if not cap.isOpened():
        print("❌ Unable to open video stream")
        return

    print("🎥 Video stream started")

    while video_running.is_set():
        ret, frame = cap.read()
        if not ret:
            print("⚠️ Failed to read frame")
            break

        frame_resized = cv2.resize(frame, (640, 640))
        frame_enhanced = enhance_image(frame_resized)
        frame_rgb = cv2.cvtColor(frame_enhanced, cv2.COLOR_BGR2RGB)
        frame_rgb = np.transpose(frame_rgb, (2, 0, 1))
        frame_tensor = torch.tensor(frame_rgb.copy()).float().div(255.0).unsqueeze(0)

        if model is not None:
            frame_tensor = frame_tensor.to(device)
            results = model(frame_tensor, conf=0.5)
            frame_output = results[0].plot()
            detected_classes = []

            names = results[0].names if hasattr(results[0], "names") else {}
            if hasattr(results[0], "boxes") and results[0].boxes and results[0].boxes.cls.numel() > 0:
                for cls_tensor in results[0].boxes.cls:
                    cls_id = int(cls_tensor.item())
                    cls_name = names.get(cls_id, str(cls_id))
                    detected_classes.append(cls_name)

                print("🔎 Detected:", detected_classes)

                cls_name = detected_classes[0]
                if cls_name == last_class:
                    if time.time() - start_time >= DETECTION_CONFIRM_SEC and not sent_flag:
                        ble_queue.put(cls_name)
                        sent_flag = True
                else:
                    last_class = cls_name
                    start_time = time.time()
                    sent_flag = False

                now_t = time.time()
                if any(cls in ["leaf blight", "curly leaf blight"] for cls in detected_classes):
                    if now_t - last_disease_log_time > DISEASE_LOG_INTERVAL:
                        push_event("disease")
                        push_event("spray")
                        last_disease_log_time = now_t
                if "fresh" in detected_classes:
                    if now_t - last_fresh_log_time > FRESH_LOG_INTERVAL:
                        push_event("fresh")
                        last_fresh_log_time = now_t
            else:
                last_class = None
                start_time = None
                sent_flag = False

            frame_bgr = cv2.cvtColor(frame_output, cv2.COLOR_RGB2BGR)
        else:
            frame_bgr = frame_resized

        ret2, jpeg = cv2.imencode('.jpg', frame_bgr)
        if ret2:
            latest_jpeg = jpeg.tobytes()

    cap.release()
    print("🎥 Video stream stopped")

def auto_flight_sequence(takeoff=True):
    time.sleep(2)
    send_tello_command("command")
    time.sleep(2)
    send_tello_command("streamon")
    time.sleep(5)
    if takeoff:
        print("takeoff")
        '''send_tello_command("takeoff")
        time.sleep(8)
        send_tello_command("forward 60")
        time.sleep(5)
        send_tello_command("forward 60")
        time.sleep(5)
        send_tello_command("cw 180")
        time.sleep(8)
        send_tello_command("forward 60")
        time.sleep(5)
        send_tello_command("forward 60")
        time.sleep(7)
        send_tello_command("land")'''

def stop_flight_sequence():
    try:
        send_tello_command("streamoff")
        time.sleep(1)
        send_tello_command("land")
    except Exception as e:
        print("❌ stop_flight_sequence error:", e)

def keep_alive_loop():
    while True:
        try:
            send_tello_command("command")
        except Exception as e:
            print("❌ Keep-alive error:", e)
        time.sleep(10)

def mjpeg_generator():
    global latest_jpeg
    while True:
        if latest_jpeg is not None:
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + latest_jpeg + b'\r\n')
        time.sleep(0.03)

@app.route("/events_json")
def events_json():
    with event_lock:
        payload = {
            "disease": list(events["disease"]),
            "fresh": list(events["fresh"]),
            "spray": list(events["spray"]),
        }
    print("➡️ /events_json ->", {k: len(v) for k, v in payload.items()})
    return jsonify(payload)

@app.route("/")
def index():
    return render_template("index.html")

@app.route("/health")
def health():
    return {"status": "ok", "video_running": video_running.is_set()}

@app.route("/start", methods=["POST"])
def start():
    global video_thread, flight_thread, ble_thread, test_thread, keep_alive_thread

    if ble_thread is None or not ble_thread.is_alive():
        ble_thread = threading.Thread(target=start_ble_background_loop, daemon=True)
        ble_thread.start()

    if video_thread is None or not video_thread.is_alive():
        video_running.set()
        video_thread = threading.Thread(target=receive_video, daemon=True)
        video_thread.start()

    if flight_thread is None or not flight_thread.is_alive():
        takeoff = request.args.get("takeoff", "true").lower() == "true"
        flight_thread = threading.Thread(target=auto_flight_sequence, args=(takeoff,), daemon=True)
        flight_thread.start()

    if ENABLE_TEST_EVENTS and (test_thread is None or not test_thread.is_alive()):
        test_thread = threading.Thread(target=test_event_generator, daemon=True)
        test_thread.start()

    if keep_alive_thread is None or not keep_alive_thread.is_alive():
        keep_alive_thread = threading.Thread(target=keep_alive_loop, daemon=True)
        keep_alive_thread.start()

    return {"status": "started"}

@app.route("/stop", methods=["POST"])
def stop():
    video_running.clear()
    threading.Thread(target=stop_flight_sequence, daemon=True).start()
    return {"status": "landing"}

@app.route("/video_feed")
def video_feed():
    return Response(mjpeg_generator(), mimetype='multipart/x-mixed-replace; boundary=frame')

def find_free_port(start_port=5000):
    port = start_port
    while True:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            if s.connect_ex(("0.0.0.0", port)) != 0:
                return port
            port += 1

if __name__ == "__main__":
    if ENABLE_TEST_EVENTS:
        test_thread = threading.Thread(target=test_event_generator, daemon=True)
        test_thread.start()

    port = find_free_port(5000)
    print(f"🚀 Flask running on port {port}")
    app.run(host="0.0.0.0", port=port, debug=True, threaded=True, use_reloader=False)
