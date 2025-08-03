# 🚀 IntelliAgro Drone – AI-Powered Autonomous Crop Diagnosis & Treatment

**Final Year Project**
Empowering modern agriculture with intelligent automation.

---

## 🌟 Overview

**IntelliAgro Drone** is an AI-powered, fully autonomous drone system designed to revolutionize traditional farming practices. It integrates deep learning, computer vision, and IoT technologies to **detect plant diseases in real-time** and **perform precision spraying**, improving both crop health and resource efficiency.

---

## 🔍 Key Features

* ✅ **Real-Time Disease Detection (YOLOv8)**

  * Uses the YOLO deep learning model to identify cotton leaf diseases from aerial footage.
  * Supports detection of multiple classes (e.g., fresh, leaf blight, curly leaf blight).

* 🧠 **Smart Spraying System**

  * Sprays pesticide *only* on infected plants.
  * Reduces chemical usage and protects healthy crops.

* 📹 **Live Video Streaming**

  * Live drone camera feed via a Flask web dashboard.
  * Visual tracking of detection and spraying actions.

* 📊 **Operation Logging**

  * Logs detections, actions, timestamps, and GPS metadata.
  * Enables post-operation review and traceability.

* 🕹 **One-Click Full Field Operation**

  * Starts detection, logging, and spraying with a single click.
  * Designed for simplicity and end-to-end automation.

---

## 🧰 Tech Stack

| Component     | Technology                     |
| ------------- | ------------------------------ |
| Model         | YOLOv8 (Ultralytics)           |
| Drone         | DJI Tello                      |
| BLE Interface | ESP32 with MicroPython         |
| Sprayer       | Servo-controlled via ESP32     |
| Backend       | Python, Flask, OpenCV          |
| Web Dashboard | HTML, JS (Chart.js), Bootstrap |

---

## 📂 Repository Structure

```
📁 intelliagro-drone/
├── drone_stream/          # Video stream and YOLO detection
├── esp32_ble/             # MicroPython BLE + servo control
├── static/                # Frontend assets (CSS, JS)
├── templates/             # Flask HTML templates
├── dataset/               # Training data (optional)
├── app.py                 # Flask app (dashboard + BLE interface)
├── requirements.txt       # Python dependencies
└── README.md              # Project documentation
```

---

## 🚀 Getting Started

### 1. Clone the Repository

```bash
git clone https://github.com/your-username/intelliagro-drone.git
cd intelliagro-drone
```

### 2. Install Dependencies

```bash
pip install -r requirements.txt
```

### 3. Flash ESP32 with MicroPython Code

* Use `Thonny` or `ampy` to upload `esp32_ble/main.py` to your ESP32.
* Ensure BLE service is advertising as `"ESP-DISEASE-DETECTOR"`.

### 4. Run Flask Web Dashboard

```bash
python app.py
```

Access the dashboard at: `http://localhost:5000`

---

## 🧪 Demo & Results

> 🔬 The system was successfully tested in real-world field conditions using cotton crops.
> 🎯 Achieved over 90% detection accuracy with minimal false spraying.
> 🛰️ Drone operated fully autonomously, requiring no manual intervention post-takeoff.

---

## 🤝 Acknowledgments

* Ultralytics YOLOv8
* ESP32 MicroPython Community
* OpenCV & Flask Developers
* DJI Tello SDK

* **PDF report or presentation**
  I can help integrate those too.
