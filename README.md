# Context Awareness System

A multimodal context-aware desk system built using a Raspberry Pi, ESP32, and sensors.  
The system recognizes **objects**, **gestures**, and **RFID tags** to automatically switch between modes and control connected devices.  
It integrates **Spotify**, **KDE Connect**, and a **Flask web dashboard** to create a seamless smart-desk experience.

---

## 🚀 Features

### 🎥 Object Detection (camera.py)
- Detects Book / Pen → **Study Mode**
- Detects Mug / Cup → **Relax Mode**
- Uses Raspberry Pi Camera + OpenCV
- Combines visual features with IR temperature sensing for reliability

### ✋ Gesture Recognition (ESP32)
- Supported gestures: Tap, Flick, Hold, Up, Down, Custom
- Controls on the active device:
  - Play/Pause  
  - Next track  
  - Volume up/down  
  - Toggle active device  
  - Send clipboard  

### 🏷 RFID-Based Device Switching
- Each RFID tag corresponds to a device (via KDE Connect)
- Placing a tag activates the associated device  
- Removing the tag deactivates it  

### 🎵 Media & Device Control
- Spotify playlist and playback management  
- KDE Connect integration:
  - Volume  
  - Clipboard sharing  
  - File transfer  
  - MPRIS control  
- Auto mute/unmute notifications  

### 🌐 Web Dashboard (Flask)
Shows:
- System mode (Study/Relax)  
- Last gesture  
- Last detected object  
- Active/previous device  
- Temperature readings  
- Diagnostic tools  

---

## 📁 Repository Structure

.
├── main.py # Core logic: serial handler, Spotify, KDE Connect, Flask UI
├── camera.py # Object detection using OpenCV + PiCamera2
├── ESP32_Code/
│ └── ESP32_Code.ino # Gesture + RFID firmware (ESP32)
├── docs/
│ └── Context Awareness System Project Report.pdf
├── requirements.txt # Python dependencies
└── README.md # Project documentation

yaml
Copy code

---

## 🧩 Hardware Used

- Raspberry Pi 4  
- Pi Camera  
- ESP32 DevKit  
- MFRC522 RFID Reader  
- MLX90614 IR Temperature Sensor  
- DHT11 Temp & Humidity Sensor  
- VL53L0X or similar ToF Gesture Sensor  

---

## 📦 Installation

### 1️⃣ Install Python Packages

```bash
pip install -r requirements.txt
2️⃣ Install Raspberry Pi Dependencies
bash
Copy code
sudo apt install python3-opencv python3-picamera2
sudo apt install kdeconnect dunst
sudo apt install wl-clipboard xclip xsel
3️⃣ Spotify Setup
Create a Spotify Developer App and add this redirect URI:

arduino
Copy code
http://127.0.0.1:8888/callback
Export credentials:

bash
Copy code
export SPOTIPY_CLIENT_ID="your_id"
export SPOTIPY_CLIENT_SECRET="your_secret"
export SPOTIPY_REDIRECT_URI="http://127.0.0.1:8888/callback"
4️⃣ Flash the ESP32 Firmware
bash
Copy code
# Using Arduino IDE or PlatformIO
# Upload the following file:
ESP32_Code/ESP32_Code.ino
▶️ Running the System
1️⃣ Start the main program
bash
Copy code
python3 main.py /dev/ttyUSB0
(Replace /dev/ttyUSB0 with your ESP32’s port.)

2️⃣ Open the dashboard
cpp
Copy code
http://<raspberry-pi-ip>:5000
