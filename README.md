# Vision-Based Sensorless Bot

This project is a closed-loop, camera-guided differential drive robot that works without onboard distance sensors. An overhead IP camera tracks an ArUco marker on the robot, and a Python controller sends wheel-speed commands to an ESP32 over Wi-Fi.

## What You Can Deploy Right Now

- Real-time path drawing from a desktop/laptop UI (`main.py`)
- ArUco-based pose tracking (`DICT_4X4_50`, marker ID `0`)
- HTTP motor control on ESP32 via `/set?la=<int>&ra=<int>`
- Low-latency threaded camera reader for older 2.4 GHz networks

## Repository Layout

```
.
|-- main.py                    # Vision + path-follow controller (Python)
|-- code.ino                   # ESP32 firmware (Wi-Fi + motor HTTP server)
|-- sketch_mar3a/sketch_mar3a.ino
`-- README.md
```

`code.ino` and `sketch_mar3a/sketch_mar3a.ino` currently contain the same firmware logic.

## Hardware Requirements

- ESP32 development board
- 2x DC gear motors (differential drive)
- Motor driver (firmware is currently written for TB6612-style direction + PWM control)
- Android phone running an IP camera app (for example, IP Webcam)
- Battery supply for motors + stable 5V supply for ESP32
- Printed ArUco marker (`DICT_4X4_50`, ID `0`) mounted on robot top

## ESP32 Pin Mapping (Current Firmware)

Defined in `code.ino`:

- `PWMA = 25`
- `AIN1 = 16`
- `AIN2 = 21`
- `PWMB = 22`
- `BIN1 = 26`
- `BIN2 = 27`
- `STBY = 13`

If your wiring differs, update these constants before upload.

## Software Requirements

### PC / Laptop

- Python 3.9+
- `opencv-contrib-python`
- `numpy`
- `requests`

### ESP32 Build Environment

- Arduino IDE 2.x (or PlatformIO)
- ESP32 board package installed
- `WiFi.h` and `WebServer.h` (bundled with ESP32 core)

## Network Requirements

- Phone, ESP32, and computer must be on the same local network
- Prefer stable 2.4 GHz Wi-Fi with good signal strength
- If possible, reserve static DHCP leases for ESP32 and phone to avoid IP changes

## Deploy Step-by-Step

### 1. Flash ESP32 Firmware

1. Open `code.ino` in Arduino IDE.
2. Set your Wi-Fi credentials:

```cpp
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";
```

3. Select your ESP32 board and COM port.
4. Upload firmware.
5. Open Serial Monitor (`115200`) and note the printed ESP32 IP address.

### 2. Start Phone Camera Stream

1. Install and open IP Webcam (or equivalent).
2. Set a moderate resolution (example: `640x480`) for lower latency.
3. Start server and note phone IP.
4. Confirm stream URL works in browser:

```text
http://<PHONE_IP>:8080/video
```

### 3. Configure Python Controller

1. From project root, create and activate a virtual environment.

Windows PowerShell:

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
```

2. Install dependencies:

```powershell
pip install opencv-contrib-python numpy requests
```

3. Edit `main.py` configuration:

```python
ESP_IP = "http://<ESP32_IP>"
url = "http://<PHONE_IP>:8080/video"
ROBOT_ARUCO_ID = 0
MARKER_REAL_SIZE_CM = 5.7
```

Set `MARKER_REAL_SIZE_CM` to your printed marker's real edge length in centimeters.

### 4. Run the System

```powershell
python main.py
```

In the OpenCV window:

- Draw a path below the top header bar
- Click `Follow` to start tracking
- Click `Clear` to stop and erase path
- Press `ESC` for safe shutdown (motors stop)

## Quick Validation Checklist

- Opening `http://<ESP32_IP>/` shows control webpage
- Calling `http://<ESP32_IP>/set?la=30&ra=30` moves robot forward
- Camera stream URL opens and updates live
- ArUco marker is detected and robot dot appears in UI

## Runtime API (ESP32)

- `GET /set?la=<int>&ra=<int>`: set signed wheel speeds (`-255` to `255` recommended)
- `GET /F`, `GET /B`, `GET /L`, `GET /R`, `GET /S`: legacy movement endpoints

## Control Tuning (in `main.py`)

- `MAX_SPEED`
- `TURN_GAIN`
- `FWD_GAIN`
- `DEADZONE_DIST_CM`
- `DEADZONE_ANGLE`
- `LOOKAHEAD_DIST_CM`

Start with small changes and test in short runs.

## Troubleshooting

- High delay in video: lower camera resolution/FPS and keep phone close to router
- Robot not moving: verify `/set` endpoint manually in browser and check motor wiring
- Marker not detected: increase lighting and print marker with sharp edges
- Wrong turning direction: swap motor polarity or left/right channel mapping in `setMotors`
- Intermittent control: use stronger Wi-Fi and avoid crowded hotspot channels

## Safety Notes

- Test on a stand first (wheels off ground)
- Keep an emergency stop method ready (power switch or unplug)
- Do not run near edges, stairs, or fragile objects

## Credits

Core concept inspired by Vikas Singh Thakur's vision-based robotics work; this version adds low-latency threaded streaming and deployment-oriented tuning for constrained 2.4 GHz setups.