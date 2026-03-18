# Vision-Based Sensorless Bot

A closed-loop, camera-guided differential-drive robot that navigates **without any onboard sensors**. An overhead IP camera tracks an ArUco marker on the robot, and a Python controller sends wheel-speed commands to an ESP32 over Wi-Fi. The project was developed incrementally across five phases, from basic path drawing to full A\* obstacle avoidance with PID-stabilized control.

## Features

- **ArUco pose tracking** — real-time position and heading from a `DICT_4X4_50` marker (ID `0`)
- **Click-to-place checkpoint navigation** — drop waypoints on the video feed; the robot visits each in order
- **Breadcrumb interpolation** — auto-generated intermediate waypoints between checkpoints for smooth curves
- **HSV-based obstacle detection** — blue objects detected in real time with configurable danger zones
- **A\* pathfinding** — eight-direction grid search that routes around obstacles with safety margins
- **Dynamic re-routing** — if an obstacle moves into the planned path, the route is recalculated on the fly
- **PID steering & speed control** — proportional-integral-derivative controller for stable, oscillation-free driving
- **Short-term tracking memory** — tolerates brief ArUco detection dropouts (~0.5 s) without losing control
- **ESP32 web UI** — built-in keyboard (WASD) and virtual joystick control page for manual driving
- **Threaded camera reader** — low-latency frame grabbing suitable for 2.4 GHz Wi-Fi links

## Repository Layout

```
.
├── main.py                 # Original prototype: freehand path drawing + pure-pursuit follower
├── Update_phase1.py        # Phase 1: Click-to-place checkpoints + state machine (IDLE/DRIVING/WAITING)
├── Update_phase2.py        # Phase 2: Breadcrumb interpolation between checkpoints
├── Update_phase3.py        # Phase 3: HSV obstacle detection + danger-zone visualization
├── Update_phase_4.py       # Phase 4: A* pathfinding + dynamic re-routing around obstacles
├── claude.py               # Phase 5 (latest): Phase 4 + PID control + tracking memory
├── code.ino                # ESP32 firmware (Wi-Fi + TB6612 motor HTTP server + web UI)
└── README.md
```

### File Descriptions

| File | Purpose |
|------|---------|
| **main.py** | The original controller. Draw a freehand path with click-drag; the robot follows using pure-pursuit with a configurable lookahead distance. Uses proportional-only steering. |
| **Update_phase1.py** | Replaces freehand drawing with click-to-place checkpoints. Adds a three-state machine (`IDLE` → `DRIVING` → `WAITING`) so the robot stops for 3 seconds at each checkpoint to simulate loading/unloading. |
| **Update_phase2.py** | Adds breadcrumb generation: when you click a new checkpoint, intermediate waypoints are auto-inserted every 20 px so the robot follows a smooth line instead of beelining between distant points. |
| **Update_phase3.py** | Adds real-time obstacle detection using HSV color filtering (default: blue). Detected objects get a padded "Danger Zone" bounding box drawn on screen. Path planning is still manual. |
| **Update_phase_4.py** | Replaces manual path segments with A\* grid search (20 px cells, 8-direction connectivity). Obstacles feed a padded invisible collision box into the planner. If the obstacle moves into the active path, the route is recalculated automatically after a 1.5 s cooldown. |
| **claude.py** | **The latest and most complete version.** Builds on Phase 4 and adds: PID steering controller (Kp/Ki/Kd with anti-windup), PD forward speed controller (smooth deceleration), and short-term ArUco tracking memory (ghost position for up to 15 lost frames). |
| **code.ino** | ESP32 firmware. Connects to Wi-Fi, serves a web page with WASD keyboard and virtual joystick controls, and exposes HTTP endpoints for programmatic motor control. Uses TB6612-style dual-channel PWM. |

## Development Phases

### Phase 1 — Checkpoint State Machine
Click to place waypoints. The robot drives to each one, stops for 3 seconds (simulating a logistics pickup/drop), then proceeds to the next. Proportional steering only.

### Phase 2 — Breadcrumb Interpolation
Breadcrumbs are auto-inserted between checkpoints (every `BREADCRUMB_SPACING` px). The robot drives through breadcrumbs without stopping but pauses at real checkpoints. This produces smoother trajectories.

### Phase 3 — Obstacle Vision
A second processing pass converts each frame to HSV and masks for blue objects. Detected blobs above `MIN_OBSTACLE_AREA` get a padded bounding box (`DANGER_ZONE_PADDING`). The boxes are drawn on the display but do not yet affect the path.

### Phase 4 — A\* Pathfinding & Dynamic Re-routing
Clicking a new checkpoint now triggers an A\* search from the last waypoint to the click location, routing around detected obstacle boxes (inflated by `PATH_BUFFER_PX`). While driving, the system checks every frame whether any breadcrumb has been swallowed by a moved obstacle; if so, a full re-route is computed from the robot's current position through all remaining checkpoints.

### Phase 5 — PID Control & Tracking Memory
Replaces the proportional-only steering with a full PID controller:

| Parameter | Default | Role |
|-----------|---------|------|
| `STEER_KP` | 2.00 | Proportional: corrects current heading error |
| `STEER_KI` | 0.05 | Integral: eliminates persistent drift over time |
| `STEER_KD` | 0.80 | Derivative: dampens oscillation |
| `STEER_INTEGRAL_LIMIT` | 2.0 | Anti-windup cap on the integral accumulator |
| `SPEED_KP` | 4.50 | Distance-proportional forward speed |
| `SPEED_KD` | 1.00 | Smooth deceleration as target approaches |

Short-term memory keeps the last-known position and heading for up to `MAX_LOST_FRAMES` (15) frames when the ArUco marker is temporarily occluded. A gray "ghost" dot indicates memory-based tracking.

## Hardware Requirements

- ESP32 development board
- 2× DC gear motors (differential drive)
- TB6612FNG (or equivalent) dual H-bridge motor driver
- Android phone running an IP camera app (e.g., IP Webcam)
- Battery supply for motors + stable 5 V for ESP32
- Printed ArUco marker (`DICT_4X4_50`, ID `0`) mounted on robot top

## ESP32 Pin Mapping

Defined in `code.ino`:

| Function | GPIO |
|----------|------|
| PWMA (left motor speed) | 25 |
| AIN1 (left motor dir) | 16 |
| AIN2 (left motor dir) | 21 |
| PWMB (right motor speed) | 22 |
| BIN1 (right motor dir) | 26 |
| BIN2 (right motor dir) | 27 |
| STBY (driver enable) | 13 |

Update these constants before uploading if your wiring differs.

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

- Phone, ESP32, and computer must be on the **same local network**
- Prefer stable 2.4 GHz Wi-Fi with good signal strength
- Reserve static DHCP leases for ESP32 and phone to avoid IP changes between sessions

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
2. Set resolution to `640×480` for lower latency.
3. Start server and note phone IP.
4. Confirm stream URL works in browser:

```
http://<PHONE_IP>:8080/video
```

### 3. Configure Python Controller

1. Create and activate a virtual environment:

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
```

2. Install dependencies:

```powershell
pip install opencv-contrib-python numpy requests
```

3. Edit configuration in your chosen script (e.g., `claude.py`):

```python
ESP_IP = "http://<ESP32_IP>"
url = "http://<PHONE_IP>:8080/video"
ROBOT_ARUCO_ID = 0
MARKER_REAL_SIZE_CM = 5.7   # Measure your printed marker in cm
```

### 4. Run the System

To run the latest version with all features (PID + obstacle avoidance + A\*):

```powershell
python claude.py
```

Or run any earlier phase to test incrementally:

```powershell
python Update_phase1.py   # Checkpoints only
python Update_phase2.py   # + Breadcrumbs
python Update_phase3.py   # + Obstacle vision
python Update_phase_4.py  # + A* pathfinding
```

### 5. Using the UI

- **Click** on the video feed (below the header bar) to place checkpoints
- **Follow** button — starts the robot driving through the planned route
- **Clear** button — stops the robot and erases all waypoints
- **ESC** — safe shutdown (motors stop, camera releases)

## Quick Validation Checklist

- [ ] Opening `http://<ESP32_IP>/` shows the WASD + joystick control page
- [ ] Calling `http://<ESP32_IP>/set?la=30&ra=30` moves the robot forward
- [ ] Camera stream URL opens and updates live in a browser
- [ ] ArUco marker is detected and a green dot appears on the robot in the UI

## Runtime API (ESP32)

| Endpoint | Description |
|----------|-------------|
| `GET /set?la=<int>&ra=<int>` | Set signed wheel speeds (−255 to 255) |
| `GET /F` | Forward at default speed |
| `GET /B` | Backward at default speed |
| `GET /L` | Spin left |
| `GET /R` | Spin right |
| `GET /S` | Stop both motors |

## PID Tuning Guide

All PID gains are defined at the top of `claude.py`. Adjust and test in short runs:

| Symptom | Fix |
|---------|-----|
| Robot wobbles/oscillates side to side | Increase `STEER_KD` (e.g., 1.0–1.5) or decrease `STEER_KP` |
| Robot turns too slowly | Increase `STEER_KP` |
| Robot drifts consistently to one side | Increase `STEER_KI` slightly (e.g., 0.08–0.15) |
| Robot overshoots and stops too late | Increase `SPEED_KD` |
| Robot moves too fast near targets | Decrease `SPEED_KP` |
| Robot stalls during point turns | Increase the minimum `turn` floor (currently 35) |

Other key parameters:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `MAX_SPEED` | 100 | PWM cap for motor commands (0–255) |
| `DEADZONE_DIST_CM` | 3.0 | Distance threshold to consider a waypoint reached |
| `DEADZONE_ANGLE` | 0.3 rad | Angle threshold below which steering correction is skipped |
| `POINT_TURN_ANGLE` | 0.6 rad | Above this error, robot stops forward motion and spins in place |
| `DANGER_ZONE_PADDING` | 60 px | How far the safety box extends beyond the detected obstacle edge |
| `PATH_BUFFER_PX` | 65 px | Extra invisible margin fed to A\* to keep the path away from obstacles |
| `RECALC_COOLDOWN` | 1.5 s | Minimum time between dynamic re-route calculations |

## Troubleshooting

| Problem | Solution |
|---------|----------|
| High video delay | Lower camera resolution/FPS; keep phone close to router |
| Robot not moving | Verify `/set` endpoint in a browser; check motor wiring and `STBY` pin |
| Marker not detected | Improve lighting; print marker with sharp black-on-white edges |
| Wrong turning direction | Swap motor polarity or left/right channel mapping in `setMotors()` |
| Intermittent control | Use stronger Wi-Fi; avoid crowded hotspot channels |
| A\* finds no path | Reduce `DANGER_ZONE_PADDING` / `PATH_BUFFER_PX` if obstacles are close together |
| Robot panics near obstacles | Increase `RECALC_COOLDOWN` or the jitter immunity radius (currently 60 px) |

## Safety Notes

- Test on a stand first (wheels off ground) before deploying on the floor
- Keep an emergency stop method ready (power switch or unplug battery)
- Do not run near edges, stairs, or fragile objects

## License

This project is licensed under the **GNU General Public License v3.0**. See the license header in `code.ino` for details.

## Credits

Created by **Vikas Singh Thakur**. This version adds threaded camera streaming, A\* obstacle avoidance, PID control, and deployment-oriented tuning for constrained 2.4 GHz setups.