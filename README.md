# Vision-Based Sensorless AMR — CCTV-Guided Differential Drive Robot

A closed-loop, camera-guided differential-drive robot that navigates **without any onboard sensors**. Overhead CCTV cameras track an ArUco marker on the robot; a Raspberry Pi 4 runs all navigation, obstacle detection, and pathfinding; and a Raspberry Pi Zero 2W on the robot executes PID motor control via MQTT. The project was developed across five phases, from basic path drawing through full multi-camera D\* Lite obstacle avoidance with MQTT-based fail-safe communication.

---

## Architecture Overview

```
[ Overhead CCTV Cameras ] ──RTSP──► [ Raspberry Pi 4 ]
                                          │
                               ┌──────────┼───────────────┐
                               │          │               │
                         ArUco Pose   MOG2 Obstacle   D* Lite
                         Tracking     Detection       Planner
                               │          │               │
                               └──────────┼───────────────┘
                                          │ MQTT (QoS 1)
                                          ▼
                                [ Raspberry Pi Zero 2W ] ── on robot
                                          │
                                    PID Motor Control
                                          │
                                    TB6612FNG H-Bridge
                                       /       \
                               Left Motor     Right Motor
```

---

## Features

- **Multi-camera RTSP ingestion** — RPi 4 pulls simultaneous streams; each runs in its own thread for non-blocking frame grabs
- **Web-based camera calibration** — Flask app served on LAN; click 4 floor reference points per camera, homography computed instantly, stored in `cameras.json`
- **Unified floor-map construction** — each frame warped from pixel space to real-world cm coordinates; all views composited into one top-down 2D map
- **MOG2 background subtraction** — detects any object regardless of color, shape, or type; no pre-configured HSV range required
- **Static vs dynamic obstacle classification** — blobs stable for >N frames trigger D\* Lite reroute; moving blobs trigger stop-and-wait
- **D\* Lite incremental pathfinding** — 20 cm grid cells, 8-direction connectivity; replans only the invalidated portion of the search graph on obstacle change (~<20 ms on RPi 4 vs ~100 ms for full A\* replan)
- **Dynamic re-routing with cooldown** — every frame checks path against live obstacle map; recalculates from current robot position through all remaining checkpoints if blocked
- **MQTT communication (QoS 1)** — Mosquitto broker on RPi 4; guaranteed at-least-once delivery; `{"la": 60, "ra": 55}` payload format
- **Watchdog fail-safe** — RPi 4 publishes heartbeat every 200 ms; RPi Zero 2W stops all motors immediately if no heartbeat received within 500 ms
- **PID steering and speed control** — full PID (Kp/Ki/Kd + anti-windup) on RPi Zero 2W; runs locally between MQTT messages
- **Short-term ArUco tracking memory** — tolerates brief detection dropouts (~0.5 s) without losing control; gray ghost dot shown during memory-based tracking
- **Click-to-place checkpoint navigation** — click waypoints on the unified floor map; robot visits each in order, stops for configurable dwell time
- **Breadcrumb interpolation** — intermediate waypoints auto-generated between checkpoints for smooth trajectories
- **Camera handoff protocol** — on stream loss, affected zone marked BLIND; robot stops if entering blind zone; overlap zones allow seamless camera switch

---

## Repository Layout

```
.
├── rpi4_navigation_server.py      # Central navigation: RTSP ingestion, homography, MOG2, D* Lite, MQTT publisher
├── dstar_lite.py                  # Standalone D* Lite pathfinding class (imported by navigation server)
├── rpizero_motor_controller.py    # On-robot: MQTT subscriber, PID control, watchdog, GPIO motor output
├── camera_calibration_app.py      # Flask web app for per-camera homography calibration
├── cameras.json                   # Auto-generated camera config (RTSP URLs + homography matrices)
│
│── Legacy Prototype (ESP32 / single-camera / A*)
├── main.py                        # Original: freehand path drawing + pure-pursuit follower (ESP32 HTTP)
├── Update_phase1.py               # Phase 1: Click checkpoints + IDLE/DRIVING/WAITING state machine
├── Update_phase2.py               # Phase 2: Breadcrumb interpolation between checkpoints
├── Update_phase3.py               # Phase 3: HSV obstacle detection + danger-zone visualization
├── Update_phase_4.py              # Phase 4: A* pathfinding + dynamic re-routing + PID + tracking memory
├── code.ino                       # Legacy ESP32 firmware (HTTP motor server + WASD web UI)
│
└── README.md
```

### File Descriptions

| File | Platform | Purpose |
|------|----------|---------|
| **rpi4_navigation_server.py** | Raspberry Pi 4 | Production controller. Multi-camera RTSP ingestion, homography stitching, MOG2 obstacle detection, D\* Lite planning, MQTT publish, ArUco tracking with memory, watchdog heartbeat |
| **dstar_lite.py** | Raspberry Pi 4 | Standalone D\* Lite pathfinding class with `calculate_key()`, `initialize()`, `update_vertex()`, `compute_shortest_path()`, `move_and_replan()`, `is_obstacle()` — imported by navigation server, testable independently |
| **rpizero_motor_controller.py** | Raspberry Pi Zero 2W | On-robot. Subscribes to MQTT motor commands, runs PID loop locally, watchdog consumer (stops if heartbeat lost), GPIO → TB6612FNG PWM output |
| **camera_calibration_app.py** | Any browser (served by RPi 4) | Flask web app; shows live RTSP stream, accepts 4 reference point clicks, computes `cv2.getPerspectiveTransform()`, saves result to `cameras.json` |
| **main.py** | PC (legacy) | Original single-camera prototype; freehand path drawing, pure-pursuit follower, ESP32 HTTP control |
| **Update_phase1.py** | PC (legacy) | Click-to-place checkpoints; three-state machine; ESP32 HTTP |
| **Update_phase2.py** | PC (legacy) | Adds breadcrumb generation between checkpoints |
| **Update_phase3.py** | PC (legacy) | Adds HSV-based blue obstacle detection and danger-zone visualization |
| **Update_phase_4.py** | PC (legacy) | A\* pathfinding, dynamic re-routing, full PID, ArUco tracking memory; ESP32 HTTP |
| **code.ino** | ESP32 (legacy) | TB6612FNG H-bridge motor server; WASD + joystick web UI; HTTP `/set?la=X&ra=Y` endpoint |

---

## Development Phases

### Prototype Phase (Legacy — PC + ESP32 + Single Camera)

#### Phase 0 — Freehand Path Follower (`main.py`)
Click-drag to draw a path; robot follows using pure-pursuit with a configurable lookahead distance. Proportional-only steering. ESP32 controlled via HTTP GET.

#### Phase 1 — Checkpoint State Machine (`Update_phase1.py`)
Click to place waypoints. Robot drives to each, stops for 3 seconds (simulating logistics pickup/drop), then proceeds. Proportional steering only.

#### Phase 2 — Breadcrumb Interpolation (`Update_phase2.py`)
Breadcrumbs auto-inserted between checkpoints every `BREADCRUMB_SPACING` px. Robot drives through breadcrumbs without stopping, pauses only at real checkpoints. Produces smoother trajectories.

#### Phase 3 — Obstacle Vision (`Update_phase3.py`)
HSV color filtering detects blue objects. Detected blobs get a padded "Danger Zone" bounding box. Boxes drawn on display but do not yet affect the path.

#### Phase 4 — A\* + Dynamic Re-routing + PID + Tracking Memory (`Update_phase_4.py`)
New checkpoints trigger A\* search from last waypoint, routing around detected obstacles. Every frame checks if any breadcrumb is inside a moved obstacle box; if so, full reroute from robot's current position. Full PID controller replaces proportional-only steering. Short-term ArUco memory for occlusion tolerance.

---

### Production System (RPi 4 + RPi Zero 2W + Multi-Camera)

#### Block A — Perceive
Multi-camera RTSP ingestion → per-camera homography → unified top-down floor map → MOG2 background subtraction → static/dynamic obstacle classification.

#### Block B — Decide
Supervisor clicks waypoints on the unified 2D floor map via web UI. D\* Lite incrementally replans around obstacles. Dynamic re-routing every frame with 1.5 s cooldown. Checkpoint arrival triggers 3 s dwell.

#### Block C — Act
MQTT QoS 1 commands from RPi 4 to RPi Zero 2W. PID motor loop runs locally on RPi Zero 2W. Watchdog: if no heartbeat in 500 ms → immediate motor stop.

---

## Hardware Requirements

### Production System
- Raspberry Pi 4 (4 GB recommended) — navigation server
- Raspberry Pi Zero 2W — on-robot motor controller
- TB6612FNG dual H-bridge motor driver (connected to RPi Zero 2W GPIO)
- 2× DC gear motors (differential drive)
- Existing facility CCTV cameras with RTSP output (or any IP camera)
- Printed ArUco marker (`DICT_4X4_50`, ID `0`) mounted on robot top
- Battery supply for motors + stable 5 V for RPi Zero 2W
- Local Wi-Fi network (2.4 GHz, stable signal)

### Legacy Prototype (for reference)
- ESP32 development board
- TB6612FNG motor driver
- Android phone running IP Webcam app

---

## RPi Zero 2W GPIO Pin Mapping

Defined in `rpizero_motor_controller.py`:

| Function | GPIO (BCM) |
|----------|-----------|
| PWMA (left motor speed) | 12 |
| AIN1 (left direction) | 23 |
| AIN2 (left direction) | 24 |
| PWMB (right motor speed) | 13 |
| BIN1 (right direction) | 27 |
| BIN2 (right direction) | 22 |
| STBY (driver enable) | 25 |

Update these constants before deploying if your wiring differs.

---

## Software Requirements

### Raspberry Pi 4 (Navigation Server)

```bash
pip install opencv-contrib-python numpy paho-mqtt flask
# Mosquitto MQTT broker
sudo apt install mosquitto mosquitto-clients
sudo systemctl enable mosquitto
```

### Raspberry Pi Zero 2W (Motor Controller)

```bash
pip install paho-mqtt RPi.GPIO
```

### PC (Legacy Prototype Only)

```bash
pip install opencv-contrib-python numpy requests
```

---

## Network Requirements

- RPi 4, RPi Zero 2W, and CCTV cameras must be on the same local network
- Prefer stable 2.4 GHz Wi-Fi with good signal strength
- Reserve static DHCP leases for all devices to avoid IP changes between sessions
- Mosquitto broker on RPi 4 must be reachable from RPi Zero 2W (default port 1883)

---

## Deploy Step-by-Step (Production System)

### 1. Configure and Start Mosquitto (RPi 4)

```bash
# /etc/mosquitto/mosquitto.conf
listener 1883
allow_anonymous true
```

```bash
sudo systemctl restart mosquitto
```

### 2. Calibrate Cameras (One-Time Setup)

```bash
# On RPi 4
python camera_calibration_app.py
```

Open `http://<RPI4_IP>:5000` in any browser. For each camera:
1. Enter RTSP URL (e.g., `rtsp://192.168.1.X/stream`)
2. Use a tape measure to mark 4 real-world reference points on the floor (note their cm coordinates)
3. Click the 4 points on the live video feed in matching order
4. Click "Compute Homography" — matrix saved to `cameras.json`

Repeat for every camera. This is a one-time setup per session (or whenever a camera is moved).

### 3. Configure Navigation Server

Edit constants at the top of `rpi4_navigation_server.py`:

```python
MQTT_BROKER_IP = "localhost"          # Mosquitto is local to RPi 4
ROBOT_ARUCO_ID = 0
MARKER_REAL_SIZE_CM = 5.7             # Measure your printed marker
CAMERAS_CONFIG = "cameras.json"       # Output from calibration step
```

### 4. Configure Motor Controller (RPi Zero 2W)

Edit constants at the top of `rpizero_motor_controller.py`:

```python
MQTT_BROKER_IP = "192.168.X.X"       # RPi 4 IP address
HEARTBEAT_TIMEOUT_MS = 500            # Stop motors if no heartbeat within this window
```

### 5. Start Motor Controller First (RPi Zero 2W)

```bash
python rpizero_motor_controller.py
```

The motor controller subscribes to MQTT and waits. It will stop immediately if the heartbeat topic goes quiet.

### 6. Start Navigation Server (RPi 4)

```bash
python rpi4_navigation_server.py
```

Opens an OpenCV window showing the unified floor map. The server also begins publishing heartbeats.

### 7. Using the UI

- **Click** on the floor map to place checkpoints (D\* Lite routes around live obstacles automatically)
- **Follow** button — starts the robot driving the planned route
- **Clear** button — stops the robot and erases all waypoints
- **ESC** — safe shutdown (motors stop, MQTT disconnect, camera release)

---

## MQTT Topics

| Topic | Publisher | Payload | Description |
|-------|-----------|---------|-------------|
| `factory/robot/01/cmd` | RPi 4 | `{"la": int, "ra": int}` | Signed wheel speeds (−255 to 255) |
| `factory/robot/01/heartbeat` | RPi 4 | `"alive"` | Published every 200 ms; loss triggers motor stop |
| `factory/robot/01/status` | RPi Zero 2W | `{"state": str}` | `"moving"` / `"stopped"` / `"watchdog_stop"` |

---

## PID Tuning Guide (Production System)

All PID gains are defined at the top of `rpi4_navigation_server.py`:

| Symptom | Fix |
|---------|-----|
| Robot wobbles / oscillates side to side | Increase `STEER_KD` (try 1.0–1.5) or decrease `STEER_KP` |
| Robot turns too slowly | Increase `STEER_KP` |
| Robot drifts consistently to one side | Increase `STEER_KI` slightly (try 0.08–0.15) |
| Robot overshoots and stops too late | Increase `SPEED_KD` |
| Robot moves too fast near targets | Decrease `SPEED_KP` |
| Robot stalls during point turns | Raise the minimum turn floor (currently 35 PWM counts) |

| Parameter | Default | Role |
|-----------|---------|------|
| `STEER_KP` | 3.00 | Proportional: corrects current heading error |
| `STEER_KI` | 0.05 | Integral: eliminates persistent drift |
| `STEER_KD` | 0.80 | Derivative: dampens oscillation |
| `STEER_INTEGRAL_LIMIT` | 2.0 | Anti-windup cap on integral accumulator |
| `SPEED_KP` | 4.50 | Distance-proportional forward speed |
| `SPEED_KD` | 1.00 | Smooth deceleration as target approaches |

---

## D\* Lite vs A\* — Why It Matters

| Property | A\* (Phase 4 legacy) | D\* Lite (production) |
|----------|----------------------|----------------------|
| Replan strategy | Full replan from scratch | Propagates only invalidated portion |
| Replan time (large floor) | ~100 ms | ~20 ms |
| Suitable for moving obstacles | No — too slow | Yes |
| Implementation complexity | Low | Moderate |

In a factory with moving humans and forklifts, the difference between 20 ms and 100 ms replanning is operationally significant — it determines whether the robot reacts before or after collision.

---

## MOG2 vs HSV Color Filtering — Why It Matters

| Property | HSV filtering (Phase 3 legacy) | MOG2 (production) |
|----------|-------------------------------|-------------------|
| Detects | Only the configured color range | Any object on the floor |
| Requires tuning | Yes — per object color | No — learns background at startup |
| Handles lighting changes | Poorly | Yes — adaptive model |
| False positives from floor markings | Common | Rare (floor is background) |
| Detects cardboard box, pallet, human | Only if color matches | Yes |

MOG2 requires a 30–50 frame background initialization at startup (empty floor). All subsequent frames are compared against this model.

---

## Key Configuration Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `MAX_SPEED` | 100 | PWM cap for motor commands (0–255) |
| `DEADZONE_DIST_CM` | 3.0 | Distance threshold to consider a waypoint reached |
| `DEADZONE_ANGLE` | 0.3 rad | Angle threshold below which steering correction is skipped |
| `POINT_TURN_ANGLE` | 0.6 rad | Above this error, robot stops forward motion and spins in place |
| `DANGER_ZONE_PADDING_CM` | 12 cm | Safety box extends this far beyond detected obstacle edge |
| `PATH_BUFFER_CM` | 13 cm | Extra invisible margin fed to D\* Lite to keep path away from obstacles |
| `RECALC_COOLDOWN` | 1.5 s | Minimum time between dynamic re-route calculations (absorbs camera jitter) |
| `MAX_LOST_FRAMES` | 15 | ArUco tracking memory: tolerate this many lost frames before giving up |
| `HEARTBEAT_INTERVAL_MS` | 200 | RPi 4 publishes heartbeat at this interval |
| `HEARTBEAT_TIMEOUT_MS` | 500 | RPi Zero 2W stops motors if no heartbeat within this window |
| `GRID_SIZE_CM` | 20 | D\* Lite grid cell size in cm |
| `STOP_DURATION` | 3.0 s | Robot dwell time at each checkpoint (simulated load/unload) |
| `BREADCRUMB_SPACING_CM` | 15 cm | Intermediate waypoint interval between checkpoints |
| `MOG2_HISTORY` | 50 | Background model frames (empty-floor capture at startup) |
| `MOG2_VAR_THRESHOLD` | 40 | Sensitivity of foreground detection |

---

## Troubleshooting

| Problem | Solution |
|---------|----------|
| Robot doesn't move | Check MQTT connection; verify `heartbeat` topic is publishing; inspect GPIO wiring |
| Watchdog fires immediately | RPi 4 heartbeat not publishing; check Mosquitto broker is running |
| High video delay | Lower camera resolution/FPS; ensure cameras are on 5 GHz if available |
| Marker not detected | Improve lighting; print marker with sharp black-on-white edges |
| MOG2 flags entire floor | Background model captured with robot already on floor; restart with empty floor |
| D\* Lite finds no path | Reduce `DANGER_ZONE_PADDING_CM` / `PATH_BUFFER_CM` if obstacles are tightly packed |
| Camera calibration drift | Re-run `camera_calibration_app.py`; check that reference points are not moved |
| Wrong turning direction | Swap `AIN1`/`AIN2` or `BIN1`/`BIN2` pin assignments in `rpizero_motor_controller.py` |
| Robot oscillates at speed | Decrease `STEER_KP`; increase `STEER_KD` |
| Robot drifts on straight runs | Increase `STEER_KI` gradually |

---

## Quick Validation Checklist

- [ ] Mosquitto broker running on RPi 4: `mosquitto_sub -t factory/robot/01/heartbeat` shows `"alive"` every 200 ms
- [ ] RPi Zero 2W subscribed and motors respond: publish `{"la": 40, "ra": 40}` to `factory/robot/01/cmd`
- [ ] Camera RTSP streams open in VLC before starting the server
- [ ] Background model initialized with empty floor (no robot, no obstacles)
- [ ] ArUco marker detected — green dot appears on robot in the UI
- [ ] Placing a checkpoint routes around visible obstacles automatically

---

## Safety Notes

- Always start `rpizero_motor_controller.py` **before** `rpi4_navigation_server.py` so the watchdog is armed before any commands are sent
- Test on a stand first (wheels off ground) before deploying on the floor
- Keep an emergency stop ready (cut motor battery or publish `{"la": 0, "ra": 0}` to the cmd topic)
- Do not run near edges, stairs, or fragile objects
- The watchdog only covers network/software faults — it does not replace a physical emergency stop for high-speed or heavy-load deployments

---

## Runtime API (MQTT)

```bash
# Move forward
mosquitto_pub -h <RPI4_IP> -t factory/robot/01/cmd -m '{"la": 50, "ra": 50}'

# Stop
mosquitto_pub -h <RPI4_IP> -t factory/robot/01/cmd -m '{"la": 0, "ra": 0}'

# Monitor heartbeat
mosquitto_sub -h <RPI4_IP> -t factory/robot/01/heartbeat

# Monitor robot status
mosquitto_sub -h <RPI4_IP> -t factory/robot/01/status
```

---

## References

**[1]** Koenig, S., & Likhachev, M. (2002). D\* Lite. *Proceedings of the 18th National Conference on Artificial Intelligence (AAAI '02)*, pp. 476–483.

**[2]** Garrido-Jurado, S. et al. (2014). Automatic generation and detection of highly reliable fiducial markers under occlusion. *Pattern Recognition*, 47(6), 2280–2292.

**[3]** Zivkovic, Z. (2004). Improved adaptive Gaussian mixture model for background subtraction. *ICPR 2004*.

**[4]** World Bank. (2021). *Gearing Up for the Future of Manufacturing in Bangladesh*. World Bank Group.

---

## License

This project is licensed under the **GNU General Public License v3.0**.

