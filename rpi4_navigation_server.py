# rpi4_navigation_server.py — Central navigation brain (RPi 4)
# FALLBACK: single camera mode
# Set CAMERAS_CONFIG = None and SINGLE_RTSP_URL = "rtsp://..." to use one camera
# without the full multi-camera setup. The system will skip homography warping
# and treat the raw frame as the floor map directly.
import cv2, numpy as np, math, json, time, threading, os, sys
try:
    import paho.mqtt.client as mqtt
except ImportError:
    mqtt = None
from collections import deque
from dstar_lite import DStarLite

# ========================== CONSTANTS ==========================
MQTT_BROKER_IP = "localhost"
ROBOT_ARUCO_ID = 0
MARKER_REAL_SIZE_CM = 5.7
CAMERAS_CONFIG = "cameras.json"
SINGLE_RTSP_URL = None
FLOOR_WIDTH_CM = 400
FLOOR_HEIGHT_CM = 300
MAX_SPEED = 100
DEADZONE_DIST_CM = 3.0
DEADZONE_ANGLE = 0.3
POINT_TURN_ANGLE = 0.6
BREADCRUMB_SPACING_CM = 15
STEER_KP = 3.00
STEER_KI = 0.05
STEER_KD = 0.80
STEER_INTEGRAL_LIMIT = 2.0
SPEED_KP = 4.50
SPEED_KD = 1.00
GRID_SIZE_CM = 20
PATH_BUFFER_CM = 13
DANGER_ZONE_PADDING_CM = 12
RECALC_COOLDOWN = 1.5
MAX_LOST_FRAMES = 15
MOG2_HISTORY = 50
MOG2_VAR_THRESHOLD = 40
MIN_OBSTACLE_AREA = 1500
STOP_DURATION = 3.0
HEARTBEAT_INTERVAL_MS = 200
HEADER_HEIGHT = 60
DISPLAY_W, DISPLAY_H = 800, 600

def log(msg):
    print(f"[{time.strftime('%H:%M:%S')}] {msg}")

# ========================== VIDEO STREAM ==========================
class VideoStream:
    def __init__(self, src=0):
        self.stream = cv2.VideoCapture(src)
        self.grabbed, self.frame = self.stream.read()
        self.stopped = False
        self.alive = True
    def start(self):
        threading.Thread(target=self._update, daemon=True).start()
        return self
    def _update(self):
        while not self.stopped:
            self.grabbed, self.frame = self.stream.read()
            if not self.grabbed:
                self.alive = False
    def read(self):
        return self.frame
    def stop(self):
        self.stopped = True
        self.stream.release()

# ========================== CAMERA MANAGER ==========================
class CameraManager:
    def __init__(self):
        self.cameras = []
        self.streams = []
        self.bg_subtractors = []
        self.blind_zones = []
    def load(self, config_path):
        if config_path and os.path.exists(config_path):
            with open(config_path) as f:
                self.cameras = json.load(f)
            log(f"Loaded {len(self.cameras)} cameras from {config_path}")
        elif SINGLE_RTSP_URL:
            self.cameras = [{"rtsp_url": SINGLE_RTSP_URL, "homography_matrix": None, "zone_bounds": [0,0,FLOOR_WIDTH_CM,FLOOR_HEIGHT_CM]}]
            log(f"Fallback: single camera mode ({SINGLE_RTSP_URL})")
        else:
            log("WARNING: No cameras configured"); return
        for i, cam in enumerate(self.cameras):
            s = VideoStream(cam["rtsp_url"]).start()
            self.streams.append(s)
            self.bg_subtractors.append(cv2.createBackgroundSubtractorMOG2(history=MOG2_HISTORY, varThreshold=MOG2_VAR_THRESHOLD))
            self.blind_zones.append(False)
            threading.Thread(target=self._heartbeat, args=(i,), daemon=True).start()
            log(f"Camera {i} started: {cam['rtsp_url']}")
        time.sleep(1.0)
        self._warmup_bg()
    def _heartbeat(self, idx):
        while True:
            time.sleep(1.0)
            if idx < len(self.streams) and not self.streams[idx].alive:
                if not self.blind_zones[idx]:
                    self.blind_zones[idx] = True
                    log(f"ALERT: Camera {idx} dead — blind zone flagged")
    def _warmup_bg(self):
        log("Warming up background subtractors...")
        for _ in range(MOG2_HISTORY):
            for i, s in enumerate(self.streams):
                f = s.read()
                if f is not None:
                    self.bg_subtractors[i].apply(f)
            time.sleep(0.03)
        log("Background models initialized.")
    def composite(self):
        floor = np.zeros((DISPLAY_H, DISPLAY_W, 3), np.uint8)
        sx = DISPLAY_W / FLOOR_WIDTH_CM
        sy = DISPLAY_H / FLOOR_HEIGHT_CM
        for i, cam in enumerate(self.cameras):
            if self.blind_zones[i]: continue
            f = self.streams[i].read()
            if f is None: continue
            H = cam.get("homography_matrix")
            if H:
                Hm = np.float32(H)
                warped = cv2.warpPerspective(f, Hm, (FLOOR_WIDTH_CM, FLOOR_HEIGHT_CM))
                resized = cv2.resize(warped, (DISPLAY_W, DISPLAY_H))
                mask = (resized > 0).any(axis=2).astype(np.uint8) * 255
                np.copyto(floor, resized, where=mask[:,:,None]>0)
            else:
                floor = cv2.resize(f, (DISPLAY_W, DISPLAY_H))
        return floor
    def detect_obstacles(self):
        raw_obs = []
        sx = DISPLAY_W / FLOOR_WIDTH_CM
        sy = DISPLAY_H / FLOOR_HEIGHT_CM
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7,7))
        for i, cam in enumerate(self.cameras):
            if self.blind_zones[i]: continue
            f = self.streams[i].read()
            if f is None: continue
            fg = self.bg_subtractors[i].apply(f, learningRate=0)
            fg = cv2.morphologyEx(fg, cv2.MORPH_CLOSE, kernel)
            fg = cv2.morphologyEx(fg, cv2.MORPH_OPEN, kernel)
            cnts, _ = cv2.findContours(fg, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            H = cam.get("homography_matrix")
            for cnt in cnts:
                if cv2.contourArea(cnt) < MIN_OBSTACLE_AREA: continue
                x, y, w, h = cv2.boundingRect(cnt)
                if H:
                    Hm = np.float32(H)
                    corners = np.float32([[x,y],[x+w,y],[x+w,y+h],[x,y+h]]).reshape(-1,1,2)
                    tc = cv2.perspectiveTransform(corners, Hm).reshape(-1,2)
                    xc, yc = tc[:,0].min(), tc[:,1].min()
                    wc, hc = tc[:,0].max()-xc, tc[:,1].max()-yc
                else:
                    xc = x / sx; yc = y / sy; wc = w / sx; hc = h / sy
                raw_obs.append((float(xc), float(yc), float(wc), float(hc)))
        return raw_obs

# ========================== OBSTACLE CLASSIFIER ==========================
class ObstacleClassifier:
    def __init__(self, window=5, threshold=5.0):
        self.history = {}
        self.window = window
        self.threshold = threshold
    def classify(self, obstacles):
        static_obs, dynamic_obs = [], []
        new_hist = {}
        for i, (x, y, w, h) in enumerate(obstacles):
            cx, cy = x + w/2, y + h/2
            key = f"{int(cx/20)}_{int(cy/20)}"
            prev = self.history.get(key, deque(maxlen=self.window))
            prev.append((cx, cy))
            new_hist[key] = prev
            if len(prev) >= 2:
                dx = prev[-1][0] - prev[0][0]
                dy = prev[-1][1] - prev[0][1]
                if math.hypot(dx, dy) < self.threshold:
                    static_obs.append((x, y, w, h))
                else:
                    dynamic_obs.append((x, y, w, h))
            else:
                static_obs.append((x, y, w, h))
        self.history = new_hist
        return static_obs, dynamic_obs

# ========================== MQTT HANDLER ==========================
class MQTTHandler:
    def __init__(self):
        self.client = None
        self.robot_status = "unknown"
        self.connected = False
    def connect(self):
        if mqtt is None:
            log("paho-mqtt not installed — MQTT disabled"); return
        self.client = mqtt.Client(client_id="rpi4_nav_server")
        self.client.on_connect = self._on_connect
        self.client.message_callback_add("factory/robot/01/status", self._on_status)
        try:
            self.client.connect(MQTT_BROKER_IP, 1883, keepalive=60)
            self.client.loop_start()
            self.connected = True
            threading.Thread(target=self._heartbeat_loop, daemon=True).start()
        except Exception as e:
            log(f"MQTT connect failed: {e}")
    def _on_connect(self, c, u, f, rc):
        log(f"MQTT connected (rc={rc})")
        c.subscribe("factory/robot/01/status", qos=1)
    def _on_status(self, c, u, msg):
        try: self.robot_status = json.loads(msg.payload.decode()).get("state","?")
        except: pass
    def _heartbeat_loop(self):
        while True:
            self.publish_raw("factory/robot/01/heartbeat", "alive")
            time.sleep(HEARTBEAT_INTERVAL_MS / 1000.0)
    def send_motor(self, la, ra):
        self.publish_raw("factory/robot/01/cmd", json.dumps({"la":int(la),"ra":int(ra)}))
    def publish_raw(self, topic, payload):
        if not self.connected: return
        try: self.client.publish(topic, payload, qos=1)
        except Exception as e: log(f"MQTT pub error: {e}")

# ========================== ARUCO TRACKER ==========================
class ArucoTracker:
    def __init__(self):
        d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        p = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(d, p)
        self.last_pos = None
        self.last_angle = None
        self.lost_count = 0
    def detect(self, frame):
        corners, ids, _ = self.detector.detectMarkers(frame)
        if ids is not None:
            for i, mid in enumerate(ids.flatten()):
                if mid == ROBOT_ARUCO_ID:
                    c = corners[i][0]
                    cx, cy = float(np.mean(c[:,0])), float(np.mean(c[:,1]))
                    fx = (c[0][0]+c[1][0])/2; fy = (c[0][1]+c[1][1])/2
                    angle = math.atan2(fy-cy, fx-cx)
                    self.last_pos = (cx, cy)
                    self.last_angle = angle
                    self.lost_count = 0
                    return (cx, cy), angle, True
        self.lost_count += 1
        if self.lost_count < MAX_LOST_FRAMES and self.last_pos:
            return self.last_pos, self.last_angle, False
        return None, None, False

# ========================== MAIN ==========================
def main():
    cam_mgr = CameraManager()
    cam_mgr.load(CAMERAS_CONFIG)
    if not cam_mgr.streams:
        log("No cameras. Exiting."); return
    mq = MQTTHandler(); mq.connect()
    tracker = ArucoTracker()
    classifier = ObstacleClassifier()
    planner = DStarLite(FLOOR_WIDTH_CM, FLOOR_HEIGHT_CM, GRID_SIZE_CM)
    planner.path_buffer = PATH_BUFFER_CM
    # State
    checkpoints = []
    path_cells = []
    following = False
    robot_state = "IDLE"
    wait_start = 0
    last_recalc = 0
    breadcrumbs = deque(maxlen=200)
    pid_i, pid_pe, pid_pd, pid_t = 0.0, 0.0, 0.0, time.time()
    detected_static = []
    detected_dynamic = []
    sx = DISPLAY_W / FLOOR_WIDTH_CM
    sy = DISPLAY_H / FLOOR_HEIGHT_CM

    FOLLOW_BTN = {"x1":10,"y1":10,"x2":130,"y2":50}
    CLEAR_BTN = {"x1":150,"y1":10,"x2":270,"y2":50}

    def px(cm_x): return int(cm_x * sx)
    def py(cm_y): return int(cm_y * sy)

    def place_checkpoint(x_cm, y_cm):
        nonlocal path_cells, following, checkpoints
        checkpoints.append((x_cm, y_cm))
        planner.goal = planner.cell_from_cm(x_cm, y_cm)
        if tracker.last_pos:
            rp = tracker.last_pos
            rpx_cm = rp[0] / sx; rpy_cm = rp[1] / sy
            planner.start = planner.cell_from_cm(rpx_cm, rpy_cm)
        planner.initialize()
        planner.compute_shortest_path()
        path_cells = planner.extract_path()
        log(f"Checkpoint at ({x_cm:.0f},{y_cm:.0f})cm, path={len(path_cells)} cells")

    def mouse_cb(event, mx, my, flags, param):
        nonlocal following, robot_state, checkpoints, path_cells
        nonlocal pid_i, pid_pe, pid_pd, pid_t
        if event != cv2.EVENT_LBUTTONDOWN: return
        if my < HEADER_HEIGHT:
            if FOLLOW_BTN["x1"]<=mx<=FOLLOW_BTN["x2"] and FOLLOW_BTN["y1"]<=my<=FOLLOW_BTN["y2"]:
                if path_cells:
                    following = True; robot_state = "DRIVING"
                    pid_i=0; pid_pe=0; pid_pd=0; pid_t=time.time()
                return
            if CLEAR_BTN["x1"]<=mx<=CLEAR_BTN["x2"] and CLEAR_BTN["y1"]<=my<=CLEAR_BTN["y2"]:
                checkpoints=[]; path_cells=[]; following=False; robot_state="IDLE"
                mq.send_motor(0,0); breadcrumbs.clear()
                return
        else:
            x_cm = (mx / sx); y_cm = ((my - HEADER_HEIGHT) / sy)
            place_checkpoint(x_cm, y_cm)

    cv2.namedWindow("AMR Navigation")
    cv2.setMouseCallback("AMR Navigation", mouse_cb)
    log("Navigation server running. Click to place checkpoints.")

    while True:
        floor = cam_mgr.composite()
        raw_obs = cam_mgr.detect_obstacles()
        detected_static, detected_dynamic = classifier.classify(raw_obs)
        planner.obstacles = [(x-DANGER_ZONE_PADDING_CM, y-DANGER_ZONE_PADDING_CM,
                              w+2*DANGER_ZONE_PADDING_CM, h+2*DANGER_ZONE_PADDING_CM)
                             for x,y,w,h in detected_static]
        pos, angle, live = tracker.detect(floor)
        # Build display
        header = np.zeros((HEADER_HEIGHT, DISPLAY_W, 3), np.uint8)
        cv2.rectangle(header, (FOLLOW_BTN["x1"],FOLLOW_BTN["y1"]),
                      (FOLLOW_BTN["x2"],FOLLOW_BTN["y2"]), (0,200,0), -1)
        cv2.putText(header, "Follow", (FOLLOW_BTN["x1"]+12,42),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255,255,255), 2)
        cv2.rectangle(header, (CLEAR_BTN["x1"],CLEAR_BTN["y1"]),
                      (CLEAR_BTN["x2"],CLEAR_BTN["y2"]), (0,0,200), -1)
        cv2.putText(header, "Clear", (CLEAR_BTN["x1"]+18,42),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255,255,255), 2)
        n_obs = len(detected_static) + len(detected_dynamic)
        status_txt = f"State:{robot_state} | Obs:{n_obs} | MQTT:{mq.robot_status}"
        cv2.putText(header, status_txt, (290,38), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200,200,200), 1)
        display = np.vstack([header, floor])
        # Draw obstacles
        for ox,oy,ow,oh in detected_static:
            cv2.rectangle(display, (px(ox),py(oy)+HEADER_HEIGHT),
                          (px(ox+ow),py(oy+oh)+HEADER_HEIGHT), (0,0,255), 2)
        for ox,oy,ow,oh in detected_dynamic:
            cv2.rectangle(display, (px(ox),py(oy)+HEADER_HEIGHT),
                          (px(ox+ow),py(oy+oh)+HEADER_HEIGHT), (0,165,255), 2)
        # Draw path
        if path_cells and len(path_cells) > 1:
            pts = [(px(planner.cm_from_cell(c)[0]), py(planner.cm_from_cell(c)[1])+HEADER_HEIGHT)
                   for c in path_cells]
            cv2.polylines(display, [np.array(pts)], False, (0,0,255), 2)
        # Draw checkpoints
        for cp in checkpoints:
            cv2.circle(display, (px(cp[0]),py(cp[1])+HEADER_HEIGHT), 12, (255,100,0), -1)
            cv2.putText(display, "CP", (px(cp[0])+14,py(cp[1])+HEADER_HEIGHT-8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255,100,0), 1)
        # Draw breadcrumbs
        for b in breadcrumbs:
            cv2.circle(display, (int(b[0]),int(b[1])+HEADER_HEIGHT), 2, (0,255,255), -1)
        # Draw robot
        if pos:
            rpx, rpy = int(pos[0]), int(pos[1])
            color = (0,255,0) if live else (150,150,150)
            cv2.circle(display, (rpx, rpy+HEADER_HEIGHT), 8, color, -1)
            if angle is not None:
                ex = int(rpx + 25*math.cos(angle))
                ey = int(rpy + 25*math.sin(angle))
                cv2.arrowedLine(display, (rpx,rpy+HEADER_HEIGHT),
                                (ex,ey+HEADER_HEIGHT), color, 2, tipLength=0.3)
        # Draw target
        if path_cells and len(path_cells)>0:
            tc = planner.cm_from_cell(path_cells[0])
            cv2.circle(display, (px(tc[0]),py(tc[1])+HEADER_HEIGHT), 6, (255,0,0), -1)
        # Dynamic obstacle: stop robot
        if detected_dynamic and following:
            mq.send_motor(0, 0)
            cv2.putText(display, "DYNAMIC OBS — HOLDING", (200,HEADER_HEIGHT+30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,165,255), 2)
        elif following and pos and angle is not None and path_cells:
            rp_cm = (pos[0]/sx, pos[1]/sy)
            # Check path blockage
            blocked = False
            for cell in path_cells:
                ccm = planner.cm_from_cell(cell)
                for ox,oy,ow,oh in detected_static:
                    if ox+10<=ccm[0]<=ox+ow-10 and oy+10<=ccm[1]<=oy+oh-10:
                        blocked = True; break
                if blocked: break
            if blocked and (time.time()-last_recalc) > RECALC_COOLDOWN:
                last_recalc = time.time()
                log("Path blocked — replanning...")
                path_cells = planner.move_and_replan(rp_cm)
            # Drive toward next cell
            if not path_cells:
                mq.send_motor(0,0); following=False; robot_state="IDLE"
            else:
                target_cm = planner.cm_from_cell(path_cells[0])
                dx_cm = target_cm[0] - rp_cm[0]
                dy_cm = target_cm[1] - rp_cm[1]
                dist_cm = math.hypot(dx_cm, dy_cm)
                if robot_state == "DRIVING":
                    if dist_cm < DEADZONE_DIST_CM:
                        path_cells.pop(0)
                        if not path_cells:
                            mq.send_motor(0,0); following=False; robot_state="IDLE"
                            log("Route complete!")
                        elif path_cells[0] == planner.goal:
                            mq.send_motor(0,0); robot_state="WAITING"; wait_start=time.time()
                            pid_i=0; pid_pe=0; pid_pd=0
                            log("Arrived at checkpoint!")
                    else:
                        tgt_a = math.atan2(dy_cm, dx_cm)
                        ae = math.atan2(math.sin(tgt_a-angle), math.cos(tgt_a-angle))
                        now=time.time(); dt=max(now-pid_t,0.01); pid_t=now
                        p_s = STEER_KP*ae
                        pid_i = max(-STEER_INTEGRAL_LIMIT,min(STEER_INTEGRAL_LIMIT,pid_i+ae*dt))
                        i_s = STEER_KI*pid_i
                        d_s = STEER_KD*(ae-pid_pe)/dt; pid_pe=ae
                        steer = p_s+i_s+d_s
                        turn = min(int(abs(steer)*MAX_SPEED/math.pi), MAX_SPEED)
                        d_sp = SPEED_KD*(pid_pd-dist_cm)/dt; pid_pd=dist_cm
                        if abs(ae)>POINT_TURN_ANGLE:
                            fwd=0; turn=max(turn,35)
                        else:
                            fwd=max(0,min(int(dist_cm*SPEED_KP+d_sp),MAX_SPEED))
                        if abs(ae)<DEADZONE_ANGLE:
                            la=fwd; ra=fwd
                        elif steer>0:
                            la=fwd-turn; ra=fwd+turn
                        else:
                            la=fwd+turn; ra=fwd-turn
                        la=max(-MAX_SPEED,min(MAX_SPEED,la))
                        ra=max(-MAX_SPEED,min(MAX_SPEED,ra))
                        mq.send_motor(la, ra)
                        if breadcrumbs:
                            lb=breadcrumbs[-1]
                            if math.hypot(pos[0]-lb[0],pos[1]-lb[1])>BREADCRUMB_SPACING_CM*sx:
                                breadcrumbs.append(pos)
                        else:
                            breadcrumbs.append(pos)
                elif robot_state == "WAITING":
                    tl = STOP_DURATION-(time.time()-wait_start)
                    cv2.putText(display,f"WAIT:{tl:.1f}s",(10,HEADER_HEIGHT+30),
                                cv2.FONT_HERSHEY_SIMPLEX,0.8,(0,255,255),2)
                    if tl<=0:
                        path_cells.pop(0)
                        if not path_cells:
                            following=False; robot_state="IDLE"; log("Route complete!")
                        else:
                            robot_state="DRIVING"
                            pid_i=0;pid_pe=0;pid_pd=0;pid_t=time.time()
        else:
            if not detected_dynamic:
                mq.send_motor(0,0)
        cv2.imshow("AMR Navigation", display)
        if cv2.waitKey(1) & 0xFF == 27:
            break
    mq.send_motor(0,0)
    for s in cam_mgr.streams: s.stop()
    cv2.destroyAllWindows()
    log("Shutdown complete.")

if __name__ == "__main__":
    main()
