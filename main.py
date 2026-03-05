import cv2
import numpy as np
import math
import requests
from threading import Thread 
import time


class VideoStream:
    def __init__(self, src=0):
        # Initialize the video camera stream and read the first frame
        self.stream = cv2.VideoCapture(src)
        (self.grabbed, self.frame) = self.stream.read()
        self.stopped = False

    def start(self):
        Thread(target=self.update, args=(), daemon=True).start()
        return self

    def update(self):
        while True:
            if self.stopped:
                return
            (self.grabbed, self.frame) = self.stream.read()

    def read(self):
        return self.frame

    def stop(self):
        self.stopped = True
        self.stream.release()
# -----------------------------------------------------------------------

ESP_IP = "http://172.20.10.4"           
ROBOT_ARUCO_ID = 0                       
MARKER_REAL_SIZE_CM = 5.7                

MAX_SPEED = 50                         
TURN_GAIN = 2.00                          # How aggressively to turn (higher = sharper turns)
FWD_GAIN = 3.500                           # Forward speed scaling with REAL distance (cm)
DEADZONE_DIST_CM = 5.0                   # Now in real cm (stop if closer than this)
DEADZONE_ANGLE = 0.3                    # Radians (~7°) — consider "straight" if smaller

# Drawing settings
DRAW_COLOR = (0, 0, 255)                 # Red for drawn path
DRAW_THICKNESS = 3                       # Line thickness
LOOKAHEAD_DIST_CM = 20.0                # Shorter lookahead for better path adherence

# UI settings
HEADER_HEIGHT = 60                       # Height of top control bar
FOLLOW_BUTTON = {"x1": 10, "y1": 10, "x2": 110, "y2": 50, "text": "Follow", "color": (0, 255, 0)}
CLEAR_BUTTON = {"x1": 130, "y1": 10, "x2": 230, "y2": 50, "text": "Clear", "color": (0, 0, 255)}

# ArUco setup
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
aruco_params = cv2.aruco.DetectorParameters()
aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

url = "http://172.20.10.3:8080/video" 

# --- CHANGED: Start the threaded video stream instead of standard VideoCapture ---
print("Connecting to camera stream...")
stream = VideoStream(url).start()
time.sleep(1.0) # Give the camera a second to warm up
# ---------------------------------------------------------------------------------

drawing = False
following = False   
path_points = []    
ix, iy = -1, -1     

def mouse_callback(event, x, y, flags, param):
    global drawing, ix, iy, path_points, following

    if event == cv2.EVENT_LBUTTONDOWN:
        if y < HEADER_HEIGHT:   
             
            if FOLLOW_BUTTON["x1"] <= x <= FOLLOW_BUTTON["x2"] and FOLLOW_BUTTON["y1"] <= y <= FOLLOW_BUTTON["y2"]:
                following = True
                return
            
            if CLEAR_BUTTON["x1"] <= x <= CLEAR_BUTTON["x2"] and CLEAR_BUTTON["y1"] <= y <= CLEAR_BUTTON["y2"]:
                path_points = []
                following = False
                return
        else:
            drawing = True
            adj_y = y - HEADER_HEIGHT
            ix, iy = x, adj_y
            path_points = [(x, adj_y)]   

    elif event == cv2.EVENT_MOUSEMOVE:
        if drawing and y >= HEADER_HEIGHT:
            adj_y = y - HEADER_HEIGHT
            path_points.append((x, adj_y))   

    elif event == cv2.EVENT_LBUTTONUP:
        drawing = False
        if path_points and y >= HEADER_HEIGHT:
            adj_y = y - HEADER_HEIGHT
            path_points.append((x, adj_y))   

cv2.namedWindow("ArUco Path Follower - Draw Below Buttons, ESC to quit")
cv2.setMouseCallback("ArUco Path Follower - Draw Below Buttons, ESC to quit", mouse_callback)

def get_robot_pose(corners, ids):
    if ids is None or len(ids) == 0:
        return None, None, None

    ids = ids.flatten()

    for i, marker_id in enumerate(ids):
        if marker_id == ROBOT_ARUCO_ID:
            corner = corners[i][0]   

            cx = int(np.mean(corner[:, 0]))
            cy = int(np.mean(corner[:, 1]))

            front_mid_x = int((corner[0][0] + corner[1][0]) / 2)
            front_mid_y = int((corner[0][1] + corner[1][1]) / 2)

            dx = front_mid_x - cx
            dy = front_mid_y - cy
            orientation = math.atan2(dy, dx)

            sides = [
                np.linalg.norm(corner[0] - corner[1]),
                np.linalg.norm(corner[1] - corner[2]),
                np.linalg.norm(corner[2] - corner[3]),
                np.linalg.norm(corner[3] - corner[0])
            ]
            avg_pixel_size = np.mean(sides)

            return (cx, cy), orientation, avg_pixel_size

    return None, None, None

def update_path_progress(robot_pos, path, scale_cm_per_pixel, deadzone_cm):
    i = 0
    while i < len(path) - 1:
        dist_px = math.hypot(path[i][0] - robot_pos[0], path[i][1] - robot_pos[1])
        dist_cm = dist_px * scale_cm_per_pixel
        
        if dist_cm > deadzone_cm:
            break   
        i += 1
    return path[i:]   

def get_next_target_on_path(robot_pos, path, scale_cm_per_pixel, lookahead_cm):
    if not path:
        return None

    min_dist = float('inf')
    closest_idx = 0
    for i, p in enumerate(path):
        dist = math.hypot(p[0] - robot_pos[0], p[1] - robot_pos[1])
        if dist < min_dist:
            min_dist = dist
            closest_idx = i
    
    remaining_dist = lookahead_cm / scale_cm_per_pixel   
    current_dist = 0
    for i in range(closest_idx, len(path) - 1):
        seg_dx = path[i+1][0] - path[i][0]
        seg_dy = path[i+1][1] - path[i][1]
        seg_len = math.hypot(seg_dx, seg_dy)
        
        if current_dist + seg_len >= remaining_dist:
            t = (remaining_dist - current_dist) / seg_len
            tx = path[i][0] + t * seg_dx
            ty = path[i][1] + t * seg_dy
            return (tx, ty)
        
        current_dist += seg_len

    return path[-1]

def send_motor_command(left_speed, right_speed):
    try:
        url = f"{ESP_IP}/set?la={left_speed}&ra={right_speed}"
        requests.get(url, timeout=0.15)
    except Exception as e:
        pass # Silenced the print so it doesn't flood your console, adjust as needed

print("Starting ArUco + Path Drawing Robot Follower...")
print("Draw path with mouse (left click drag) BELOW the buttons.")
print("Click 'Follow' button to start following.")
print("Click 'Clear' button to delete the path.")
print("Press ESC to quit.")

while True:
    # --- CHANGED: Pull the newest frame from the background thread! ---
    frame = stream.read()
    if frame is None:
        continue # If the frame isn't ready yet, skip and try again immediately
    # ------------------------------------------------------------------

    h, w = frame.shape[:2]

    header = np.zeros((HEADER_HEIGHT, w, 3), np.uint8)      
     
    cv2.rectangle(header, (FOLLOW_BUTTON["x1"], FOLLOW_BUTTON["y1"]), (FOLLOW_BUTTON["x2"], FOLLOW_BUTTON["y2"]), FOLLOW_BUTTON["color"], -1)
    cv2.putText(header, FOLLOW_BUTTON["text"], (FOLLOW_BUTTON["x1"] + 10, FOLLOW_BUTTON["y1"] + 35), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)    
     
    cv2.rectangle(header, (CLEAR_BUTTON["x1"], CLEAR_BUTTON["y1"]), (CLEAR_BUTTON["x2"], CLEAR_BUTTON["y2"]), CLEAR_BUTTON["color"], -1)
    cv2.putText(header, CLEAR_BUTTON["text"], (CLEAR_BUTTON["x1"] + 10, CLEAR_BUTTON["y1"] + 35), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)
     
    display_frame = np.vstack([header, frame])
    
    corners, ids, _ = aruco_detector.detectMarkers(frame)
    robot_pos, robot_angle, marker_pixel_size = get_robot_pose(corners, ids)

    if robot_pos:
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
         
        display_robot_pos = (robot_pos[0], robot_pos[1] + HEADER_HEIGHT)
        cv2.circle(display_frame, display_robot_pos, 8, (0, 255, 0), -1)   
     
    if len(path_points) > 1:
         
        display_path = [(p[0], p[1] + HEADER_HEIGHT) for p in path_points]
        cv2.polylines(display_frame, [np.array(display_path)], False, DRAW_COLOR, DRAW_THICKNESS)
     
    if following and robot_pos is not None and robot_angle is not None and marker_pixel_size > 0 and len(path_points) > 1:
        scale_cm_per_pixel = MARKER_REAL_SIZE_CM / marker_pixel_size        
        
        path_points = update_path_progress(robot_pos, path_points, scale_cm_per_pixel, DEADZONE_DIST_CM)
        
        if not path_points:
            following = False
            send_motor_command(0, 0)
            continue
        
        target_pos = get_next_target_on_path(robot_pos, path_points, scale_cm_per_pixel, LOOKAHEAD_DIST_CM)
        
        if target_pos:
             
            display_target = (int(target_pos[0]), int(target_pos[1]) + HEADER_HEIGHT)
            cv2.circle(display_frame, display_target, 8, (255, 0, 0), -1)   
            
            dx = target_pos[0] - robot_pos[0]
            dy = target_pos[1] - robot_pos[1]
            pixel_dist = math.hypot(dx, dy)
            real_dist_cm = pixel_dist * scale_cm_per_pixel

            if real_dist_cm < DEADZONE_DIST_CM:
                send_motor_command(0, 0)
            else:
                target_angle = math.atan2(dy, dx)
                angle_error = math.atan2(math.sin(target_angle - robot_angle),
                                         math.cos(target_angle - robot_angle))   
                
                fwd = min(int(real_dist_cm * FWD_GAIN), MAX_SPEED)
                
                turn = int(abs(angle_error) * TURN_GAIN * MAX_SPEED / math.pi)
                turn = min(turn, MAX_SPEED)

                if abs(angle_error) < DEADZONE_ANGLE:
                     
                    left = fwd
                    right = fwd
                else:
                     
                    if angle_error > 0:   
                        left = fwd + turn
                        right = fwd - turn
                    else:                
                        left = fwd - turn
                        right = fwd + turn

                    # Clamp
                    left = max(min(left, MAX_SPEED), -MAX_SPEED)
                    right = max(min(right, MAX_SPEED), -MAX_SPEED)

                send_motor_command(left, right)

                cv2.line(display_frame, display_robot_pos, display_target, (0, 255, 255), 2)

        else:
            send_motor_command(0, 0)
            following = False
    else:
         
        send_motor_command(0, 0)
 
    cv2.imshow("ArUco Path Follower - Draw Below Buttons, ESC to quit", display_frame)

    if cv2.waitKey(1) & 0xFF == 27:
        break

send_motor_command(0, 0)  # final stop
stream.stop() # --- CHANGED: Properly stop the background thread ---
cv2.destroyAllWindows()
print("Program ended.")