import cv2
import numpy as np
import math
import requests
from threading import Thread 
import time

class VideoStream:
    def __init__(self, src=0):
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

ESP_IP = "http://172.20.10.5"           
ROBOT_ARUCO_ID = 0                       
MARKER_REAL_SIZE_CM = 5.7                

MAX_SPEED = 80                         
TURN_GAIN = 1.50                         
FWD_GAIN = 5.00                          
DEADZONE_DIST_CM = 3.0                 # Target reached threshold
DEADZONE_ANGLE = 0.3  
POINT_TURN_ANGLE = 0.6                  # Roughly 35 degrees in radians
BREADCRUMB_SPACING = 20                 # Drop a breadcrumb every 20 pixels  

# --- PHASE 3: OBSTACLE VISION SETTINGS ---

#  for a BLUE object . 

OBSTACLE_LOWER_HSV = np.array([100, 150, 50])
OBSTACLE_UPPER_HSV = np.array([140, 255, 255])

# Yellow HSV bounds
# OBSTACLE_LOWER_HSV = np.array([20, 100, 100])
# OBSTACLE_UPPER_HSV = np.array([30, 255, 255])

MIN_OBSTACLE_AREA = 1000   # Ignore tiny specks of color noise
DANGER_ZONE_PADDING = 50   # Expand the box by 50 pixels (Approx robot radius)

detected_obstacles = []    # Global list to store the danger zones for detected obstacles

# Drawing settings
DRAW_COLOR = (0, 0, 255)                 
DRAW_THICKNESS = 3                       

# UI settings
HEADER_HEIGHT = 60                       
FOLLOW_BUTTON = {"x1": 10, "y1": 10, "x2": 110, "y2": 50, "text": "Follow", "color": (0, 255, 0)}
CLEAR_BUTTON = {"x1": 130, "y1": 10, "x2": 230, "y2": 50, "text": "Clear", "color": (0, 0, 255)}

# ArUco setup
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
aruco_params = cv2.aruco.DetectorParameters()
aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

url = "http://172.20.10.4:8080/video" 

print("Connecting to camera stream...")
stream = VideoStream(url).start()
time.sleep(1.0) 
# ---------------------------------------------------------------------------------

# --- NEW: Phase 1 State Machine Variables ---
following = False   
path_points = []    # Now acts as a list of exact Checkpoints
robot_state = "IDLE" # States: "IDLE", "DRIVING", "WAITING"
wait_start_time = 0
STOP_DURATION = 3.0 # Wait 3 seconds at each checkpoint
# --------------------------------------------

def mouse_callback(event, x, y, flags, param):
    global path_points, following, robot_state

    if event == cv2.EVENT_LBUTTONDOWN:
        if y < HEADER_HEIGHT:   
            if FOLLOW_BUTTON["x1"] <= x <= FOLLOW_BUTTON["x2"] and FOLLOW_BUTTON["y1"] <= y <= FOLLOW_BUTTON["y2"]:
                if len(path_points) > 0:
                    following = True
                    robot_state = "DRIVING"
                return
            
            if CLEAR_BUTTON["x1"] <= x <= CLEAR_BUTTON["x2"] and CLEAR_BUTTON["y1"] <= y <= CLEAR_BUTTON["y2"]:
                path_points = []
                following = False
                robot_state = "IDLE"
                send_motor_command(0, 0)
                return
        else:
            adj_y = y - HEADER_HEIGHT
            new_pos = (x, adj_y)
            
            # --- PHASE 2: BREADCRUMB GENERATION ---
            if len(path_points) > 0:
                # Get the last point we added
                last_pos = path_points[-1]["pos"]
                
                # Calculate distance between last point and new click
                dx = new_pos[0] - last_pos[0]
                dy = new_pos[1] - last_pos[1]
                dist = math.hypot(dx, dy)
                
                # Figure out how many breadcrumbs to drop
                num_breadcrumbs = int(dist / BREADCRUMB_SPACING)
                
                # Interpolate and add the breadcrumbs
                for i in range(1, num_breadcrumbs):
                    bx = int(last_pos[0] + (dx * i / num_breadcrumbs))
                    by = int(last_pos[1] + (dy * i / num_breadcrumbs))
                    path_points.append({"pos": (bx, by), "is_checkpoint": False})
            
            # Finally, add the actual clicked point as a Checkpoint
            path_points.append({"pos": new_pos, "is_checkpoint": True})   

cv2.namedWindow("ArUco AMR Logistics Simulator", cv2.WINDOW_NORMAL)
cv2.setMouseCallback("ArUco AMR Logistics Simulator", mouse_callback)

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

def send_motor_command(left_speed, right_speed):
    try:
        url = f"{ESP_IP}/set?la={left_speed}&ra={right_speed}"
        requests.get(url, timeout=0.15)
    except Exception as e:
        pass 

print("Starting Phase 2: Checkpoint State Machine...")
print("Click to place checkpoints. Click 'Follow' to start the logistics route.")

while True:
    frame = stream.read()
    if frame is None:
        continue 

    h, w = frame.shape[:2]

    # =================================================================
    # --- PHASE 3: OBSTACLE VISION ---
    # Convert frame to HSV color space for robust color detection
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Create a mask that isolates only the colors within our HSV range
    obstacle_mask = cv2.inRange(hsv_frame, OBSTACLE_LOWER_HSV, OBSTACLE_UPPER_HSV)
    
    # Find the contours (outlines) of the detected color blobs
    contours, _ = cv2.findContours(obstacle_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    detected_obstacles = [] # Reset the list every frame
    
    for cnt in contours:
        if cv2.contourArea(cnt) > MIN_OBSTACLE_AREA:
            # Get the tight bounding box of the physical object
            x, y, obj_w, obj_h = cv2.boundingRect(cnt)
            
            # Inflate it to create the Configuration Space "Danger Zone"
            safe_x = max(0, x - DANGER_ZONE_PADDING)
            safe_y = max(0, y - DANGER_ZONE_PADDING)
            safe_w = obj_w + (DANGER_ZONE_PADDING * 2)
            safe_h = obj_h + (DANGER_ZONE_PADDING * 2)
            
            # Store the Danger Zone coordinates (x, y, width, height)
            detected_obstacles.append((safe_x, safe_y, safe_w, safe_h))
            
            # Draw the physical object box (Yellow)
            cv2.rectangle(frame, (x, y), (x + obj_w, y + obj_h), (0, 255, 255), 2)
            
            # Draw the Danger Zone box (Red & Thick)
            cv2.rectangle(frame, (safe_x, safe_y), (safe_x + safe_w, safe_y + safe_h), (0, 0, 255), 4)
            cv2.putText(frame, "DANGER ZONE", (safe_x, safe_y - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
    # =================================================================

    # Now create the header and vstack it as usual...
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
     
    # --- VISUALIZE CHECKPOINTS ---
    # --- PHASE 2: VISUALIZE BREADCRUMBS & CHECKPOINTS ---
    if len(path_points) > 0:
        display_path = [(p["pos"][0], p["pos"][1] + HEADER_HEIGHT) for p in path_points]
        
        # Draw continuous line
        if len(display_path) > 1:
            cv2.polylines(display_frame, [np.array(display_path)], False, DRAW_COLOR, 2)
        
        # Draw points distinctly
        for i, pt_data in enumerate(path_points):
            p = (pt_data["pos"][0], pt_data["pos"][1] + HEADER_HEIGHT)
            if pt_data["is_checkpoint"]:
                # Draw big circles for actual checkpoints
                color = (0, 255, 0) if (following and i == 0) else (0, 0, 255)
                cv2.circle(display_frame, p, 10, color, -1)
                cv2.putText(display_frame, "CP", (p[0] + 15, p[1] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            else:
                # Draw small yellow dots for breadcrumbs
                cv2.circle(display_frame, p, 3, (0, 255, 255), -1)

    # --- PHASE 2: BREADCRUMB STATE MACHINE ---
    if following and robot_pos is not None and robot_angle is not None and marker_pixel_size > 0 and len(path_points) > 0:
        scale_cm_per_pixel = MARKER_REAL_SIZE_CM / marker_pixel_size        
        
        target_data = path_points[0]
        target_pos = target_data["pos"]
        display_target = (int(target_pos[0]), int(target_pos[1]) + HEADER_HEIGHT)
        
        dx = target_pos[0] - robot_pos[0]
        dy = target_pos[1] - robot_pos[1]
        pixel_dist = math.hypot(dx, dy)
        real_dist_cm = pixel_dist * scale_cm_per_pixel

        if robot_state == "DRIVING":
            # 1. Did we arrive at the current target?
            if real_dist_cm < DEADZONE_DIST_CM:
                if target_data["is_checkpoint"]:
                    # It's a major checkpoint, stop and wait
                    send_motor_command(0, 0)
                    robot_state = "WAITING"
                    wait_start_time = time.time()
                    print("Arrived at Checkpoint! Loading/Unloading for 3 seconds...")
                else:
                    # It's just a breadcrumb! Pop it and keep driving seamlessly
                    path_points.pop(0) 
            
            # 2. Steer towards target (Using your perfectly fixed math)
            else:
                target_angle = math.atan2(dy, dx)
                angle_error = math.atan2(math.sin(target_angle - robot_angle),
                                         math.cos(target_angle - robot_angle))   
                
                POINT_TURN_ANGLE = 0.6 
                turn = int(abs(angle_error) * TURN_GAIN * MAX_SPEED / math.pi)
                turn = min(turn, MAX_SPEED)

                if abs(angle_error) > POINT_TURN_ANGLE:
                    fwd = 0
                    turn = max(turn, 35) 
                else:
                    fwd = min(int(real_dist_cm * FWD_GAIN), MAX_SPEED)

                if abs(angle_error) < DEADZONE_ANGLE:
                    left = fwd
                    right = fwd
                else:
                    if angle_error > 0:   
                        left = fwd - turn  
                        right = fwd + turn 
                    else:                
                        left = fwd + turn  
                        right = fwd - turn 

                left = max(min(left, MAX_SPEED), -MAX_SPEED)
                right = max(min(right, MAX_SPEED), -MAX_SPEED)
                send_motor_command(left, right)
                cv2.line(display_frame, display_robot_pos, display_target, (0, 255, 255), 2)

        elif robot_state == "WAITING":
            time_left = STOP_DURATION - (time.time() - wait_start_time)
            cv2.putText(display_frame, f"WAITING: {time_left:.1f}s", (10, HEADER_HEIGHT + 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 3)

            if time_left <= 0:
                print("Timer complete. Moving to next target.")
                path_points.pop(0) 
                if len(path_points) == 0:
                    following = False
                    robot_state = "IDLE"
                    print("Route Complete!")
                else:
                    robot_state = "DRIVING"
    else:
        send_motor_command(0, 0) 
    cv2.imshow("ArUco AMR Logistics Simulator", display_frame)

    if cv2.waitKey(1) & 0xFF == 27:
        break

send_motor_command(0, 0) 
stream.stop() 
cv2.destroyAllWindows()
print("Program ended.")