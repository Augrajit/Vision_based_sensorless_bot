import heapq
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

MAX_SPEED = 100                         
TURN_GAIN = 2.00                        
FWD_GAIN = 4.50                          
DEADZONE_DIST_CM = 3.0                 # Target reached threshold
DEADZONE_ANGLE = 0.3  
POINT_TURN_ANGLE = 0.6                  # Roughly 35 degrees in radians
BREADCRUMB_SPACING = 20                 # Drop a breadcrumb every 20 pixels  

# --- PID CONTROLLER GAINS (Steering) ---
STEER_KP = 3.00                          # Proportional gain (same as old TURN_GAIN)
STEER_KI = 0.05                          # Integral gain (eliminates steady-state drift)
STEER_KD = 0.80                          # Derivative gain (dampens oscillation)
STEER_INTEGRAL_LIMIT = 2.0               # Anti-windup: cap the integral accumulator

# --- PID CONTROLLER GAINS (Forward Speed) ---
SPEED_KP = 4.50                          # Proportional gain for distance
SPEED_KD = 1.00                          # Derivative gain (smooth deceleration)

# --- PHASE 3: OBSTACLE VISION SETTINGS ---
# Blue object tracking by default
OBSTACLE_LOWER_HSV = np.array([100, 150, 50])
OBSTACLE_UPPER_HSV = np.array([140, 255, 255])

MIN_OBSTACLE_AREA = 1000   # Ignore tiny specks of color noise
DANGER_ZONE_PADDING = 60   # Expand the box by 80 pixels (Approx robot radius)
PATH_BUFFER_PX = 65        # ~6.5 cm. The invisible forcefield pushing the line away

detected_obstacles = []    # Global list to store the danger zones

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

# --- Phase 1 State Machine Variables ---
following = False   
path_points = []    
robot_state = "IDLE" 
wait_start_time = 0
STOP_DURATION = 3.0 
# --------------------------------------------

# # --- NEW: Short-Term Memory Variables ---
last_known_pos = None
last_known_angle = None
lost_frame_count = 0
MAX_LOST_FRAMES = 15                                     # Tolerate ~0.5 seconds of lost tracking before stopping

# --- PID State Variables ---
pid_steer_integral = 0.0
pid_steer_prev_error = 0.0
pid_speed_prev_dist = 0.0
pid_last_time = time.time()

def mouse_callback(event, x, y, flags, param):
    global path_points, following, robot_state, detected_obstacles 

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
            
            if len(path_points) > 0:
                last_pos = path_points[-1]["pos"]
                safe_path = calculate_a_star_path(last_pos, new_pos, detected_obstacles)
                
                if safe_path:
                    path_points.extend(safe_path)
                else:
                    print("A* Blocked: Could not find a safe path to that point!")
            else:
                # Snap the first checkpoint to safe ground if placed on an obstacle
                safe_pos = find_nearest_safe_point(new_pos, detected_obstacles)
                if safe_pos is not None:
                    path_points.append({"pos": safe_pos, "is_checkpoint": True})
                else:
                    print("No safe location found near that point!")   

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
        req_url = f"{ESP_IP}/set?la={left_speed}&ra={right_speed}"
        requests.get(req_url, timeout=0.15)
    except Exception as e:
        pass 

print("Starting Phase 4: Dynamic Route Re-calculation...")

last_recalc_time = 0
RECALC_COOLDOWN = 1.5  # Seconds between route recalculations (absorbs jitter)

def find_nearest_safe_point(point, obstacles, grid_size=20, width=640, height=480):
    """Spiral outward from a point to find the nearest navigable grid cell."""
    def snap(p):
        return (int(p[0] // grid_size) * grid_size, int(p[1] // grid_size) * grid_size)

    def is_collision(p):
        for (ox, oy, ow, oh) in obstacles:
            if ox <= p[0] <= ox + ow and oy <= p[1] <= oy + oh:
                return True
        return False

    snapped = snap(point)
    if not is_collision(snapped):
        return snapped

    max_rings = 200 // grid_size
    for radius in range(1, max_rings + 1):
        best = None
        best_dist = float('inf')
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                if abs(dx) != radius and abs(dy) != radius:
                    continue  # Only check the perimeter of each ring
                candidate = (snapped[0] + dx * grid_size, snapped[1] + dy * grid_size)
                if 0 <= candidate[0] < width and 0 <= candidate[1] < height:
                    if not is_collision(candidate):
                        dist = math.hypot(candidate[0] - point[0], candidate[1] - point[1])
                        if dist < best_dist:
                            best_dist = dist
                            best = candidate
        if best is not None:
            return best
    return None

def calculate_a_star_path(start_pos, goal_pos, obstacles, grid_size=20, width=640, height=480):
    def snap(p): 
        return (int(p[0] // grid_size) * grid_size, int(p[1] // grid_size) * grid_size)
    
    start_g = snap(start_pos)
    goal_g = snap(goal_pos)

    def is_collision(p):
        for (ox, oy, ow, oh) in obstacles:
            if ox <= p[0] <= ox + ow and oy <= p[1] <= oy + oh: 
                return True
        return False

    # Smart Targeting: snap start/goal to nearest safe cell instead of failing
    if is_collision(start_g):
        safe_start = find_nearest_safe_point(start_pos, obstacles, grid_size, width, height)
        if safe_start is None:
            print("Robot is fully enclosed — no safe start found.")
            return []
        start_g = safe_start

    if is_collision(goal_g):
        safe_goal = find_nearest_safe_point(goal_pos, obstacles, grid_size, width, height)
        if safe_goal is None:
            print("No reachable point near target — all surrounding cells blocked.")
            return []
        print(f"Target inside obstacle — snapped to nearest safe point {safe_goal}")
        goal_g = safe_goal

    open_set = []
    heapq.heappush(open_set, (0, start_g))
    came_from = {}
    g_score = {start_g: 0}

    while open_set:
        _, current = heapq.heappop(open_set)

        if current == goal_g:
            new_path = []
            while current in came_from:
                new_path.append({"pos": current, "is_checkpoint": False})
                current = came_from[current]
            new_path.reverse()
            if new_path: 
                new_path[-1]["is_checkpoint"] = True
            return new_path

        for dx, dy in [(-grid_size,0), (grid_size,0), (0,-grid_size), (0,grid_size),
                       (-grid_size,-grid_size), (grid_size,-grid_size), (-grid_size,grid_size), (grid_size,grid_size)]:
            neighbor = (current[0] + dx, current[1] + dy)

            if 0 <= neighbor[0] < width and 0 <= neighbor[1] < height:
                if is_collision(neighbor): 
                    continue 
                
                tentative_g = g_score[current] + math.hypot(dx, dy)
                
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + math.hypot(goal_g[0] - neighbor[0], goal_g[1] - neighbor[1])
                    heapq.heappush(open_set, (f_score, neighbor))
                    
    return [] 

while True:
    # 1. Grab the raw image from the camera thread
    raw_frame = stream.read()
    if raw_frame is None:
        continue 

    # 2. THE FIX: Make a complete, disposable copy of the image!
    frame = raw_frame.copy()

    h, w = frame.shape[:2]

    
    # --- ARUCO DETECTION WITH MEMORY ---
    corners, ids, _ = aruco_detector.detectMarkers(frame)
    raw_robot_pos, raw_robot_angle, marker_pixel_size = get_robot_pose(corners, ids)

    if raw_robot_pos is not None:
        # We see the robot! Update our memory.
        robot_pos = raw_robot_pos
        robot_angle = raw_robot_angle
        last_known_pos = robot_pos
        last_known_angle = robot_angle
        lost_frame_count = 0
        
        # Draw ArUco overlay and green dot for live tracking directly on 'frame'
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        cv2.circle(frame, (int(robot_pos[0]), int(robot_pos[1])), 8, (0, 255, 0), -1)
    
    else:
        # Tracking lost! Use memory if we have it.
        lost_frame_count += 1
        if lost_frame_count < MAX_LOST_FRAMES and last_known_pos is not None:
            robot_pos = last_known_pos
            robot_angle = last_known_angle
            
            # Draw gray "ghost" dot to indicate we are using memory
            cv2.circle(frame, (int(robot_pos[0]), int(robot_pos[1])), 8, (150, 150, 150), -1)
        else:
            # It's been lost for too long. Truly give up.
            robot_pos = None
            robot_angle = None

    # Detect Obstacles
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    obstacle_mask = cv2.inRange(hsv_frame, OBSTACLE_LOWER_HSV, OBSTACLE_UPPER_HSV)
    contours, _ = cv2.findContours(obstacle_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    detected_obstacles = [] 
     
    for cnt in contours:
        if cv2.contourArea(cnt) > MIN_OBSTACLE_AREA:
            x, y, obj_w, obj_h = cv2.boundingRect(cnt)
            
            # 1. The VISUAL Red Box (What you see on screen)
            safe_x = max(0, x - DANGER_ZONE_PADDING)
            safe_y = max(0, y - DANGER_ZONE_PADDING)
            safe_w = obj_w + (DANGER_ZONE_PADDING * 2)
            safe_h = obj_h + (DANGER_ZONE_PADDING * 2)
            
            cv2.rectangle(frame, (x, y), (x + obj_w, y + obj_h), (0, 255, 255), 2)
            cv2.rectangle(frame, (safe_x, safe_y), (safe_x + safe_w, safe_y + safe_h), (0, 0, 255), 4)
            cv2.putText(frame, "DANGER ZONE", (safe_x, safe_y - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

            # 2. The INVISIBLE Math Box (Pushes the A* line 6-7cm away!)
            route_x = safe_x - PATH_BUFFER_PX
            route_y = safe_y - PATH_BUFFER_PX
            route_w = safe_w + (PATH_BUFFER_PX * 2)
            route_h = safe_h + (PATH_BUFFER_PX * 2)

            # Feed THIS larger, invisible box to the A* math to keep the line away
            detected_obstacles.append((route_x, route_y, route_w, route_h))

    # --- HEADER ---
    header = np.zeros((HEADER_HEIGHT, w, 3), np.uint8)   
    cv2.rectangle(header, (FOLLOW_BUTTON["x1"], FOLLOW_BUTTON["y1"]), (FOLLOW_BUTTON["x2"], FOLLOW_BUTTON["y2"]), FOLLOW_BUTTON["color"], -1)
    cv2.putText(header, FOLLOW_BUTTON["text"], (FOLLOW_BUTTON["x1"] + 10, FOLLOW_BUTTON["y1"] + 35), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)    
    cv2.rectangle(header, (CLEAR_BUTTON["x1"], CLEAR_BUTTON["y1"]), (CLEAR_BUTTON["x2"], CLEAR_BUTTON["y2"]), CLEAR_BUTTON["color"], -1)
    cv2.putText(header, CLEAR_BUTTON["text"], (CLEAR_BUTTON["x1"] + 10, CLEAR_BUTTON["y1"] + 35), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)
    display_frame = np.vstack([header, frame])

    # Compute display_robot_pos from the single source of truth set above.
    # Guards against NameError further down in the driving logic.
    if robot_pos is not None:
        display_robot_pos = (robot_pos[0], robot_pos[1] + HEADER_HEIGHT)
    else:
        display_robot_pos = None

    # If driving, constantly verify that our path hasn't been blocked!
    if following and robot_pos is not None and len(path_points) > 0:
        path_blocked = False
        for p in path_points:
            px, py = p["pos"]
            
            # 1. JITTER IMMUNITY: Ignore breadcrumbs within 60 pixels of the robot.
            # This prevents the robot from panicking if it accidentally brushes the danger zone.
            if math.hypot(px - robot_pos[0], py - robot_pos[1]) < 60:
                continue
                
            for (ox, oy, ow, oh) in detected_obstacles:
                # 2. HYSTERESIS: Shrink the collision check by 10 pixels. 
                # A* builds the path using the full box, but we only trigger a recalculation 
                # if the box deeply swallows the path. This absorbs camera wobble!
                if ox + 10 <= px <= ox + ow - 10 and oy + 10 <= py <= oy + oh - 10:
                    path_blocked = True
                    break
            if path_blocked:
                break
                
        if path_blocked and (time.time() - last_recalc_time) > RECALC_COOLDOWN:
            last_recalc_time = time.time()
            print("WARNING: Obstacle moved into path! Recalculating route...")
            
            # 1. Save all the major Checkpoints we still need to hit
            remaining_checkpoints = [pt["pos"] for pt in path_points if pt["is_checkpoint"]]
            
            # 2. Wipe the compromised path
            path_points = []
            
            # 3. Stitch a brand new path together from our current position to the checkpoints!
            current_start = robot_pos
            for cp in remaining_checkpoints:
                safe_part = calculate_a_star_path(current_start, cp, detected_obstacles)
                if safe_part:
                    path_points.extend(safe_part)
                    current_start = cp
                else:
                    print("Trapped! No safe detour available. Stopping.")
                    following = False
                    robot_state = "IDLE"
                    send_motor_command(0, 0)
                    break
    # =================================================================
      
    # --- VISUALIZE PATH ---
    if len(path_points) > 0:
        display_path = [(p["pos"][0], p["pos"][1] + HEADER_HEIGHT) for p in path_points]
        
        if len(display_path) > 1:
            cv2.polylines(display_frame, [np.array(display_path)], False, DRAW_COLOR, 2)
        
        for i, pt_data in enumerate(path_points):
            p = (pt_data["pos"][0], pt_data["pos"][1] + HEADER_HEIGHT)
            if pt_data["is_checkpoint"]:
                color = (0, 255, 0) if (following and i == 0) else (0, 0, 255)
                cv2.circle(display_frame, p, 10, color, -1)
                cv2.putText(display_frame, "CP", (p[0] + 15, p[1] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            else:
                cv2.circle(display_frame, p, 3, (0, 255, 255), -1)

    # --- DRIVING LOGIC ---
    if following and robot_pos is not None and robot_angle is not None and marker_pixel_size is not None and marker_pixel_size > 0 and len(path_points) > 0:
        scale_cm_per_pixel = MARKER_REAL_SIZE_CM / marker_pixel_size        
        
        target_data = path_points[0]
        target_pos = target_data["pos"]
        display_target = (int(target_pos[0]), int(target_pos[1]) + HEADER_HEIGHT)
        
        dx = target_pos[0] - robot_pos[0]
        dy = target_pos[1] - robot_pos[1]
        pixel_dist = math.hypot(dx, dy)
        real_dist_cm = pixel_dist * scale_cm_per_pixel

        if robot_state == "DRIVING":
            if real_dist_cm < DEADZONE_DIST_CM:
                if target_data["is_checkpoint"]:
                    send_motor_command(0, 0)
                    robot_state = "WAITING"
                    wait_start_time = time.time()
                    # Reset PID state on arrival
                    pid_steer_integral = 0.0
                    pid_steer_prev_error = 0.0
                    pid_speed_prev_dist = 0.0
                    print("Arrived at Checkpoint! Loading/Unloading for 3 seconds...")
                else:
                    path_points.pop(0) 
            else:
                target_angle = math.atan2(dy, dx)
                angle_error = math.atan2(math.sin(target_angle - robot_angle),
                                         math.cos(target_angle - robot_angle))   
                
                # --- PID STEERING CONTROLLER ---
                now = time.time()
                dt = now - pid_last_time
                if dt <= 0:
                    dt = 0.033  # Fallback ~30fps
                pid_last_time = now

                # P term
                p_steer = STEER_KP * angle_error

                # I term with anti-windup
                pid_steer_integral += angle_error * dt
                pid_steer_integral = max(-STEER_INTEGRAL_LIMIT, min(STEER_INTEGRAL_LIMIT, pid_steer_integral))
                i_steer = STEER_KI * pid_steer_integral

                # D term
                d_steer = STEER_KD * (angle_error - pid_steer_prev_error) / dt
                pid_steer_prev_error = angle_error

                pid_steer_output = p_steer + i_steer + d_steer

                turn = int(abs(pid_steer_output) * MAX_SPEED / math.pi)
                turn = min(turn, MAX_SPEED)

                # --- PID FORWARD SPEED ---
                d_speed = SPEED_KD * (pid_speed_prev_dist - real_dist_cm) / dt
                pid_speed_prev_dist = real_dist_cm

                POINT_TURN_ANGLE = 0.6 
                if abs(angle_error) > POINT_TURN_ANGLE:
                    fwd = 0
                    turn = max(turn, 35) 
                else:
                    fwd = int(real_dist_cm * SPEED_KP + d_speed)
                    fwd = max(0, min(fwd, MAX_SPEED))

                if abs(angle_error) < DEADZONE_ANGLE:
                    left = fwd
                    right = fwd
                else:
                    if pid_steer_output > 0:   
                        left = fwd - turn  
                        right = fwd + turn 
                    else:                
                        left = fwd + turn  
                        right = fwd - turn 

                left = max(min(left, MAX_SPEED), -MAX_SPEED)
                right = max(min(right, MAX_SPEED), -MAX_SPEED)
                send_motor_command(left, right)
                if display_robot_pos is not None:
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
                    # Reset PID for next segment
                    pid_steer_integral = 0.0
                    pid_steer_prev_error = 0.0
                    pid_speed_prev_dist = 0.0
                    pid_last_time = time.time()
    else:
        # Failsafe: If we aren't following, OR if tracking is lost while driving, STOP!
        send_motor_command(0, 0)
        # Reset PID when stopped
        pid_steer_integral = 0.0
        pid_steer_prev_error = 0.0
        pid_speed_prev_dist = 0.0
            
    cv2.imshow("ArUco AMR Logistics Simulator", display_frame)

    if cv2.waitKey(1) & 0xFF == 27:
        break

send_motor_command(0, 0) 
stream.stop() 
cv2.destroyAllWindows()
print("Program ended.")