"""grocery controller."""

# Apr 1, 2025

from controller import Robot
import math
import numpy as np
from path_planning.rrt import RRT
from path_planning.rrt_star import RRTStar
import matplotlib.pyplot as plt
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
import cv2
import random

#Initialization
print("=== Initializing Grocery Shopper...")
#Consts
MAX_SPEED = 7.0  # [rad/s]
MAX_SPEED_MS = 0.633 # [m/s]
AXLE_LENGTH = 0.4044 # m
MOTOR_LEFT = 10
MOTOR_RIGHT = 11
N_PARTS = 12
LIDAR_ANGLE_BINS = 667
LIDAR_SENSOR_MAX_RANGE = 5.5 # Meters
LIDAR_ANGLE_RANGE = math.radians(240)

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# The Tiago robot has multiple motors, each identified by their names below
part_names = ("head_2_joint", "head_1_joint", "torso_lift_joint", "arm_1_joint",
              "arm_2_joint",  "arm_3_joint",  "arm_4_joint",      "arm_5_joint",
              "arm_6_joint",  "arm_7_joint",  "wheel_left_joint", "wheel_right_joint",
              "gripper_left_finger_joint","gripper_right_finger_joint")

# 

# All motors except the wheels are controlled by position control. The wheels
# are controlled by a velocity controller. We therefore set their position to infinite.
target_pos = (0.0, 0.0, 0.35, 0.07, 1.02, -3.16, 1.27, 1.32, 0.0, 1.41, 'inf', 'inf',0.045,0.045)

robot_parts={}
for i, part_name in enumerate(part_names):
    robot_parts[part_name]=robot.getDevice(part_name)
    robot_parts[part_name].setPosition(float(target_pos[i]))
    robot_parts[part_name].setVelocity(robot_parts[part_name].getMaxVelocity() / 2.0)

# Enable gripper encoders (position sensors)
left_gripper_enc=robot.getDevice("gripper_left_finger_joint_sensor")
right_gripper_enc=robot.getDevice("gripper_right_finger_joint_sensor")
left_gripper_enc.enable(timestep)
right_gripper_enc.enable(timestep)

# Enable Camera
camera = robot.getDevice('camera')
camera.enable(timestep)
camera.recognitionEnable(timestep)

# Enable GPS and compass localization
gps = robot.getDevice("gps")
gps.enable(timestep)
compass = robot.getDevice("compass")
compass.enable(timestep)

# Enable LiDAR
lidar = robot.getDevice('Hokuyo URG-04LX-UG01')
lidar.enable(timestep)
lidar.enablePointCloud()

# Enable display
display = robot.getDevice("display")

# Odometry
right_wheel_sensor = robot.getDevice("wheel_right_joint_sensor")
right_wheel_sensor.enable(timestep)
left_wheel_sensor = robot.getDevice("wheel_left_joint_sensor")
left_wheel_sensor.enable(timestep)
robot.step(timestep)
od_x = gps.getValues()[0]
od_y = gps.getValues()[1]
n = compass.getValues()
od_theta = ((math.atan2(n[0], n[1])))
prev_right_position = 0.02078221705252888
prev_left_position = 0.02079795535576762

vL = 0
vR = 0

lidar_sensor_readings = [] # List to hold sensor readings
lidar_offsets = np.linspace(-LIDAR_ANGLE_RANGE/2., +LIDAR_ANGLE_RANGE/2., LIDAR_ANGLE_BINS)
lidar_offsets = lidar_offsets[83:len(lidar_offsets)-83] # Only keep lidar readings not blocked by robot chassis

try:
    map = np.load("map.npy")
    map = np.fliplr(map)  # Flip columns
    map = np.flipud(map)  # Flip rows
    print("Map loaded successfully:", map.shape)
except FileNotFoundError:
    print("map.npy not found! Using empty map. To create map change mode to MODES['MAPPING']")
    map = np.zeros(shape=[322,600])

# ------------------------------------------------------------------
# Helper Functions

#################
#
# [START] Search Path with RRT====
#
#################
plot_counter = 0

# Greedy Smoothing Function
def smooth_path_greedy(path, obstacle_list, robot_radius, max_iter=100, num_samples=10):
    """
    Greedily smooth the path by removing redundant waypoints, ensuring no collisions with obstacles.

    Args:
        path (list): List of (x, y) tuples representing the path.
        obstacle_list (list): List of (x, y, radius) tuples representing obstacles.
        robot_radius (float): Radius of the robot for collision checking.
        max_iter (int): Maximum number of iterations.
        num_samples (int): Number of points to sample along each line segment for collision checking.

    Returns:
        list: Smoothed path as a list of (x, y) tuples.
    """
    if not path or len(path) < 3:
        return path

    smoothed_path = path.copy()
    iteration = 0
    while iteration < max_iter:
        if len(smoothed_path) < 3:
            break

        simplified = False
        i = 0
        while i < len(smoothed_path) - 2:
            j = i + 2
            p1 = smoothed_path[i]
            p2 = smoothed_path[j]

            # Sample points along the line segment p1 to p2
            collision_free = True
            for t in np.linspace(0, 1, num_samples):
                x = p1[0] + t * (p2[0] - p1[0])
                y = p1[1] + t * (p2[1] - p1[1])
                # Check distance to each obstacle
                for ox, oy, radius in obstacle_list:
                    dist = ((x - ox)**2 + (y - oy)**2)**0.5
                    if dist < radius + robot_radius:  # Use full robot_radius
                        collision_free = False
                        print(f"Collision detected between {p1} and {p2} at sampled point ({x:.2f}, {y:.2f})")
                        break
                if not collision_free:
                    break

            # Additional check: Closest point on line segment to obstacle
            if collision_free:
                for ox, oy, radius in obstacle_list:
                    dx1, dy1 = p1[0] - ox, p1[1] - oy
                    dx12, dy12 = p2[0] - p1[0], p2[1] - p1[1]
                    length = (dx12**2 + dy12**2)**0.5
                    if length == 0:
                        continue
                    t = max(0, min(1, (dx1 * dx12 + dy1 * dy12) / (length**2)))
                    closest_x = p1[0] + t * dx12
                    closest_y = p1[1] + t * dy12
                    dist = ((closest_x - ox)**2 + (closest_y - oy)**2)**0.5
                    if dist < radius + robot_radius:
                        collision_free = False
                        print(f"Collision detected between {p1} and {p2} at closest point ({closest_x:.2f}, {closest_y:.2f}) to obstacle ({ox:.2f}, {oy:.2f})")
                        break

            if collision_free:
                print(f"Applying shortcut: Removing point {smoothed_path[i+1]} between {p1} and {p2}")
                smoothed_path.pop(i + 1)
                simplified = True
            else:
                i += 1

        if not simplified:
            break
        iteration += 1

    print(f"Greedy smoothing: Reduced path from {len(path)} to {len(smoothed_path)} points")
    return smoothed_path


def find_path(start, goal, map, show_path=False):
    # Map properties
    map_size_x = 600  # pixels (width, x-axis)
    map_size_y = 322  # pixels (height, y-axis)
    resolution = 0.05  # meters/pixel (30 m / 600 ≈ 0.05 m)
    origin = [0, 0]  # Center in world coordinates (meters)
    top_right = [15, 8.05]    # Top-right corner in world coordinates (meters)
    bottom_left = [-15, -8.05]  # Bottom-left corner in world coordinates

    # Load map
    map_array = map  

    # Create obstacleList in world coordinates
    obstacleList = []
    occupied_indices = np.where(map_array >= 1)
    rows, cols = occupied_indices
    radius = resolution * 2  # Small radius based on resolution (e.g., 0.1 m)

    for row, col in zip(rows, cols):
        # Convert pixel indices to world coordinates with origin at top-right
        x = (col - map_size_x / 2) * resolution  # col 0 → -15, col 600 → 15
        y = (map_size_y / 2 - row) * resolution   # row 0 → 8.05, row 322 → -8.05
        obstacleList.append((x, y, radius))

    # RRT setup
    rrt = RRTStar(
        start=start,  # Use world coordinates directly
        goal=goal,
        rand_area=[bottom_left[0], top_right[0], bottom_left[1], top_right[1]],  
        obstacle_list=obstacleList,
        expand_dis=3,
        max_iter=2000,
        connect_circle_dist=8,
        goal_sample_rate=10,
        search_until_max_iter=False,
        robot_radius=0.6
    )
    path = rrt.planning(animation=False)

    # Path smoothing (assuming you have this function from PythonRobotics)
    if path:
        smoothedPath = smooth_path_greedy(path, obstacleList, 0.4)
    else:
        smoothedPath = None
        print("No path found!")

    # Draw final path
    if path and show_path:
        rrt.draw_graph()
        plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r', label='Path')
        if smoothedPath:
            plt.plot([x for (x, y) in smoothedPath], [y for (x, y) in smoothedPath], '--c', label="Smoothed Path")
        plt.plot(start[0], start[1], 'go')
        plt.plot(goal[0], goal[1], 'ro')
        plt.grid(True)
        plt.legend()
        plt.axis("equal")
        # global plot_counter
        # plot_counter += 1
        # file = f"path_planning_plots/path_{plot_counter}.jpg"
        # plt.savefig(file, format='jpg', dpi=300, bbox_inches='tight')
        plt.show()
        

    return smoothedPath

# ==== [END] Search Path with RRT ====


###################
#
# [START] Inverse Kinematics
#
###################

base_elements = ["base_link", "base_link_Torso_joint", "Torso", "torso_lift_joint", "torso_lift_link", "torso_lift_link_TIAGo front arm_11367_joint", "TIAGo front arm_11367"]
last_link_vector = [0.004, 0,-0.1741]
my_chain = Chain.from_urdf_file("robot_urdf.urdf", last_link_vector=last_link_vector, base_elements=base_elements)

torso_lift_motor = robot.getDevice("torso_lift_joint")
torso_lift_position_sensor = torso_lift_motor.getPositionSensor()
torso_lift_position_sensor.enable(timestep)

# print(my_chain.links)

for link_id in range(len(my_chain.links)):

    # This is the actual link object
    link = my_chain.links[link_id]
    
    # I've disabled "torso_lift_joint" manually as it can cause
    # the TIAGO to become unstable.
    if link.name not in part_names or link.name =="torso_lift_joint":
        print("Disabling {}".format(link.name))
        my_chain.active_links_mask[link_id] = False

# Initialize the arm motors and encoders.
motors = []
for link in my_chain.links:
    if link.name in part_names and link.name != "torso_lift_joint":
        motor = robot.getDevice(link.name)

        # Make sure to account for any motors that
        # require a different maximum velocity!
        if link.name == "torso_lift_joint":
            motor.setVelocity(0.07)
        else:
            motor.setVelocity(1)
            
        position_sensor = motor.getPositionSensor()
        position_sensor.enable(timestep)
        motors.append(motor)

robot.getDevice("torso_lift_joint").setPosition(0.0)

def calculateIk(offset_target, orient=True, orientation_mode="Y", target_orientation=[0,0,1]):
    '''
    Parameters
    ----------
    offset_target : list
        A vector specifying the target position of the end effector
    orient : bool, optional
        Whether or not to orient, default True
    orientation_mode : str, optional
        Either "X", "Y", or "Z", default "Y"
    target_orientation : list, optional
        The target orientation vector, default [0,0,1]

    Returns
    -------
    list
        The calculated joint angles from inverse kinematics
    '''

    # Get the number of links in the chain
    num_links = len(my_chain.links)
    print(num_links)

    # Create initial position array with the correct size
    # initial_position = [0,0,0,0] + [m.getPositionSensor().getValue() for m in motors] + [0,0,0,0]
    initial_position = [0] * num_links


    # Map each motor to its corresponding link position
    motor_idx = 0
    for i in range(num_links):
        link_name = my_chain.links[i].name
        print(link_name)
        if my_chain.active_links_mask[i]:
            initial_position[i] = round(motors[motor_idx].getPositionSensor().getValue(), 3)
            motor_idx += 1
            print("1: ",  initial_position[i])
        elif link_name == "torso_lift_joint":
            # Set the initial position to the current position of torso_lift_joint
            initial_position[i] = round(torso_lift_position_sensor.getValue(), 3)
            print("0: ", initial_position[i])
        else:
            print(0)

    # Calculate IK
    ikResults = my_chain.inverse_kinematics(
        offset_target, 
        initial_position=initial_position,
        target_orientation=target_orientation, 
        orientation_mode=orientation_mode
    )

    # Validate result
    position = my_chain.forward_kinematics(ikResults)
    squared_distance = math.sqrt(
        (position[0, 3] - offset_target[0])**2 + 
        (position[1, 3] - offset_target[1])**2 + 
        (position[2, 3] - offset_target[2])**2
    )
    print(f"IK calculated with error - {squared_distance}")

    return ikResults, squared_distance

def moveArmToTarget(ikResults):
    '''Moves arm given ikResults'''

    for res in range(len(ikResults)):
        # This if check will ignore anything that isn't controllable
        if my_chain.links[res].name in part_names:
            robot.getDevice(my_chain.links[res].name).setPosition(ikResults[res])
            print("Setting {} to {}".format(my_chain.links[res].name, ikResults[res]))

def closeGrip():
    robot.getDevice("gripper_right_finger_joint").setPosition(0.020)
    robot.getDevice("gripper_left_finger_joint").setPosition(0.020)

def openGrip():
    robot.getDevice("gripper_right_finger_joint").setPosition(0.045)
    robot.getDevice("gripper_left_finger_joint").setPosition(0.045)

def placeInBox():
    robot.getDevice("arm_1_joint").setPosition(0.1)
    robot.getDevice("arm_2_joint").setPosition(0)
    robot.getDevice("arm_3_joint").setPosition(-1.6)
    robot.getDevice("arm_4_joint").setPosition(2.29)
    robot.getDevice("arm_5_joint").setPosition(0)
    robot.getDevice("arm_6_joint").setPosition(1.39)
    robot.getDevice("arm_7_joint").setPosition(-1.8)

    
def resetArm():
    robot.getDevice("arm_1_joint").setPosition(0.07)
    robot.getDevice("arm_2_joint").setPosition(1.02)
    robot.getDevice("arm_3_joint").setPosition(-3.16)
    robot.getDevice("arm_4_joint").setPosition(1.27)
    robot.getDevice("arm_5_joint").setPosition(1.32)
    robot.getDevice("arm_6_joint").setPosition(0)
    robot.getDevice("arm_7_joint").setPosition(1.41)
    

# ==== [END] Inverse Kinematics ====

###################
#
# [START] Computer Vision
#
###################
# Initialize video writer
width = camera.getWidth()
height = camera.getHeight()
fps = 60  # Frames per second (adjust as needed)
fourcc = cv2.VideoWriter_fourcc(*'H264')  # Codec (XVID is widely compatible)
video_writer = cv2.VideoWriter('ColorBlob.mp4', fourcc, fps, (width, height))


def detect_cube(camera, video_writer=video_writer):
    # Get the camera image  
    img = camera.getImage()
    width = camera.getWidth()
    height = camera.getHeight()

    if not img:
        return None

    # Convert Webots RGBA image to RGB
    img_array = np.frombuffer(img, dtype=np.uint8).reshape((height, width, 4))
    bgr_image = img_array[:, :, :3].copy()  

    # Convert to HSV
    hsv_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)

    # Yellow detection in HSV (Hue 30 for yellow)
    yellow_lower = (25, 80, 80)  # Hue 30 - 5
    yellow_upper = (35, 255, 255)  # Hue 30 + 5
    yellow_mask = cv2.inRange(hsv_image, yellow_lower, yellow_upper)

    # Find contours
    yellow_contours, _ = cv2.findContours(yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Get recognition objects from the camera
    recognized_objects = camera.getRecognitionObjects()
    cube = None

    # Contour area threshold
    min_contour_area = 12

    # Process contours to find yellow cubes
    for cnt in yellow_contours:
        if cv2.contourArea(cnt) > min_contour_area:
            x, y, w, h = cv2.boundingRect(cnt)
            centroid_x = x + w / 2
            centroid_y = y + h / 2

            for obj in recognized_objects:
                position = obj.getPositionOnImage()
                px, py = position[0], position[1]
                x, y = obj.getPosition()[0], obj.getPosition()[1]
                
                # Match contour location with cube on image
                if abs(centroid_x - px) < 5 and abs(centroid_y - py) < 5 and abs(y) < 0.2 and abs(x) <= 1.3:
                    cube = obj.getPosition()
                    break

            if cube:
                break

    # Visualize the result
    
    for cnt in yellow_contours:
        if cv2.contourArea(cnt) > min_contour_area:
            x, y, w, h = cv2.boundingRect(cnt)
            cv2.rectangle(bgr_image, (x, y), (x + w, y + h), (0, 255, 255), 2)
            cv2.putText(bgr_image, "YELLOW", (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

    cv2.imshow("Detected Objects", bgr_image)
    cv2.waitKey(1)

    # Write the frame to the video if video_writer is provided
    if video_writer is not None:
        video_writer.write(bgr_image)
        
    return cube
# ==== [END] Computer Vision ====


###################
#
# [START] Autonomous Mapping Variables
#
###################
keyboard = robot.getKeyboard()
keyboard.enable(timestep)

FORWARD_SPEED = MAX_SPEED * 0.8
TURN_SPEED    = MAX_SPEED * 0.2
SAFE_DIST     = 2
AHEAD_WINDOW  = 2  # check ±2 beams

# ==== [END] Autonomous Mapping Variables ====


###################
#
# [START] States and State Machine Variables
#
###################
MODES = {
    'START': 0,
    'STOP': 1,
    'MAPPING': 2,
    'PICK': 3,
    'GRAB': 4,
    'PLACE': 5,
    'RELEASE': 6,
    'SET_ARM': 7,
    'PLAN': 8,
    'NAVIGATE': 9,
    'SCAN': 10,
    'ADJUST_POSE' : 11,
    'RETREAT': 12,
    'RESET_ARM': 13,
}

mode = MODES['START']

# Delay variable for various tasks
delay = 0

# Error for Inverse Kinematics
error = math.inf

# SCAN and ADJUST_POSE variables
target = None
lower_shelf = False
offset_target = None


## NAVIGATE variables
cube_locations = [(3.5, 6.30), (3.5, 2.64), (2.62, 2.59), (0.777, 1.37), 
                   (-2.77, 1.31), (-2.72, 1.2), (-3.16, 2.63), (2.34, -2.53), 
                  (4.4, -2.53), (-1.33, -5.4)]

current_cube_index = 0
waypoints = []
current_waypoint_index = 0

## SCAN variables
rotation_speed = 0.8
# ==== [END] States and State Machine Variables ====

#print(list(my_chain.active_links_mask))

# Main Loop
while robot.step(timestep) != -1:
    # Color blob detection
    target = detect_cube(camera)

    if mode == MODES['START']:
        delay += 1
        if delay == 200:
            delay = 0
            mode = MODES['PLAN']

    elif mode == MODES['STOP']:
        vL = 0
        vR = 0 

    elif mode == MODES['MAPPING']:
        # Save map press [S]
        key = keyboard.getKey()
        if key == ord('S'):
            map = map > 0.5
            map = np.multiply(map, 1)
            np.save("map.npy", map)
            print("Map file saved")
        elif key == ord('L'):
            map = np.load("map.npy")
            print("Map loaded")
            np.flipud(map)
            plt.imshow(map)
            plt.show()
        

        num_beams = len(lidar_sensor_readings)
        CENTER_IDX   = num_beams // 2
        begin = max(0, CENTER_IDX - AHEAD_WINDOW)
        end   = min(num_beams, CENTER_IDX + AHEAD_WINDOW + 1)
        ahead_ranges = lidar_sensor_readings[begin:end]

        #Display#
        pose_x = gps.getValues()[0]
        pose_y = gps.getValues()[1]
        n = compass.getValues()
        rad = ((math.atan2(n[0], n[1])))
        pose_theta = rad

        lidar_sensor_readings = lidar.getRangeImage()
        lidar_sensor_readings = lidar_sensor_readings[83:len(lidar_sensor_readings)-83]

        for i, rho in enumerate(lidar_sensor_readings):
            alpha = lidar_offsets[i]
            if rho > LIDAR_SENSOR_MAX_RANGE:
                continue

            # The Webots coordinate system doesn't match the robot-centric axes we're used to
            rx = math.cos(alpha)*rho
            ry = -math.sin(alpha)*rho

            t = pose_theta
            # Convert detection from robot coordinates into world coordinates
            wx =  math.cos(t)*rx - math.sin(t)*ry + pose_x
            wy =  math.sin(t)*rx + math.cos(t)*ry + pose_y

            ################ ^ [End] Do not modify ^ ##################

            # print("Rho: %f Alpha: %f rx: %f ry: %f wx: %f wy: %f" % (rho,alpha,rx,ry,wx,wy))
            if wx >= 15:
                wx = 14.999
            elif wx <= -15:
                wx = -14.999
            if wy >= 8.05:
                wy = 8.049
            elif wy <= -8.05:
                wy = -8.049
            if rho < LIDAR_SENSOR_MAX_RANGE:
                # px = 300-(int(wx*20))
                # py = 161-(int(wy*20))
                px = int((15 - wx) * 20)  # shift x from [-15, 15] → [0, 600]
                py = int((8.05 - wy) * 20)   # flip y from [-8.05, 8.05] → [0, 322]
                if (px > 599):
                    px = 599
                if (py > 321):
                    py = 321
                # if 0 <= px < 600 and 0 <= py < 322:
                    # display.setColor(0x0000FF)  # Blue
                    # display.drawPixel(py, px)
                if (map[py][px] <= (1 - 5e-3)):
                    map[py][px] += 5e-3
                
                # Part 1.3: visualize map gray values.
                g = map[py][px]
                
                color = (g*256**2+g*256+g)*255 
                display.setColor(int(color))
                display.drawPixel(py, px)

        left_ahead_ranges = lidar_sensor_readings[
            CENTER_IDX - 20 : CENTER_IDX
        ]
        right_ahead_ranges = lidar_sensor_readings[
            CENTER_IDX  : CENTER_IDX + 20
        ]
        left_ranges = lidar_sensor_readings[:CENTER_IDX]
        right_ranges  = lidar_sensor_readings[CENTER_IDX + 1:]
        valid_right = [r for r in right_ranges if not math.isnan(r)]
        valid_left  = [r for r in left_ranges  if not math.isnan(r)]
        right_dist  = min(valid_right) if valid_right else float('inf')
        left_dist   = min(valid_left)  if valid_left  else float('inf')
        print(ahead_ranges)
        # print("right: ", right_ahead_ranges)
        # print("left: ", left_ahead_ranges)
        if any(a < 1 for a in left_ranges):
            vL, vR = TURN_SPEED , -TURN_SPEED 
            print("Left too close")
        elif any(b < 1 for b in right_ranges):
            vL, vR = -TURN_SPEED , TURN_SPEED
            print("Right too close")
        elif any(r < SAFE_DIST for r in ahead_ranges):
            # Something is too close straight ahead → spin
            print("Obstacles!")
            random_num = random.random()
            # while any(r < 2 for r in ahead_ranges):
                # print("RIght: ", right_dist, "Left: ", left_dist)
            if right_dist < left_dist:
                for _ in range (50):
                    vL, vR =  -TURN_SPEED, TURN_SPEED
                    robot_parts["wheel_left_joint"].setVelocity(vL)
                    robot_parts["wheel_right_joint"].setVelocity(vR)
                    robot.step(timestep)
            elif right_dist > left_dist:
                for _ in range (50):
                    vL, vR = TURN_SPEED,  -TURN_SPEED
                    robot_parts["wheel_left_joint"].setVelocity(vL)
                    robot_parts["wheel_right_joint"].setVelocity(vR)
                    robot.step(timestep)
            else:
                # no side is closer (or no valid readings) → go straight or use neighbor logic
                vL = vR = FORWARD_SPEED
            robot_parts["wheel_left_joint"].setVelocity(vL)
            robot_parts["wheel_right_joint"].setVelocity(vR)
            robot.step(timestep)
            # refresh lidar and ahead_ranges
            lidar_sensor_readings = lidar.getRangeImage()[83:-83]
            ahead_ranges = lidar_sensor_readings[
                max(0, CENTER_IDX - AHEAD_WINDOW) : 
                min(len(lidar_sensor_readings), CENTER_IDX + AHEAD_WINDOW + 1)
            ]
            vL = vR = FORWARD_SPEED

        else:
            vL, vR = FORWARD_SPEED, FORWARD_SPEED

    elif mode == MODES['SCAN']:
        if target:
            if target[2] < 0:
                lower_shelf = True
            else:
                lower_shelf = False
            
            mode = MODES['ADJUST_POSE']
            vL = 0
            vR = 0
        else:
            vL = -rotation_speed
            vR = rotation_speed

    elif mode == MODES['PICK']:
        
        if error > 0.32:
            # Lower shelf pickup
            if (lower_shelf):
                ikResults, error = calculateIk(offset_target, orientation_mode="Y", target_orientation=[0,0,-1])
                print("Lower Shelf Pickup")
            # Upper Sheld pickup
            else:
                ikResults, error = calculateIk(offset_target, orientation_mode="Y", target_orientation=[0,0,-1])
                print("Upper Shelf Pickup")
        else:
            if delay == 0:
                moveArmToTarget(ikResults)
                delay += 1
            elif delay == 440:
                vL = 0
                vR = 0
                delay = 0
                error = math.inf
                mode = MODES['GRAB']
            else:
                delay +=1
                if delay >= 150:
                    vL = 0.13
                    vR = 0.13
            
    elif mode == MODES['GRAB']:
        if delay == 0:
            closeGrip()
            delay += 1
        elif delay == 30:
            delay = 0
            mode = MODES['RETREAT']
        else:
            delay += 1

    elif mode == MODES['RETREAT']:
        if delay == 200:
            delay = 0
            vL = 0
            vR = 0
            mode = MODES['PLACE']
        else:
            vL = -1.0
            vR = -1.0
            delay += 1

    elif mode == MODES['PLACE']:
        if delay == 50:
            placeInBox()
            torso_lift_motor.setPosition(0.0)
            delay += 1
        elif delay == 300:
            robot.getDevice("arm_5_joint").setPosition(-1.8)
            delay += 1
        elif delay == 400:
            delay = 0
            mode = MODES['RELEASE']
        else:
            delay += 1
    
    elif mode == MODES['RELEASE']:
        if delay == 0:
            openGrip()
            delay += 1
        elif delay == 50:
            delay = 0
            mode = MODES['RESET_ARM']
        else:
            delay += 1

    elif mode == MODES['SET_ARM']:
        if delay == 0:
            # Lower shelf pickup
            if (lower_shelf):
                set_position = [0.5, -0.3 , 0.7]
                ikResults, error = calculateIk(set_position, orientation_mode="Y", target_orientation=[0,0,-1])
                moveArmToTarget(ikResults)
                pass
            # Upper shelf pickup
            else:
                set_position = [0.5, -0.3 , 1.35]
                ikResults, error = calculateIk(set_position, orientation_mode="Y", target_orientation=[0,0,-1])
                moveArmToTarget(ikResults)
            ikResults = None
            error = math.inf
            delay += 1
        elif delay == 200:
            mode = MODES['PICK']
            delay = 0
        else:
            delay += 1
    
    elif mode == MODES['ADJUST_POSE']:
        if delay == 0 and not lower_shelf:
            torso_lift_motor.setPosition(0.35)
            delay+=1
        elif delay > 300 or lower_shelf:
            if(target[1] > 0.08):
                    vL = -0.05
                    vR = 0.05
            elif(target[1] < -0.08):
                    vL = 0.05
                    vR = -0.05
            elif target[0] < 0.965 and lower_shelf:
                vL = -0.1
                vR = -0.1
            elif target[0] > 0.97 and lower_shelf:
                vL = 0.1
                vR = 0.1
            elif target[0] < 1.005 and not lower_shelf:
                vL = -0.1
                vR = -0.1
            elif target[0] > 1.01 and not lower_shelf:
                vL = 0.1
                vR = 0.1
            else:
                vL = 0
                vR = 0
                delay = 0
                if (lower_shelf):
                    offset_target = [target[0]+0.4, target[1]+0.125, (target[2])+0.97+0.015]
                else:
                    offset_target = [target[0]+0.4, target[1]+0.12, (target[2])+0.97+0.12+0.35]
                mode = MODES['SET_ARM']
        else:
            delay += 1
        

    elif mode == MODES['RESET_ARM']:
        resetArm()
        mode = MODES['PLAN']
        
    elif mode == MODES['PLAN']:

        if (current_cube_index >= len(cube_locations)):
            mode = MODES['STOP']
        else:
            # Get current pose
            start = (gps.getValues()[0], gps.getValues()[1])

            # If odometry 
            # start = (od_x, od_y)

            # Get the next cube location
            goal = cube_locations[current_cube_index]

            # Find path to next cube
            waypoints = find_path(start, goal, map, True)[::-1]
            waypoints.pop(0)
            print(waypoints)

            current_cube_index += 1
            mode = MODES['NAVIGATE']

    elif mode == MODES['NAVIGATE']:
        current_time = robot.getTime()

        # Controller gains
        p1 = 15.0
        p2 = 2.0

        distance_threshold = 0.05

        # If odometry comment this out
        #-------------------------------------
        # Get current pose
        od_x = gps.getValues()[0]
        od_y = gps.getValues()[1]

        # Get compass heading
        compass_vals = compass.getValues()
        od_theta = np.arctan2(compass_vals[0], compass_vals[1])
        #--------------------------------------

        # Current waypoint
        current_waypoint = waypoints[current_waypoint_index]

        # Calculate error and heading
        rho = np.sqrt((od_x - current_waypoint[0]) ** 2 + (od_y - current_waypoint[1]) ** 2)
        desired_heading = np.arctan2(current_waypoint[1] - od_y, current_waypoint[0] - od_x)
        alpha = desired_heading - od_theta

        # Normalize alpha to be between -pi and pi
        if alpha > np.pi:
            alpha = alpha - 2 * np.pi
        elif alpha < -np.pi:
            alpha = alpha + 2 * np.pi

        # Calculate velocity based on error and heading
        vL = -p1 * alpha + p2 * rho
        vR = +p1 * alpha + p2 * rho

        # Normalize wheelspeed
        max_vel = max(abs(vL), abs(vR))
        target_speed = 0.6 * MAX_SPEED  # Target upper bound
        if max_vel > target_speed:
            scaling_factor = target_speed / max_vel
            vL *= scaling_factor
            vR *= scaling_factor

        # Log movement occasionally
        if int(current_time) % 3 == 0 and abs(current_time - int(current_time)) < 0.1:
           print(f"Position=({od_x:.2f}, {od_y:.2f}), heading={np.degrees(od_theta):.1f}°, dist to WP={rho:.2f}, desired heading={np.degrees(desired_heading):.1f}°, alpha={np.degrees(alpha):.1f}°")

        # Check if waypoint reached
        if rho < distance_threshold:
            print(f"Reached waypoint: {current_waypoint} (Position: {od_x:.2f}, {od_y:.2f})")

            # Move to next waypoint
            current_waypoint_index += 1

            # Check if all waypoints complete
            if current_waypoint_index >= len(waypoints):
                vL = 0
                vR = 0
                current_waypoint_index = 0
                mode = MODES['SCAN']
                print("All waypoints reached!")

    '''
    # Odometry
    right_position = right_wheel_sensor.getValue()
    # print("Right Wheel position: ", right_position)
    left_position = left_wheel_sensor.getValue()
    # print("Left Wheel position:", left_position)
    delta_right_position = right_position - prev_right_position
    delta_left_position = left_position - prev_left_position
    # print("Delta Right Wheel position: ", delta_right_position)
    # print("Delta Left Wheel position:", delta_left_position)
    prev_right_position = right_position
    prev_left_position = left_position
    delta_vR = delta_right_position * 1000 / timestep
    delta_vL = delta_left_position * 1000 / timestep
    # print("Delta Right Wheel speed: ", delta_vR)
    # print("Delta Left Wheel speed:", delta_vL)
    od_x += (delta_vL+delta_vR)/2/MAX_SPEED*MAX_SPEED_MS*timestep/1000.0*math.cos(od_theta)
    od_y += (delta_vL+delta_vR)/2/MAX_SPEED*MAX_SPEED_MS*timestep/1000.0*math.sin(od_theta)
    od_theta += (delta_vR-delta_vL)/AXLE_LENGTH/MAX_SPEED*MAX_SPEED_MS*timestep/1000.0
    # print("Odometry_X: %f Odometry_Y: %f Odometry_Theta: %f" % (od_x, od_y, od_theta))
    '''

    robot_parts["wheel_left_joint"].setVelocity(vL)
    robot_parts["wheel_right_joint"].setVelocity(vR)

# Release the video writer and close OpenCV windows
video_writer.release()
cv2.destroyAllWindows()