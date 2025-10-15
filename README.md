## About 
This is a project where a TIAGo robot has to collect the 10 yellow cubes throughout the store. The robot has to map the store, plan the paths it will take, and pick and place the cubes inside the red basket it is carrying. This was all done in the Webots simulation environment. 


## Video Demo
[![Shopping Robot Demo](docs/thumbnail.jpg)](http://www.youtube.com/watch?v=DUbBy7oAF0g "Shopping Robot Demo")

## Mapping
  ### Method: Autonomous

### Description
  - Autonomous mapping was achieved with LIDAR sensors that would record obstacle locations. The LIDAR sensors at the same time would tell how the robot should move to continue to map the store, this is what made it autonomous. If the LIDAR detects obstacles too close to the right the robot would turn left and vice versa for the left. If the robot detects obstacles too close in front it simply rotates away. The mapping process is basically a drive obstacle avoidance mode that collects map data in the process.

## Path Planning
### Method: RRT* with path smoothing

### Description: 
  - For path plannning the RRT* algorithm from [PythonRobotics](https://github.com/AtsushiSakai/PythonRobotics) was used with some slight adjustments to make it work. The mapping process creates a numpy array of booleans dictating whether an obstacle exists in that space or not. The RRT* algorithm does not take in a grid map for obstacles but rather an obstacle list that takes an (x,y) coordinate and a radius to tell the size of the obstacle. To resolve this each occupied pixel coordinate was converted to world cordinates and given a small radius based on the resolution of the grid map. Each was then added to the obstacle list which made up the map for RRT*. However, the RRT algorithm only allowed for a square random sampling area, while a rectangular area was needed. So this was adjusted in the algorithm by simply adding two morre parameters and checks to the random area.

- For path smoothing, a greedy smoothing function was used. The function would simply try to cut out unnecessary points, while trying to avoid creating collisions. More specifically, the function iteratively removes points from the path by taking every other point and checking if an obstacle free path exists between them. The function continues this process until there are less than 3 points in the path or it reaches the max iterations, specified as a parameter. The function checks for collisions by sampling points along the prospective path and checking if the distance between a sample point and an obstacle is less than the robot radius plus obstacle radius. 

## Manipulation
### Method: Inverse Kinematics

### Description
- Inverse kinematics was done with the ikpy library. Most of the setup for ikpy followed this [guide](https://gist.github.com/ItsMichal/4a8fcb330d04f2ccba582286344dd9a7). The whole pick and place operation involved several states to fully perform. The states follow this order SCAN &rarr; ADJUST_POSE &rarr; SET_ARM &rarr; PICK &rarr; GRAB &rarr; RETREAT &rarr; PLACE &rarr; RELEASE.
  * **SCAN**: This is the state where computer vision is utilized. The robot rotates around looking for a yellow cube to focus. Once a yellow cube is found it records whether the cube resides on an upper or lower shelf.
    
  * **ADJUST_POSE**: If the cube is on an upper shelf the torso lift joint is adjusted accordingly. Also in this state the robot moves closer or away from the target cube to a specified distance to avoid collisions with the shelves when using inverse kinematics. The robot also rotates in order to stay centered with the target cube. Once in the ideal location the target coordinates are converted from camera frame to robot frame and stored in offset_target.
    
  * **SET_ARM**: The robot adjusts the arm to an optimal position for pickup depending on whether the cube is on an upper or lower shelf. This step is done once again to avoid shelf collisions and to move the arm and gripper joint into positions and orientations that make the actual pick operation with inverse kinematics more fluid.
 
  * **PICK**: The pick operation calculates the inverse kinematics and executes the results on the arm. The operation recalculates the inverse kinematics until it is below a certain error threshold. This error is calculated by using the ikpy forward kinematics function on the IK results and finding the distance between the result position and the target position. Once the inverse kinematics is applied on the arm the robot drives forward slowly and stops after some delay. This ensures the cube is within the robot grippers.
 
  *  **GRAB**: This operation simply closes the robot grippers to hold onto the cube.
 
  *  **RETREAT**: The robot moves back for some delay to clear the arm from the shelf space to avoid collisions in the PLACE operation.
 
  *  **PLACE**: The place operation moves the torso lift joint to its lowest position and moves the arm above the red basket. This step was initially attempted with inverse kinematics but there were many inconsistencies in how the arm moved which increased likelihood of collisions and caused strange orienations and imprecise positioning of the cubes above the red basket. As a result, the arm positions for this step were hardcoded.
 
  *  **RELEASE**: Opens the robot grippers holding the cube. 
   
## Computer Vision
### Method: Color Blob Detection with CV2

### Description
- Computer Vision is performed with color blob detection using the cv2 library in Python. It works by taking the camera data from the TIAGo robot and converting the image into HSV format for better color distinction. Once in this format, a yellow mask is applied over the image to show only the yellow cubes in the image. Then, with the cv2 findContours function, the yellow contours from the image are matched with objects based on their position. Once matched, a yellow bounding rectangle is placed around the yellow cube in the image to highlight that the computer vision has identified a possible cube for pickup. To avoid highlighting yellow cubes that are far away a minimum contour area threshold was setup. Only yellow contours from the image larger than this threshold area are considered for possible yellow cubes. This was setup so as the robot rotates in the SCAN state, it does not focus on a cube far down the aisle or on the opposite shelf.

## Localization
### Method: GPS

### Description
- For localization webots builtin GPS was used.

### Odometry
- The code can be tested with odometry but it still has slight calculation errors and often crashes into obstacles. If you wish to test with odometry follow the comments in the PLAN and NAVIGATE modes and uncomment the block of code at the bottom highlighted as odometry.

## Libraries Used
- math
- numpy
- matplotlib
- ikpy
- cv2
- random

