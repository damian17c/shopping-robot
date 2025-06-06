## About 
This is a project where a TIAGo robot has to collect the 10 yellow cubes throughout the store. The robot has to map the store, plan the paths it will take, and pick and place the cubes inside the red basket it is carrying. This was all done in the Webots simulation environment. 


## Video Demo
[![Shopping Robot Demo](http://img.youtube.com/vi/DUbBy7oAF0g/0.jpg)](http://www.youtube.com/watch?v=DUbBy7oAF0g "Shopping Robot Demo")

## Mapping
  ### Method: Autonomous

### Description
  - Autonomous mapping was achieved with LIDAR sensors that would record obstacle locations. The LIDAR sensors also helped tell where how the robot should move to continue to map the store, this is what made it autonomous. If the LIDAR detects obstacles too close to the right the robot would turn left and vice versa for the left. If the robot detects obstacles too close in front it simply rotates away. The mapping process is basically a drive obstacle avoidance mode that collects data map data in the process.

## Path Planning
### Method: RRT* with path smoothing

### Description: 
  - For path plannning the RRT* algorithm from [PythonRobotics](https://github.com/AtsushiSakai/PythonRobotics) with some slight adjustments to make it work. The mapping process created an numpy array of 1s and 0s dictating whether an obstacle existed in that space or not. The RRT* algorithm did not 

### Description

## Manipulation
### Method: Inverse Kinematics

### Description

## Computer Vision
### Method: Color Blob Detection with CV2

### Description

## Localization
### Method: GPS

### Description

### Odometry
- The code can be tested with odometry but it still has slight calculation errors and often crashes into obstacles. If you wish to test with odometry follow the comments in the PLAN and NAVIGATE modes and uncomment the block of code at the bottom highlighted as odometry.

## Libraries Used
- math
- numpy
- matplotlib
- ikpy
- cv2
- random
