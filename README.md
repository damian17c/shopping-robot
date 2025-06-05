## About 
This is a project where a TIAGo robot has to collect the 10 yellow cubes throughout the store. The robot has to map the store, plan the paths it will take, and pick and place the cubes inside the red basket it is carrying. This was all done in the Webots simulation environment. 


## Video Demo
[![Shopping Robot Demo](http://img.youtube.com/vi/DUbBy7oAF0g/0.jpg)](http://www.youtube.com/watch?v=DUbBy7oAF0g "Shopping Robot Demo")

## Mapping
### Method: Autonomous

### Description
- Autonomous mapping was achieved with LIDAR sensors and 

## Path Planning
Path planning was done with RRT* algorithm with path smoothing. 

## Manipulation


## Computer Vision


## Localization

### GPS

### Odometry
- The code can be tested with odometry but it still has slight calculation errors and often crashes into obstacles. If you wish to test with odometry follow the comments in the PLAN and NAVIGATE modes and uncomment the block of code at the bottom highlighted as odometry.

## Libraries Used
- math
- numpy
- matplotlib
- ikpy
- cv2
- random
