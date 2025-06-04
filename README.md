## Video Demo
[![Shopping Robot Demo](http://img.youtube.com/vi/DUbBy7oAF0g/0.jpg)](http://www.youtube.com/watch?v=DUbBy7oAF0g "Shopping Robot Demo")


## Webots World Adjustments
- Tiago Lidar translation changed to:  [0.1, 0, 0.2]
- Display size changed
    - Width: 322
    - Height: 600

## Libraries Used
- math
- numpy
- matplotlib
- ikpy
- cv2
- random

## Mapping
- For mapping change the mode inside (if mode == MODES['START']:) from MODES['PLAN'] to MODES['MAPPING']

## Odometry
- The code can be tested with odometry but it still has slight calculation errors and often crashes into obstacles. If you wish to test with odometry follow the comments in the PLAN and NAVIGATE modes and uncomment the block of code at the bottom highlighted as odometry.
