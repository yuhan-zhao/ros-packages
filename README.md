# ros-packages
This repo collects small custom ros packages for future snippet. Each package will be documented as follows.

## lidar_utils
This package is for turtlebot lidar. The package mainly impelements the following functions:
- Obstacle detection based on yaml configuration. 
- Publish obstacle information as message.

The obstacle detection uses the distance info only. If lidar detects consective numbers within the detection range specified by yaml configuration, then the lidar will treat it as an obstacle.
