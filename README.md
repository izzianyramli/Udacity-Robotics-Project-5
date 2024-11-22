# Udacity-Robotics-Project5
Udacity Robotics Software Engineer Nanodegree - Project 5: Home Service Robot

## Packages
This repo contains several packages that is needed for mapping, localization and navigation:
1. `my_robot`: This package includes several things needed for the overall application:

-- `config/`: contains configuration files needed for localization and navigation.

-- `launch/`: contains the launch scripts that will be called in the bash scripts.

-- `maps/`: contains the map data to be used for navigation.

-- `rvizConfig/`: contains the pre-configured RViz configuration for mapping and navigation.

-- `worlds/`: contains the Gazebo simulation environment.

-- `urdf/`: contains robot body and sensors defined.

-- `meshes/`: contains data on for Robot's Hokuyo laser sensor.

2. `slam_gmapping`: This package contains SLAM algorithm that is used to map the enviroment.

You may need to change the param based on your simulation environment in `slam_gmapping/gmapping/src/slam_gmapping.cpp`:
```cpp
  // Parameters used by our GMapping wrapper
  if(!private_nh_.getParam("throttle_scans", throttle_scans_))
    throttle_scans_ = 1;
  if(!private_nh_.getParam("base_frame", base_frame_))
    base_frame_ = "base_link";
  if(!private_nh_.getParam("map_frame", map_frame_))
    map_frame_ = "map";
  if(!private_nh_.getParam("odom_frame", odom_frame_))
    odom_frame_ = "odom";
```

3. `teleop_twist_keyboard`: This package contains robot teleoperation package that is used to manually control the robot using keyboard for testing, especially while doing mapping. 
4. `add_markers`: Package for adding markers in RViz to be used in home service robot application.
5. `pick-objects`: Package for adding multiple navigation goal for the home service robot application.

## 1. Clone and Build
Clone repo and build workspace
```bash
$ mkdir ~/catkin_ws && cd ~/catkin_ws
$ git clone https://github.com/izzianyramli/Udacity-Robotics-Project5.git src
$ git submodule update && git submodule init
$ catkin_make
```
## 2. Mapping
For mapping, run `test_slam.sh`. This will launch 4 separate xterm terminals to run Gazebo, SLAM, RViz, and Teleop.
```bash
$ cd ~/catkin_ws
$ source devel/setup.bash
$ ./src/scripts/test_slam.sh
```

When you are satisfied with the map, save it by running `map_server` command in new terminal:
```bash
$ rosrun map_server map_saver -f <filename>
```


## 3. Navigation
For navigation, run `test_navigation.sh`. This will launch 3 separate xterm terminals to run Gazebo, AMCL, and RViz.
```bash
$ cd ~/catkin_ws
$ source devel/setup.bash
$ ./src/scripts/test_navigation.sh
```

## 4. Navigation with Multiple Goals
To test navigation with multiple goals, run `pick_objects.sh`. This will launch 4 separate xterm terminals to run Gazebo, AMCL, RViz, and pick_objects node that will automatically set 2 consecutive goals to the robot.
```bash
$ cd ~/catkin_ws
$ source devel/setup.bash
$ ./src/scripts/pick_objects.sh
```

## 5. Virtual Objects
To test for virtual objects in RViz, run `add_markers.sh`. This will launch 4 separate xterm terminals to run Gazebo, AMCL, RViz, and add_markers node that will automatically add 2 virtual objects in RViz to visualize the pickup and drop-off points.
```bash
$ cd ~/catkin_ws
$ source devel/setup.bash
$ ./src/scripts/add_markers.sh
```

## 6. Home Service Robot
To test for the end-to-end home service robot application, run `home_service.sh`. This will launch 5 separate xterm terminals to run Gazebo, AMCL, RViz, pick_object node and add_markers node. You will see robot moves to 2 different points, along with the 2 virtual objects to visualize the robot picking up object and delivers it to the drop-off point.
```bash
$ cd ~/catkin_ws
$ source devel/setup.bash
$ ./src/scripts/home_service.sh
```
