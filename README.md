# Udacity-Robotics-Project5
Udacity Robotics Software Engineer Nanodegree - Project 5: Home Service Robot

## Packages Description
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

3. `teleop_twist_keyboard`: This package contains robot teleoperation package that is used to manually control and move the robot using keyboard for testing, especially while doing mapping. 
4. `add_markers`: Package for adding cube markers in RViz to be used in home service robot application, to visualize object being picked up and dropped off by the robot.
5. `pick-objects`: Package for adding multiple navigation goal for the robot to navigate within the Gazebo environment.

## 1. Clone and Build
Clone repo and build workspace
```bash
$ mkdir ~/catkin_ws && cd ~/catkin_ws
$ git clone https://github.com/izzianyramli/Udacity-Robotics-Project5.git src
$ git submodule update && git submodule init
$ catkin_make
```
## 2. Mapping
For mapping, run `test_slam.sh`. This will launch 4 separate xterm terminals :
1. Gazebo: `roslaunch my_robot world.launch`. This will use package from `my_robot` to launch Gazebo world (`my_robot/world/HexagonWorld.world`) including the robot model.
2. SLAM: `roslaunch gmapping slam_gmapping_pr2.launch`. This will use package run SLAM algorithm from gmapping package which uses `/scan` data from the robot to map the building in Gazebo environment. 
3. RViz: `roslaunch my_robot view_navigation.launch`. This will run preconfigured RViz to visualize the robot's odometry and scan and also to view the map generated while running SLAM. 
4. Teleop: `roslaunch my_robot teleop.launch`. This launch file will levearage the `teleop_twist_keyboard` package which serves as robot teleoperation package to manually control and move the robot using keyboard while mapping the environment.
```bash
$ cd ~/catkin_ws
$ source devel/setup.bash
$ ./src/scripts/test_slam.sh
```

When you are satisfied with the map, save it by running `map_server` command in new terminal:
```bash
$ rosrun map_server map_saver -f <filename>
```
This will save the .pgm and .yaml file that will be used in Navigation part.

## 3. Navigation
For navigation, run `test_navigation.sh`. This will launch 3 separate xterm terminals :
1. Gazebo: `roslaunch my_robot world.launch`. This will use package from `my_robot` to launch Gazebo world (`my_robot/world/HexagonWorld.world`) including the robot model.
2. AMCL: `roslaunch my_robot amcl.launch`. This uses AMCL package from ROS to run localization and load the saved map.yaml file (`my_robot/maps/map.yaml`). AMCL node will also load the local and global costmap config file from `my_robot/config/` which contains parameters needed when doing navigation (such as inflation radius, robot radius, obstacle layer, etc.). You can also tweak these value based on your environment and use cases.
3. RViz: `roslaunch my_robot view_navigation.launch`. This will run preconfigured RViz to visualize the robot's odometry and position. We will use the `2DNavGoal` in RViz to send navigation goal to the robot move_base node. 
```bash
$ cd ~/catkin_ws
$ source devel/setup.bash
$ ./src/scripts/test_navigation.sh
```

## 4. Navigation with Multiple Goals
To test navigation with multiple goals, run `pick_objects.sh`. This will launch 4 separate xterm terminals : 
1. Gazebo: `roslaunch my_robot world.launch`. This will use package from `my_robot` to launch Gazebo world (`my_robot/world/HexagonWorld.world`) including the robot model.
2. AMCL: `roslaunch my_robot amcl.launch`. This uses AMCL package from ROS to run localization and load the saved map.yaml file (`my_robot/maps/map.yaml`). AMCL node will also load the local and global costmap config file from `my_robot/config/` which contains parameters needed when doing navigation (such as inflation radius, robot radius, obstacle layer, etc.). You can also tweak these value based on your environment and use cases.
3. RViz: `roslaunch my_robot view_navigation.launch`. This will run preconfigured RViz to visualize the robot's odometry and position.
4. Pick Objects: `rosrun pick_objects pick_objects`. This node will send 2 consecutive goals to the robot (pickup point, then dropoff point).
```bash
$ cd ~/catkin_ws
$ source devel/setup.bash
$ ./src/scripts/pick_objects.sh
```

## 5. Virtual Objects
To test for virtual objects in RViz, run `add_markers.sh`. This will launch 4 separate xterm terminals :
1. Gazebo: `roslaunch my_robot world.launch`. This will use package from `my_robot` to launch Gazebo world (`my_robot/world/HexagonWorld.world`) including the robot model.
2. AMCL: `roslaunch my_robot amcl.launch`. This uses AMCL package from ROS to run localization and load the saved map.yaml file (`my_robot/maps/map.yaml`). AMCL node will also load the local and global costmap config file from `my_robot/config/` which contains parameters needed when doing navigation (such as inflation radius, robot radius, obstacle layer, etc.). You can also tweak these value based on your environment and use cases.
3. RViz: `roslaunch my_robot view_navigation.launch`. This will run preconfigured RViz to visualize the robot's odometry and position.
4. Add Markers: `rosrun add_markers add_markers_time`. This node will visualize 2 virtual objects (cube object) in RViz to visualize the pickup and dropoff points. Marker will appear at the pickup point, and after 5 seconds it will hide, and appear at the dropoff point.
```bash
$ cd ~/catkin_ws
$ source devel/setup.bash
$ ./src/scripts/add_markers.sh
```

## 6. Home Service Robot
To test for the end-to-end home service robot application, run `home_service.sh`. This will launch 5 separate xterm terminals :
1. Gazebo: `roslaunch my_robot world.launch`. This will use package from `my_robot` to launch Gazebo world (`my_robot/world/HexagonWorld.world`) including the robot model.
2. AMCL: `roslaunch my_robot amcl.launch`. This uses AMCL package from ROS to run localization and load the saved `map.yaml` file (`my_robot/maps/map.yaml`). AMCL node will also load the local and global costmap config file from `my_robot/config/` which contains parameters needed when doing navigation (such as inflation radius, robot radius, obstacle layer, etc.). You can also tweak these value based on your environment and use cases.
3. RViz: `roslaunch my_robot view_navigation.launch`. This will run preconfigured RViz to visualize the robot's odometry and position.
4. Add Markers: `rosrun add_markers add_markers`. This node will visualize 2 virtual objects (cube object) in RViz to visualize the pickup and dropoff points. Marker will appear at the pickup point, and wait for the robot to reach the pickup point, then disappear. Once the robot reached the dropoff point, it will appear again at the dropoff point.
5. Pick Objects: `rosrun pick_objects pick_objects`. This node will send 2 consecutive goals to the robot (pickup point, then dropoff point).
```bash
$ cd ~/catkin_ws
$ source devel/setup.bash
$ ./src/scripts/home_service.sh
```
