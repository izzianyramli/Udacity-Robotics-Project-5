#!/bin/sh

xterm -e "roslaunch my_robot world.launch" &
sleep 10
xterm -e "roslaunch my_robot amcl.launch" &
sleep 10
xterm -e "roslaunch my_robot view_navigation.launch" &
sleep 10
xterm -e "rosrun add_markers add_markers_time"
