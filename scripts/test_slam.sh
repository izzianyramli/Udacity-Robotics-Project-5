#!/bin/sh

xterm -e "roslaunch my_robot world.launch" &
sleep 10
xterm -e "roslaunch gmapping slam_gmapping_pr2.launch " &
sleep 10
xterm -e "roslaunch my_robot view_navigation.launch" &
sleep 10
xterm -e "roslaunch my_robot teleop.launch"
