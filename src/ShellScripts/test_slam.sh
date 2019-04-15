#! /bin/sh
#Import turtlebot_world.launch
#world_file:= /home/workspace/catkin_ws/src/World/map01.world
xterm -e " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/workspace/catkin_ws/src/World/map01.world" &
sleep 3
#gmapping_demo.launch or rosrun slam_gmapping
xterm -e " roslaunch turtlebot_gazebo gmapping_demo.launch " &
sleep 3
#view_navigation.launch
xterm -e " roslaunch turtlebot_rviz_launchers view_navigation.launch " &
sleep 3
#keyboard_teleop.launch
xterm -e " roslaunch turtlebot_teleop keyboard_teleop.launch " 