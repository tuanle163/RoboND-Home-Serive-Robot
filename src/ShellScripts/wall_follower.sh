#! /bin/sh
#Turblebot: Import turtlebot_world.launch
xterm -e " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/workspace/catkin_ws/src/World/map01.world" &
sleep 3
#Gmapping: gmapping_demo.launch or rosrun slam_gmapping
xterm -e " roslaunch turtlebot_gazebo gmapping_demo.launch " &
sleep 3
#Rviz: view_navigation.launch
xterm -e " roslaunch turtlebot_rviz_launchers view_navigation.launch " &
sleep 5
#wall_follower Node: wall_follower.cpp
xterm -e " rosrun wall_follower wall_follower" 