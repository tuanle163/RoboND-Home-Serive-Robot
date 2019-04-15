#! /bin/sh
#Turtlebot: Import turtlebot_world.launch
xterm -e " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/workspace/catkin_ws/src/World/map01.world " &
sleep 3
#AMCL: amcl_demo.launch
xterm -e " roslaunch turtlebot_gazebo amcl_demo.launch map_file:=/home/workspace/catkin_ws/src/World/wall_follower_map.yaml 3d_sensor:=kinect " & 
sleep 3
#Rviz: view_navigation.launch
xterm -e " roslaunch turtlebot_rviz_launchers view_navigation.launch "