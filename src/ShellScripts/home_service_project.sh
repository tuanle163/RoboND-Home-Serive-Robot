#! /bin/sh
#Turblebot: Import turtlebot_world.launch
xterm -e " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/workspace/catkin_ws/src/World/map01.world " &
sleep 5
#AMCL: amcl_demo.launch
xterm -e " roslaunch turtlebot_gazebo amcl_demo.launch map_file:=/home/workspace/catkin_ws/src/World/wall_follower_map.yaml 3d_sensor:=kinect " &
sleep 5
#Rviz: view_navigation.launch
xterm -e " roslaunch turtlebot_rviz_launchers view_navigation.launch " &
#pick_objects
sleep 5
xterm -e " rosrun pick_objects pick_objects " &
#add_markers
sleep 3
xterm -e " rosrun add_markers add_markers " 
