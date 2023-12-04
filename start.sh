#!/bin/bash

gnome-terminal --tab -- bash -c 'export PROJECT_HOME=$(pwd); cd webapp; python3 webserver.py'
gnome-terminal --tab -- bash -c 'roscore'
sleep 5
gnome-terminal --tab -- bash -c 'export PROJECT_HOME=$(pwd); cd catkin_ws; source devel/setup.bash; roslaunch rosbridge_server rosbridge_websocket.launch'
gnome-terminal --window -- bash -c 'export PROJECT_HOME=$(pwd); cd catkin_ws; source devel/setup.bash; roslaunch --wait rasa_ros dialogue.xml'
gnome-terminal --window -- bash -c 'export PROJECT_HOME=$(pwd); cd catkin_ws; source devel/setup.bash; roslaunch --wait pepper_nodes pepper_bringup.launch'
gnome-terminal --window -- bash -c 'export PROJECT_HOME=$(pwd); cd catkin_ws; source devel/setup.bash; roslaunch --wait ros_audio_pkg speech_recognition.launch'
gnome-terminal --window -- bash -c 'export PROJECT_HOME=$(pwd); cd catkin_ws; source devel/setup.bash; roslaunch --wait face_recognition recognition.launch'
