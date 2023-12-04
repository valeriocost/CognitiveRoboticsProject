#!/bin/bash
pip install -r requirements.txt

wget -qO- https://get.haskellstack.org/ | sh 
sudo apt install libicu-dev libpcre3-dev
git clone https://github.com/facebook/duckling
cd duckling/
stack upgrade 
stack build

cd catkin_ws
chmod u+x src/rasa_ros/scripts/*
chmod u+x src/pepper_nodes/src/*
chmod u+x src/face_recognition/src/*
chmod u+x src/ros_audio_pkg/src/*
catkin build

pip install virtualenv
cd ../toDoList
virtualenv venv
source venv/bin/activate
pip install -r requirements.txt
python3 -m spacy download en_core_web_md
deactivate


