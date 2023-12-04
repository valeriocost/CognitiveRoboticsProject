Group 4:
	Antonello Avella
	Eugenio Carpentieri
	Valerio Costantino
	Claudio De Pisapia

Install and execution instruction:
# Install spacy model
python3 -m spacy download en_core_web_md

# Install duckling
wget -qO- https://get.haskellstack.org/ | sh 
sudo apt install libicu-dev libpcre3-dev
cd Group4 
git clone https://github.com/facebook/duckling
cd duckling/
stack upgrade 
stack build

# Set environment variable
cd ..
export PROJECT_HOME=$(pwd)
‎
# Build ROS project
cd rosGroup4
chmod u+x src/rasa_ros/scripts/*
catkin build
source devel/setup.bash

# Run launchfile
roslaunch rasa_ros dialogue.xml
