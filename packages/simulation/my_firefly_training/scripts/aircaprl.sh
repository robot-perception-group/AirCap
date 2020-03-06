#!/bin/bash
##DIR is Directory where all the workspaces are created. Currently initialized to /home/${USER}/

HOMEDIR=$(eval echo ~$USER)
DIR=$HOMEDIR/aircaprl
#DIR=$(echo $PWD)

##Add a directory name where github repos related to aircaprl have to be downloaded
GIT_DIR="/git" 
AIRCAPWS_DIR="/aircap_ws"

if [ -d $DIR ] 
then
	echo "Directory $DIR exists. Please edit the variable-- DIR --in the script"
else
	echo "Downloading git repositories to folder echo $DIR$GIT_DIR"
	mkdir -p $DIR$GIT_DIR
	cd $DIR$GIT_DIR
	# Download AIRCAPRL and Rotors Simulator Repos
	git clone -b drl_experimental_rahul https://github.com/rahul-tallamraju/rotors_simulator.git 
	git clone https://github.com/robot-perception-group/openairos.git &&\
	git clone https://github.com/ros/geometry.git &&\
	git clone https://github.com/ros/geometry2.git &&\
	git clone https://github.com/ros-perception/vision_opencv.git &&\
	git clone https://github.com/openai/spinningup.git

	echo "INSTALLING PACKAGES FOR ROTORS SIMULATOR.."
	sudo apt install screen ros-melodic-octomap-msgs ros-melodic-octomap-ros ros-melodic-mrpt-bridge ros-melodic-cv-camera ros-melodic-mav-msgs \
	ros-melodic-mavlink python-wstool python-catkin-tools protobuf-compiler libgoogle-glog-dev ros-melodic-control-toolbox libmrpt* -y

	echo "INSTALLING PACKAGES FOR SPINNINGUP"
	sudo apt-get install libopenmpi-dev  cmake python3-dev zlib1g-dev

	if [ -d $DIR$AIRCAPWS_DIR ]
	then 
	echo "Directory $DIR$AIRCAPWS_DIR exists. Please edit the variable-- AIRCAPWS_DIR --in the script"
	else
	mkdir -p $DIR$AIRCAPWS_DIR/src && cd $DIR$AIRCAPWS_DIR/src
	echo "linking files to workspace from $AIRCAP_PATH"
	ln -s $AIRCAP_PATH/packages/flight/ && \
	ln -s $AIRCAP_PATH/scripts/ && \
	ln -s $AIRCAP_PATH/packages/simulation/aircap/ && \
	ln -s $AIRCAP_PATH/packages/simulation/fake_communication_failure/ && \
	ln -s $AIRCAP_PATH/packages/simulation/Gazebo_Plugins/ && \
	ln -s $AIRCAP_PATH/packages/simulation/librepilot_gazebo_bridge/ && \
	ln -s $AIRCAP_PATH/packages/simulation/random_moving_target/ && \
	ln -s $DIR$GIT_DIR/rotors_simulator/ 

	echo "CREATING ROS WORKSPACE FOR AIRCAP"
	cd $DIR$AIRCAPWS_DIR;catkin_make
	echo "Adding: 'source $DIR$AIRCAPWS_DIR/devel/setup.bash >> ~/.bashrc' "
	if grep -Fxq "source $DIR$AIRCAPWS_DIR/devel/setup.bash" ~/.bashrc
	then
	echo "line already in bashrc"
	else
	echo "source $DIR$AIRCAPWS_DIR/devel/setup.bash" >> ~/.bashrc && . ~/.bashrc
	sleep 5
	fi
	fi
fi




