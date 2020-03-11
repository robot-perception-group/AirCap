#!/bin/bash
##DIR is Directory where all the workspaces are created. Currently initialized to /home/${USER}/

HOMEDIR=$(eval echo ~$USER)

if [[ -v AIRCAPDIR ]];then
	echo "AIRCAPDIR is set to $AIRCAPDIR"
else
	AIRCAPDIR=$HOMEDIR/aircaprl
fi
#DIR=$(echo $PWD)

##Add a directory name where github repos related to aircaprl have to be downloaded
GIT_DIR="/git" 
AIRCAPWS_DIR="/aircap_ws"

#Check if ROS version is melodic
ROSVER=$(rosversion -d)
if [ $ROSVER == "melodic" ]; then
	echo ros version $ROSVER
	if [ -d $AIRCAPDIR ]; then
        source /home/${USER}/.bashrc
        source $AIRCAPDIR/aircap_ws/devel/setup.bash
		echo "Downloading git repositories to folder echo $AIRCAPDIR$GIT_DIR"
		cd $AIRCAPDIR$GIT_DIR
		# Download AIRCAPRL and Rotors Simulator Repos
		git clone -b pytorch https://github.com/nitin-ppnp/AlphaPose.git &&\
		git clone https://github.com/ros-perception/vision_opencv.git 

		echo "INSTALLING PYTHON3 ESSENTIALS"
		sudo apt-get install python-catkin-tools python3-dev python3-numpy python3-pip python3-yaml python3-venv -y
		if [ $? -eq 0 ]; then
			echo "OK"
		else
			echo "PYTHON3 PACKAGES INSTALL FAILED."
			exit
		fi			
        cd 	$AIRCAP_PATH/packages/simulation/alphapose_node/src
        rm AlphaPose
        ln -s $AIRCAPDIR$GIT_DIR/AlphaPose

        echo "Creating Directory To Keep All Virtual Environments"			
        mkdir -p $AIRCAPDIR/venv

        echo "Setting up alphapose_ws and alphapose venv"
        cd $AIRCAPDIR/venv && python3 -m venv alphapose			
        mkdir -p $AIRCAPDIR/alphapose_ws/src && cd $AIRCAPDIR/alphapose_ws/src
        ln -s $AIRCAPDIR$GIT_DIR/vision_opencv/
        source $AIRCAPDIR/venv/alphapose/bin/activate
        pip install torch torchvision opencv-python scipy matplotlib tqdm visdom rospkg catkin_pkg ipdb
        cd $AIRCAPDIR/alphapose_ws/ && catkin_make -DPYTHON_EXECUTABLE=$AIRCAPDIR/venv/alphapose/bin/python3	
        deactivate

        echo "set rosparam use_sim_time 1" >> ~/.bashrc && . ~/.bashrc
    else
        echo "$AIRCAPDIR not found. Follow steps 1 to 3 in https://github.com/robot-perception-group/AirCap/tree/aircaprl/packages/simulation/my_firefly_training"
	fi
else
	echo ros version $ROSVER
	echo "ros melodic has not been installed or initialized. Please check ros melodic installation in wiki"
fi

