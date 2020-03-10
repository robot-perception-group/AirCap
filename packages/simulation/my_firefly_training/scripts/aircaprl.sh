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
		echo "Directory $AIRCAPDIR exists. Delete ~/aircaprl directory - OR- Please edit the variable-- AIRCAPDIR --in the script"
	else
		echo "Downloading git repositories to folder echo $AIRCAPDIR$GIT_DIR"
		mkdir -p $AIRCAPDIR/ros_logs
		mkdir -p $AIRCAPDIR$GIT_DIR
		cd $AIRCAPDIR$GIT_DIR
		# Download AIRCAPRL and Rotors Simulator Repos
		git clone -b drl_experimental_rahul https://github.com/rahul-tallamraju/rotors_simulator.git 
		git clone -b pytorch https://github.com/nitin-ppnp/AlphaPose.git
		git clone https://github.com/robot-perception-group/openairos.git &&\
		git clone https://github.com/ros/geometry.git &&\
		git clone https://github.com/ros/geometry2.git &&\
		git clone https://github.com/ros-perception/vision_opencv.git &&\
		git clone https://github.com/openai/spinningup.git

		echo "INSTALLING PACKAGES FOR ROTORS SIMULATOR.."
		sudo apt install screen ros-melodic-octomap-msgs ros-melodic-octomap-ros ros-melodic-mrpt-bridge ros-melodic-cv-camera ros-melodic-mav-msgs \
		ros-melodic-mavlink python-wstool python-catkin-tools protobuf-compiler libgoogle-glog-dev ros-melodic-control-toolbox libmrpt* -y
		echo "INSTALLING POSE-COV-OPS"
		sudo apt install ros-melodic-pose-cov-ops
		echo "INSTALLING PYTHON3 ESSENTIALS"
		sudo apt-get install python-catkin-tools python3-dev python3-numpy python3-pip python3-yaml python3-venv -y
		if [ $? -eq 0 ]; then
			echo "OK"
		else
			echo "ROTORS SIMULATOR PACKAGES INSTALL FAILED."
			exit
		fi


		echo "INSTALLING PACKAGES FOR SPINNINGUP"
		sudo apt-get install libopenmpi-dev  cmake python3-dev zlib1g-dev -y

		if [ -d $AIRCAPDIR$AIRCAPWS_DIR ]; then 
			echo "Directory $AIRCAPDIR$AIRCAPWS_DIR exists. Please edit the variable-- AIRCAPWS_DIR --in the script"
		else
			mkdir -p $AIRCAPDIR$AIRCAPWS_DIR/src && cd $AIRCAPDIR$AIRCAPWS_DIR/src
			echo "linking files to workspace from $AIRCAP_PATH"
			ln -s $AIRCAP_PATH/packages/flight/ && \
			ln -s $AIRCAP_PATH/scripts/ && \
			ln -s $AIRCAP_PATH/packages/simulation/aircap/ && \
			ln -s $AIRCAP_PATH/packages/simulation/fake_communication_failure/ && \
			ln -s $AIRCAP_PATH/packages/simulation/Gazebo_Plugins/ && \
			ln -s $AIRCAP_PATH/packages/simulation/librepilot_gazebo_bridge/ && \
			ln -s $AIRCAP_PATH/packages/simulation/random_moving_target/ && \
			ln -s $AIRCAP_PATH/packages/simulation/alphapose_node/ && \			
			ln -s $AIRCAPDIR$GIT_DIR/rotors_simulator/ 
			
			cd 	$AIRCAP_PATH/packages/simulation/alphapose_node/src
			ln -s $AIRCAPDIR$GIT_DIR/AlphaPose

			echo "CREATING ROS WORKSPACE FOR AIRCAP"
			cd $AIRCAPDIR$AIRCAPWS_DIR;catkin_make
			echo "Adding: 'source $AIRCAPDIR$AIRCAPWS_DIR/devel/setup.bash >> ~/.bashrc' "
			sleep 5

			if grep -Fxq "source $AIRCAPDIR$AIRCAPWS_DIR/devel/setup.bash" ~/.bashrc ; then
				echo "line already in bashrc"
			else
				echo "source $AIRCAPDIR$AIRCAPWS_DIR/devel/setup.bash" >> ~/.bashrc && . ~/.bashrc				
			fi


			echo "Creating Directory To Keep All Virtual Environments"			
			mkdir -p $AIRCAPDIR/venv

			echo "Setting up alphapose_ws and alphapose venv"
			cd $AIRCAPDIR/venv && python3 -m venv alphapose			
			mkdir -p $AIRCAPDIR/alphapose_ws/src && cd $AIRCAPDIR/alphapose_ws/src
			ln -s $AIRCAPDIR$GIT_DIR/git/vision_opencv/
			source $AIRCAPDIR/venv/alphapose/bin/activate
			pip install torch torchvision opencv-python scipy matplotlib tqdm visdom rospkg catkin_pkg		
			cd $AIRCAPDIR/alphapose_ws/ && catkin_make -DPYTHON_EXECUTABLE=$AIRCAPDIR/venv/alphapose/bin/python3	
			deactivate
		fi
	fi
else
	echo ros version $ROSVER
	echo "ros melodic has not been installed or initialized. Please check ros melodic installation in wiki"
fi

