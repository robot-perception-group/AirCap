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
		git clone https://github.com/robot-perception-group/openairos.git &&\
		git clone https://github.com/ros/geometry.git &&\
		git clone https://github.com/ros/geometry2.git &&\
		git clone https://github.com/openai/spinningup.git

		echo "INSTALLING PACKAGES FOR ROTORS SIMULATOR.."
		sudo apt install screen ros-melodic-octomap-msgs ros-melodic-octomap-ros ros-melodic-mrpt-bridge ros-melodic-cv-camera ros-melodic-mav-msgs \
		ros-melodic-mavlink python-wstool python-catkin-tools protobuf-compiler libgoogle-glog-dev ros-melodic-control-toolbox libmrpt* -y

		if [ $? -eq 0 ]; then
			echo "OK"
		else
			echo "ROTORS SIMULATOR PACKAGES INSTALL FAILED."
			exit
		fi

		echo "INSTALLING POSE-COV-OPS"
		sudo apt install ros-melodic-pose-cov-ops		

		if [ $? -eq 0 ]; then
			echo "OK"
		else
			echo "POSE-COV-OPS PACKAGES INSTALL FAILED."
			exit
		fi

		echo "INSTALLING PACKAGES FOR SPINNINGUP"
		sudo apt-get install libopenmpi-dev  cmake python3-dev zlib1g-dev -y

		if [ $? -eq 0 ]; then
			echo "OK"
		else
			echo "SPINNINGUP PACKAGES INSTALL FAILED."
			exit
		fi

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

			echo "CREATING ROS WORKSPACE FOR AIRCAP"
			cd $AIRCAPDIR$AIRCAPWS_DIR;catkin_make
			echo "Adding: 'source $AIRCAPDIR$AIRCAPWS_DIR/devel/setup.bash >> ~/.bashrc' "
			sleep 5
			if grep -Fxq "source $AIRCAPDIR$AIRCAPWS_DIR/devel/setup.bash" ~/.bashrc ; then
				echo "line already in bashrc"
			else
				source $AIRCAPDIR$AIRCAPWS_DIR/devel/setup.bash
				echo "source $AIRCAPDIR$AIRCAPWS_DIR/devel/setup.bash" >> ~/.bashrc && . ~/.bashrc								
			fi
		fi
	fi
else
	echo ros version $ROSVER
	echo "ros melodic has not been installed or initialized. Please check ros melodic installation in wiki"
fi

