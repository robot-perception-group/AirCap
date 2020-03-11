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
		git clone -b pytorch https://github.com/nitin-ppnp/AlphaPose.git 
		git clone https://github.com/ros-perception/vision_opencv.git 
		git clone https://github.com/nkolot/SPIN.git 
		git clone https://github.com/williamljb/HumanMultiView.git


		echo "INSTALLING PYTHON3 ESSENTIALS"
		sudo apt-get install python-catkin-tools python3-dev python3-numpy python3-pip python3-yaml python3-venv -y		
		if [ $? -eq 0 ]; then
			echo "OK"
		else
			echo "PYTHON3 PACKAGES INSTALL FAILED."
			exit
		fi			
		echo "INSTALLING GFORTRAN "
		sudo apt-get install gfortran -y

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
        pip install torch torchvision opencv-python scipy matplotlib tqdm visdom rospkg catkin_pkg ipdb future
        cd $AIRCAPDIR/alphapose_ws/ && catkin_make -DPYTHON_EXECUTABLE=$AIRCAPDIR/venv/alphapose/bin/python3	
		source devel/setup.bash
        deactivate
		sleep 3

        echo "Setting up spin_ws and spin venv"
        cd $AIRCAPDIR/venv && virtualenv spin -p python3.6
        mkdir -p $AIRCAPDIR/spin_ws/src && cd $AIRCAPDIR/spin_ws/src
        ln -s $AIRCAPDIR$GIT_DIR/vision_opencv/ 
		ln -s $AIRCAPDIR$GIT_DIR/geometry/
		ln -s $AIRCAPDIR$GIT_DIR/geometry2/
		ln -s $AIRCAP_PATH/packages/simulation/hmr_node/		
		cd hmr_node/src && ln -s $AIRCAPDIR$GIT_DIR/SPIN/	
		mv hmr_pub.py SPIN/
		mv spin.py SPIN/
		mv config.py SPIN/		
		if [ -f SPIN/data/J_regressor_extra.npy ]; then 
			echo "DATA DOWNLOADED IN A PREVIOUS RUN"
		else 
			cd SPIN && ./fetch_data.sh
		fi 
		mkdir -p ${AIRCAPDIR}/git/SPIN/data/smpl
        source $AIRCAPDIR/venv/spin/bin/activate
		pip install -U pip
		cd $AIRCAPDIR/spin_ws/src/hmr_node && pip install -r requirements.txt
        pip install rospkg catkin_pkg empy numpy
        cd $AIRCAPDIR/spin_ws/ && catkin_make_isolated -DPYTHON_EXECUTABLE=$AIRCAPDIR/venv/spin/bin/python3
		source devel_isolated/setup.bash
        deactivate
		sleep 3

        echo "Setting up multihmr_ws and mhmr venv"
        cd $AIRCAPDIR/venv && virtualenv mhmr -p python2
        mkdir -p $AIRCAPDIR/multihmr_ws/src && cd $AIRCAPDIR/multihmr_ws/src
        ln -s $AIRCAPDIR$GIT_DIR/vision_opencv/ 
		ln -s $AIRCAPDIR$GIT_DIR/geometry/
		ln -s $AIRCAPDIR$GIT_DIR/geometry2/
		ln -s $AIRCAP_PATH/packages/simulation/multihmr_node/		
		cd multihmr_node/src && ln -s $AIRCAPDIR$GIT_DIR/HumanMultiView/	
		mv multihmr_pub.py HumanMultiView/
		mv multihmr.py HumanMultiView/
		mv renderer.py HumanMultiView/src_ortho/util/
		mv RunModel.py HumanMultiView/src_ortho/
        source $AIRCAPDIR/venv/mhmr/bin/activate
		cd $AIRCAPDIR/multihmr_ws/src/multihmr_node && pip install -r requirements.txt
        pip install rospkg catkin_pkg empy numpy
        cd $AIRCAPDIR/multihmr_ws/ && catkin_make_isolated -DPYTHON_EXECUTABLE=$AIRCAPDIR/venv/mhmr/bin/python2
		source devel_isolated/setup.bash
        deactivate


		if grep -Fxq "set rosparam use_sim_time 1" ~/.bashrc ; then
			echo "line set rosparam use_sim_time 1 , already in ~/.bashrc" 
		else
        	echo "set rosparam use_sim_time 1" >> ~/.bashrc && . ~/.bashrc
		fi
    else
        echo "$AIRCAPDIR not found. Follow steps 1 to 3 in https://github.com/robot-perception-group/AirCap/tree/aircaprl/packages/simulation/my_firefly_training"
	fi
else
	echo ros version $ROSVER
	echo "ros melodic has not been installed or initialized. Please check ros melodic installation in wiki"
fi

