#!/bin/bash
HOMEDIR=$(eval echo ~$USER)
if [[ -v AIRCAPDIR ]];then
	echo "AIRCAPDIR is set to $AIRCAPDIR"
else
	AIRCAPDIR=$HOMEDIR/aircaprl
fi

GIT_DIR="/git"
DRLWS_DIR="/drl_ws"
ANACONDA_DIR=$HOMEDIR/anaconda3 #Add the path to anaconda3 directory

if [ -d $ANACONDA_DIR ]; then
	# >>> conda initialize >>>
	# !! Contents within this block are managed by 'conda init' !!
	__conda_setup="$('$ANACONDA_DIR/bin/conda' 'shell.bash' 'hook' 2> /dev/null)"
	if [ $? -eq 0 ]; then
	    eval "$__conda_setup"
	else
	    if [ -f "$ANACONDA_DIR/etc/profile.d/conda.sh" ]; then
		 . "$ANACONDA_DIR/etc/profile.d/conda.sh"
	    else
		 export PATH="$ANACONDA_DIR/bin:$PATH"
	    fi
	fi
	unset __conda_setup	
	
	echo "creating conda env"
	conda create -n spinningup python=3.6 -y
	conda activate spinningup
	cd $AIRCAPDIR$GIT_DIR
	cd spinningup && pip install -e .
	pip install stable-baselines[mpi] rospkg catkin_pkg empy defusedxml gitpython
	mkdir -p $AIRCAPDIR$DRLWS_DIR/src && cd $AIRCAPDIR$DRLWS_DIR/src
	ln -s $AIRCAP_PATH/packages/simulation/my_firefly_training && \
	ln -s $AIRCAPDIR$GIT_DIR/openairos/openai_ros
	ln -s $AIRCAPDIR$GIT_DIR/geometry2
	ln -s $AIRCAPDIR$GIT_DIR/geometry	
	cd .. && catkin_make_isolated -DPYTHON_EXECUTABLE=$ANACONDA_DIR/envs/spinningup/bin/python3
	mkdir logs
	conda deactivate
else
	echo " ~/anaconda3 directory does not exist. Please change variable -- ANACONDA_DIR -- in the script $0 "		
fi