#!/bin/bash
NUM_ENVS=$1
LOG_PATH=$2
ROBOT_ID=$3
NUM_ROBOTS=$4
PREDICTION=$5
ROSIP=$(hostname -I | cut -d' ' -f1);
# >>> conda initialize >>>
# !! Contents within this block are managed by 'conda init' !!
__conda_setup="$('/home/${USER}/anaconda3/bin/conda' 'shell.bash' 'hook' 2> /dev/null)"
 if [ $? -eq 0 ]; then
     eval "$__conda_setup"
 else
     if [ -f "/home/${USER}/anaconda3/etc/profile.d/conda.sh" ]; then
         . "/home/${USER}/anaconda3/etc/profile.d/conda.sh"
     else
         export PATH="/home/${USER}/anaconda3/bin:$PATH"
     fi
 fi
 unset __conda_setup
# <<< conda initialize <<<

cd ~/aircaprl/drl_ws/src/my_firefly_training/src
conda activate spinningup
source ~/aircaprl/drl_ws/devel/setup.bash
export ROS_MASTER_URI=http://$ROSIP:11311; export ROS_IP=$ROSIP; export ROS_HOSTNAME=${ROSIP}
#roslaunch my_firefly_training start_training.launch num_envs:=$NUM_ENVS log_file:=$LOG_PATH robotID:=$ROBOT_ID num_robots:=$NUM_ROBOTS prediction_only:=$PREDICTION
source ~/aircaprl/drl_ws/devel/setup.bash && python test_multiagent.py $NUM_ENVS $LOG_PATH $ROBOT_ID $NUM_ROBOTS $PREDICTION
