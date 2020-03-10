#!/bin/bash
NUM_ENVS=$1
LOG_PATH=$2
ROBOT_ID=$3
NUM_ROBOTS=$4
PREDICTION=$5
ROSIP=$(hostname -I | cut -d' ' -f1);
ANACONDA_DIR=/home/${USER}/anaconda3

if [ -d $ANACONDA_DIR ]; then
    # >>> conda initialize >>>
    # !! Contents within this block are managed by 'conda init' !!
    __conda_setup="$('/bin/conda' 'shell.bash' 'hook' 2> /dev/null)"
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
    # <<< conda initialize <<<
    cd ${AIRCAPDIR}/drl_ws/src/my_firefly_training/src
    conda activate spinningup
    export ROS_MASTER_URI=http://$ROSIP:11311; export ROS_IP=$ROSIP; export ROS_HOSTNAME=${ROSIP}
    source ${AIRCAPDIR}/drl_ws/devel_isolated/setup.bash && python test_singleagent.py $NUM_ENVS $LOG_PATH $ROBOT_ID $NUM_ROBOTS $PREDICTION
else
echo "In the script $0 set --ANACONDA_DIR-- to the directory where anaconda3 is installed"
fi