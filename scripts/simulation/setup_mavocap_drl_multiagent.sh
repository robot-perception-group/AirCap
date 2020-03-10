#!/bin/bash


ROBOS=$1

USEGT=$2

COMSUCCESSRATE=$3

NAME=$4

NUM_ENVS=$5

OBS=$6

ROSIP=$(hostname -I | cut -d' ' -f1);
ROS_MASTER=http://$ROSIP:1131


if [ -z "$COMSUCCESSRATE" ]; then
   COMSUCCESSRATE=100
fi

if [ -z "$NAME" ]; then
   NAME="gazebo_flight_$( date + '%s' )"
fi

if [ -z "$OBS" ]; then
   OBS=6
fi

# For world file with obstacles
# WORLD="arena_RAL_obs_$OBS"
WORLD="arena_RAL"

ROBOT_IDS="["
HUMAN_INPUT="[1"

Xs=( 0 -10 -8 -6 -4 5 0 2 4 6 8 10 15)
Ys=( -5 -10 -8 -6 -4 5 0 2 4 6 8 10 15)
LOGPATH="/home/${USER}/ros_logs"

if [ $# -lt 1 ]; then
        echo "usage: $0 <number of robots> <boolean flag to indicate ground truth or estimation version> <communication success rate> <experiment title> <number of envs> <number of obstacles>"
        exit 1
fi

LOGFILE=$( echo ${LOGPATH}/${NAME}*.bag )
if [ -e $LOGFILE ]; then
	echo Experiment result exists, exiting
	exit 0
fi

killall screen

for env in $(seq 0 $(($NUM_ENVS-1))); do

  env_id=$(($env+1))

  # This changes the gazebo master to different computers. the statement below
  # changes the gazebo master ip when number of environments  is greater than or equal to 3.
  # presently the gazebo master is not changed in the if then else statements. This is recommended
  # when using several environments for RL training
  if [ ${env_id} -lt 3 ]
  then
    GAZEBOIP=$(hostname -I | cut -d' ' -f1);
    GAZEBO_MASTER=http://${GAZEBOIP}:1135
    GAZEBO_MASTER_ID=${GAZEBO_MASTER}${env_id}
  else
    GAZEBOIP=$(hostname -I | cut -d' ' -f1);
    GAZEBO_MASTER=http://${GAZEBOIP}:1135
    GAZEBO_MASTER_ID=${GAZEBO_MASTER}${env_id}
  fi


  ROS_MASTER_URI=${ROS_MASTER}${env_id}; GAZEBO_MASTER_URI=${GAZEBO_MASTER_ID}; ROS_IP=${ROSIP}; ROS_HOSTNAME=${ROSIP} screen -d -m -S env_${env_id} bash -i
  screen -S env_${env_id} -X caption always
  screen -S env_${env_id} -X screen bash -i -c "ROS_MASTER_URI=${ROS_MASTER}${env_id}; GAZEBO_MASTER_URI=$GAZEBO_MASTER_ID; ROS_IP=${ROSIP}; ROS_HOSTNAME=${ROSIP}; roscore"
  screen -S  env_${env_id} -X title ROSMASTER

  echo "Started ENV_${env_id}"


  sleep 10
  # timeout 400 ./rossleep.py 20

  for i in $(seq 0 $(($ROBOS-1))); do
  	id=$(($i+1))
    echo "Starting AIRCAP for robot $id"
    screen -S env_${env_id} -X screen  bash -i -c "ROS_MASTER_URI=${ROS_MASTER}${env_id}; GAZEBO_MASTER_URI=$GAZEBO_MASTER_ID; ROS_IP=${ROSIP}; ROS_HOSTNAME=${ROSIP};
    roslaunch aircap simulation.launch robotID:=$id numRobots:=$ROBOS comSuccessRate:=$COMSUCCESSRATE obstacles_number:=$OBS useGTforTarget:=$USEGT useMPC:=true  --screen"
    screen -S  env_${env_id} -X title AIRCAP${env_id}_$id
    sleep 2
  done




  # export ROS_MASTER_URI=${ROS_MASTER}${env_id}; export GAZEBO_MASTER_URI=$GAZEBO_MASTER_ID; export ROS_IP=${ROSIP}; export ROS_HOSTNAME=${ROSIP}
  # check robot status
  echo "Checking robot status"
  result=1
 
done

