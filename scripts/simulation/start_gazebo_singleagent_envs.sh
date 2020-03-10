#!/bin/bash


ROBOS=$1

USEGT=$2

COMSUCCESSRATE=$3

NAME=$4

NUM_ENVS=$5

RECORD=$6

ROSIP=$(hostname -I | cut -d' ' -f1)
ROS_MASTER=http://$ROSIP:1131
GAZEBOIP=$ROSIP
GAZEBO_MASTER=http://${GAZEBOIP}:1135

if [ -z "$COMSUCCESSRATE" ]; then
   COMSUCCESSRATE=100
fi
if [ -z "$NAME" ]; then
   NAME="gazebo_flight_$( date + '%s' )"
fi
if [ -z "$RECORD" ]; then
   RECORD=0
fi
# For world file with obstacles
WORLD="arena_RAL"

ROBOT_IDS="["
HUMAN_INPUT="[1"

Xs=( 0 0 -8 -6 -4 5 0 2 4 6 8 10 15)
Ys=( -5 5 -8 -6 -4 5 0 2 4 6 8 10 15)



if [ $# -lt 1 ]; then
        echo "usage: $0 <number of robots> <boolean flag to indicate ground truth or estimation version> <communication success rate> <experiment title> <number of envs> <number of obstacles>"
        exit 1
fi


for env in $(seq 0 $(($NUM_ENVS-1))); do
  env_id=$(($env+1))

  ROS_MASTER_URI=${ROS_MASTER}${env_id}; GAZEBO_MASTER_URI=${GAZEBO_MASTER}${env_id} ; ROS_IP=${GAZEBOIP}; ROS_HOSTNAME=${GAZEBOIP} screen -d -m -S envsim_${env_id} bash -i
  screen -S envsim_${env_id} -X caption always
  echo "Started envsim_${env_id}"

  echo "Starting Gazebo"

  screen -S envsim_${env_id} -X screen bash -i -c "export ROS_MASTER_URI=${ROS_MASTER}${env_id};export GAZEBO_MASTER_URI=${GAZEBO_MASTER}${env_id} ;export ROS_IP=${GAZEBOIP};export ROS_HOSTNAME=${GAZEBOIP}; roslaunch rotors_gazebo world.launch world_name:=$WORLD gui:=true --screen"
  screen -S envsim_${env_id} -X title "GAZEBO${env_id}"
  sleep 5

  for i in $(seq 0 $(($ROBOS-1))); do
  	id=$(($i+1))
  	echo "launching robot $id"
  	screen -S envsim_${env_id} -X screen bash -i -c "export ROS_MASTER_URI=${ROS_MASTER}${env_id};export GAZEBO_MASTER_URI=${GAZEBO_MASTER}${env_id};export ROS_IP=${GAZEBOIP};export ROS_HOSTNAME=${GAZEBOIP}; roslaunch rotors_gazebo mav_with_joy_and_ID.launch roboID:=$id Z:=10 X:=${Xs[$i]}  Y:=${Ys[$i]} --screen"
    screen -S envsim_${env_id} -X title FIREFLY${env_id}_$id
  	sleep 5
    screen -S envsim_${env_id} -X screen bash -i -c " export ROS_MASTER_URI=${ROS_MASTER}${env_id};export GAZEBO_MASTER_URI=${GAZEBO_MASTER}${env_id} ;export ROS_IP=${GAZEBOIP};export ROS_HOSTNAME=${GAZEBOIP};roslaunch tf_from_uav_pose for_one_robot.launch robotID:=$id --screen"
    screen -S envsim_${env_id} -X title TF4RMUAVPOSE${env_id}_${id}
    echo "Started tf from uav pose"
    sleep 2

    screen -S envsim_${env_id} -X screen bash -i -c "export  ROS_MASTER_URI=${ROS_MASTER}${env_id};export GAZEBO_MASTER_URI=${GAZEBO_MASTER}${env_id};export ROS_IP=${GAZEBOIP};export ROS_HOSTNAME=${GAZEBOIP}; rosrun random_moving_target  actor_joint_GT_image_overlay.py $id --screen"
    screen -S envsim_${env_id} -X title IMAGEOVERLAY${env_id}_${id}
    echo "Started image overlay node"

  done


# echo "Waiting 10 seconds for everyone to come up"
# sleep 10

screen -S envsim_${env_id} -X screen bash -i -c " export ROS_MASTER_URI=${ROS_MASTER}${env_id};export GAZEBO_MASTER_URI=${GAZEBO_MASTER}${env_id} ;export ROS_IP=${GAZEBOIP};export ROS_HOSTNAME=${GAZEBOIP};rosrun random_moving_target actor_joint_publisher.py --screen"
screen -S envsim_${env_id} -X title "ACTOR_JOINTS${env_id}"
echo "Started Actor Joint Publisher"


# echo "Starting SPIN"
# screen -S envsim_${env_id} -X screen bash -i -c "export ROS_MASTER_URI=${ROS_MASTER}${env_id}; export GAZEBO_MASTER_URI=${GAZEBO_MASTER}${env_id}; export ROS_IP=${GAZEBOIP}; export ROS_HOSTNAME=${GAZEBOIP}; . ~/spin/bin/activate; . ~/hmr_ws/devel/setup.bash ; rosrun hmr_node hmr_pub.py 1"
# screen -S envsim_${env_id} -X title HMR${env_id}
# sleep 5


# sleep 5
# echo "Starting MHMR"
# screen -S envsim_${env_id} -X screen bash -i -c "export ROS_MASTER_URI=${ROS_MASTER}${env_id}; export GAZEBO_MASTER_URI=${GAZEBO_MASTER}${env_id}; export ROS_IP=${GAZEBOIP}; export ROS_HOSTNAME=${GAZEBOIP}; . ~/mhmr/bin/activate; . ~/multihmr_ws/devel/setup.bash ; rosrun multihmr_node multihmr_pub.py 1"
# screen -S envsim_${env_id} -X title MHMR${env_id}
# sleep 20


# if [ ${env_id} -lt 3 ]; then
# echo "Starting Alphapose"
# screen -S envsim_${env_id} -X screen bash -i -c "export ROS_MASTER_URI=${ROS_MASTER}${env_id}; export GAZEBO_MASTER_URI=${GAZEBO_MASTER}${env_id}; export ROS_IP=${GAZEBOIP}; export ROS_HOSTNAME=${GAZEBOIP}; . ~/alphapose/bin/activate; . ~/alphapose_ws/devel/setup.bash ; rosrun alphapose_node alphapose_cropped.py --num_machines=2"
# screen -S envsim_${env_id} -X title ALPHA${env_id}
# sleep 5
# fi

##### START DRL TESTING #############
# params for reinforcement learning script (in order):
# <num_environments>
# <log file path and name> (create log folder in drl_ws: mkdir $LOGPATH/<filename>)
# <robotID> (unused parameter)
# <num_robots> (per env)
# <Test (unused parameter)>
echo "Starting Single Agent Deep RL testing"
LOGPATH=$AIRCAPDIR/ros_logs
screen -d -m -S DRL_Training bash -i -c "./start_drl_singleagent.sh 1 $LOGPATH/${NAME} 1 1 True"
# result=1

sleep 20
for env in $(seq 0 $(($NUM_ENVS-1))); do
  env_id=$(($env+1))
  for i in $(seq 0 $(($ROBOS-1))); do
  	id=$(($i+1))
  	echo "killing mpc $id"
    bash -i -c "export ROS_MASTER_URI=${ROS_MASTER}${env_id};export GAZEBO_MASTER_URI=${GAZEBO_MASTER}${env_id} ;export ROS_IP=${GAZEBOIP};export ROS_HOSTNAME=${GAZEBOIP}; rosnode kill /machine_${id}/nmpc_planner_${id}"
  done
done

if [ $RECORD -eq 1 ]; then
  if [ -d $AIRCAPDIR ]; then
    LOGPATH=$AIRCAPDIR/ros_logs
    LOGFILE=$( echo ${LOGPATH}/${NAME}*.bag )
    if [ -e $LOGFILE ]; then    
      echo "Experiment result exists not recording bag"
    else
      echo "saving rosbags to ${LOGPATH}"echo "saving rosbags to ${LOGPATH}"
      #START ROSBAG RECORDING. Topic to record are defined in drl_topics.txt  
      echo "recording topics mentioned in file drl_topics.txt for 120s. "
      echo "Warning image topics are recorded by default and bag sizes can get very big very quickly. Comment the following topics in drl_topics.txt to disable image recording"
      echo "/firefly_1/firefly_1/xtion/rgb/image_raw , /firefly_1/firefly_1/xtion/rgb/camera_info"
      nohup ./kill.sh &  rosbag record -b 0 -o ${LOGPATH}/${NAME}.bag $( cat drl_topics.txt | tr '\n' ' ' ) __name:=my_bag 
    fi
  else
    "$AIRCAPDIR does not exist. Not recording bag. Check git readme on how to set $AIRCAPDIR"
  fi
else
  echo "RECORD not set. Not recording bag"
fi
done

