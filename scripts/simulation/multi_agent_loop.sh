RUNS=$1
NAME=$2
for i in $(seq 0 $(($RUNS-1))); do
    ./setup_mavocap_drl_multiagent.sh 2 True 100 ${NAME}$i 1
    ./start_gazebo_multiagent_envs.sh 2 True 100 ${NAME}$i 1 
done