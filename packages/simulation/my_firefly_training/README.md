

## 1. install ros-melodic-desktop-full
```
# Instructions copied from http://wiki.ros.org/melodic/Installation/Ubuntu
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update  &&   sudo apt install ros-melodic-desktop-full -y
sudo rosdep init 
rosdep update -y
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential
```

## 2. clone aircaprl repository
```
git clone -b aircaprl https://github.com/robot-perception-group/AirCap.git
```

## 3. setup script to download all dependencies for aircap
```
export AIRCAP_PATH=<path_to_aircap_git_repo>
cd ${AIRCAP_PATH}/packages/simulation/my_firefly_training/scripts
bash aircaprl.sh
```

## 4. Setup OpenAI-gym, stable-baselines and reinforcement learning ros workspace for conda virtual env
- Note: install anaconda for python 3.7 from https://docs.continuum.io/anaconda/install/linux/ 
- After anaconda installation the command prompt asks "Do you wish the installer to initialize Anaconda3 by running conda init? [yes|no]" no
```
bash setup_drl_ws.sh
```

## 5. Build Actor Plugin

```
cd ${AIRCAP_PATH}/packages/simulation/Gazebo_Plugins
mkdir build && cd build
cmake ..
make
```
- In bashrc file add the following line:
```
echo "export GAZEBO_PLUGIN_PATH=${AIRCAP_PATH}/packages/simulation/Gazebo_Plugins/build" >> ~/.bashrc
source ~/.bashrc && cd
```

## 6. Download trained networks
'''
Download all the trained networks from here: https://owncloud.tuebingen.mpg.de/index.php/s/oD6N9smx7xHe9Ad
Extract/Copy the networks into the folder ~/aircaprl/drl_ws/logs
The directory hierarchy should look like:
~/aircaprl/drl_ws 
     | logs 
            || 1.1
            || 1.2
            ...so on
  
'''
## 6. Make manual edits to setup scripts to enable networked setup

- edit ~/aircaprl/aircap_ws/src/scripts/setup_mavocap_drl_multiagent.sh
- find and replace : ROSIP with approporiate master IP address
- In the script the gazebo master is different for different environment IDs as they can run on multiple computers
- find and replace : GAZEBOIP with approporiate gazebo server IP address in the if else bash statements

- same edits to the following gazebo simulation launch file
- ~/aircaprl/aircap_ws/src/scripts/start_gazebo_singleagent_envs.sh
- ~/aircaprl/aircap_ws/src/scripts/start_gazebo_multiagent_envs.sh

- same edits to the following drl startup scripts
- ~/aircaprl/aircap_ws/src/scripts/start_drl_singleagent.sh
- ~/aircaprl/aircap_ws/src/scripts/start_drl_multiagent.sh

- in the file: ~/aircaprl/drl_ws/src/openai_ros/src/openai_ros/robot_gazebo_env.py,
- find and replace ROSIP and GAZEBOIP appropriately


- define ros bag log file in line: LOGPATH="/home/${USER}/ros_logs"
- NOTE: Alphapose network is always launched with start_gazebo_multiagent_envs.sh i.e. along with the gazebo servers


## 7. startup all nodes and environments
```
# For single agent drl
./single_agent_loop.sh 1 test
# params (in order):
# <num_envs>
# <rosbag file name or experiment name (currently option not enabled)>

# For multi agent drl
./multi_agent_loop.sh 1 test
# <num_envs>
# <rosbag file name or experiment name (currently option not enabled)>
```
