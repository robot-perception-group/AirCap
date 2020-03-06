

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
Download all the trained networks from here: https://owncloud.tuebingen.mpg.de/index.php/s/oD6N9smx7xHe9Ad
Extract/Copy the networks into the folder ~/aircaprl/drl_ws/logs

The directory hierarchy should look like:

~/aircaprl/drl_ws 

     | logs

            || 1.1
            || 1.2
            ...so on
  

## 6. startup all nodes and environments

#The following scripts should startup ros nodes, gazebo and drl testing
- For single agent drl
#params (in order):
#<num_envs>
#<rosbag file name or experiment name (currently option not enabled)>
```
cd ~/aircaprl/aircap_ws/src/scripts/simulation
./single_agent_loop.sh 1 test
```

- For multi agent drl
#Note! for networks 2.1 and 2.2, the actor has to be static. 
#Uncomment line 141 "this->velocity  = 0.0;" in file ~/aircalrl/aircap_ws/src/Gazebo_Plugins/ActorPlugin.cc
#Run make as suggested in STEP 5 above. 
# For network 2.3 can operate on a moving actor
```
./multi_agent_loop.sh 1 test
```


## 7. Kill all nodes
```
cd ~/aircaprl/aircap_ws/src/scripts/simulation
./killswitch 1
```

## 7. Change Networks
### Single Agent:

- Navigate to ~/aircaprl/drl_ws/src/my_firefly_training/src/test_singleagent.py
- Line 29 has the yaml file pointing to the network parameters. 
- currently it is running Network 1.4 >> "test_network14.yaml"
- Options to change this file to 
- Network1.1-test_network11.yaml
- Network1.2-test_network12.yaml
- Network1.3-test_network13.yaml
- Network1.4-test_network14.yaml

### Multi Agent
- Navigate to ~/aircaprl/drl_ws/src/my_firefly_training/src/test_multiagent.py
- Line 29 has the yaml file pointing to the network parameters. 
- currently it is running Network 2.3 >> "test_network23.yaml"
- Options to change this file to 
- Network2.1-test_network21.yaml
- Network2.2-test_network22.yaml
- Network2.3-test_network23.yaml


## 7. DEBUGGING
Each ros node runs in its own screen window in bash.

The screens running are env_$id,envsim_$id and $DRL_Training

To access the screens e.g.,:
```
- For aircap ros nodes
screen -R env_1

- For aircap ros nodes
$screen -R envsim_1
- For DRL ros node
$screen -R DRL_Training
```

Once you are accessing a screen

Switch between different windows in screen using the shortcut ctrl+A+N
