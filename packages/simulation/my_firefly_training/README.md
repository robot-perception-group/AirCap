

### 1. Install ros-melodic-desktop-full
**Link**:
http://wiki.ros.org/melodic/Installation/Ubuntu
```
sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential
```

### 2. Clone aircaprl repository
```
cd && git clone -b aircaprl https://github.com/robot-perception-group/AirCap.git
```

### 3. Export aircap environment variables to .bashrc and run the setup script to download all dependencies for aircap
**export "AIRCAP_PATH" to the path of the git repository you just downloaded as shown below.**<br/>  
**export "AIRCAPDIR" to the path where all aircaprl packages need to be installed as shown below. We recommend setting this to  /home/${USER}/aircaprl as seen below.**  
```
echo "export AIRCAP_PATH=/home/${USER}/AirCap" >> ~/.bashrc
echo "export AIRCAPDIR=/home/$USER/aircaprl" >> ~/.bashrc
source ~/.bashrc
cd ${AIRCAP_PATH}/packages/simulation/my_firefly_training/scripts
bash aircaprl.sh
```

### 4. Setup openAI-gym, stable-baselines and reinforcement learning ros workspace for conda virtual env
- During anaconda installation the command prompt asks "Do you wish the installer to initialize Anaconda3 by running conda init? [yes|no]" **yes**
```
#Anaconda 3 for python  3.7 install
wget -O ${AIRCAPDIR}/anaconda_install.sh https://repo.anaconda.com/archive/Anaconda3-2019.10-Linux-x86_64.sh
cd ${AIRCAPDIR} && bash anaconda_install.sh
rm anaconda_install.sh
```
- **At the end of the .bashrc file comment the lines starting from  >>> conda initialize >>> to  <<< conda initialize <<<**
```
gedit ~/.bashrc
# search for >>> conda initialize >>>
# comment lines  from >>> conda initialize >>> to <<< conda initialize <<< 
# save and close the file
```
- Setup the deep reinforcement learning - ros - openAiGym workspace
```
cd ${AIRCAP_PATH}/packages/simulation/my_firefly_training/scripts
bash setup_drl_ws.sh
```

### 5. Build the actor plugin
```
cd ${AIRCAP_PATH}/packages/simulation/Gazebo_Plugins
mkdir build && cd build
cmake ..
make 
echo "export GAZEBO_PLUGIN_PATH=${AIRCAP_PATH}/packages/simulation/Gazebo_Plugins/build" >> ~/.bashrc
source ~/.bashrc && cd ${AIRCAPDIR}
```

### 6. Download the trained networks
```
wget -O networks.zip https://owncloud.tuebingen.mpg.de/index.php/s/oD6N9smx7xHe9Ad/download
unzip networks.zip
mv networks/* ~/aircaprl/drl_ws/logs
rm networks.zip
```

**The directory hierarchy should look like:**

| ~/aircaprl/drl_ws 

     | logs

            || 1.1
            || 1.2
            ...so on
  

### 7. Startup all the ros nodes and the gazebo test environment

- The following scripts should startup ros nodes, gazebo and drl testing
- **It may take sometime to start gazebo for first time so be patient (around 30 seconds):)** 


- To start script the format is as follows
- script.sh  <number_of_experiment_runs (default value: 1)> <rosbag file name or experiment name (default name: test)> <record-bag-flag (default value: 0)>
- **We noticed thatI Gazebo could crash on the first run if it was newly installed.**
- **If Gazebo client does not startup in 30 seconds, run ./killswitch to kill all nodes and rerun the below script**
- **If record-bag-flag = 1 , then the bags save images and rewards for 120s. The saving of bags after 120s of experiment run code take a few minutes as images have to be written from memory.**
- For single agent drl
```
cd ~/aircaprl/aircap_ws/src/scripts/simulation
./single_agent_loop.sh 1 test 0
```
- For multi agent drl
```
cd ~/aircaprl/aircap_ws/src/scripts/simulation
./multi_agent_loop.sh 1 test 0
```


### 8. Kill all the ros nodes
Always execute the killswitch node after each experimental run to kill all ros nodes
```
cd ~/aircaprl/aircap_ws/src/scripts/simulation
./killswitch.sh 1
```

### 9. Test different networks
#### Single Agent:

- Navigate to ~/aircaprl/drl_ws/src/my_firefly_training/src/test_singleagent.py
- Line 29 has the yaml file pointing to the network parameters. 
- Currently it is running Network 1.4 >> "test_network14.yaml"
- Options to change this file :
```
-- Network1.1: test_network11.yaml
-- Network1.2: test_network12.yaml
-- Network1.3: test_network13.yaml
-- Network1.4: test_network14.yaml
```

#### Multi Agent
- Navigate to ~/aircaprl/drl_ws/src/my_firefly_training/src/test_multiagent.py
- Line 29 has the yaml file pointing to the network parameters. 
- Currently it is running Network 2.3 >> "test_network23.yaml"
- Options to change this file:
```
-- Network2.1: test_network21.yaml
-- Network2.2: test_network22.yaml
-- Network2.3: test_network23.yaml
```
- Note! for networks 2.1 and 2.2, the actor has to be static. 
- Uncomment line 141 "this->velocity  = 0.0;" in file ~/aircalrl/aircap_ws/src/Gazebo_Plugins/ActorPlugin.cc
- Run "make" as suggested in STEP 5 above. 
- Network 2.3 however can operate on a moving actor


### 10. View the debug output 
Each ros node runs in its own screen window in bash.

The screens running are env_$id,envsim_$id and $DRL_Training

To access the screens e.g.,:
```
- For aircap ros nodes
screen -R env_1

- For aircap ros nodes
screen -R envsim_1
- For DRL ros node
screen -R DRL_Training
```

Once you are accessing a screen

Switch between different windows in screen using the shortcut ctrl+A+N
