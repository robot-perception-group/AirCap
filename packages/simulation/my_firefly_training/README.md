

### 1. Install ros-melodic-desktop-full
**Link**:
http://wiki.ros.org/melodic/Installation/Ubuntu
```
sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential
```

### 2. Clone aircaprl repository
```
cd && git clone -b aircaprl https://gitlab.tuebingen.mpg.de/aahmad/aircap.git
```

### 3. Export aircap environment variables to .bashrc and run the setup script to download all dependencies for aircap
**export "AIRCAP_PATH" to the path of the git repository you just downloaded as shown below.**<br/>  
**export "AIRCAPDIR" to the path where all aircaprl packages need to be installed as shown below. We recommend setting this to  /home/${USER}/aircaprl as seen below.**  
```
echo "export AIRCAP_PATH=/home/${USER}/aircap" >> ~/.bashrc
echo "export AIRCAPDIR=/home/$USER/aircaprl" >> ~/.bashrc
source ~/.bashrc
cd ${AIRCAP_PATH}/packages/simulation/my_firefly_training/scripts
bash aircaprl.sh
```

### (Optional) 3. Enable reward recording in rosbags. 
- Note: This would require download of large files and several external dependencies. 
- Note: Rewards depend on pose and shape estimation neural networks running alongside the deep reinforcement learning node.
- Note: Only if you plan to run these networks and record the rewards this step is required otherwise skip it.
- Note: Neural networks like Alphapose, SPIN, Multi-view HMR will be downloaded and their respective ros nodes, python virtual environments and workspaces will be created
- Note: GPU requirements for Multi-view HMR are fairly high (around 4GB on an Nvidia GTX 1080Ti) per instance
- Note: GPU requirements for Alphapose  (around 2GB on an Nvidia GTX 1080Ti) per instance
- Note: GPU requirements for SPIN  (around 2GB on an Nvidia GTX 1080Ti) per instance
```
cd ${AIRCAP_PATH}/packages/simulation/my_firefly_training/scripts
bash aircaprl_record.sh
```

### 4. Setup openAI-gym, stable-baselines and reinforcement learning ros workspace for conda virtual env
- During anaconda installation the command prompt asks "Do you wish the installer to initialize Anaconda3 by running conda init? [yes|no]" **yes**
- Recommended to use the default installation location for anaconda as prompted during installation.
```
#Anaconda 3 for python  3.7 install
cd ${AIRCAPDIR} && wget -O ${AIRCAPDIR}/anaconda_install.sh https://repo.anaconda.com/archive/Anaconda3-2019.10-Linux-x86_64.sh
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
- Setup the deep reinforcement learning - ros - openAIGym workspace
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
cd ${AIRCAPDIR} && wget -O networks.zip https://owncloud.tuebingen.mpg.de/index.php/s/oD6N9smx7xHe9Ad/download
unzip networks.zip
mv networks/* ~/aircaprl/drl_ws/logs
rm networks.zip
rm -r networks
```

**The directory hierarchy should look like:**

     | ${AIRCAPDIR}/drl_ws 

          | logs

                 || 1.1
                 || 1.2
                 ...so on
  

### 7. Startup all the ros nodes and the gazebo test environment

- The following scripts should startup ros nodes, gazebo and drl testing
- **It may take sometime to start gazebo for first time so be patient (around 30 seconds):)** 


- To start script the format is as follows
- script.sh  <number_of_experiment_runs (default value: 1)> <rosbag file name or experiment name (default name: test)> <record-bag-flag (default value: 0)>
- **We noticed that Gazebo could crash on the first run if it was newly installed.**
- **If Gazebo client does not startup in 30 seconds, run ./killswitch to kill all nodes and rerun the below script**
- **If record-bag-flag = 1 , then the bags save images and rewards for 120s. The saving of bags after 120s of experiment run could take a few minutes as images have to be written from memory.**
- For single agent drl
```
cd ${AIRCAPDIR}/aircap_ws/src/scripts/simulation
./single_agent_loop.sh 1 test 0
```
- For multi agent drl
```
cd ${AIRCAPDIR}/aircap_ws/src/scripts/simulation
./multi_agent_loop.sh 1 test 0
```


### 8. Kill all the ros nodes
Always execute the killswitch node after each experimental run to kill all ros nodes
```
cd ${AIRCAPDIR}/aircap_ws/src/scripts/simulation
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


### (Optional) 10. View the debug output 
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


### (Optional) 11. Recording rewards in rosbags
- Until step 9 setting record-bag-flag will record images but it will not record rewards. This is because the rewards are obtained from pose and shape-estimation neural networks.
- Here we describe how to record the rewards into rosbags during testing.

1. Enabling alphapose ros node

     - Download the models manually: duc_se.pth : https://drive.google.com/uc?id=1OPORTWB2cwd5YTVBX-NE8fsauZJWsrtW&export=download, yolov3-spp.weights: https://drive.google.com/uc?id=1D47msNOOiJKvPOXlnpyzdKA3k6E97NTC&export=download

     - Place them into ${AIRCAPDIR}/git/Alphapose/models/sppe and ${AIRCAPDIR}/git/Alphapose/models/yolo respectively.

2. Enabling SPIN

     - Download the *SMPL* model. You will need the [neutral model](http://smplify.is.tue.mpg.de) for running the.  Please go to the websites for the corresponding projects and register to get access to the downloads section. In case you need to convert the models to be compatible with python3, please follow the instructions [here](https://github.com/vchoutas/smplx/tree/master/tools).

     - Rename downloaded smpl neutral model as "SMPL_NEUTRAL.pkl". Place the smpl neutral model in ${AIRCAPDIR}/git/SPIN/data/smpl

3.  Enabling Multi-View HMR
     -  We use tensorflow 1.12.0 specified in ${AIRCAP_PATH}/multihmr_node/requirements.txt. Edit this file to use a different version.

     -  We tested on cuda-9.0 and cudnn-7.3-cu9.0. Do not forget to export all environment variables

     -  Download pretrained multi-view HMR model here: https://drive.google.com/file/d/1grEX6HmqL6CKittCyl_N6nggqIRIEOCt/view

     -   Extract the downloaded file into ${AIRCAPDIR}/git/HumanMultiView/

