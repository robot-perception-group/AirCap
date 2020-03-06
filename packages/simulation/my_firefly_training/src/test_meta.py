#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os, sys
import numpy as np

from openai_ros.openai_ros_common import StartOpenAI_ROS_Environment_Parallel
from openai_ros.task_envs.task_envs_list_testing import RegisterOpenAI_Ros_Env
from openai_ros.task_envs.task_commons import LoadYamlFileParamsTest
import rospy

import tensorflow as tf
import gym
from stable_baselines.common.vec_env import SubprocVecEnv
from stable_baselines import PPO2
from stable_baselines.bench import Monitor


os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3' 
LoadYamlFileParamsTest(rospackage_name="my_firefly_training",
                        rel_path_from_package_to_file="config",
                        yaml_file_name="test_meta.yaml")

task_and_robot_environment_name = rospy.get_param('task_and_robot_environment_name')
num_envs = int(sys.argv[1])
robotID = int(sys.argv[3])
num_robots = int(sys.argv[4])

'''Ensure arguments are valid''' 
'''Only 2 robots are currently supported for the multi-agent case'''
assert num_envs>0
assert robotID>0
assert num_robots==2, "Number of Robots must be 2"



def MakeOpenAIros_ENV(task_and_robot_environment_name, env_id, robotID, num_robots):
    """
    1) Registers the TaskEnvironment wanted, if it exists in the Task_Envs.
    2) It will import the Gym Env and Make it.
    """
    rospy.logwarn("Env: {} will be imported".format(
        task_and_robot_environment_name))
    result = RegisterOpenAI_Ros_Env(task_env=task_and_robot_environment_name)


    if result:
        rospy.logwarn("Register of Task Env went OK, lets make the env..."+str(task_and_robot_environment_name))
        env = gym.make(task_and_robot_environment_name, env_id=int(str(env_id)+str(robotID)), robotID=robotID, num_robots=num_robots)
    else:
        rospy.logwarn("Registration Failed. Check given environment Name")
        env = None
    return env

if __name__ == "__main__":
    
    # Create log dir
    try:
        log_dir=sys.argv[2]+"/snapshots/"
        print("Saving Models in: "+log_dir)
        os.makedirs(log_dir, exist_ok=True)
    except ValueError:
            print('Unable to create directory, Check the path provided')


        
    '''
    Make Both The Gym Environments 
    '''
    '''Helper function for creating parallel environments for training'''
    def create_parallel_envs(env_id,robID,env_name):
        eid = env_id+1
        robID = robID+1
        print(str(eid)+str(robID))
        env = MakeOpenAIros_ENV(env_name, eid, robID, num_robots)
        env = Monitor(env, log_dir+'/monitor'+str(eid)+str(robID)+'.csv', allow_early_resets=True)
        return env
    
    envs = []
    def ret_lambda_func(k,l,name):
        return lambda : create_parallel_envs(k,l,name)
    
    for k in range(num_envs):
        for l in range(num_robots):
            envs.append(ret_lambda_func(k,l,task_and_robot_environment_name))
    
    env = SubprocVecEnv(envs, start_method='forkserver')    
    print("Gym environment done")
    
    nw_path1 = rospy.get_param('neural_nw_path1')
    assert os.path.exists(nw_path1)
    
    '''Load the network parameters to test the network'''
    model1 = PPO2.load(nw_path1,env=env)
    
    #Multi Agent Following
    nw_path2 = rospy.get_param('neural_nw_path2')
    assert os.path.exists(nw_path2)
    
    '''Load the network parameters to test the network'''
    model2 = PPO2.load(nw_path2,env=env)

    '''Testing the model'''
    rospy.logerr("TESTING THE LEARNED MODEL")
    
    obs = env.reset();
    steps_elapsed = 0
    while True:
        obs1 = obs[:,:65]
        obs2 = obs[:,65:]        
        action1, _states = model1.predict(observation=obs1, deterministic = True)
        action2, _states = model2.predict(observation=obs2, deterministic = True)
        
        '''Take the average of the translational velocities in x and y. Supply the yawrate from single agent model'''
        action_meta = (action1[:,0:2]+action2[:,0:2])/2
        action_meta = np.c_[action_meta,action1[:,2]]

        obs, rewards, dones, info = env.step(action_meta)
        steps_elapsed+=1
        print('########################## Steps Elapsed:'+str(steps_elapsed)+'#######################################')
