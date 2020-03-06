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
from os.path import expanduser
'''
########## LOAD NETWORK HERE ########################
yaml_file_name (options for testing different networks):
Network2.1: test_network21.yaml
Network2.2: test_network22.yaml
Network2.3: test_network23.yaml
'''
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3' 
LoadYamlFileParamsTest(rospackage_name="my_firefly_training",
                        rel_path_from_package_to_file="config",
                        yaml_file_name="test_network23.yaml")

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
    Make The Gym Environment 
    '''
    '''Helper function for creating parallel environments for training'''
    def create_parallel_envs(env_id,robID):
        eid = env_id+1
        robID = robID+1
        print(str(eid)+str(robID))
        env = MakeOpenAIros_ENV(task_and_robot_environment_name, eid, robID, num_robots)
        env = Monitor(env, log_dir+'/monitor'+str(eid)+str(robID)+'.csv', allow_early_resets=True)
        return env
    
    envs = []
    def ret_lambda_func(k,l):
        return lambda : create_parallel_envs(k,l)

    for k in range(num_envs):
        for l in range(num_robots):
            envs.append(ret_lambda_func(k,l))

    env = SubprocVecEnv(envs, start_method='forkserver')    
    print("Gym environment done")
        
    nw_path = expanduser('~')+rospy.get_param('neural_nw_path')
    assert os.path.exists(nw_path)
    
    '''Load the network parameters to test the network'''
    model = PPO2.load(nw_path,env=env)

    '''Testing the model'''
    rospy.logerr("TESTING THE LEARNED MODEL")
    obs = env.reset()
    steps_elapsed = 0
    while True:
        action, _states = model.predict(observation=obs, deterministic = True)
        obs, rewards, dones, info = env.step(action)
        steps_elapsed+=1
        if steps_elapsed>1000000:
            steps_elapsed = 0
            obs = env.reset()
