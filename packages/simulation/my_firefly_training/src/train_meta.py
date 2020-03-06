#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os, sys
import numpy as np

from meta_network import custom_load, MetaPolicy

from openai_ros.openai_ros_common import StartOpenAI_ROS_Environment_Parallel
from openai_ros.task_envs.task_envs_list_testing import RegisterOpenAI_Ros_Env
from openai_ros.task_envs.task_commons import LoadYamlFileParamsTest
import rospy

import tensorflow as tf
import gym
from stable_baselines.common.vec_env import SubprocVecEnv
from stable_baselines import PPO2
from stable_baselines.bench import Monitor
from stable_baselines.results_plotter import load_results, ts2xy


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
best_mean_reward, n_steps = -np.inf, 0


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


def callback(_locals, _globals):
  """
  Callback called after n steps (PPO2)
  :param _locals: (dict)
  :param _globals: (dict)
  """
  global n_steps, best_mean_reward
  # import ipdb; ipdb.set_trace()
  # Print stats every 10 calls
  if (n_steps + 1) % 10 == 0:
      # Evaluate policy performance
      _locals['self'].save(log_dir+'checkpoint_model'+str(n_steps + 1)+'.pkl')

      # import ipdb; ipdb.set_trace()
      err_flag = True
      try:
          x, y = ts2xy(load_results(log_dir), 'timesteps')


      except Exception:
          import ipdb; ipdb.set_trace()
          logf = open(log_dir+"ts2xyerr"+str(n_steps+1)+".log", "w")
          traceback.print_exc(file=logf)
          logf.close()
          err_flag = False

      if err_flag:
	      if len(x) > 0:
    		  try:
    		      mean_reward = np.mean(y[-10:])
    		      print(x[-1], 'timesteps')
    		      print("Best mean reward: {:.2f} - Last mean reward per episode: {:.2f}".format(best_mean_reward, mean_reward))

    		      # New best model, you could save the agent here
    		      if mean_reward > best_mean_reward:
    		          best_mean_reward = mean_reward
    		          # Example for saving best model
    		          print("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@Saving new best model")
    		          _locals['self'].save(log_dir + 'best_model.pkl')
    		  except Exception:
    		      logf = open(log_dir+"meanerr"+str(n_steps+1)+".log", "w")

    		      traceback.print_exc(file=logf)
    		      logf.close()
    	      # except:
    		  # rospy.logwarn("Weird")
    		  # _locals['self'].save(log_dir + 'weird_model.pkl')
  n_steps += 1
  return True


if __name__ == "__main__":

    # Create log dir
    try:
        log_dir=sys.argv[2]+"/snapshots/"
        print("Saving Models in: "+log_dir)
        os.makedirs(log_dir, exist_ok=True)
    except ValueError:
            print('Unable to create directory, Check the path provided')



    '''
    Make The Gym Environments
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

    ##Single Agent Following
    nw_path1 = rospy.get_param('neural_nw_path1')
    assert os.path.exists(nw_path1)

    ##Multi Agent Following
    nw_path2 = rospy.get_param('neural_nw_path2')
    assert os.path.exists(nw_path2)

    PPO2.custom_load = custom_load
    model = PPO2(MetaPolicy, env, verbose=1,tensorboard_log=log_dir)

    model,params = model.custom_load(nw_path1,sa_or_ma='sa')
    model,_ = model.custom_load(nw_path2,sa_or_ma='ma')

    '''Training the model'''
    rospy.logerr("LEARNING A MODEL")
    model.learn(total_timesteps=4000000,tb_log_name = log_dir,callback = callback)
    model.save(log_dir+"/trained_model")
    rospy.logwarn("DONE TRAINING")
