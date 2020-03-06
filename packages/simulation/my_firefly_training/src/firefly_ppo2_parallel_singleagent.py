#!/usr/bin/env python


# -*- coding: utf-8 -*-


import gym
import numpy as np
from gym.envs.registration import register
import multiprocessing as mp
from openai_ros.openai_ros_common import StartOpenAI_ROS_Environment,StartOpenAI_ROS_Environment_Parallel
import traceback

import tensorflow as tf
from tensorflow import keras
from tensorflow.keras import layers

import rospy
import rospkg
# import our training environment
import openai_ros.task_envs.firefly.formation
import datetime
#import stable_baselines.ddpg.policies as pol

# from stable_baselines.a2c.utils import linear
from stable_baselines.a2c.utils import ortho_init
from stable_baselines.common.policies import MlpPolicy, ActorCriticPolicy, MlpLnLstmPolicy, FeedForwardPolicy
from stable_baselines.common.vec_env import DummyVecEnv, SubprocVecEnv
from stable_baselines import PPO2, A2C, DDPG
from stable_baselines.bench import Monitor
from stable_baselines.results_plotter import load_results, ts2xy
import os, sys
# import cProfile
# import re
# import pstats
# import pstats
# cProfile.run('re.compile("foo|bar")', 'restats')
# p = pstats.Stats('restats')
# p.sort_stats('time').print_stats(10)
# cProfile.run('re.compile("foo|bar")')


LOGDIR="/home/rtallamraju/drl_ws/logs"
ALG = "/ppo2_mlp_aamas"
task_and_robot_environment_name = 'DeepFollow-v11'
num_envs = int(sys.argv[1])
robotID = int(sys.argv[3])
num_robots = int(sys.argv[4])
prediction_only = sys.argv[5]
#rospy.init_node('test_firefly_env',anonymous=True, log_level=rospy.WARN)
best_mean_reward, n_steps = -np.inf, 0

def linear(input_tensor, scope, n_hidden, *, init_scale=1.0, init_bias=0.0, trainable=True):
    """
    Creates a fully connected layer for TensorFlow

    :param input_tensor: (TensorFlow Tensor) The input tensor for the fully connected layer
    :param scope: (str) The TensorFlow variable scope
    :param n_hidden: (int) The number of hidden neurons
    :param init_scale: (int) The initialization scale
    :param init_bias: (int) The initialization offset bias
    :return: (TensorFlow Tensor) fully connected layer
    """
    with tf.variable_scope(scope):
        n_input = input_tensor.get_shape()[1].value
        weight = tf.get_variable("w", [n_input, n_hidden], initializer=ortho_init(init_scale), trainable=trainable)
        bias = tf.get_variable("b", [n_hidden], initializer=tf.constant_initializer(init_bias), trainable=trainable)
        return tf.matmul(input_tensor, weight) + bias


class CustomPolicy(FeedForwardPolicy):
    def __init__(self, *args, **kwargs):
        super(CustomPolicy, self).__init__(*args, **kwargs, act_fun=tf.nn.relu,
                                           net_arch=[dict(pi=[256,256],
                                                          vf=[256,256])],
                                           feature_extraction="mlp")

class CustomPolicyYawrate(FeedForwardPolicy):
    def __init__(self, *args, **kwargs):
        super(CustomPolicyYawrate, self).__init__(*args, **kwargs, act_fun=tf.nn.relu,
                                           net_arch=[dict(pi=[64, 64],
                                                          vf=[64, 64])],
                                           feature_extraction="mlp")

class CustomPolicy2(ActorCriticPolicy):
    def __init__(self, sess, ob_space, ac_space, n_env, n_steps, n_batch, reuse=False, **kwargs):
        super(CustomPolicy2, self).__init__(sess, ob_space, ac_space, n_env, n_steps, n_batch, reuse=reuse, scale=False)

        with tf.variable_scope("model", reuse=reuse):
            act_fun = tf.nn.relu

            tracking_obs, joint_obs = tf.split(self.processed_obs, [5,34],1)
            pi_h = tracking_obs
            vf_h = tracking_obs

            #Tracking Network
            for i, layer_size in enumerate([256, 256]):
                pi_h = act_fun(linear(pi_h, "pi_fc{}".format(i), layer_size, init_scale=np.sqrt(2), trainable=False))
            for i, layer_size in enumerate([256, 256]):
                vf_h = act_fun(linear(vf_h, "vf_fc{}".format(i), layer_size, init_scale=np.sqrt(2), trainable=False))


            pi_latent_t = pi_h
            vf_latent_t = vf_h

            # Person joint n/w (from Alphapose)
            pi_h = joint_obs
            vf_h = joint_obs

            #Tracking Network
            for i, layer_size in enumerate([256, 256]):
                pi_h = act_fun(linear(pi_h, "pi_j_fc{}".format(i), layer_size, init_scale=np.sqrt(2)))

            for i, layer_size in enumerate([256, 256]):
                vf_h = act_fun(linear(vf_h, "vf_j_fc{}".format(i), layer_size, init_scale=np.sqrt(2)))


            pi_latent_j = pi_h
            vf_latent_j = vf_h
            pi_concat_h = tf.concat([pi_latent_t,pi_latent_j],1)
            vf_concat_h = tf.concat([vf_latent_t,vf_latent_j],1)

            pi_latent = act_fun(linear(pi_concat_h, "pi_tj_fc", 256, init_scale=np.sqrt(2)))
            vf_latent = act_fun(linear(vf_concat_h, "vf_tj_fc", 256, init_scale=np.sqrt(2)))
            value_fn = linear(vf_latent, 'vf', 1)

            self._proba_distribution, self._policy, self.q_value = \
                self.pdtype.proba_distribution_from_latent(pi_latent, vf_latent, init_scale=0.01)

        self._value_fn = value_fn
        self._setup_init()

    def step(self, obs, state=None, mask=None, deterministic=False):
        if deterministic:
            action, value, neglogp = self.sess.run([self.deterministic_action, self.value_flat, self.neglogp],
                                                   {self.obs_ph: obs})
        else:
            action, value, neglogp = self.sess.run([self.action, self.value_flat, self.neglogp],
                                                   {self.obs_ph: obs})
        return action, value, self.initial_state, neglogp

    def proba_step(self, obs, state=None, mask=None):
        return self.sess.run(self.policy_proba, {self.obs_ph: obs})

    def value(self, obs, state=None, mask=None):
        return self.sess.run(self.value_flat, {self.obs_ph: obs})

class TriangulationPolicy(ActorCriticPolicy):
    def __init__(self, sess, ob_space, ac_space, n_env, n_steps, n_batch, reuse=False, **kwargs):
        super(TriangulationPolicy, self).__init__(sess, ob_space, ac_space, n_env, n_steps, n_batch, reuse=reuse, scale=False)

        with tf.variable_scope("model", reuse=reuse):
            act_fun = tf.nn.relu

            joint_obs, formation_obs, angles_obs = tf.split(self.processed_obs, [56,4,4],1)
            pi_h = joint_obs
            vf_h = joint_obs

            #Joints Network
            for i, layer_size in enumerate([128,128]):
                pi_h = act_fun(linear(pi_h, "pi_j_fc{}".format(i), layer_size, init_scale=np.sqrt(2)))
            for i, layer_size in enumerate([128,128]):
                vf_h = act_fun(linear(vf_h, "vf_j_fc{}".format(i), layer_size, init_scale=np.sqrt(2)))


            pi_latent_j = pi_h
            vf_latent_j = vf_h

            pi_latent_visj = tf.get_variable("act_j",[1,128])
            pi_latent_visj.assign(tf.layers.flatten(pi_latent_j))

            # Formation Network 1
            pi_h = formation_obs
            vf_h = formation_obs

            for i, layer_size in enumerate([64,64]):
                pi_h = act_fun(linear(pi_h, "pi_f_fc{}".format(i), layer_size, init_scale=np.sqrt(2)))

            for i, layer_size in enumerate([64,64]):
                vf_h = act_fun(linear(vf_h, "vf_f_fc{}".format(i), layer_size, init_scale=np.sqrt(2)))


            pi_latent_f1 = pi_h
            vf_latent_f1 = vf_h

            pi_latent_visf = tf.get_variable("act_f", [1,64])
            pi_latent_visf.assign(pi_latent_f1)

            # Formation Network 2
            pi_h = angles_obs
            vf_h = angles_obs

            for i, layer_size in enumerate([64,64]):
                pi_h = act_fun(linear(pi_h, "pi_a_fc{}".format(i), layer_size, init_scale=np.sqrt(2)))

            for i, layer_size in enumerate([64,64]):
                vf_h = act_fun(linear(vf_h, "vf_a_fc{}".format(i), layer_size, init_scale=np.sqrt(2)))


            pi_latent_f2 = pi_h
            vf_latent_f2 = vf_h

            pi_latent_visa = tf.get_variable("act_a", [1,64])
            pi_latent_visa.assign(pi_latent_f2)

            pi_concat_h = tf.concat([pi_latent_j,pi_latent_f1,pi_latent_f2],1)
            vf_concat_h = tf.concat([vf_latent_j,vf_latent_f1,vf_latent_f2],1)

            pi_latent = act_fun(linear(pi_concat_h, "pi_jfa_fc1", 256, init_scale=np.sqrt(2)))
            pi_latent_vis1 = tf.get_variable("act_jfa1", [1,256])
            pi_latent_vis1.assign(pi_latent)
            pi_latent = act_fun(linear(pi_latent, "pi_jfa_fc2", 256, init_scale=np.sqrt(2)))
            pi_latent_vis2 = tf.get_variable("act_jfa2", [1,256])
            pi_latent_vis2.assign(pi_latent)
            vf_latent = act_fun(linear(vf_concat_h, "vf_jfa_fc1", 256, init_scale=np.sqrt(2)))
            vf_latent = act_fun(linear(vf_latent, "vf_jfa_fc2", 256, init_scale=np.sqrt(2)))
            value_fn = linear(vf_latent, 'vf', 1)

            self._proba_distribution, self._policy, self.q_value = \
                self.pdtype.proba_distribution_from_latent(pi_latent, vf_latent, init_scale=0.01)

        self._value_fn = value_fn
        self._setup_init()

    def step(self, obs, state=None, mask=None, deterministic=False):
        if deterministic:
            action, value, neglogp = self.sess.run([self.deterministic_action, self.value_flat, self.neglogp],
                                                   {self.obs_ph: obs})
        else:
            action, value, neglogp = self.sess.run([self.action, self.value_flat, self.neglogp],
                                                   {self.obs_ph: obs})
        return action, value, self.initial_state, neglogp

    def proba_step(self, obs, state=None, mask=None):
        return self.sess.run(self.policy_proba, {self.obs_ph: obs})

    def value(self, obs, state=None, mask=None):
        return self.sess.run(self.value_flat, {self.obs_ph: obs})

class TriangulationPolicyGT(ActorCriticPolicy):
    def __init__(self, sess, ob_space, ac_space, n_env, n_steps, n_batch, reuse=False, **kwargs):
        super(TriangulationPolicyGT, self).__init__(sess, ob_space, ac_space, n_env, n_steps, n_batch, reuse=reuse, scale=False)

        with tf.variable_scope("model", reuse=reuse):
            act_fun = tf.nn.relu
            joint_obs, formation_obs, angles_obs, gt_obs = tf.split(self.processed_obs, [56,4,4,48],1)
            pi_h = joint_obs
            #Joints Network
            for i, layer_size in enumerate([128,128]):
                pi_h = act_fun(linear(pi_h, "pi_j_fc{}".format(i), layer_size, init_scale=np.sqrt(2)))
            pi_latent_j = pi_h
            pi_latent_visj = tf.get_variable("act_j",[1,128])
            pi_latent_visj.assign(tf.layers.flatten(pi_latent_j))
            # Formation Network 1
            pi_h = formation_obs
            for i, layer_size in enumerate([64,64]):
                pi_h = act_fun(linear(pi_h, "pi_f_fc{}".format(i), layer_size, init_scale=np.sqrt(2)))
            pi_latent_f1 = pi_h
            pi_latent_visf = tf.get_variable("act_f", [1,64])
            pi_latent_visf.assign(pi_latent_f1)

            # Formation Network 2
            pi_h = angles_obs
            for i, layer_size in enumerate([64,64]):
                pi_h = act_fun(linear(pi_h, "pi_a_fc{}".format(i), layer_size, init_scale=np.sqrt(2)))
            pi_latent_f2 = pi_h
            pi_latent_visa = tf.get_variable("act_a", [1,64])
            pi_latent_visa.assign(pi_latent_f2)

            pi_concat_h = tf.concat([pi_latent_j,pi_latent_f1,pi_latent_f2],1)
            pi_latent = act_fun(linear(pi_concat_h, "pi_jfa_fc1", 256, init_scale=np.sqrt(2)))
            pi_latent_vis1 = tf.get_variable("act_jfa1", [1,256])
            pi_latent_vis1.assign(pi_latent)
            pi_latent = act_fun(linear(pi_latent, "pi_jfa_fc2", 256, init_scale=np.sqrt(2)))
            pi_latent_vis2 = tf.get_variable("act_jfa2", [1,256])
            pi_latent_vis2.assign(pi_latent)

            vf_h = gt_obs
            for i, layer_size in enumerate([256,256]):
                vf_h = act_fun(linear(vf_h, "vf_j_fc{}".format(i), layer_size, init_scale=np.sqrt(2)))
            vf_latent = vf_h

            value_fn = linear(vf_latent, 'vf', 1)

            self._proba_distribution, self._policy, self.q_value = \
                self.pdtype.proba_distribution_from_latent(pi_latent, vf_latent, init_scale=0.01)

        self._value_fn = value_fn
        self._setup_init()

    def step(self, obs, state=None, mask=None, deterministic=False):
        if deterministic:
            action, value, neglogp = self.sess.run([self.deterministic_action, self.value_flat, self.neglogp],
                                                   {self.obs_ph: obs})
        else:
            action, value, neglogp = self.sess.run([self.action, self.value_flat, self.neglogp],
                                                   {self.obs_ph: obs})
        return action, value, self.initial_state, neglogp

    def proba_step(self, obs, state=None, mask=None):
        return self.sess.run(self.policy_proba, {self.obs_ph: obs})

    def value(self, obs, state=None, mask=None):
        return self.sess.run(self.value_flat, {self.obs_ph: obs})

class CustomPolicyTest(ActorCriticPolicy):
    def __init__(self, sess, ob_space, ac_space, n_env, n_steps, n_batch, reuse=False, **kwargs):
        super(CustomPolicyTest, self).__init__(sess, ob_space, ac_space, n_env, n_steps, n_batch, reuse=reuse, scale=False)

        with tf.variable_scope("model", reuse=reuse):
            act_fun = tf.nn.relu

            tracking_obs = self.processed_obs
            pi_h = tracking_obs
            vf_h = tracking_obs

            #Tracking Network
            for i, layer_size in enumerate([256, 256]):
                pi_h = act_fun(linear(pi_h, "pi_t_fc{}".format(i), layer_size, init_scale=np.sqrt(2)))
            for i, layer_size in enumerate([256, 256]):
                vf_h = act_fun(linear(vf_h, "vf_t_fc{}".format(i), layer_size, init_scale=np.sqrt(2)))

            pi_latent = pi_h
            vf_latent = vf_h
            value_fn = linear(vf_latent, 'vf', 1)

            self._proba_distribution, self._policy, self.q_value = \
                self.pdtype.proba_distribution_from_latent(pi_latent, vf_latent, init_scale=0.01)

        self._value_fn = value_fn
        self._setup_init()

    def step(self, obs, state=None, mask=None, deterministic=False):
        if deterministic:
            action, value, neglogp = self.sess.run([self.deterministic_action, self.value_flat, self.neglogp],
                                                   {self.obs_ph: obs})
        else:
            action, value, neglogp = self.sess.run([self.action, self.value_flat, self.neglogp],
                                                   {self.obs_ph: obs})
        return action, value, self.initial_state, neglogp

    def proba_step(self, obs, state=None, mask=None):
        return self.sess.run(self.policy_proba, {self.obs_ph: obs})

    def value(self, obs, state=None, mask=None):
        return self.sess.run(self.value_flat, {self.obs_ph: obs})

class MLPPolicyGT(ActorCriticPolicy):
    def __init__(self, sess, ob_space, ac_space, n_env, n_steps, n_batch, reuse=False, **kwargs):
        super(MLPPolicyGT, self).__init__(sess, ob_space, ac_space, n_env, n_steps, n_batch, reuse=reuse, scale=False)

        with tf.variable_scope("model", reuse=reuse):
            act_fun = tf.nn.relu
            policy_obs, gt_obs = tf.split(self.processed_obs, [12,54],1)
            pi_h = policy_obs
            #Joints Network
            for i, layer_size in enumerate([256,256]):
                pi_h = act_fun(linear(pi_h, "pi_j_fc{}".format(i), layer_size, init_scale=np.sqrt(2)))
            pi_latent = pi_h

            vf_h = gt_obs
            for i, layer_size in enumerate([256,256]):
                vf_h = act_fun(linear(vf_h, "vf_j_fc{}".format(i), layer_size, init_scale=np.sqrt(2)))
            vf_latent = vf_h

            value_fn = linear(vf_latent, 'vf', 1)

            self._proba_distribution, self._policy, self.q_value = \
                self.pdtype.proba_distribution_from_latent(pi_latent, vf_latent, init_scale=0.01)

        self._value_fn = value_fn
        self._setup_init()

    def step(self, obs, state=None, mask=None, deterministic=False):
        if deterministic:
            action, value, neglogp = self.sess.run([self.deterministic_action, self.value_flat, self.neglogp],
                                                   {self.obs_ph: obs})
        else:
            action, value, neglogp = self.sess.run([self.action, self.value_flat, self.neglogp],
                                                   {self.obs_ph: obs})
        return action, value, self.initial_state, neglogp

    def proba_step(self, obs, state=None, mask=None):
        return self.sess.run(self.policy_proba, {self.obs_ph: obs})

    def value(self, obs, state=None, mask=None):
        return self.sess.run(self.value_flat, {self.obs_ph: obs})


class MLPPolicySingleAgent(ActorCriticPolicy):
    def __init__(self, sess, ob_space, ac_space, n_env, n_steps, n_batch, reuse=False, **kwargs):
        super(MLPPolicySingleAgent, self).__init__(sess, ob_space, ac_space, n_env, n_steps, n_batch, reuse=reuse, scale=False)

        with tf.variable_scope("model", reuse=reuse):
            act_fun = tf.nn.relu
            policy_obs, gt_obs = tf.split(self.processed_obs, [6,51],1)
            pi_h = policy_obs
            #Joints Network
            for i, layer_size in enumerate([256,256]):
                pi_h = act_fun(linear(pi_h, "pi_j_fc{}".format(i), layer_size, init_scale=np.sqrt(2)))
            pi_latent = pi_h

            vf_h = gt_obs
            for i, layer_size in enumerate([256,256]):
                vf_h = act_fun(linear(vf_h, "vf_j_fc{}".format(i), layer_size, init_scale=np.sqrt(2)))
            vf_latent = vf_h

            value_fn = linear(vf_latent, 'vf', 1)

            self._proba_distribution, self._policy, self.q_value = \
                self.pdtype.proba_distribution_from_latent(pi_latent, vf_latent, init_scale=0.01)

        self._value_fn = value_fn
        self._setup_init()

    def step(self, obs, state=None, mask=None, deterministic=False):
        if deterministic:
            action, value, neglogp = self.sess.run([self.deterministic_action, self.value_flat, self.neglogp],
                                                   {self.obs_ph: obs})
        else:
            action, value, neglogp = self.sess.run([self.action, self.value_flat, self.neglogp],
                                                   {self.obs_ph: obs})
        return action, value, self.initial_state, neglogp

    def proba_step(self, obs, state=None, mask=None):
        return self.sess.run(self.policy_proba, {self.obs_ph: obs})

    def value(self, obs, state=None, mask=None):
        return self.sess.run(self.value_flat, {self.obs_ph: obs})


class MLPPolicySingleAgentMitYawRate(ActorCriticPolicy):
    def __init__(self, sess, ob_space, ac_space, n_env, n_steps, n_batch, reuse=False, **kwargs):
        super(MLPPolicySingleAgentMitYawRate, self).__init__(sess, ob_space, ac_space, n_env, n_steps, n_batch, reuse=reuse, scale=False)

        with tf.variable_scope("model", reuse=reuse):
            act_fun = tf.nn.relu
            policy_obs, gt_obs = tf.split(self.processed_obs, [8,53],1)
            pi_h = policy_obs
            #Joints Network
            for i, layer_size in enumerate([256,256]):
                pi_h = act_fun(linear(pi_h, "pi_j_fc{}".format(i), layer_size, init_scale=np.sqrt(2)))
            pi_latent = pi_h

            vf_h = gt_obs
            for i, layer_size in enumerate([256,256]):
                vf_h = act_fun(linear(vf_h, "vf_j_fc{}".format(i), layer_size, init_scale=np.sqrt(2)))
            vf_latent = vf_h

            value_fn = linear(vf_latent, 'vf', 1)

            self._proba_distribution, self._policy, self.q_value = \
                self.pdtype.proba_distribution_from_latent(pi_latent, vf_latent, init_scale=0.01)

        self._value_fn = value_fn
        self._setup_init()

    def step(self, obs, state=None, mask=None, deterministic=False):
        if deterministic:
            action, value, neglogp = self.sess.run([self.deterministic_action, self.value_flat, self.neglogp],
                                                   {self.obs_ph: obs})
        else:
            action, value, neglogp = self.sess.run([self.action, self.value_flat, self.neglogp],
                                                   {self.obs_ph: obs})
        return action, value, self.initial_state, neglogp

    def proba_step(self, obs, state=None, mask=None):
        return self.sess.run(self.policy_proba, {self.obs_ph: obs})

    def value(self, obs, state=None, mask=None):
        return self.sess.run(self.value_flat, {self.obs_ph: obs})

class MLPPolicySingleAgentMitYawRateHMR(ActorCriticPolicy):
    def __init__(self, sess, ob_space, ac_space, n_env, n_steps, n_batch, reuse=False, **kwargs):
        super(MLPPolicySingleAgentMitYawRateHMR, self).__init__(sess, ob_space, ac_space, n_env, n_steps, n_batch, reuse=reuse, scale=False)

        with tf.variable_scope("model", reuse=reuse):
            act_fun = tf.nn.relu
            policy_obs, gt_obs = tf.split(self.processed_obs, [10,55],1)
            pi_h = policy_obs
            #Joints Network
            for i, layer_size in enumerate([256,256]):
                pi_h = act_fun(linear(pi_h, "pi_j_fc{}".format(i), layer_size, init_scale=np.sqrt(2)))
            pi_latent = pi_h

            vf_h = gt_obs
            for i, layer_size in enumerate([256,256]):
                vf_h = act_fun(linear(vf_h, "vf_j_fc{}".format(i), layer_size, init_scale=np.sqrt(2)))
            vf_latent = vf_h

            value_fn = linear(vf_latent, 'vf', 1)

            self._proba_distribution, self._policy, self.q_value = \
                self.pdtype.proba_distribution_from_latent(pi_latent, vf_latent, init_scale=0.01)

        self._value_fn = value_fn
        self._setup_init()

    def step(self, obs, state=None, mask=None, deterministic=False):
        if deterministic:
            action, value, neglogp = self.sess.run([self.deterministic_action, self.value_flat, self.neglogp],
                                                   {self.obs_ph: obs})
        else:
            action, value, neglogp = self.sess.run([self.action, self.value_flat, self.neglogp],
                                                   {self.obs_ph: obs})
        return action, value, self.initial_state, neglogp

    def proba_step(self, obs, state=None, mask=None):
        return self.sess.run(self.policy_proba, {self.obs_ph: obs})

    def value(self, obs, state=None, mask=None):
        return self.sess.run(self.value_flat, {self.obs_ph: obs})



def callback(_locals, _globals):
  """
  Callback called at each step (for DQN an others) or after n steps (see ACER or PPO2)
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
    now = datetime.datetime.now()
    if len(sys.argv) < 2:
        log_dir = LOGDIR+ALG+str(now)+"/snapshots/"
    else:
        log_dir=sys.argv[2]+"/snapshots/"

    print("Saving Models in: "+log_dir)
    os.makedirs(log_dir, exist_ok=True)


    def create_parallel_envs(env_id,robID):
        eid = env_id+1
        robID = robID+1
        print(str(eid)+str(robID))
        env = StartOpenAI_ROS_Environment_Parallel(task_and_robot_environment_name, eid, robID, num_robots)
        env = Monitor(env, log_dir+'/monitor'+str(eid)+str(robID)+'.csv', allow_early_resets=True)
        return env


    #env = SubprocVecEnv([lambda worker_id=k: create_parallel_envs(k) for k in range(num_envs)])
    envs = []
    def ret_lambda_func(k,l):
        return lambda : create_parallel_envs(k,l)

    for k in range(num_envs):
        for l in range(num_robots):
            envs.append(ret_lambda_func(k,l))

    env = SubprocVecEnv(envs, start_method='forkserver')
    # Create the Gym environment
    print("Gym environment done")

    #policy_kwargs = {n_steps:128}
    # TRAINING
    # model = PPO2(MlpPolicy, env, PPO2verbose=1,tensorboard_log=log_dir, nminibatches=1, n_steps=300)
    # model = PPO2(MlpLnLstmPolicy, env, verbose=1,tensorboard_log=log_dir,nminibatches=num_envs)
    model = PPO2(MLPPolicySingleAgentMitYawRateHMR, env, verbose=1,tensorboard_log=log_dir)

    '''
    80k iterations model with 256x256 and unobserved rewards
    '''
    # model = PPO2.load("/home/rtallamraju/drl_ws/logs/ppo2_mlp_aamas2019-09-13 01:03:23.109209/snapshots/best_model.pkl",env=env, tensorboard_log= LOGDIR+ALG+"/tboard_logs/")
    '''
    1M iterations model with 256x256 and unobserved rewards
    '''
    # model = PPO2.load("/home/rtallamraju/drl_ws/logs/ppo2_mlp_aamas2019-09-14 00:17:24.857364/snapshots/best_model.pkl",env=env, tensorboard_log= LOGDIR+ALG+"/tboard_logs/")
    '''
    500k iterations model with 256x256 and unobserved rewards-- velocity matching first attempt
    '''
    # model = PPO2.load("/home/rtallamraju/drl_ws/logs/ppo2_mlp_aamas2019-09-16 16:44:39.081008/snapshots/best_model.pkl",env=env, tensorboard_log= LOGDIR+ALG+"/tboard_logs/")
    '''
    400k iterations model with 256x256 and unobserved rewards-- velocity matching and height ratio corrected
    '''
    # model = PPO2.load("/home/rtallamraju/drl_ws/logs/ppo2_mlp_aamas2019-09-17 19:09:00.941485/snapshots/best_model.pkl",env=env, tensorboard_log= LOGDIR+ALG+"/tboard_logs/")
    '''
    400k iterations model with 256x256 and unobserved rewards-- straight line actor motion
    '''
    # model = PPO2.load("/home/rtallamraju/drl_ws/logs/ppo2_mlp_aamas2019-09-18 16:40:06.344479/snapshots/best_model.pkl",env=env, tensorboard_log= LOGDIR+ALG+"/tboard_logs/")

    '''
    1M iterations model with 256x256 and unobserved rewards-- straight line actor motion+velocity matching
    '''
    #model = PPO2.load("/home/rtallamraju/drl_ws/logs/ppo2_mlp_aamasaction, _states = model.predict(obs)2019-09-19 13:04:15.100493/snapshots/best_model.pkl",env=env, tensorboard_log= LOGDIR+ALG+"/tboard_logs/")

    '''
    400k iterations model with 256x256 and unobserved rewards-- straight line actor motion+velocity matching
    '''
    # model = PPO2.load("/home/rtallamraju/drl_ws/logs/ppo2_mlp_aamas2019-09-22 22:00:34.063792/snapshots/best_model.pkl",env=env, tensorboard_log= LOGDIR+ALG+"/tboard_logs/")


    '''
    1 M iterations model with 256x256 and unobserved rewards-- straight line actor motion+velocity matching
    '''
    # model = PPO2.load("/home/rtallamraju/drl_ws/logs/parallel_ppo_wtf/snapshots/trained_model.pkl",env=env, tensorboard_log= LOGDIR+ALG+"/tboard_logs/")

    '''
    1 M + 1 M iterations model with 256x256 and unobserved rewards-- straight line actor motion+velocity matching
    '''
    #model = PPO2.load("/home/rtallamraju/drl_ws/logs/parallel_ppo_cartesian_att2/snapshots/trained_model.pkl",env=env, tensorboard_log= LOGDIR+ALG+"/tboard_logs/")

    '''
    1 M + 1 M+ 1M iterations model with 256x256 and unobserved rewards-- straight line actor motion+velocity matching
    '''
    # model = PPO2.load("/home/rtallamraju/drl_ws/logs/parallel_ppo_cartesian_att3_try6/snapshots/trained_model.pkl",env=env, tensorboard_log= LOGDIR+ALG+"/tboard_logs/")

    '''
    (do not use. MAV does not move) 1 M + 1 M + 1 M + 1 M iterations model with 256x256 and unobserved rewards-- straight line actor motion+velocity matching
    '''

    # model = PPO2.load("/home/rtallamraju/drl_ws/logs/parallel_ppo_cartesian_att4/snapshots/trained_model.pkl",env=env, tensorboard_log= LOGDIR+ALG+"/tboard_logs/")

    '''
    1 M iterations model with 256x256 and action concat --maximizing joint visibility
    '''
    # model = PPO2.load("/home/rtallamraju/drl_ws/logs/drl_action_concat_2/snapshots/trained_model.pkl",env=env, tensorboard_log= LOGDIR+ALG+"/tboard_logs/")

    '''
    1 M + 1M iterations model with 256x256 and action concat --maximizing joint visibility
    '''

    # model = PPO2.load("/home/rtallamraju/drl_ws/logs/drl_action_concat_3/snapshots/trained_model.pkl",env=env, tensorboard_log= LOGDIR+ALG+"/tboard_logs/")

    ''' 1M Multi-Agent -- works horribly (MAV flies away) do not use'''
    # model = PPO2.load("/home/rtallamraju/drl_ws/logs/drl_multiagent_try9/snapshots/trained_model.pkl",env=env, tensorboard_log= LOGDIR+ALG+"/tboard_logs/")

    ''' 1M Multi-Agent with inter-agent distance input : change formation parameterization (and network observation) to only inter-agent distance'''
    # model = PPO2.load("/home/rtallamraju/drl_ws/logs/drl_multiagent_try10/snapshots/trained_model.pkl",env=env, tensorboard_log= LOGDIR+ALG+"/tboard_logs/")

    ''' 1M+800k Multi-Agent with inter-agent distance input : change formation parameterization (and network observation) to only inter-agent distance '''
    # model = PPO2.load("/home/rtallamraju/drl_ws/logs/drl_multiagent_try11/snapshots/best_model.pkl",env=env, tensorboard_log= LOGDIR+ALG+"/tboard_logs/")

    ''' 1M Multi-Agent with  formation parameterization  '''
    # model = PPO2.load("/home/rtallamraju/drl_ws/logs/drl_multiagent_try12/snapshots/trained_model.pkl",env=env, tensorboard_log= LOGDIR+ALG+"/tboard_logs/")

    ''' 1M+1M Multi-Agent with  formation parameterization  '''
    # model = PPO2.load("/home/rtallamraju/drl_ws/logs/drl_multiagent_try13/snapshots/trained_model.pkl",env=env, tensorboard_log= LOGDIR+ALG+"/tboard_logs/")

    ''' 1M+1M+5M Multi-Agent with  formation parameterization  '''
    # model = PPO2.load("/home/rtallamraju/drl_ws/logs/drl_multiagent_try14/snapshots/best_model.pkl",env=env, tensorboard_log= LOGDIR+ALG+"/tboard_logs/")

    ''' 3-D estimate of only root joint, Computer shutdown. Starting training from saved model '''
    # model = PPO2.load("/home/rtallamraju/drl_ws/logs/drl_multiagent_try15/snapshots/checkpoint_model170.pkl",env=env, tensorboard_log= LOGDIR+ALG+"/tboard_logs/")

    '''2.3 M trained on 3-D estimate of only root joint '''
    # model = PPO2.load("/home/rtallamraju/drl_ws/logs/drl_multiagent_try16/snapshots/best_model.pkl",env=env, tensorboard_log= LOGDIR+ALG+"/tboard_logs/")

    '''1.2 M single agent following'''
    # model = PPO2.load("/home/rtallamraju/drl_ws/logs/drl_singleagent_try4/snapshots/best_model.pkl",env=env, tensorboard_log= LOGDIR+ALG+"/tboard_logs/")

    '''4 M single agent following'''
    #model = PPO2.load("/home/rtallamraju/drl_ws/logs/drl_singleagent_try5/snapshots/best_model.pkl",env=env, tensorboard_log= LOGDIR+ALG+"/tboard_logs/")

    ''' 1M single agent following'''
    # model = PPO2.load("/home/rtallamraju/drl_ws/logs/drl_singleagent_try8/snapshots/checkpoint_model990.pkl",env=env, tensorboard_log= LOGDIR+ALG+"/tboard_logs/")

    ''' 5 M single agent following'''
    # model = PPO2.load("/home/rtallamraju/drl_ws/logs/drl_singleagent_try9/snapshots/best_model.pkl",env=env, tensorboard_log= LOGDIR+ALG+"/tboard_logs/")


    ''' 3 M multi-agent centralized training: only 3d estimate reward and collision penalty'''
    # model = PPO2.load("/home/rtallamraju/drl_ws/logs/drl_multiagent_1/snapshots/best_model.pkl",env=env, tensorboard_log= LOGDIR+ALG+"/tboard_logs/")

    ''' 3 M multi-agent centralized training: only 3d estimate reward, collision penalty and w-space violation'''
    # model = PPO2.load("/home/rtallamraju/drl_ws/logs/drl_multiagent_2/snapshots/best_model.pkl",env=env, tensorboard_log= LOGDIR+ALG+"/tboard_logs/")

    ''' 3 M MPC+  multi-agent centralized training with l-hip and r-hip observations: only 3d estimate reward, collision penalty and w-space violation'''
    # model = PPO2.load("/home/rtallamraju/drl_ws/logs/drl_multiagent_3/snapshots/best_model.pkl",env=env, tensorboard_log= LOGDIR+ALG+"/tboard_logs/")

    ''' 3M MPC+LSTM'''
    # model = PPO2.load("/home/rtallamraju/drl_ws/logs/drl_ma_fullbody3/snapshots/best_model.pkl",env=env, tensorboard_log= LOGDIR+ALG+"/tboard_logs/")

    ''' 2.5 M MPC+Multi-Agent tanh'''
    # model = PPO2.load("/home/rtallamraju/drl_ws/logs/drl_ma_fullbody4/snapshots/checkpoint_model1840.pkl",env=env, tensorboard_log= LOGDIR+ALG+"/tboard_logs/")

    '''1 M MPC+Multi-Agent tanh with triangulation network'''
    # model = PPO2.load("/home/rtallamraju/drl_ws/logs/drl_ma_fullbody_correct5/snapshots/trained_model.pkl",env=env, tensorboard_log= LOGDIR+ALG+"/tboard_logs/")

    '''1 M MPC+Multi-Agent tanh with multi-layered triangulation network'''
    #model = PPO2.load("/home/rtallamraju/drl_ws/logs/drl_ma_fullbody_correct6/snapshots/trained_model.pkl",env=env, tensorboard_log= LOGDIR+ALG+"/tboard_logs/")
    
    ''' 3 M GT Observations Single Agent Following: Env to use DeepFollow-v8. Robot Environment:firefly_singleagentcartesian_env.py '''
    #model = PPO2.load("/home/aamir/ros_logs/drl_sa_gt1/snapshots/trained_model.zip",env=env, tensorboard_log= LOGDIR+ALG+"/tboard_logs/")

    ''' 3 M GT Observations Single Agent Following with yawrate: Env to use DeepFollow-v9. Robot Environment:firefly_singleagentcartesian_env.py. Add orientation of person to actor and critic observation vectors '''
    #model = PPO2.load("/home/aamir/ros_logs/drl_sa_gt2/snapshots/trained_model.zip",env=env, tensorboard_log= LOGDIR+ALG+"/tboard_logs/")
    
    
    ''' 3 M GT Observations Single Agent Following with yawrate: Env to use DeepFollow-v9. Robot Environment:firefly_singleagentcartesian_env.py.  '''
    #model = PPO2.load("/home/aamir/ros_logs/drl_sa_gt3/snapshots/trained_model.zip",env=env, tensorboard_log= LOGDIR+ALG+"/tboard_logs/")
    '''Random Actor Velocity. Same Model as Above: Only with Centering Rewards. No visibility reward. Wrong Bearing'''
    #model = PPO2.load("/home/aamir/ros_logs/drl_sa_gt4/snapshots/best_model.pkl",env=env, tensorboard_log= LOGDIR+ALG+"/tboard_logs/")
    
    ''' 
    Test 1 : States in World Frame, Bearing Corrected, 4M iterations trained for DeepFollow-v9: Only Centering Reward
    Test 2 : States in Ego Frame, Bearing Corrected, 4M iterations trained for DeepFollow-v10: Only Centering Reward
    Test 2-Alpha : States in Ego Frame, Bearing Corrected, 4M iterations trained for DeepFollow-v10: Only Centering Reward using alphapose. Reward: self.is_image_centered_alpha
    Test 3 : States in World Frame, Bearing Corrected, 4M iterations trained for DeepFollow-v9: Only Visibility Reward
    Test 4 : States in Ego Frame, Bearing Corrected, 4M iterations trained for DeepFollow-v10: Only Visibility Reward
    Test 5 : States in World Frame, Bearing Corrected, 4M iterations trained for DeepFollow-v9: Centering and Visibility Reward
    Test 6 : States in Ego Frame, Bearing Corrected, 4M iterations trained for DeepFollow-v10: Centering and Visibility Reward    

    Is Ego or world better?
    
    Test 7 : States in Ego/World Frame, Bearing Corrected, 4M iterations trained for DeepFollow-v9: Only 3D HMR error Reward
    Test 8 : States in Ego/World Frame, Bearing Corrected, 4M iterations trained for DeepFollow-v9: Only Centering + 3D HMR error Reward
    Test 9 : States in Ego/World Frame, Bearing Corrected, 4M iterations trained for DeepFollow-v9: Only Visibility + 3D HMR errorReward
    Test 10 : States in Ego/World Frame, Bearing Corrected, 4M iterations trained for DeepFollow-v9: Centering + Visibility + 3D HMR error Reward
    
    '''
    
    '''Test 1 : ** Bearing Corrected 4M iterations trained for DeepFollow-v9: Only Centering Reward'''
    #model = PPO2.load("/home/aamir/ros_logs/drl_sa_gt5/snapshots/trained_model.zip",env=env, tensorboard_log= LOGDIR+ALG+"/tboard_logs/")
    
    '''Test2-P1: Training Crashed. Continue from latest checkpoint'''
    #model = PPO2.load("/home/aamir/ros_logs/drl_sa_gt6/snapshots/checkpoint_model750.pkl",env=env, tensorboard_log= LOGDIR+ALG+"/tboard_logs/")
    '''Test2-P2: ACtor Plugin Issue. Actor walking backwards. Continue from latest checkpoint'''
    #model = PPO2.load("/home/aamir/ros_logs/drl_sa_gt6_2/snapshots/checkpoint_model2320.pkl",env=env, tensorboard_log= LOGDIR+ALG+"/tboard_logs/")
    '''Test2-P3: States in Ego Frame, Bearing Corrected, 4M iterations trained for DeepFollow-v10: Only Centering Reward'''
    #model = PPO2.load("/home/aamir/ros_logs/drl_sa_gt6_2/snapshots/best_model.pkl",env=env, tensorboard_log= LOGDIR+ALG+"/tboard_logs/")    
    '''Test3 : Network -  MLPPolicySingleAgentMitYawRate 3 M iterations trained on DeepFollow-v10. Only Centering reward:self.is_image_centered_alpha'''
    #model = PPO2.load("/home/aamir/ros_logs/drl_sa_gt8/snapshots/best_model.pkl",env=env, tensorboard_log= LOGDIR+ALG+"/tboard_logs/") 
'''Test4 : Network -  MLPPolicySingleAgentMitYawRateHMR 4 M iterations trained on DeepFollow-v11. Only HMR reward:self.hmr_error'''
    #model = PPO2.load("/home/aamir/ros_logs/drl_sa_gt9/snapshots/best_model.pkl",env=env, tensorboard_log= LOGDIR+ALG+"/tboard_logs/") 
'''Test4 : Network -  MLPPolicySingleAgentMitYawRateHMR 4 M iterations trained on DeepFollow-v11. Only Weighted HMR reward:self.hmr_error'''
    #model = PPO2.load("/home/aamir/ros_logs/drl_sa_gt10/snapshots/best_model.pkl",env=env, tensorboard_log= LOGDIR+ALG+"/tboard_logs/") 
        
    rospy.logerr("PREDICTION_MODE")
    print(prediction_only)

    if prediction_only=='True':
        rospy.logerr("TESTING THE LEARNED MODEL")

        ##input("Press Enter to continue...")
        obs = env.reset()
        steps_elapsed = 0
        while True:
            action, _states = model.predict(observation=obs, deterministic = True)
            # rospy.logwarn("PREDICTED ACTION ==>"+str(action))
            obs, rewards, dones, info = env.step(action)
            steps_elapsed+=1
            if steps_elapsed>1000000:
                steps_elapsed = 0
                obs = env.reset()
    else:
        rospy.logerr("LEARNING A MODEL")
        model.learn(total_timesteps=4000000,tb_log_name = log_dir,callback = callback)
        model.save(log_dir+"/trained_model")
        rospy.logwarn("DONE TRAINING")
