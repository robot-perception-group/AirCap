from stable_baselines.common.policies import MlpPolicy, ActorCriticPolicy, MlpLnLstmPolicy, FeedForwardPolicy
import tensorflow as tf
import os
import numpy as np
from stable_baselines.a2c.utils import ortho_init
from stable_baselines.common.distributions import  DiagGaussianProbabilityDistributionType
from stable_baselines.common.base_class import  BaseRLModel
from collections import OrderedDict

os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'



def _setup_custom_load_operations(self):
    """
    Create tensorflow operations for loading model parameters
    """
    # Assume tensorflow graphs are static -> check
    # that we only call this function once
    if self._param_load_ops is not None:
        raise RuntimeError("Parameter load operations have already been created")
    # For each loadable parameter, create appropiate
    # placeholder and an assign op, and store them to
    # self.load_param_ops as dict of variable.name -> (placeholder, assign)
    loadable_parameters = self.all_params
    # Use OrderedDict to store order for backwards compatibility with
    # list-based params
    self._param_load_ops = OrderedDict()
    with self.graph.as_default():
        for param in loadable_parameters:
            placeholder = tf.placeholder(dtype=param.dtype, shape=param.shape)
            # param.name is unique (tensorflow variables have unique names)
            self._param_load_ops[param.name] = (placeholder, param.assign(placeholder))


def custom_load_parameters(self, load_path_or_dict, exact_match=True):
    # Make sure we have assign ops
    if self._param_load_ops is None:
        self._setup_custom_load_operations()

    if isinstance(load_path_or_dict, dict):
        # Assume `load_path_or_dict` is dict of variable.name -> ndarrays we want to load
        params = load_path_or_dict
    elif isinstance(load_path_or_dict, list):
        warnings.warn("Loading model parameters from a list. This has been replaced " +
                      "with parameter dictionaries with variable names and parameters. " +
                      "If you are loading from a file, consider re-saving the file.",
                      DeprecationWarning)
        # Assume `load_path_or_dict` is list of ndarrays.
        # Create param dictionary assuming the parameters are in same order
        # as `get_parameter_list` returns them.
        params = dict()
        for i, param_name in enumerate(self._param_load_ops.keys()):
            params[param_name] = load_path_or_dict[i]
    else:
        # Assume a filepath or file-like.
        # Use existing deserializer to load the parameters.
        # We only need the parameters part of the file, so
        # only load that part.
        _, params = BaseRLModel._load_from_file(load_path_or_dict, load_data=False)
        params = dict(params)

    feed_dict = {}
    param_update_ops = []
    # Keep track of not-updated variables
    not_updated_variables = set(self._param_load_ops.keys())
    for param_name, param_value in params.items():
        placeholder, assign_op = self._param_load_ops[param_name]
        feed_dict[placeholder] = param_value
        # Create list of tf.assign operations for sess.run
        param_update_ops.append(assign_op)
        # Keep track which variables are updated
        not_updated_variables.remove(param_name)

    # Check that we updated all parameters if exact_match=True
    if exact_match and len(not_updated_variables) > 0:
        raise RuntimeError("Load dictionary did not contain all variables. " +
                           "Missing variables: {}".format(", ".join(not_updated_variables)))

    self.sess.run(param_update_ops, feed_dict=feed_dict)

BaseRLModel.custom_load_parameters= custom_load_parameters
BaseRLModel._setup_custom_load_operations = _setup_custom_load_operations

def custom_load(self, load_path, sa_or_ma='sa', env=None, custom_objects=None, **kwargs):
    """
    Load the model from file

    :param load_path: (str or file-like) the saved parameter location
    :param env: (Gym Environment) the new environment to run the loaded model on
        (can be None if you only need prediction from a trained model)
    :param custom_objects: (dict) Dictionary of objects to replace
        upon loading. If a variable is present in this dictionary as a
        key, it will not be deserialized and the corresponding item
        will be used instead. Similar to custom_objects in
        `keras.models.load_model`. Useful when you have an object in
        file that can not be deserialized.
    :param kwargs: extra arguments to change the model when loading
    """
    data, params = self._load_from_file(load_path)
    new_params = self.get_parameters()

    custom_params = {}
    for name, val in params.items():
        if name[:len('model/pi_j_fc')]+'_'+sa_or_ma+name[len('model/pi_j_fc'):] in new_params:
            name=name[:len('model/pi_j_fc')]+'_'+sa_or_ma+name[len('model/pi_j_fc'):]
            custom_params[name]=val
        elif name[:len('model/pi')]+'_'+sa_or_ma+name[len('model/pi'):] in new_params:
            name=name[:len('model/pi')]+'_'+sa_or_ma+name[len('model/pi'):]
            custom_params[name]=val

    self.load_parameters(custom_params,exact_match=False)

    return self,params


def custom_load_value_and_policy(self, load_path, sa_or_ma='sa', env=None, custom_objects=None, **kwargs):
    """
    Load the model from file

    :param load_path: (str or file-like) the saved parameter location
    :param env: (Gym Environment) the new environment to run the loaded model on
        (can be None if you only need prediction from a trained model)
    :param custom_objects: (dict) Dictionary of objects to replace
        upon loading. If a variable is present in this dictionary as a
        key, it will not be deserialized and the corresponding item
        will be used instead. Similar to custom_objects in
        `keras.models.load_model`. Useful when you have an object in
        file that can not be deserialized.
    :param kwargs: extra arguments to change the model when loading
    """
    data, params = self._load_from_file(load_path)
    parameter_values = self.sess.run(self.all_params)
    new_params = OrderedDict((param.name, value) for param, value in zip(self.all_params, parameter_values))

    custom_params = {}
    for name, val in params.items():
        if name[:len('model/pi_j_fc')]+'_'+sa_or_ma+name[len('model/pi_j_fc'):] in new_params:
            name=name[:len('model/pi_j_fc')]+'_'+sa_or_ma+name[len('model/pi_j_fc'):]
            custom_params[name]=val
        elif name[:len('model/pi')]+'_'+sa_or_ma+name[len('model/pi'):] in new_params:
            name=name[:len('model/pi')]+'_'+sa_or_ma+name[len('model/pi'):]
            custom_params[name]=val

        elif name[:len('model/vf_j_fc')]+'_'+sa_or_ma+name[len('model/vf_j_fc'):] in new_params:
            name=name[:len('model/vf_j_fc')]+'_'+sa_or_ma+name[len('model/vf_j_fc'):]
            custom_params[name]=val
        elif name[:len('model/vf')]+'_'+sa_or_ma+name[len('model/vf'):] in new_params:
            name=name[:len('model/vf')]+'_'+sa_or_ma+name[len('model/vf'):]
            custom_params[name]=val

        elif name[:len('model/q')]+'_'+sa_or_ma+name[len('model/q'):] in new_params:
            name=name[:len('model/q')]+'_'+sa_or_ma+name[len('model/q'):]
            custom_params[name]=val

    self.custom_load_parameters(custom_params,exact_match=False)
    return self,params,custom_params


def custom_load_policy(self, load_path, sa_or_ma='sa', env=None, custom_objects=None, **kwargs):
    """
    Load the model from file

    :param load_path: (str or file-like) the saved parameter location
    :param env: (Gym Environment) the new environment to run the loaded model on
        (can be None if you only need prediction from a trained model)
    :param custom_objects: (dict) Dictionary of objects to replace
        upon loading. If a variable is present in this dictionary as a
        key, it will not be deserialized and the corresponding item
        will be used instead. Similar to custom_objects in
        `keras.models.load_model`. Useful when you have an object in
        file that can not be deserialized.
    :param kwargs: extra arguments to change the model when loading
    """
    data, params = self._load_from_file(load_path)
    parameter_values = self.sess.run(self.all_params)
    new_params = OrderedDict((param.name, value) for param, value in zip(self.all_params, parameter_values))

    custom_params = {}
    for name, val in params.items():
        if name[:len('model/pi_j_fc')]+'_'+sa_or_ma+name[len('model/pi_j_fc'):] in new_params:
            name=name[:len('model/pi_j_fc')]+'_'+sa_or_ma+name[len('model/pi_j_fc'):]
            custom_params[name]=val
        elif name[:len('model/pi')]+'_'+sa_or_ma+name[len('model/pi'):] in new_params:
            name=name[:len('model/pi')]+'_'+sa_or_ma+name[len('model/pi'):]
            custom_params[name]=val

    self.custom_load_parameters(custom_params,exact_match=False)
    return self,params,custom_params


"""
Custom function to sample from diagonal gaussian probability distribution.
The yawrate is directly used from the single agent policy network.
The translational velocities are combined using a fully connected network.
"""
def custom_proba_distribution_from_latent(self, pi_latent_vector, yawrate, vf_latent_vector, init_scale=1.0, init_bias=0.0):
    mean = pi_latent_vector#MetaPolicy.linear(pi_latent_vector, 'pi', self.size-1, init_scale=init_scale, init_bias=init_bias)
    mean = tf.concat([mean,yawrate],1)
    logstd = tf.get_variable(name='pi/logstd', shape=[1, self.size], initializer=tf.zeros_initializer())
    pdparam = tf.concat([mean, mean * 0.0 + logstd], axis=1)
    q_values = MetaPolicy.linear(vf_latent_vector, 'q', self.size, init_scale=init_scale, init_bias=init_bias)
    return self.proba_distribution_from_flat(pdparam), mean, q_values

def pretrained_proba_distribution_from_latent(self, pi_latent_vector, vf_latent_vector, q_values, init_scale=1.0, init_bias=0.0):
    mean = pi_latent_vector
    logstd = tf.get_variable(name='pi/logstd', shape=[1, self.size], initializer=tf.zeros_initializer())
    pdparam = tf.concat([mean, mean * 0.0 + logstd], axis=1)
    return self.proba_distribution_from_flat(pdparam), mean, q_values


#Add the custom method to the distribution class
DiagGaussianProbabilityDistributionType.custom_proba_distribution_from_latent = custom_proba_distribution_from_latent
DiagGaussianProbabilityDistributionType.pretrained_proba_distribution_from_latent = pretrained_proba_distribution_from_latent


class MetaPolicy(ActorCriticPolicy):
    def __init__(self, sess, ob_space, ac_space, n_env, n_steps, n_batch, reuse=False, **kwargs):
        super(MetaPolicy, self).__init__(sess, ob_space, ac_space, n_env, n_steps, n_batch, reuse=reuse, scale=False)

        with tf.variable_scope("model", reuse=reuse):
            act_fun = tf.nn.relu
            policy_obs,gt_obs = tf.split(self.processed_obs, [15,63],1)


            #Single Agent Network
            pi_h = policy_obs
            for i, layer_size in enumerate([256,256]):
                pi_h = act_fun(MetaPolicy.linear(pi_h, "pi_j_fc_sa{}".format(i), layer_size, init_scale=np.sqrt(2)))
            pi_latent = pi_h



            vf_h = gt_obs
            for i, layer_size in enumerate([256,256]):
                vf_h = act_fun(MetaPolicy.linear(vf_h, "vf_j_fc{}".format(i), layer_size, init_scale=np.sqrt(2)))
            vf_latent = vf_h

            value_fn = MetaPolicy.linear(vf_latent, 'vf', 1)

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

    @staticmethod
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



class MetaPolicyPretrained(ActorCriticPolicy):
    def __init__(self, sess, ob_space, ac_space, n_env, n_steps, n_batch, reuse=False, **kwargs):
        super(MetaPolicyPretrained, self).__init__(sess, ob_space, ac_space, n_env, n_steps, n_batch, reuse=reuse, scale=False)

        with tf.variable_scope("model", reuse=reuse):
            act_fun = tf.nn.relu
            policy_obs_sa, policy_obs_ma,gt_obs = tf.split(self.processed_obs, [8,7,63],1)

            ##############################################################################################################
            #################################### POLICY NETWORK ###########################################################
            ##############################################################################################################

            #Single Agent Network
            pi_h = policy_obs_sa
            for i, layer_size in enumerate([256,256]):
                pi_h = act_fun(MetaPolicy.linear(pi_h, "pi_j_fc_sa{}".format(i), layer_size, init_scale=np.sqrt(2),trainable=True))
            pi_latent_sa = MetaPolicy.linear(pi_h, 'pi_sa', 3, trainable=True)
            trans_sa,yawrate = tf.split(pi_latent_sa, [2,1],1)

            pi_h = policy_obs_ma
            for i, layer_size in enumerate([256,256]):
                pi_h = act_fun(MetaPolicy.linear(pi_h, "pi_j_fc_ma{}".format(i), layer_size, init_scale=np.sqrt(2),trainable=True))
            trans_ma = MetaPolicy.linear(pi_h, 'pi_ma', 2, trainable=True)

            #sum the actions
            pi_latent = tf.add(trans_sa,trans_ma)
            #concatenate the yawrate to the action sum
            # pi_latent = tf.concat([pi_latent,yawrate],1)

            ##############################################################################################################
            #################################### VALUE NETWORK ###########################################################
            ##############################################################################################################

            # vf_h = gt_obs_sa
            # for i, layer_size in enumerate([256,256]):
            #     vf_h = act_fun(MetaPolicy.linear(vf_h, "vf_j_fc_sa{}".format(i), layer_size, init_scale=np.sqrt(2),trainable=True))
            # vf_latent_sa = MetaPolicy.linear(vf_h, "vf_sa", 1,trainable=True)
            # q_sa = MetaPolicy.linear(vf_h, 'q_sa',3,trainable=True)
            # q_sa,q_y_sa = tf.split(q_sa, [2,1],1)
            #
            # vf_h = gt_obs_ma
            # for i, layer_size in enumerate([256,256]):
            #     vf_h = act_fun(MetaPolicy.linear(vf_h, "vf_j_fc_ma{}".format(i), layer_size, init_scale=np.sqrt(2),trainable=True))
            # vf_latent_ma = MetaPolicy.linear(vf_h, "vf_ma", 1,trainable=True)
            # q_ma = MetaPolicy.linear(vf_h, 'q_ma',2,trainable=True)
            #
            # vf_latent = tf.add(vf_latent_sa,vf_latent_ma)
            # q_values = tf.add(q_sa,q_ma)
            # q_values = tf.concat([q_values,q_y_sa],1)

            # self._proba_distribution, self._policy, self.q_value = \
            #     self.pdtype.pretrained_proba_distribution_from_latent(pi_latent, vf_latent, q_values, init_scale=0.01)

            vf_h = gt_obs
            for i, layer_size in enumerate([256,256]):
                vf_h = act_fun(MetaPolicy.linear(vf_h, "vf_j_fc{}".format(i), layer_size, init_scale=np.sqrt(2)))
            vf_latent = vf_h

            value_fn = MetaPolicy.linear(vf_latent, 'vf', 1)

            self._proba_distribution, self._policy, self.q_value = \
                self.pdtype.custom_proba_distribution_from_latent(pi_latent, yawrate, vf_latent, init_scale=0.01)

        # self._value_fn = vf_latent
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

    @staticmethod
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