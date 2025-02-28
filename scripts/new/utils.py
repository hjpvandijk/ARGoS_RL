# Copyright (c) 2017 OpenAI (http://openai.com)
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

import os
import gym
import numpy as np
import tensorflow as tf
from gym import spaces
from collections import deque

def sample(logits):
    noise = tf.compat.v1.random_uniform(tf.compat.v1.shape(logits))
    return tf.compat.v1.argmax(logits - tf.compat.v1.log(-tf.compat.v1.log(noise)), 1)

def cat_entropy(logits):
    a0 = logits - tf.compat.v1.reduce_max(logits, 1, keep_dims=True)
    ea0 = tf.compat.v1.exp(a0)
    z0 = tf.compat.v1.reduce_sum(ea0, 1, keep_dims=True)
    p0 = ea0 / z0
    return tf.compat.v1.reduce_sum(p0 * (tf.compat.v1.log(z0) - a0), 1)

def cat_entropy_softmax(p0):
    return - tf.compat.v1.reduce_sum(p0 * tf.compat.v1.log(p0 + 1e-6), axis = 1)

def mse(pred, target):
    return tf.compat.v1.square(pred-target)/2.

def ortho_init(scale=1.0):
    def _ortho_init(shape, dtype, partition_info=None):
        #lasagne ortho init for tf
        shape = tuple(shape)
        if len(shape) == 2:
            flat_shape = shape
        elif len(shape) == 4: # assumes NHWC
            flat_shape = (np.prod(shape[:-1]), shape[-1])
        else:
            raise NotImplementedError
        a = np.random.normal(0.0, 1.0, flat_shape)
        u, _, v = np.linalg.svd(a, full_matrices=False)
        q = u if u.shape == flat_shape else v # pick the one with the correct shape
        q = q.reshape(shape)
        return (scale * q[:shape[0], :shape[1]]).astype(np.float32)
    return _ortho_init

def conv(x, scope, nf, rf, stride, pad='VALID', init_scale=1.0, data_format='NHWC', one_dim_bias=False):
    if data_format == 'NHWC':
        channel_ax = 3
        strides = [1, stride, stride, 1]
        bshape = [1, 1, 1, nf]
    elif data_format == 'NCHW':
        channel_ax = 1
        strides = [1, 1, stride, stride]
        bshape = [1, nf, 1, 1]
    else:
        raise NotImplementedError
    bias_var_shape = [nf] if one_dim_bias else [1, nf, 1, 1]
    nin = x.get_shape()[channel_ax].value
    wshape = [rf, rf, nin, nf]
    with tf.compat.v1.variable_scope(scope):
        w = tf.compat.v1.get_variable("w", wshape, initializer=ortho_init(init_scale))
        b = tf.compat.v1.get_variable("b", bias_var_shape, initializer=tf.compat.v1.constant_initializer(0.0))
        if not one_dim_bias and data_format == 'NHWC':
            b = tf.compat.v1.reshape(b, bshape)
        return b + tf.compat.v1.nn.conv2d(x, w, strides=strides, padding=pad, data_format=data_format)

def fc(x, scope, nh, init_scale=1.0, init_bias=0.0):
    with tf.compat.v1.variable_scope(scope):
        nin = x.get_shape()[1]#.value
        w = tf.compat.v1.get_variable("w", [nin, nh], initializer=ortho_init(init_scale))
        b = tf.compat.v1.get_variable("b", [nh], initializer=tf.compat.v1.constant_initializer(init_bias))
        return tf.compat.v1.matmul(x, w)+b

def batch_to_seq(h, nbatch, nsteps, flat=False):
    if flat:
        h = tf.compat.v1.reshape(h, [nbatch, nsteps])
    else:
        h = tf.compat.v1.reshape(h, [nbatch, nsteps, -1])
    return [tf.compat.v1.squeeze(v, [1]) for v in tf.compat.v1.split(axis=1, num_or_size_splits=nsteps, value=h)]

def seq_to_batch(h, flat = False):
    shape = h[0].get_shape().as_list()
    if not flat:
        assert(len(shape) > 1)
        nh = h[0].get_shape()[-1].value
        return tf.compat.v1.reshape(tf.compat.v1.concat(axis=1, values=h), [-1, nh])
    else:
        return tf.compat.v1.reshape(tf.compat.v1.stack(values=h, axis=1), [-1])

def lstm(xs, ms, s, scope, nh, init_scale=1.0):
    nbatch, nin = [v.value for v in xs[0].get_shape()]
    nsteps = len(xs)
    with tf.compat.v1.variable_scope(scope):
        wx = tf.compat.v1.get_variable("wx", [nin, nh*4], initializer=ortho_init(init_scale))
        wh = tf.compat.v1.get_variable("wh", [nh, nh*4], initializer=ortho_init(init_scale))
        b = tf.compat.v1.get_variable("b", [nh*4], initializer=tf.compat.v1.constant_initializer(0.0))

    c, h = tf.compat.v1.split(axis=1, num_or_size_splits=2, value=s)
    for idx, (x, m) in enumerate(zip(xs, ms)):
        c = c*(1-m)
        h = h*(1-m)
        z = tf.compat.v1.matmul(x, wx) + tf.compat.v1.matmul(h, wh) + b
        i, f, o, u = tf.compat.v1.split(axis=1, num_or_size_splits=4, value=z)
        i = tf.compat.v1.nn.sigmoid(i)
        f = tf.compat.v1.nn.sigmoid(f)
        o = tf.compat.v1.nn.sigmoid(o)
        u = tf.compat.v1.tanh(u)
        c = f*c + i*u
        h = o*tf.compat.v1.tanh(c)
        xs[idx] = h
    s = tf.compat.v1.concat(axis=1, values=[c, h])
    return xs, s

def _ln(x, g, b, e=1e-5, axes=[1]):
    u, s = tf.compat.v1.nn.moments(x, axes=axes, keep_dims=True)
    x = (x-u)/tf.compat.v1.sqrt(s+e)
    x = x*g+b
    return x

def lnlstm(xs, ms, s, scope, nh, init_scale=1.0):
    nbatch, nin = [v.value for v in xs[0].get_shape()]
    nsteps = len(xs)
    with tf.compat.v1.variable_scope(scope):
        wx = tf.compat.v1.get_variable("wx", [nin, nh*4], initializer=ortho_init(init_scale))
        gx = tf.compat.v1.get_variable("gx", [nh*4], initializer=tf.compat.v1.constant_initializer(1.0))
        bx = tf.compat.v1.get_variable("bx", [nh*4], initializer=tf.compat.v1.constant_initializer(0.0))

        wh = tf.compat.v1.get_variable("wh", [nh, nh*4], initializer=ortho_init(init_scale))
        gh = tf.compat.v1.get_variable("gh", [nh*4], initializer=tf.compat.v1.constant_initializer(1.0))
        bh = tf.compat.v1.get_variable("bh", [nh*4], initializer=tf.compat.v1.constant_initializer(0.0))

        b = tf.compat.v1.get_variable("b", [nh*4], initializer=tf.compat.v1.constant_initializer(0.0))

        gc = tf.compat.v1.get_variable("gc", [nh], initializer=tf.compat.v1.constant_initializer(1.0))
        bc = tf.compat.v1.get_variable("bc", [nh], initializer=tf.compat.v1.constant_initializer(0.0))

    c, h = tf.compat.v1.split(axis=1, num_or_size_splits=2, value=s)
    for idx, (x, m) in enumerate(zip(xs, ms)):
        c = c*(1-m)
        h = h*(1-m)
        z = _ln(tf.compat.v1.matmul(x, wx), gx, bx) + _ln(tf.compat.v1.matmul(h, wh), gh, bh) + b
        i, f, o, u = tf.compat.v1.split(axis=1, num_or_size_splits=4, value=z)
        i = tf.compat.v1.nn.sigmoid(i)
        f = tf.compat.v1.nn.sigmoid(f)
        o = tf.compat.v1.nn.sigmoid(o)
        u = tf.compat.v1.tanh(u)
        c = f*c + i*u
        h = o*tf.compat.v1.tanh(_ln(c, gc, bc))
        xs[idx] = h
    s = tf.compat.v1.concat(axis=1, values=[c, h])
    return xs, s

def conv_to_fc(x):
    nh = np.prod([v.value for v in x.get_shape()[1:]])
    x = tf.compat.v1.reshape(x, [-1, nh])
    return x

def discount_with_dones(rewards, dones, gamma):
    discounted = []
    r = 0
    for reward, done in zip(rewards[::-1], dones[::-1]):
        r = reward + gamma*r*(1.-done) # fixed off by one bug
        discounted.append(r)
    return discounted[::-1]

def find_trainable_variables(key):
    with tf.compat.v1.variable_scope(key):
        return tf.compat.v1.trainable_variables()

def make_path(f):
    return os.makedirs(f, exist_ok=True)

def constant(p):
    return 1

def linear(p):
    return 1-p

def middle_drop(p):
    eps = 0.75
    if 1-p<eps:
        return eps*0.1
    return 1-p

def double_linear_con(p):
    p *= 2
    eps = 0.125
    if 1-p<eps:
        return eps
    return 1-p

def double_middle_drop(p):
    eps1 = 0.75
    eps2 = 0.25
    if 1-p<eps1:
        if 1-p<eps2:
            return eps2*0.5
        return eps1*0.1
    return 1-p

schedules = {
    'linear':linear,
    'constant':constant,
    'double_linear_con': double_linear_con,
    'middle_drop': middle_drop,
    'double_middle_drop': double_middle_drop
}

class Scheduler(object):

    def __init__(self, v, nvalues, schedule):
        self.n = 0.
        self.v = v
        self.nvalues = nvalues
        self.schedule = schedules[schedule]

    def value(self):
        current_value = self.v*self.schedule(self.n/self.nvalues)
        self.n += 1.
        return current_value

    def value_steps(self, steps):
        return self.v*self.schedule(steps/self.nvalues)


class EpisodeStats:
    def __init__(self, nsteps, nenvs):
        self.episode_rewards = []
        for i in range(nenvs):
            self.episode_rewards.append([])
        self.lenbuffer = deque(maxlen=40)  # rolling buffer for episode lengths
        self.rewbuffer = deque(maxlen=40)  # rolling buffer for episode rewards
        self.nsteps = nsteps
        self.nenvs = nenvs

    def feed(self, rewards, masks):
        rewards = np.reshape(rewards, [self.nenvs, self.nsteps])
        masks = np.reshape(masks, [self.nenvs, self.nsteps])
        for i in range(0, self.nenvs):
            for j in range(0, self.nsteps):
                self.episode_rewards[i].append(rewards[i][j])
                if masks[i][j]:
                    l = len(self.episode_rewards[i])
                    s = sum(self.episode_rewards[i])
                    self.lenbuffer.append(l)
                    self.rewbuffer.append(s)
                    self.episode_rewards[i] = []

    def mean_length(self):
        if self.lenbuffer:
            return np.mean(self.lenbuffer)
        else:
            return 0  # on the first params dump, no episodes are finished

    def mean_reward(self):
        if self.rewbuffer:
            return np.mean(self.rewbuffer)
        else:
            return 0


# For ACER
def get_by_index(x, idx):
    assert(len(x.get_shape()) == 2)
    assert(len(idx.get_shape()) == 1)
    idx_flattened = tf.compat.v1.range(0, x.shape[0]) * x.shape[1] + idx
    y = tf.compat.v1.gather(tf.compat.v1.reshape(x, [-1]),  # flatten input
                  idx_flattened)  # use flattened indices
    return y

def check_shape(ts,shapes):
    i = 0
    for (t,shape) in zip(ts,shapes):
        assert t.get_shape().as_list()==shape, "id " + str(i) + " shape " + str(t.get_shape()) + str(shape)
        i += 1

def avg_norm(t):
    return tf.compat.v1.reduce_mean(tf.compat.v1.sqrt(tf.compat.v1.reduce_sum(tf.compat.v1.square(t), axis=-1)))

def gradient_add(g1, g2, param):
    print([g1, g2, param.name])
    assert (not (g1 is None and g2 is None)), param.name
    if g1 is None:
        return g2
    elif g2 is None:
        return g1
    else:
        return g1 + g2

def q_explained_variance(qpred, q):
    _, vary = tf.compat.v1.nn.moments(q, axes=[0, 1])
    _, varpred = tf.compat.v1.nn.moments(q - qpred, axes=[0, 1])
    check_shape([vary, varpred], [[]] * 2)
    return 1.0 - (varpred / vary)
