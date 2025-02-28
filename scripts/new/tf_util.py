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

import numpy as np
import tensorflow as tf  # pylint: ignore-module
import copy
import os
import functools
import collections
import multiprocessing

def switch(condition, then_expression, else_expression):
    """Switches between two operations depending on a scalar value (int or bool).
    Note that both `then_expression` and `else_expression`
    should be symbolic tensors of the *same shape*.

    # Arguments
        condition: scalar tensor.
        then_expression: TensorFlow operation.
        else_expression: TensorFlow operation.
    """
    x_shape = copy.copy(then_expression.get_shape())
    x = tf.compat.v1.cond(tf.compat.v1.cast(condition, 'bool'),
                lambda: then_expression,
                lambda: else_expression)
    x.set_shape(x_shape)
    return x

# ================================================================
# Extras
# ================================================================

def lrelu(x, leak=0.2):
    f1 = 0.5 * (1 + leak)
    f2 = 0.5 * (1 - leak)
    return f1 * x + f2 * abs(x)

# ================================================================
# Mathematical utils
# ================================================================

def huber_loss(x, delta=1.0):
    """Reference: https://en.wikipedia.org/wiki/Huber_loss"""
    return tf.compat.v1.where(
        tf.compat.v1.abs(x) < delta,
        tf.compat.v1.square(x) * 0.5,
        delta * (tf.compat.v1.abs(x) - 0.5 * delta)
    )

# ================================================================
# Global session
# ================================================================

def make_session(num_cpu=None, make_default=False, graph=None):
    """Returns a session that will use <num_cpu> CPU's only"""
    if num_cpu is None:
        num_cpu = int(os.getenv('RCALL_NUM_CPU', multiprocessing.cpu_count()))
    tf_config = tf.compat.v1.ConfigProto(
        inter_op_parallelism_threads=num_cpu,
        intra_op_parallelism_threads=num_cpu)
    if make_default:
        return tf.compat.v1.InteractiveSession(config=tf_config, graph=graph)
    else:
        return tf.compat.v1.Session(config=tf_config, graph=graph)

def single_threaded_session():
    """Returns a session which will only use a single CPU"""
    return make_session(num_cpu=1)

def in_session(f):
    @functools.wraps(f)
    def newfunc(*args, **kwargs):
        with tf.compat.v1.Session():
            f(*args, **kwargs)
    return newfunc

ALREADY_INITIALIZED = set()

def initialize():
    """Initialize all the uninitialized variables in the global scope."""
    new_variables = set(tf.compat.v1.global_variables()) - ALREADY_INITIALIZED
    tf.compat.v1.get_default_session().run(tf.compat.v1.variables_initializer(new_variables))
    ALREADY_INITIALIZED.update(new_variables)

# ================================================================
# Model components
# ================================================================

def normc_initializer(std=1.0, axis=0):
    def _initializer(shape, dtype=None, partition_info=None):  # pylint: disable=W0613
        out = np.random.randn(*shape).astype(np.float32)
        out *= std / np.sqrt(np.square(out).sum(axis=axis, keepdims=True))
        return tf.compat.v1.constant(out)
    return _initializer

def conv2d(x, num_filters, name, filter_size=(3, 3), stride=(1, 1), pad="SAME", dtype=tf.compat.v1.float32, collections=None,
           summary_tag=None):
    with tf.compat.v1.variable_scope(name):
        stride_shape = [1, stride[0], stride[1], 1]
        filter_shape = [filter_size[0], filter_size[1], int(x.get_shape()[3]), num_filters]

        # there are "num input feature maps * filter height * filter width"
        # inputs to each hidden unit
        fan_in = intprod(filter_shape[:3])
        # each unit in the lower layer receives a gradient from:
        # "num output feature maps * filter height * filter width" /
        #   pooling size
        fan_out = intprod(filter_shape[:2]) * num_filters
        # initialize weights with random weights
        w_bound = np.sqrt(6. / (fan_in + fan_out))

        w = tf.compat.v1.get_variable("W", filter_shape, dtype, tf.compat.v1.random_uniform_initializer(-w_bound, w_bound),
                            collections=collections)
        b = tf.compat.v1.get_variable("b", [1, 1, 1, num_filters], initializer=tf.compat.v1.zeros_initializer(),
                            collections=collections)

        if summary_tag is not None:
            tf.compat.v1.summary.image(summary_tag,
                             tf.compat.v1.transpose(tf.compat.v1.reshape(w, [filter_size[0], filter_size[1], -1, 1]),
                                          [2, 0, 1, 3]),
                             max_images=10)

        return tf.compat.v1.nn.conv2d(x, w, stride_shape, pad) + b

# ================================================================
# Theano-like Function
# ================================================================

def function(inputs, outputs, updates=None, givens=None):
    """Just like Theano function. Take a bunch of tensorflow placeholders and expressions
    computed based on those placeholders and produces f(inputs) -> outputs. Function f takes
    values to be fed to the input's placeholders and produces the values of the expressions
    in outputs.

    Input values can be passed in the same order as inputs or can be provided as kwargs based
    on placeholder name (passed to constructor or accessible via placeholder.op.name).

    Example:
        x = tf.compat.v1.placeholder(tf.compat.v1.int32, (), name="x")
        y = tf.compat.v1.placeholder(tf.compat.v1.int32, (), name="y")
        z = 3 * x + 2 * y
        lin = function([x, y], z, givens={y: 0})

        with single_threaded_session():
            initialize()

            assert lin(2) == 6
            assert lin(x=3) == 9
            assert lin(2, 2) == 10
            assert lin(x=2, y=3) == 12

    Parameters
    ----------
    inputs: [tf.compat.v1.placeholder, tf.compat.v1.constant, or object with make_feed_dict method]
        list of input arguments
    outputs: [tf.compat.v1.Variable] or tf.compat.v1.Variable
        list of outputs or a single output to be returned from function. Returned
        value will also have the same shape.
    """
    if isinstance(outputs, list):
        return _Function(inputs, outputs, updates, givens=givens)
    elif isinstance(outputs, (dict, collections.OrderedDict)):
        f = _Function(inputs, outputs.values(), updates, givens=givens)
        return lambda *args, **kwargs: type(outputs)(zip(outputs.keys(), f(*args, **kwargs)))
    else:
        f = _Function(inputs, [outputs], updates, givens=givens)
        return lambda *args, **kwargs: f(*args, **kwargs)[0]


class _Function(object):
    def __init__(self, inputs, outputs, updates, givens):
        for inpt in inputs:
            if not hasattr(inpt, 'make_feed_dict') and not (type(inpt) is tf.compat.v1.Tensor and len(inpt.op.inputs) == 0):
                assert False, "inputs should all be placeholders, constants, or have a make_feed_dict method"
        self.inputs = inputs
        updates = updates or []
        self.update_group = tf.compat.v1.group(*updates)
        self.outputs_update = list(outputs) + [self.update_group]
        self.givens = {} if givens is None else givens

    def _feed_input(self, feed_dict, inpt, value):
        if hasattr(inpt, 'make_feed_dict'):
            feed_dict.update(inpt.make_feed_dict(value))
        else:
            feed_dict[inpt] = value

    def __call__(self, *args):
        assert len(args) <= len(self.inputs), "Too many arguments provided"
        feed_dict = {}
        # Update the args
        for inpt, value in zip(self.inputs, args):
            self._feed_input(feed_dict, inpt, value)
        # Update feed dict with givens.
        for inpt in self.givens:
            feed_dict[inpt] = feed_dict.get(inpt, self.givens[inpt])
        results = tf.compat.v1.get_default_session().run(self.outputs_update, feed_dict=feed_dict)[:-1]
        return results

# ================================================================
# Flat vectors
# ================================================================

def var_shape(x):
    out = x.get_shape().as_list()
    assert all(isinstance(a, int) for a in out), \
        "shape function assumes that shape is fully known"
    return out

def numel(x):
    return intprod(var_shape(x))

def intprod(x):
    return int(np.prod(x))

def flatgrad(loss, var_list, clip_norm=None):
    grads = tf.compat.v1.gradients(loss, var_list)
    if clip_norm is not None:
        grads = [tf.compat.v1.clip_by_norm(grad, clip_norm=clip_norm) for grad in grads]
    return tf.compat.v1.concat(axis=0, values=[
        tf.compat.v1.reshape(grad if grad is not None else tf.compat.v1.zeros_like(v), [numel(v)])
        for (v, grad) in zip(var_list, grads)
    ])

class SetFromFlat(object):
    def __init__(self, var_list, dtype=tf.compat.v1.float32):
        assigns = []
        shapes = list(map(var_shape, var_list))
        total_size = np.sum([intprod(shape) for shape in shapes])

        self.theta = theta = tf.compat.v1.placeholder(dtype, [total_size])
        start = 0
        assigns = []
        for (shape, v) in zip(shapes, var_list):
            size = intprod(shape)
            assigns.append(tf.compat.v1.assign(v, tf.compat.v1.reshape(theta[start:start + size], shape)))
            start += size
        self.op = tf.compat.v1.group(*assigns)

    def __call__(self, theta):
        tf.compat.v1.get_default_session().run(self.op, feed_dict={self.theta: theta})

class GetFlat(object):
    def __init__(self, var_list):
        self.op = tf.compat.v1.concat(axis=0, values=[tf.compat.v1.reshape(v, [numel(v)]) for v in var_list])

    def __call__(self):
        return tf.compat.v1.get_default_session().run(self.op)

_PLACEHOLDER_CACHE = {}  # name -> (placeholder, dtype, shape)

def get_placeholder(name, dtype, shape):
    if name in _PLACEHOLDER_CACHE:
        out, dtype1, shape1 = _PLACEHOLDER_CACHE[name]
        assert dtype1 == dtype and shape1 == shape
        return out
    else:
        out = tf.compat.v1.placeholder(dtype=dtype, shape=shape, name=name)
        _PLACEHOLDER_CACHE[name] = (out, dtype, shape)
        return out

def get_placeholder_cached(name):
    return _PLACEHOLDER_CACHE[name][0]

def flattenallbut0(x):
    return tf.compat.v1.reshape(x, [-1, intprod(x.get_shape().as_list()[1:])])


# ================================================================
# Diagnostics 
# ================================================================

def display_var_info(vars):
    from baselines import logger
    count_params = 0
    for v in vars:
        name = v.name
        if "/Adam" in name or "beta1_power" in name or "beta2_power" in name: continue
        v_params = np.prod(v.shape.as_list())
        count_params += v_params
        if "/b:" in name or "/biases" in name: continue    # Wx+b, bias is not interesting to look at => count params, but not print
        logger.info("   %s%s %i params %s" % (name, " "*(55-len(name)), v_params, str(v.shape)))

    logger.info("Total model parameters: %0.2f million" % (count_params*1e-6))


def get_available_gpus():
    # recipe from here:
    # https://stackoverflow.com/questions/38559755/how-to-get-current-available-gpus-in-tensorflow?utm_medium=organic&utm_source=google_rich_qa&utm_campaign=google_rich_qa
 
    from tensorflow.python.client import device_lib
    local_device_protos = device_lib.list_local_devices()
    return [x.name for x in local_device_protos if x.device_type == 'GPU']

# ================================================================
# Saving variables
# ================================================================

def load_state(fname):
    saver = tf.compat.v1.train.Saver()
    saver.restore(tf.compat.v1.get_default_session(), fname)

def save_state(fname):
    os.makedirs(os.path.dirname(fname), exist_ok=True)
    saver = tf.compat.v1.train.Saver()
    saver.save(tf.compat.v1.get_default_session(), fname)


