# Copyright (c) 2017 OpenAI (http://openai.com)
# Copyright (C) 2018 deeplearningrobotics.ai
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

import tensorflow as tf
import numpy as np
import tf_util as U
from utils import fc
from tensorflow.python.ops import math_ops

class Pd(object):
    """
    A particular probability distribution
    """
    def flatparam(self):
        raise NotImplementedError
    def mode(self):
        raise NotImplementedError
    def neglogp(self, x):
        # Usually it's easier to define the negative logprob
        raise NotImplementedError
    def kl(self, other):
        raise NotImplementedError
    def entropy(self):
        raise NotImplementedError
    def sample(self):
        raise NotImplementedError
    def logp(self, x):
        return - self.neglogp(x)

class PdType(object):
    """
    Parametrized family of probability distributions
    """
    def pdclass(self):
        raise NotImplementedError
    def pdfromflat(self, flat):
        return self.pdclass()(flat)
    def pdfromlatent(self, latent_vector):
        raise NotImplementedError
    def param_shape(self):
        raise NotImplementedError
    def sample_shape(self):
        raise NotImplementedError
    def sample_dtype(self):
        raise NotImplementedError

    def param_placeholder(self, prepend_shape, name=None):
        return tf.compat.v1.placeholder(dtype=tf.compat.v1.float32, shape=prepend_shape+self.param_shape(), name=name)
    def sample_placeholder(self, prepend_shape, name=None):
        return tf.compat.v1.placeholder(dtype=self.sample_dtype(), shape=prepend_shape+self.sample_shape(), name=name)

class CategoricalPdType(PdType):
    def __init__(self, ncat):
        self.ncat = ncat
    def pdclass(self):
        return CategoricalPd
    def pdfromlatent(self, latent_vector, init_scale=1.0, init_bias=0.0):
        pdparam = fc(latent_vector, 'pi', self.ncat, init_scale=init_scale, init_bias=init_bias)
        return self.pdfromflat(pdparam), pdparam

    def param_shape(self):
        return [self.ncat]
    def sample_shape(self):
        return []
    def sample_dtype(self):
        return tf.compat.v1.int32


class MultiCategoricalPdType(PdType):
    def __init__(self, nvec):
        self.ncats = nvec
    def pdclass(self):
        return MultiCategoricalPd
    def pdfromflat(self, flat):
        return MultiCategoricalPd(self.ncats, flat)
    def param_shape(self):
        return [sum(self.ncats)]
    def sample_shape(self):
        return [len(self.ncats)]
    def sample_dtype(self):
        return tf.compat.v1.int32

class DiagGaussianPdType(PdType):
    def __init__(self, size):
        self.size = size
    def pdclass(self):
        return DiagGaussianPd

    def pdfromlatent(self, latent_vector, init_scale=1.0, init_bias=0.0):
        mean = tf.compat.v1.tanh(fc(latent_vector, 'pi', self.size, init_scale=init_scale, init_bias=init_bias))
        logstd = tf.compat.v1.get_variable(name='logstd', shape=[1, self.size], initializer=tf.compat.v1.zeros_initializer())
        pdparam = tf.compat.v1.concat([mean, mean * 0.0 + logstd], axis=1)
        return self.pdfromflat(pdparam), mean

    def param_shape(self):
        return [2*self.size]
    def sample_shape(self):
        return [self.size]
    def sample_dtype(self):
        return tf.compat.v1.float32

class BernoulliPdType(PdType):
    def __init__(self, size):
        self.size = size
    def pdclass(self):
        return BernoulliPd
    def param_shape(self):
        return [self.size]
    def sample_shape(self):
        return [self.size]
    def sample_dtype(self):
        return tf.compat.v1.int32

# WRONG SECOND DERIVATIVES
# class CategoricalPd(Pd):
#     def __init__(self, logits):
#         self.logits = logits
#         self.ps = tf.compat.v1.nn.softmax(logits)
#     @classmethod
#     def fromflat(cls, flat):
#         return cls(flat)
#     def flatparam(self):
#         return self.logits
#     def mode(self):
#         return U.argmax(self.logits, axis=-1)
#     def logp(self, x):
#         return -tf.compat.v1.nn.sparse_softmax_cross_entropy_with_logits(self.logits, x)
#     def kl(self, other):
#         return tf.compat.v1.nn.softmax_cross_entropy_with_logits(other.logits, self.ps) \
#                 - tf.compat.v1.nn.softmax_cross_entropy_with_logits(self.logits, self.ps)
#     def entropy(self):
#         return tf.compat.v1.nn.softmax_cross_entropy_with_logits(self.logits, self.ps)
#     def sample(self):
#         u = tf.compat.v1.random_uniform(tf.compat.v1.shape(self.logits))
#         return U.argmax(self.logits - tf.compat.v1.log(-tf.compat.v1.log(u)), axis=-1)

class CategoricalPd(Pd):
    def __init__(self, logits):
        self.logits = logits
    def flatparam(self):
        return self.logits
    def mode(self):
        return tf.compat.v1.argmax(self.logits, axis=-1)
    def neglogp(self, x):
        # return tf.compat.v1.nn.sparse_softmax_cross_entropy_with_logits(logits=self.logits, labels=x)
        # Note: we can't use sparse_softmax_cross_entropy_with_logits because
        #       the implementation does not allow second-order derivatives...
        one_hot_actions = tf.compat.v1.one_hot(x, self.logits.get_shape().as_list()[-1])
        return tf.compat.v1.nn.softmax_cross_entropy_with_logits(
            logits=self.logits,
            labels=one_hot_actions)
    def kl(self, other):
        a0 = self.logits - tf.compat.v1.reduce_max(self.logits, axis=-1, keep_dims=True)
        a1 = other.logits - tf.compat.v1.reduce_max(other.logits, axis=-1, keep_dims=True)
        ea0 = tf.compat.v1.exp(a0)
        ea1 = tf.compat.v1.exp(a1)
        z0 = tf.compat.v1.reduce_sum(ea0, axis=-1, keep_dims=True)
        z1 = tf.compat.v1.reduce_sum(ea1, axis=-1, keep_dims=True)
        p0 = ea0 / z0
        return tf.compat.v1.reduce_sum(p0 * (a0 - tf.compat.v1.log(z0) - a1 + tf.compat.v1.log(z1)), axis=-1)
    def entropy(self):
        a0 = self.logits - tf.compat.v1.reduce_max(self.logits, axis=-1, keep_dims=True)
        ea0 = tf.compat.v1.exp(a0)
        z0 = tf.compat.v1.reduce_sum(ea0, axis=-1, keep_dims=True)
        p0 = ea0 / z0
        return tf.compat.v1.reduce_sum(p0 * (tf.compat.v1.log(z0) - a0), axis=-1)
    def sample(self):
        u = tf.compat.v1.random_uniform(tf.compat.v1.shape(self.logits))
        return tf.compat.v1.argmax(self.logits - tf.compat.v1.log(-tf.compat.v1.log(u)), axis=-1)
    @classmethod
    def fromflat(cls, flat):
        return cls(flat)

class MultiCategoricalPd(Pd):
    def __init__(self, nvec, flat):
        self.flat = flat
        self.categoricals = list(map(CategoricalPd, tf.compat.v1.split(flat, nvec, axis=-1)))
    def flatparam(self):
        return self.flat
    def mode(self):
        return tf.compat.v1.cast(tf.compat.v1.stack([p.mode() for p in self.categoricals], axis=-1), tf.compat.v1.int32)
    def neglogp(self, x):
        return tf.compat.v1.add_n([p.neglogp(px) for p, px in zip(self.categoricals, tf.compat.v1.unstack(x, axis=-1))])
    def kl(self, other):
        return tf.compat.v1.add_n([p.kl(q) for p, q in zip(self.categoricals, other.categoricals)])
    def entropy(self):
        return tf.compat.v1.add_n([p.entropy() for p in self.categoricals])
    def sample(self):
        return tf.compat.v1.cast(tf.compat.v1.stack([p.sample() for p in self.categoricals], axis=-1), tf.compat.v1.int32)
    @classmethod
    def fromflat(cls, flat):
        raise NotImplementedError

class DiagGaussianPd(Pd):
    def __init__(self, flat):
        self.flat = flat
        mean, logstd = tf.compat.v1.split(axis=len(flat.shape)-1, num_or_size_splits=2, value=flat)
        self.mean = mean
        self.logstd = logstd
        self.std = tf.compat.v1.exp(logstd)
    def flatparam(self):
        return self.flat
    def mode(self):
        return self.mean
    def neglogp(self, x):
        return 0.5 * tf.compat.v1.reduce_sum(tf.compat.v1.square((x - self.mean) / self.std), axis=-1) \
               + 0.5 * np.log(2.0 * np.pi) * tf.compat.v1.to_float(tf.compat.v1.shape(x)[-1]) \
               + tf.compat.v1.reduce_sum(self.logstd, axis=-1)
    def kl(self, other):
        assert isinstance(other, DiagGaussianPd)
        return tf.compat.v1.reduce_sum(other.logstd - self.logstd + (tf.compat.v1.square(self.std) + tf.compat.v1.square(self.mean - other.mean)) / (2.0 * tf.compat.v1.square(other.std)) - 0.5, axis=-1)
    def entropy(self):
        return tf.compat.v1.reduce_sum(self.logstd + .5 * np.log(2.0 * np.pi * np.e), axis=-1)
    def sample(self):
        return self.mean + self.std * tf.compat.v1.random_normal(tf.compat.v1.shape(self.mean))
    @classmethod
    def fromflat(cls, flat):
        return cls(flat)

class BernoulliPd(Pd):
    def __init__(self, logits):
        self.logits = logits
        self.ps = tf.compat.v1.sigmoid(logits)
    def flatparam(self):
        return self.logits
    def mode(self):
        return tf.compat.v1.round(self.ps)
    def neglogp(self, x):
        return tf.compat.v1.reduce_sum(tf.compat.v1.nn.sigmoid_cross_entropy_with_logits(logits=self.logits, labels=tf.compat.v1.to_float(x)), axis=-1)
    def kl(self, other):
        return tf.compat.v1.reduce_sum(tf.compat.v1.nn.sigmoid_cross_entropy_with_logits(logits=other.logits, labels=self.ps), axis=-1) - tf.compat.v1.reduce_sum(tf.compat.v1.nn.sigmoid_cross_entropy_with_logits(logits=self.logits, labels=self.ps), axis=-1)
    def entropy(self):
        return tf.compat.v1.reduce_sum(tf.compat.v1.nn.sigmoid_cross_entropy_with_logits(logits=self.logits, labels=self.ps), axis=-1)
    def sample(self):
        u = tf.compat.v1.random_uniform(tf.compat.v1.shape(self.ps))
        return tf.compat.v1.to_float(math_ops.less(u, self.ps))
    @classmethod
    def fromflat(cls, flat):
        return cls(flat)

def make_pdtype(ac_space):
    from gym import spaces
    if isinstance(ac_space, spaces.Box):
        assert len(ac_space.shape) == 1
        return DiagGaussianPdType(ac_space.shape[0])
    elif isinstance(ac_space, spaces.Discrete):
        return CategoricalPdType(ac_space.n)
    elif isinstance(ac_space, spaces.MultiDiscrete):
        return MultiCategoricalPdType(ac_space.nvec)
    elif isinstance(ac_space, spaces.MultiBinary):
        return BernoulliPdType(ac_space.n)
    else:
        raise NotImplementedError

def shape_el(v, i):
    maybe = v.get_shape()[i]
    if maybe is not None:
        return maybe
    else:
        return tf.compat.v1.shape(v)[i]

@U.in_session
def test_probtypes():
    np.random.seed(0)

    pdparam_diag_gauss = np.array([-.2, .3, .4, -.5, .1, -.5, .1, 0.8])
    diag_gauss = DiagGaussianPdType(pdparam_diag_gauss.size // 2) #pylint: disable=E1101
    validate_probtype(diag_gauss, pdparam_diag_gauss)

    pdparam_categorical = np.array([-.2, .3, .5])
    categorical = CategoricalPdType(pdparam_categorical.size) #pylint: disable=E1101
    validate_probtype(categorical, pdparam_categorical)

    nvec = [1,2,3]
    pdparam_multicategorical = np.array([-.2, .3, .5, .1, 1, -.1])
    multicategorical = MultiCategoricalPdType(nvec) #pylint: disable=E1101
    validate_probtype(multicategorical, pdparam_multicategorical)

    pdparam_bernoulli = np.array([-.2, .3, .5])
    bernoulli = BernoulliPdType(pdparam_bernoulli.size) #pylint: disable=E1101
    validate_probtype(bernoulli, pdparam_bernoulli)


def validate_probtype(probtype, pdparam):
    N = 100000
    # Check to see if mean negative log likelihood == differential entropy
    Mval = np.repeat(pdparam[None, :], N, axis=0)
    M = probtype.param_placeholder([N])
    X = probtype.sample_placeholder([N])
    pd = probtype.pdfromflat(M)
    calcloglik = U.function([X, M], pd.logp(X))
    calcent = U.function([M], pd.entropy())
    Xval = tf.compat.v1.get_default_session().run(pd.sample(), feed_dict={M:Mval})
    logliks = calcloglik(Xval, Mval)
    entval_ll = - logliks.mean() #pylint: disable=E1101
    entval_ll_stderr = logliks.std() / np.sqrt(N) #pylint: disable=E1101
    entval = calcent(Mval).mean() #pylint: disable=E1101
    assert np.abs(entval - entval_ll) < 3 * entval_ll_stderr # within 3 sigmas

    # Check to see if kldiv[p,q] = - ent[p] - E_p[log q]
    M2 = probtype.param_placeholder([N])
    pd2 = probtype.pdfromflat(M2)
    q = pdparam + np.random.randn(pdparam.size) * 0.1
    Mval2 = np.repeat(q[None, :], N, axis=0)
    calckl = U.function([M, M2], pd.kl(pd2))
    klval = calckl(Mval, Mval2).mean() #pylint: disable=E1101
    logliks = calcloglik(Xval, Mval2)
    klval_ll = - entval - logliks.mean() #pylint: disable=E1101
    klval_ll_stderr = logliks.std() / np.sqrt(N) #pylint: disable=E1101
    assert np.abs(klval - klval_ll) < 3 * klval_ll_stderr # within 3 sigmas
    print('ok on', probtype, pdparam)

