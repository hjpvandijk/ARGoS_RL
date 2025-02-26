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
import math
import time

import numpy as np
import rospy

from tensorswarm.srv import *
from geometry_msgs.msg import Twist, Pose2D
import gym
from gym import spaces
from multiprocessing import Process, Pool, TimeoutError, Queue
from ArgosMultiEnvironment import ArgosEnvironment



class ArgosMultiProcessEnvironment(gym.Env):
    """A tensorforce environment for the Argos robotics simulator.

    """
    def __init__(self, start_poses, goal_poses, local_map_height_cells, local_map_width_cells):
        """The length of the start and end poses must match and determine the number of robots.

        :param start_poses: The desired start poses of the robots.
        :param goal_poses: The desired goal poses of the robots.
        """

        assert(len(start_poses) == len(goal_poses))
        self.start_poses = start_poses
        self.goal_poses = goal_poses

        self.num_envs = 1
        self.action_space = spaces.Box(low=-1, high=1, shape=(2,))
        # self.observation_space = spaces.Box(low=-10, high=10, shape=(512+2+2,))
        self.state_space = spaces.Box(low=-10, high=10, shape=((local_map_height_cells*local_map_width_cells)+2+2,))
        # self.observation_space = dict(laser=dict(type='float', shape=(512, 3,)),
        #                               velocities=dict(type='float', shape=(2,)),
        #                               rel_goal=dict(type='float', shape=(2,)),
        #                               id=dict(type='int', shape=(1,)))
        self.map_height_cells = local_map_height_cells
        self.map_width_cells = local_map_width_cells
        self.ep_len_counter = 0

        self.num_robots = len(start_poses)*4

        self.aq1 = Queue()
        self.st1 = Queue()
        self.p1 = Process(target=self.proc, args=('ai0', start_poses, goal_poses, self.aq1, self.st1, local_map_height_cells, local_map_width_cells))
        self.p1.start()

        self.aq2 = Queue()
        self.st2 = Queue()
        self.p2 = Process(target=self.proc, args=('ai1', start_poses, goal_poses, self.aq2, self.st2, local_map_height_cells, local_map_width_cells))
        self.p2.start()

        self.aq3 = Queue()
        self.st3 = Queue()
        self.p3 = Process(target=self.proc, args=('ai2', start_poses, goal_poses, self.aq3, self.st3, local_map_height_cells, local_map_width_cells))
        self.p3.start()

        self.aq4 = Queue()
        self.st4 = Queue()
        self.p4 = Process(target=self.proc, args=('ai3', start_poses, goal_poses, self.aq4, self.st4, local_map_height_cells, local_map_width_cells))
        self.p4.start()

        self.model = None

    def set_model(self, model):
        self.model = model

    def proc(self, name, start_poses, goal_poses, action_q, sts_q, local_map_height_cells, local_map_width_cells):
        env = ArgosEnvironment(start_poses, goal_poses, name, local_map_height_cells, local_map_width_cells)
        while True:
            action = action_q.get()
            if len(action) == 0:
                sts_q.put(env.reset())
            else:
                sts_q.put(env.step(action))

    def chunk(self, xs, n):
        '''Split the list, xs, into n chunks'''
        L = len(xs)
        assert 0 < n <= L
        s, r = divmod(L, n)
        chunks = [xs[p:p+s] for p in range(0, L, s)]
        chunks[n-1:] = [xs[-r-s:]]
        return chunks

    def step(self, actions):
        assert(len(actions) % 4 == 0)

        ids, sts, rews, dones = [], [], [], []

        acs = self.chunk(actions, 4)

        self.aq1.put(acs[0])
        self.aq2.put(acs[1])
        self.aq3.put(acs[2])
        self.aq4.put(acs[3])


        id, st, rew, done, _ = self.st1.get()
        ids.extend(id)
        sts.extend(st)
        rews.extend(rew)
        dones.extend(done)

        id, st, rew, done, _ = self.st2.get()
        ids.extend(id)
        sts.extend(st)
        rews.extend(rew)
        dones.extend(done)

        id, st, rew, done, _ = self.st3.get()
        ids.extend(id)
        sts.extend(st)
        rews.extend(rew)
        dones.extend(done)

        id, st, rew, done, _ = self.st4.get()
        ids.extend(id)
        sts.extend(st)
        rews.extend(rew)
        dones.extend(done)

        #return range(0, self.num_robots), ob`s, rews, dones, []

        infos = [{"episode": {"l": self.ep_len_counter, "r": np.mean(rews)}}]
        self.ep_len_counter = self.ep_len_counter + 1
        return range(0, self.num_robots), sts, rews, dones, infos

    def reset(self):


        self.aq1.put([])
        self.aq2.put([])
        self.aq3.put([])
        self.aq4.put([])

        ids, obs = [], []
        id, st = self.st1.get()
        ids.extend(id)
        obs.extend(st)
        id, st = self.st2.get()
        ids.extend(id)
        obs.extend(st)
        id, st = self.st3.get()
        ids.extend(id)
        obs.extend(st)
        id, st = self.st4.get()
        ids.extend(id)
        obs.extend(st)


        self.ep_len_counter = 0
        return range(0, self.num_robots), obs

    def close(self):
        print("Closing environment")
        self.p1.terminate()
        self.p2.terminate()
        self.p3.terminate()
        self.p4.terminate()
        self.p1.join()
        self.p2.join()
        self.p3.join()
        self.p4.join()
