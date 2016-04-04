#!/usr/bin/env python

import openravepy

class Trajectory(object):

	def __init__(self, T, q_ftn):
		self.T = T  # duration, in seconds
		self.q = q_ftn

from numpy import array, pi

def get_swing_trajectory(robot, T=1.):
	q0 = robot.GetDOFValues()
	q1 = array([pi, 0.])

	def q(t):
		x = 0. if t < 0. else 1. if t > T else (t / T)
		return (1. - x) * q0 + x * q1 	

	return Trajectory(T, q)

from numpy import arange
import time

def play_trajectory(robot, traj):
	dt = 1e-2  # play at 100 Hz
	for t in arange(0., traj.T + dt, dt):
		robot.SetDOFValues(traj.q(t))
		time.sleep(dt)


if __name__ == "__main__":
	env = openravepy.Environment()
	env.Load('env.xml')
	env.SetViewer('qtcoin')
	viewer = env.GetViewer()
	viewer.SetBkgndColor([.8, .85, .9])
	robot = env.GetRobots()[0]
	# traj = get_swing_trajectory(robot)
	# play_trajectory(robot, traj)

import IPython
IPython.embed()