#!/usr/bin/env python

import openravepy

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
	joint_start1 = [3.14/3, 3.14/4, 0]
	robot.SetDOFValues(joint_start1, robot.GetManipulator('left_arm').GetArmIndices())

	joint_start2 = [-3.14/5, 3.14/4, 0]
	robot.SetDOFValues(joint_start2, robot.GetManipulator('right_arm').GetArmIndices())
	# traj = get_swing_trajectory(robot)
	# play_trajectory(robot, traj)

import IPython
IPython.embed()