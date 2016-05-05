import argparse
parser = argparse.ArgumentParser()
parser.add_argument("--interactive", action="store_true")
# parser.add_argument("--collision", action="pqp")
args = parser.parse_args()

import openravepy as op
import trajoptpy
import json
import time
import IK
import IPython
from math import pi
import numpy as np

class Motion_planning:
	
	def __init__(self, env_file):
		env = op.Environment()
		env.Load(env_file)
		env.StopSimulation()
		env.SetViewer('qtcoin')
		env.GetViewer().SetBkgndColor([.8, .85, .9])
		time.sleep(0.1)                           # Give time for the environment to update

		self.Env 		= env
		self.robot 		= env.GetRobots()[0]
		self.result 	= []

	# env.Load("env.xml")
	# env.Load("../data/table.xml")

	def init_collision_checker(self, checker, collision_options):
		"""
		params: checker <string>: The name of the checker used for collision detection
				collision_options <list><op::CollisionOptions>
		"""
		self.collisionChecker = op.RaveCreateCollisionChecker(self.Env, checker)

		j = 0
		for i in collision_options:
			j = j|i

		self.collisionChecker.SetCollisionOptions(j)
		self.Env.SetCollisionChecker(collisionChecker)

	# collisionChecker = op.RaveCreateCollisionChecker(env,'pqp')
	# collisionChecker.SetCollisionOptions(op.CollisionOptions.Distance|op.CollisionOptions.Contacts)

	def init_request(self, n_steps, manip, joint_target):
		self.request = {
		  "basic_info" : {
		    "n_steps" : n_steps,
		    "manip" : manip, # see below for valid values
		    "start_fixed" : True # i.e., DOF values at first timestep are fixed based on current robot state
		  },

		  "costs" : [
		  {
		    "type" : "joint_vel", # joint-space velocity cost
		    "params": {"coeffs" : [10000,10000,1]} # a list of length one is automatically expanded to a list of length n_dofs
		    # also valid: [1.9, 2, 3, 4, 5, 5, 4, 3, 2, 1]
		  },
		  {
		    "type" : "collision",
		    "params" : {
		      "coeffs" : [1000], # penalty coefficients. list of length one is automatically expanded to a list of length n_timesteps
		      "dist_pen" : [0.4], # robot-obstacle distance that penalty kicks in. expands to length n_timesteps
		      "continuous" : True
		    }
		  },

		  {
		    "type" : "collision",
		    "params" : {
		      "coeffs" : [1000], # penalty coefficients. list of length one is automatically expanded to a list of length n_timesteps
		      "dist_pen" : [0.4], # robot-obstacle distance that penalty kicks in. expands to length n_timesteps
		      "continuous" : False
		    }
		  }    
		  ],

		  "constraints" : [
		  {
		    "type" : "joint", 						# joint-space target
		    "params" : {"vals" : joint_target['endpoint'][-1]} 	# set constaint to the final joint target point
		    },
		  {
		    "type"    : "cart_vel",
		    "name"    : "s0_vel",
		    "params"  : {
		      "max_displacement"  : 1,
		      "first_step"        : 0,
		      "last_step"         : n_steps -1, #inclusive
		      "link"              : "s0"
		    }
		  }, 
		  {
		    "type"    : "cart_vel",
		    "name"    : "s1_vel",
		    "params"  : {
		      "max_displacement"  : 1,
		      "first_step"        : 0,
		      "last_step"         : n_steps -1, #inclusive
		      "link"              : "s1"
		    }
		  }
		  ]
		}

		if length(joint_target['endpoint']) == 1:
			add = {"init_info" : {
			"type" : "straight_line", # straight line in joint space.
			"endpoint" : joint_target['endpoint']
			  # "type" : "stationary"
			}}

		elif length(joint_target['endpoint'] > 1):
			add = {"init_info" : {
			"type" : "given_traj", # straight line in joint space.
			"data" : joint_target['endpoint']
			# "type" : "stationary"
			}}
		self.result.update(add)
		return 

	def get_robot(self):
		return self.robot

	def set_arm_from_DOF(self, DOF, name):
		self.robot.SetDOFValues(DOF, self.robot.GetManipulator(name).GetArmIndices())
		return

	def DOF_to_endeff(self, DOF):
		assert length(DOF) == 3
		return IK.get_endEffector_fromDOF(DOF)

	def endeff_to_DOF(self, endeff):
		assert length(endeff) == 3
		return IK.get_joint_DOF(endeff)

	def optimize(n_steps, manip, joint_target):
		try:
			self.init_request(n_steps, manip, joint_target)		# sets the initial request of the problem
			s = json.dumps(self.request) 						# convert dictionary into json-formatted string
	 		prob = trajoptpy.ConstructProblem(s, self.Env) 		# create object that stores optimization problem
			t_start = time.time()
			self.result = trajoptpy.OptimizeProblem(prob) 			# do optimization
			t_elapsed = time.time() - t_start
			prob.SetRobotActiveDOFs()
			print ("optimization took %.3f seconds"%t_elapsed)
			return 

		except NameError:
			print('Variable results not in existence')

	def simulate(self):
		traj = self.result.GetTraj()
	  	for wp in traj:
	    	robot.SetDOFValues(wp, robot.GetManipulator(manip).GetArmIndices())
	    	time.sleep(0.1)


if __name__ == "__main__":
	planner 		= Motion_planning('env.xml')
	planner.init_collision_checker('pqp', [op.CollisionOptions.Distance, op.CollisionOptions.Contacts])

	joint_start1 	= [3.14/3, 3.14/4, 0]
	joint_start2 	= [-3.14/4, 3.14/4, 0]

	planner.set_arm_from_DOF(DOF=joint_start1, name="left_arm")
	planner.set_arm_from_DOF(DOF=joint_start2, name="right_arm")

	# Begin loop to optimize for different initial conditions 
	# Consider initializing the trajectory by retracting arm and checking if traj is safe
	target = {"type":"straight_line", "endpoint":[[-3.14/2, 3.14/4, 0]]}
	planner.optimize(n_steps=200, manip='right_arm', joint_target=target)
	planner.simulate()
	IPython.embed()
