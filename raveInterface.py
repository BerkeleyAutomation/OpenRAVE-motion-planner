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
	
	def __init__(self, env_file, arm_to_plan, indx=0):
		self.env = op.Environment()
		self.env.StopSimulation()
		self.env.Load(env_file)

		self.robot 		= self.env.GetRobots()[indx]
		self.Left_arm 	= self.robot.GetManipulator('left_arm')
		self.Right_arm 	= self.robot.GetManipulator('right_arm')

	# env.Load("env.xml")
	# env.Load("../data/table.xml")

	def init_collision_checker(self, checker, collision_options):
		"""
		params: checker <string>: The name of the checker used for collision detection
				collision_options <list><op::CollisionOptions>
		"""
		collisionChecker = op.RaveCreateCollisionChecker(self.env, checker)

		j = 0
		for i in collision_options:
			j = j|i

		collisionChecker.SetCollisionOptions(j)
		self.env.SetCollisionChecker(collisionChecker)

	# collisionChecker = op.RaveCreateCollisionChecker(env,'pqp')
	# collisionChecker.SetCollisionOptions(op.CollisionOptions.Distance|op.CollisionOptions.Contacts)

	def set_request(self, manip, joint_target):
		"""
		Starts with a straight line trajectory to the end goal
		"""
		self.plan_arm 	= manip 	# Sets the planning arm for visual purpose
		self.target 	= joint_target

		self.request = {
		  "basic_info" : {
		    "n_steps" : 200,
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
		    "type" : "joint", # joint-space target
		    "params" : {"vals" : joint_target[0]} # length of vals = # dofs of manip
		    },
		  {
		    "type"    : "cart_vel",
		    "name"    : "s0_vel",
		    "params"  : {
		      "max_displacement"  : 1,
		      "first_step"        : 0,
		      "last_step"         : 200 -1, #inclusive
		      "link"              : "s0"
		    }
		  }, 
		  {
		    "type"    : "cart_vel",
		    "name"    : "s1_vel",
		    "params"  : {
		      "max_displacement"  : 1,
		      "first_step"        : 0,
		      "last_step"         : 200 -1, #inclusive
		      "link"              : "s1"
		    }
		  }

		  ]
		}

		return

	def get_robot(self):
		return self.robot

	def get_manip(self, name):
		if 	 name == "left_arm" : return self.Left_arm
		elif name == "right_arm" : return self.Right_arm

	def optimize(self):
		"""
		Optimization of the robot via initialization of motion through different way points
		We start by performing optimization in a straight line towards the joint target
		We check if the optimal path choosen is safe by iterating through the final trajectory 
			If path is safe -> Return trajectory
			If path is not safe -> Initialize to a different trajectory

		The sequence of optimization is done as such
			1) Straight line
			2) Retraction of arm
			3) Stationary
		"""
		for i in range(3):
			try: del self.request['init_info']
			except KeyError: pass

			if i == 0:
				pull_back = self.target[0]
				pull_back[-1] = 0
				self.request.update({"init_info" : {"type" : "straight_line", "endpoint" : pull_back}})	# Straight line initialization

			elif i == 1:
				self.request.update({"init_info" : {"type" : "straight_line", "endpoint" : self.target[0]}})	# Straight line initialization

			elif i == 2:
				self.request.update({"init_info" : {"type" : "stationary"}})	# Straight line initialization

			jd 			= json.dumps(self.request) 					# convert dictionary into json-formatted string
			prob 		= trajoptpy.ConstructProblem(jd, self.env) 	# create object that stores optimization problem
			result 		= trajoptpy.OptimizeProblem(prob) 			# do optimization

			if self.__check_safe(result.GetTraj()):
				self.traj 	= result.GetTraj()
				return

		raise Exception('No path is safe')
		return

	def simulate(self):
		self.env.SetViewer('qtcoin')
	  	for t in self.traj:
	  		self.robot.SetDOFValues(t, self.robot.GetManipulator(self.plan_arm).GetArmIndices())
	  		time.sleep(0.1)

	def __check_safe(self, trajectory):
		kin = self.env.GetKinBody('dvrk')
		self.env.GetCollisionChecker().SetCollisionOptions(op.CollisionOptions.Contacts)

		for t in trajectory:			
  			self.robot.SetDOFValues(t, self.robot.GetManipulator(self.plan_arm).GetArmIndices())
  			flag = self.env.CheckCollision(kin.GetLinks()[3], kin.GetLinks()[6])		# Checks for collisions between 2 cylinder arms of the robot

	  		if flag == True:
	  			return False		# That means that collision happened
	  		time.sleep(0.05)

	  	return True

if __name__ == "__main__":
	joint_start1 = [3.14/3, 3.14/4, 0]
	joint_start2 = [-3.14/4, 3.14/4, 0]
	manip = "right_arm"

	planner = Motion_planning('env.xml', "right_arm")
	planner.init_collision_checker('pqp', [op.CollisionOptions.Distance, op.CollisionOptions.Contacts])

	planner.get_robot().SetDOFValues(joint_start1, planner.get_manip("left_arm").GetArmIndices())
	planner.get_robot().SetDOFValues(joint_start2, planner.get_manip("right_arm").GetArmIndices())

	IK_obj = IK.dVRK_IK_simple()                                # Creates an IK object 
	endEff = IK_obj.get_endEffector_fromDOF([-3.14/2, 3.14/4, 0])
	joint_target = IK_obj.get_joint_DOF(endEff)     

	planner.set_request(manip, joint_target)
	planner.optimize()
	planner.simulate()

	IPython.embed()
