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
import numpy as np
from math import pi, ceil

class Motion_planning:
	
	def __init__(self, env_file, arm_to_plan, indx=0):
		"""
		Constructor to initialize the environment and the dVRK robot

		params: 
			env_file <string>	: 	The environment file you want to load
			arm_to_plan <string>:	Either the dVRK "left_arm" or "right_arm"
			indx <int>			:	Default 0 since we only have 1 robot loaded
		"""
		self.env = op.Environment()
		self.env.StopSimulation()
		self.env.Load(env_file)
		self.env.SetViewer('qtcoin')

		self.robot 		= self.env.GetRobots()[indx]
		self.left_arm 	= self.robot.GetManipulator('left_arm')
		self.right_arm 	= self.robot.GetManipulator('right_arm')

	def init_collision_checker(self, checker, collision_options):
		"""
		params: 
			checker <string> : The name of the Checker used for collision detection. Eg. 'pqp' collision checker
			collision_options <list><op::CollisionOptions> : Various collision options available expressed as a list
		"""
		collisionChecker = op.RaveCreateCollisionChecker(self.env, checker)

		j = 0
		for i in collision_options:
			j = j|i

		collisionChecker.SetCollisionOptions(j)
		self.env.SetCollisionChecker(collisionChecker)

	# collisionChecker = op.RaveCreateCollisionChecker(env,'pqp')
	# collisionChecker.SetCollisionOptions(op.CollisionOptions.Distance|op.CollisionOptions.Contacts)

	def __set_request(self, manip, joint_goal, n_steps):
		"""
		Initializes a request dictionary that allows for TrajOpt motion planning
		You can change the collision cost penalties as well as the distance to enforce those penalties
		You can add now constraints and penalties if you want

		Params:
			manip <string>			 : "left_arm" or "right_arm"
			joint_goal <list><float> : The goal (in DOF space) to be reached by the planning arm (manip)
			n_steps <int>			 : Number of planning points for TrajOpt. Must correspond to the number of points generated by OMPL
		"""
		self.plan_arm 	= manip 	# Sets the planning arm for visual purpose
		self.goal 		= joint_goal

		self.request = {
		  "basic_info" : {
			"n_steps" : n_steps,
			"manip" : manip, # see below for valid values
			"start_fixed" : True # i.e., DOF values at first timestep are fixed based on current robot state
		  },

		  "costs" : [
		  {
			"type" : "joint_vel", # joint-space velocity cost
			"params": {"coeffs" : [100,100,1]} # a list of length one is automatically expanded to a list of length n_dofs

		  },
		  {
			"type" : "collision",
			"params" : {
			  "coeffs" : [10000], # penalty coefficients. list of length one is automatically expanded to a list of length n_timesteps
			  "dist_pen" : [0.1], # robot-obstacle distance that penalty kicks in. expands to length n_timesteps

			  "continuous" : True
			}
		  },

		  {
			"type" : "collision",
			"params" : {
			  "coeffs" : [10000], # penalty coefficients. list of length one is automatically expanded to a list of length n_timesteps
			  "dist_pen" : [0.1], # robot-obstacle distance that penalty kicks in. expands to length n_timesteps

			  "continuous" : False
			}
		  }    
		  ],

		  "constraints" : [
		  {
			"type" : "joint", # joint-space target
			"params" : {"vals" : joint_goal} # length of vals = # dofs of manip

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

		return


	def get_robot(self):
		"""
		Function to get the current robot

		Return:
			robot<op::Environemnt::robot> 
		"""		
		return self.robot

	def get_manip(self, name):
		"""
		Returns a manipulator object givn a name

		Params:
			name <string> : Either "left_arm" or "right_arm"

		Return:
			manip <op::Environment::robot::manip>
		"""

		if 	 name == "left_arm" : return self.left_arm
		elif name == "right_arm" : return self.right_arm

	def set_manip(self, name, DOF):
		"""
		Sets the DOF for a given manipulator.
		Thereafter, a call to update the global DOF status of the robot will be done

		Params:
			name <string> 		: either 'left_arm' or 'right_arm'
			DOF <list><float> 	: Desired degree of freedom for manipulator
		""" 

		assert name == "left_arm" or name == "right_arm"
		arm_indices		= self.get_manip(name).GetArmIndices()
		self.get_robot().SetDOFValues(DOF, arm_indices)
		self.__update_DOF()

	def __update_DOF(self):
		"""
		Function is called to update the left and right arm DOF values
		"""
		self.left_arm_DOF 	= self.robot.GetDOFValues()[self.get_manip("left_arm").GetArmIndices()]		# Updates arm DOF 
		self.right_arm_DOF 	= self.robot.GetDOFValues()[self.get_manip("right_arm").GetArmIndices()]

	def optimize(self, manip, joint_target, algorithm):
		"""
		Optimization of the robot via initialization of motion through different way points
		We start by perform intermediate way point calculations using the OMPL planner with given algorithm
		Optimization of the trajectory is done through a call to TrajOpt

		Params:
			manip <string> 						: either 'left_arm' or 'right_arm'
			joint_target <list><list><float>	: return type from IK.get_joint_DOF representing the goal state of manip
			algorithm <string>					: OMPL_*** algorithm. If 'RRTConnect', then we are using the 'OML_RRTConnect' algorithm

		"""
		self.__update_DOF()		# Call to refresh DOF of the robot
		trajectory 			= self.__init_traj(manip=manip, joint_target=joint_target, algorithm=algorithm) # Performs initial OMPL_*** planning
		self.traj 			= []
		trajectory_interp 	= self.__interpolate(trajectory=trajectory, n=10)	# Interpolation between waypoints generated by OMPL for finer resolution

		self.__set_request(manip=manip, joint_goal=trajectory[-1], n_steps=len(trajectory_interp))
		self.request.update({"init_info" : {"type" : "given_traj", "data" : trajectory_interp}})				# Way point initialization after path planning
		jd 			= json.dumps(self.request) 					# convert dictionary into json-formatted string
		prob 		= trajoptpy.ConstructProblem(jd, self.env) 	# create object that stores optimization problem
		result 		= trajoptpy.OptimizeProblem(prob) 			# do Optimization

		if self.__check_safe(result.GetTraj()):
			pass
		else:
			raise Exception('No path is safe')
		self.traj 	= result.GetTraj().tolist()
				
		return

	def simulate(self):
		"""
		Call to simulate the movement of the desired manip to joint goal
		"""
		for t in self.traj:
			self.robot.SetDOFValues(t, self.robot.GetManipulator(self.plan_arm).GetArmIndices())
			time.sleep(0.1)

	def __check_safe(self, trajectory):
		"""
		Asserts a safe path by calling the collision checker through every iteration of the resultant trajectory

		Return:
			bool : If the path is safe then return True, else False
		"""
		kin = self.env.GetKinBody('dvrk')
		self.env.GetCollisionChecker().SetCollisionOptions(op.CollisionOptions.Contacts)

		for t in trajectory:			
			self.robot.SetDOFValues(t, self.robot.GetManipulator(self.plan_arm).GetArmIndices())
			flag = self.env.CheckCollision(kin.GetLinks()[3], kin.GetLinks()[6])		# Checks for collisions between 2 cylinder arms of the robot

			if flag == True:
				return False		# That means that collision happened
			time.sleep(0.05)
		return True

	def __init_traj(self, manip, joint_target, algorithm):
		"""
		Creates an initial trajectory plan between start and end goal poses
		"""
		# Issue with the init and goal configuratons -> Planner thinks that the arms are in collision

		Algo 		= "OMPL_" + algorithm
		planner 	= op.RaveCreatePlanner(self.env, Algo)		# Initializes a planner with algorithm
		simplifier 	= op.RaveCreatePlanner(self.env, 'OMPL_Simplifier')
		# self.env.GetCollisionChecker().SetCollisionOptions(op.CollisionOptions.Contacts)

		with self.env:
			arm_indx = self.get_manip(name=manip).GetArmIndices()
			self.get_robot().SetActiveDOFs(arm_indx)		# Plan for only the arm specified in manip
			self.get_robot().SetActiveDOFValues(self.right_arm_DOF)
			self.get_robot().SetActiveManipulator(self.get_manip(name=manip))

		params 		 = planner.PlannerParameters()				# Creates an empty param to be filled 
		params.SetRobotActiveJoints(self.robot)
		params.SetGoalConfig(joint_target[0])

		extraParams = ('<_nmaxiterations>{:d}</_nmaxiterations>'.format(10000))

		params.SetExtraParameters(extraParams)

		with self.env:
			with self.get_robot():
				print "Starting intial plan using {:s} algorithm".format(algorithm)
				traj = op.RaveCreateTrajectory(self.env, '')
				planner.InitPlan(self.get_robot(), params)
				result = planner.PlanPath(traj)
				assert result == op.PlannerStatus.HasSolution

				print 'Calling the OMPL_Simplifier for shortcutting.'
				simplifier.InitPlan(self.get_robot(), op.Planner.PlannerParameters())
				result = simplifier.PlanPath(traj)
				assert result == op.PlannerStatus.HasSolution

				trajectory = [traj.GetWaypoint(i).tolist() for i in range(traj.GetNumWaypoints())]
				return trajectory

	def __interpolate(self,trajectory, n=10):
		"""
		Takes in a trajectory in joint space, converts it to cartesian space using FK, performs linear interpolation between waypoints
		& maps result back to joint space using IK 

		This is done to provide TrajOpt with more points to perform its optimization
		"""
		from IK import dVRK_IK_simple
		ik 			= dVRK_IK_simple()
		waypoints 	= ik.get_endEffector_fromDOF(trajectory)		# cartesian space waypoints
		lst 		= []

		for i in range(len(waypoints) -1):
			start 	=	np.array(waypoints[i])
			end 	=	np.array(waypoints[i +1])
			lst 	+=	[list(start + ii*(end - start)/(n)) for ii in range(n +1)]

		return ik.get_joint_DOF(lst)

if __name__ == "__main__":	
	joint_start1 = [3.14/3, 3.14/4, 2]
	joint_start2 = [-3.14/5, 3.14/4, 0]
	manip 		 = "right_arm"

	planner = Motion_planning('env.xml', "right_arm")
	planner.init_collision_checker('pqp', [op.CollisionOptions.Contacts])

	planner.set_manip(name="left_arm", DOF=joint_start1)
	planner.set_manip(name="right_arm", DOF=joint_start2)

	IK_obj = IK.dVRK_IK_simple()                                # Creates an IK object 	
	endEff = IK_obj.get_endEffector_fromDOF([-3.14/2, 3.14/4, -8])
	joint_target = IK_obj.get_joint_DOF(endEff)     

	planner.optimize(manip, joint_target, algorithm="RRTConnect")
	planner.simulate()

	planner.optimize(manip, [joint_start2], algorithm="RRTConnect")
	planner.simulate()

	IPython.embed()
