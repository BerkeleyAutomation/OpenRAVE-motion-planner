""" 
Script automatically calls Trajopt with OpenRave

"""
import openravepy as Rave
import trajoptpy as Traj
import json
import rospy
import roslib
import numpy as np
import IK
import time

class Planner:
	def __init__(self, arm_name, sim=None, visual=False):
		"""
		:param arm_name: Either "PSM1" or "PSM2"
		:param visual: Boolean - Turns on Rave visuals
		"""
		self.env = None
		self.robot = None
		self.manip = None
		self.request = None
		self.joint_val = None
		self.visual = False
		self.arm_name = arm_name

		assert arm_name == "PSM1" or arm_name == "PSM2"
		self.visual = visuals
		self.sim = sim

        if self.sim is None:	  
    		self.sim = simulator.Simulator()

	def set_environment(self, robot_file, env_file, viewer='qtcoin'):
		""" 
		Initalizes the environment with the robot model
		Loads the robot model automatically

		:param robot_file: robot model file
		:param env_file: environemnt model file
		"""    	
		self.evn = openrave.Environemnt()
		self.env.StopSimulation()
		self.env.Load(robot_file)
		self.env.Load(env_file)

		if self.visual == True:
			self.env.SetViewer(viewer) 
			self.env.SetBkgndColor([0.8, 0.85, 0.0])

		self.robot = self.env.GetRobots()[0]

	def set_constraints(self, tfx_target, n_steps=10, ignore_orientation=False, link_name=None, init_traj=None):
		"""
		Initializes constraint parameters

		:param tfx_target: end effector tfx_pose 
		"""
		xyz_target = tfx_target.position() 		# <<<<< to list
		quat_target = tfx_target.quaternion() 	# <<<<< to list

		self.request = {
            "basic_info" : {
                "n_steps" : n_steps,
                "manip" : str(self.manip.GetName()), 
                "start_fixed" : True 
                },
            "costs" : [
                {
                    "type" : "joint_vel",
                    "params": {"coeffs" : [1]} 
                    },
                {
                    "type" : "collision",
                    "params" : {
                        "coeffs" : [20], # 20
                        "continuous": False,
                        "dist_pen" : [0.05] # .025 
                        }
                    },
                {
                    "type" : "collision",
                    "params" : {
                        "coeffs" : [40], # 20
                        "continuous" : True,
                        "dist_pen" : [0.05] # .025 
                        }
                    },
                {
                        "type" : "pose",
                        "name" : "target_pose",
                        "params" : {"xyz" : xyz_target, 
                                    "wxyz" : quat_target,
                                    "link": link_name,
                                    "rot_coeffs" : rot_coeffs,
                                    "pos_coeffs" : [0,0,0],
                                    }
                        },
                ],
            "constraints" : [
                {
                    "type" : "pose",
                    "name" : "target_pose",
                    "params" : {"xyz" : xyz_target, 
                                "wxyz" : quat_target,
                                "link": link_name,
                                "rot_coeffs" : [0,0,0],
                                "pos_coeffs" : [1,1,1]
                                }
                     
                    },
                {
                    "type" : "cart_vel",
                    "name" : "cart_vel",
                    "params" : {
                                "max_displacement" : .02,
                                "first_step" : 0,
                                "last_step" : n_steps-1, #inclusive
                                "link" : link_name
                                },
                 }
                ],
            }

        print("Constraints initialized")
        return True

    def start_optimization(self):
    	"""
    	Call to begin optimization
    	"""
    	s = json.dumps(self.request) # convert dictionary into json-formatted string
		prob = trajoptpy.ConstructProblem(s, self.env) # create object that stores optimization problem
		t_start = time.time()
		result = trajoptpy.OptimizeProblem(prob) # do optimization
		t_elapsed = time.time() - t_start
		print "optimization took %.3f seconds"%t_elapsed

		assert __assert_route_safe(prob, result)	# Check for no collision

		return result.GetTraj()

	def __assert_route_safe(self, prob, result):
		from trajoptpy.check_traj import traj_is_safe
		prob.SetRobotActiveDOFs() # set robot DOFs to DOFs in optimization problem
		assert traj_is_safe(result.GetTraj(), self.robot) # Check that trajectory is collision free





