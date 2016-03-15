import argparse
parser = argparse.ArgumentParser()
parser.add_argument("--interactive", action="store_true")
args = parser.parse_args()

import openravepy
import trajoptpy
import json
import time
import IK

env = openravepy.Environment()
env.StopSimulation()
env.Load("dvrk2.robot.xml")
# env.Load("../data/table.xml")

trajoptpy.SetInteractive(args.interactive) # pause every iteration, until you press 'p'. Press escape to disable further plotting
robot = env.GetRobots()[0]

joint_start = [0, 0, 0]
robot.SetDOFValues(joint_start, robot.GetManipulator('arm').GetArmIndices())

joint_endEffector = [3.14/4, 3.14/4, 0]
IK_obj = IK.dVRK_IK_simple(joint_endEffector)            # Creates an IK object 
joint_target = IK_obj.getDOF()                        # Gets DOF for target end effector pose


request = {
  "basic_info" : {
    "n_steps" : 20,
    "manip" : "arm", # see below for valid values
    "start_fixed" : True # i.e., DOF values at first timestep are fixed based on current robot state
  },
  "costs" : [
  {
    "type" : "joint_vel", # joint-space velocity cost
    "params": {"coeffs" : [1]} # a list of length one is automatically expanded to a list of length n_dofs
    # also valid: [1.9, 2, 3, 4, 5, 5, 4, 3, 2, 1]
  },
  {
    "type" : "collision",
    "params" : {
      "coeffs" : [20], # penalty coefficients. list of length one is automatically expanded to a list of length n_timesteps
      "dist_pen" : [0.025] # robot-obstacle distance that penalty kicks in. expands to length n_timesteps
    },    
  }
  ],
  "constraints" : [
  {
    "type" : "joint", # joint-space target
    "params" : {"vals" : joint_target } # length of vals = # dofs of manip
  }
  ],
  "init_info" : {
      "type" : "straight_line", # straight line in joint space.
      "endpoint" : joint_target
  }
}
s = json.dumps(request) # convert dictionary into json-formatted string
prob = trajoptpy.ConstructProblem(s, env) # create object that stores optimization problem
t_start = time.time()
result = trajoptpy.OptimizeProblem(prob) # do optimization
t_elapsed = time.time() - t_start
print "optimization took %.3f seconds"%t_elapsed
import IPython
IPython.embed()

from trajoptpy.check_traj import traj_is_safe
prob.SetRobotActiveDOFs() # set robot DOFs to DOFs in optimization problem
assert traj_is_safe(result.GetTraj(), robot) # Check that trajectory is collision free
