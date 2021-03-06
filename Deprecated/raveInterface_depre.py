import argparse
parser = argparse.ArgumentParser()
parser.add_argument("--interactive", action="store_true")
<<<<<<< HEAD
# parser.add_argument("--collision", action="pqp")
args = parser.parse_args()

import openravepy as op
=======
args = parser.parse_args()

import openravepy
>>>>>>> 0d7cac2c48fdfead3e63707083f26a33f2233e13
import trajoptpy
import json
import time
import IK
import IPython
from math import pi
<<<<<<< HEAD
import numpy as np

env = op.Environment()
=======

env = openravepy.Environment()
>>>>>>> 0d7cac2c48fdfead3e63707083f26a33f2233e13
env.StopSimulation()
env.Load("env.xml")
# env.Load("../data/table.xml")

trajoptpy.SetInteractive(args.interactive) # pause every iteration, until you press 'p'. Press escape to disable further plotting
robot = env.GetRobots()[0]
<<<<<<< HEAD
collisionChecker = op.RaveCreateCollisionChecker(env,'pqp')
collisionChecker.SetCollisionOptions(op.CollisionOptions.Distance|op.CollisionOptions.Contacts)
env.SetCollisionChecker(collisionChecker)

joint_start1 = [3.14/3, 3.14/4, 0]
robot.SetDOFValues(joint_start1, robot.GetManipulator('left_arm').GetArmIndices())

joint_start2 = [-3.14/4, 3.14/4, 0]
=======
joint_start1 = [3.14/3, 3.14/4, 0]
robot.SetDOFValues(joint_start1, robot.GetManipulator('left_arm').GetArmIndices())

joint_start2 = [-3.14/5, 3.14/4, 0]
>>>>>>> 0d7cac2c48fdfead3e63707083f26a33f2233e13
robot.SetDOFValues(joint_start2, robot.GetManipulator('right_arm').GetArmIndices())

# manip = robot.SetActiveManipulator('right_arm')
time.sleep(0.1)                           # Give time for the environment to update

IK_obj = IK.dVRK_IK_simple()                                # Creates an IK object 
<<<<<<< HEAD
endEff = IK_obj.get_endEffector_fromDOF([-3.14/2, 3.14/4, 0])
joint_target = IK_obj.get_joint_DOF(endEff) 
  
manip = "right_arm"

startEff = IK_obj.get_endEffector_fromDOF(joint_start2)
print np.linalg.norm(np.array(endEff) - np.array(startEff))
# IPython.embed()
request = {
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
=======
joint_target = [[-pi/2, 3.14/4, .5]]                  # Set all waypoints here
# joint_target = IK_obj.getDOF(joint_endEffector)             # Gets DOF for target end effector pose

print joint_target

request = {
  "basic_info" : {
    "n_steps" : 20,
    "manip" : "right_arm", # see below for valid values
    "start_fixed" : True # i.e., DOF values at first timestep are fixed based on current robot state
  },
  "costs" : [
  {
    "type" : "joint_vel", # joint-space velocity cost
    "params": {"coeffs" : [10]} # a list of length one is automatically expanded to a list of length n_dofs
    # also valid: [1.9, 2, 3, 4, 5, 5, 4, 3, 2, 1]
  },
  # {
  #   "type" : "collision",
  #   "params" : {
  #     "coeffs" : [20], # penalty coefficients. list of length one is automatically expanded to a list of length n_timesteps
  #     "dist_pen" : [0.04], # robot-obstacle distance that penalty kicks in. expands to length n_timesteps
  #     "continuous" : True
  #   }
  # },
  {
    "type" : "collision",
    "params" : {
      "coeffs" : [20], # penalty coefficients. list of length one is automatically expanded to a list of length n_timesteps
>>>>>>> 0d7cac2c48fdfead3e63707083f26a33f2233e13
      "dist_pen" : [0.4], # robot-obstacle distance that penalty kicks in. expands to length n_timesteps
      "continuous" : False
    }
  }    
  ],
<<<<<<< HEAD

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

  ],
  "init_info" : {
      "type" : "straight_line", # straight line in joint space.
      "endpoint" : [-3.14/4, 3.14/4, -10]
      # "endpoint" : [-3.14/4, 3.14/4, -5]
      # "type" : "stationary"
=======
  "constraints" : [
  {
    "type" : "joint", # joint-space target
    "params" : {"vals" : joint_target[-1]} # length of vals = # dofs of manip
  }
  ],
  "init_info" : {
      "type" : "straight_line", # straight line in joint space.
      "endpoint" : joint_target[0]
>>>>>>> 0d7cac2c48fdfead3e63707083f26a33f2233e13
  }
}
s = json.dumps(request) # convert dictionary into json-formatted string
prob = trajoptpy.ConstructProblem(s, env) # create object that stores optimization problem
t_start = time.time()
result = trajoptpy.OptimizeProblem(prob) # do optimization
t_elapsed = time.time() - t_start
print "optimization took %.3f seconds"%t_elapsed
traj = result.GetTraj()
<<<<<<< HEAD
  
# IPython.embed()
from trajoptpy.check_traj import traj_is_safe
prob.SetRobotActiveDOFs() # set robot DOFs to DOFs in optimization problem
# for t in traj:
#   print t
def f(x):
  # print "origin" + str(x)
  # print "new" + str(np.append(x, joint_start1))
  return np.append(x, joint_start1)
robot_traj = map(f, traj)
# print robot_traj
print "[IS SAFE]: " + str(traj_is_safe(robot_traj, robot, 200)) # Check that trajectory is collision free
=======

# IPython.embed()
from trajoptpy.check_traj import traj_is_safe
prob.SetRobotActiveDOFs() # set robot DOFs to DOFs in optimization problem
assert traj_is_safe(result.GetTraj(), robot) # Check that trajectory is collision free
>>>>>>> 0d7cac2c48fdfead3e63707083f26a33f2233e13

joint_angles = result.GetTraj()

# Visualize across all joint_angles
# for i in range(len(joint_angles)):
#   robot.SetDOFValues(joint_angles[i], robot.GetManipulator('<arm name>').GetArmIndices()) # iterates across all joint angles
#   time.sleep(1)

<<<<<<< HEAD
# IPython.embed()
env.SetViewer('qtcoin')
viewer = env.GetViewer()
viewer.SetBkgndColor([.8, .85, .9])
robot = env.GetRobots()[0]
joint_start1 = [3.14/3, 3.14/4, 0]
robot.SetDOFValues(joint_start1, robot.GetManipulator('left_arm').GetArmIndices())

joint_start2 = [-3.14/5, 3.14/4, 0]
robot.SetDOFValues(joint_start2, robot.GetManipulator('right_arm').GetArmIndices())

def simulate():
  for t in traj:
    robot.SetDOFValues(t, robot.GetManipulator(manip).GetArmIndices())
    time.sleep(0.1)
simulate()

=======
>>>>>>> 0d7cac2c48fdfead3e63707083f26a33f2233e13
IPython.embed()
