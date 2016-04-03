  import argparse
parser = argparse.ArgumentParser()
parser.add_argument("--interactive", action="store_true")
args = parser.parse_args()

import openravepy
import trajoptpy
import json
import time
import IK
import IPython

env = openravepy.Environment()
env.StopSimulation()
env.Load("env.xml")
# env.Load("../data/table.xml")

trajoptpy.SetInteractive(args.interactive) # pause every iteration, until you press 'p'. Press escape to disable further plotting
robot = env.GetRobots()[0]

joint_start = [0, 0, 0]
robot.SetDOFValues(joint_start, robot.GetManipulator('arm').GetArmIndices())

# joint_endEffector = [3.14/2 , 0, ]
joint_target = [3.14/2, 3.14/4 , -0.3]
IK_obj = IK.dVRK_IK_simple()                            # Creates an IK object 
joint_endEffector = IK_obj.get_endEffector_fromDOF(joint_target)
joint_target = IK_obj.getDOF(joint_endEffector)                        # Gets DOF for target end effector pose

print joint_target

request = {
  "basic_info" : {
    "n_steps" : 100,
    "manip" : "arm", # see below for valid values
    "start_fixed" : True # i.e., DOF values at first timestep are fixed based on current robot state
  },
  "costs" : [
  {
    "type" : "joint_vel", # joint-space velocity cost
    "params": {"coeffs" : [50]} # a list of length one is automatically expanded to a list of length n_dofs
    # also valid: [1.9, 2, 3, 4, 5, 5, 4, 3, 2, 1]
  },
  {
    "type" : "collision",
    "params" : {
      "coeffs" : [10000], # penalty coefficients. list of length one is automatically expanded to a list of length n_timesteps
      "dist_pen" : [0.025], # robot-obstacle distance that penalty kicks in. expands to length n_timesteps
      "continuous" : True
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
      "type" : "stationary", # straight line in joint space.
      "endpoint" : joint_target
  }
}
s = json.dumps(request) # convert dictionary into json-formatted string
prob = trajoptpy.ConstructProblem(s, env) # create object that stores optimization problem
t_start = time.time()
result = trajoptpy.OptimizeProblem(prob) # do optimization
t_elapsed = time.time() - t_start
print "optimization took %.3f seconds"%t_elapsed
traj = result.GetTraj()

from trajoptpy.check_traj import traj_is_safe
prob.SetRobotActiveDOFs() # set robot DOFs to DOFs in optimization problem
assert traj_is_safe(result.GetTraj(), robot) # Check that trajectory is collision free

joint_angles = result.GetTraj()

# Visualize across all joint_angles
for i in range(len(joint_angles)):
  robot.SetDOFValues(joint_angles[i], robot.GetManipulator('<arm name>').GetArmIndices()) # iterates across all joint angles
  time.sleep(1)

IPython.embed()
