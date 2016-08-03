# robot-IK-model-

This repository contains a motion planner for a model dVRK using OMPL and Trajopt
The dVRK robot model and obstacles are found in the env.xml file

Motion planning is done on a single arm which can be controlled by setting the manip parameter to either "left_arm" or "right_arm"
We can define the joint angles for each arm by setting the 3DOF values of "joint_start1" and "joint_start2"
The robot then plans from its starting joint DOF to the target joint DOF

The script IK.py contains the IK and FK analytical solutions for each arm of the robot. 

To launch the planner:
''' sh
$ python raveInterface.py
'''
