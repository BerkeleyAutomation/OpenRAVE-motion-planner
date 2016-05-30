import openravepy as op
import IPython

if __name__ == "__main__":
	env = op.Environment()
	env.SetViewer('qtcoin')

	joint_start1 = [3.14/3, 3.14/4, 2]
	joint_start2 = [-3.14/5, 3.14/4, 0]
	joint_goal	 = [-3.14/2, 3.14/4, -8]

	with env:
		env.StopSimulation()
		env.Load("env.xml")
		robot 		= env.GetRobots()[0]
		left_arm 	= robot.GetManipulator('left_arm')
		right_arm	= robot.GetManipulator('right_arm')
		robot.SetDOFValues(joint_start1, left_arm.GetArmIndices())
		robot.SetDOFValues(joint_start2, right_arm.GetArmIndices())

	IPython.embed()