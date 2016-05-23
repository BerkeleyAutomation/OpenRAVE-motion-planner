from openravepy import *
import IPython
import time

# start_config = [  0.80487864,  0.42326865, -0.54016693,  2.28895761,
#                  -0.34930645, -1.19702164]
# goal_config  = [  2.41349473, -1.43062044, -2.69016693,  2.12681216,
#                  -0.75643783, -1.52392537]

start_config = [-3.14/5, 3.14/4, 0]
goal_config = [-3.14/2, 3.14/4, 0]

# Setup the environment.
env = Environment()
env.StopSimulation()
env.Load("env.xml")
robot = env.GetRobot('dvrk')
manipulator = robot.GetManipulator('right_arm')

planner = RaveCreatePlanner(env, 'OMPL_RRTConnect')
simplifier = RaveCreatePlanner(env, 'OMPL_Simplifier')

joint_start1 = [3.14/3, 3.14/4, 1]
robot.SetDOFValues(joint_start1, robot.GetManipulator('left_arm').GetArmIndices())
robot.SetDOFValues(start_config, robot.GetManipulator('right_arm').GetArmIndices())

with env:
    robot.SetActiveDOFs([0,1,2])
    robot.SetActiveDOFValues(start_config)
    robot.SetActiveManipulator(manipulator)

# Setup the planning instance.
params = Planner.PlannerParameters()
params.SetRobotActiveJoints(robot)
params.SetGoalConfig(goal_config)

# Set the timeout and planner-specific parameters. You can view a list of
# supported parameters by calling: planner.SendCommand('GetParameters')
print 'Parameters:'
print planner.SendCommand('GetParameters')

params.SetExtraParameters('<range>0.02</range>')

collisionChecker = RaveCreateCollisionChecker(env,'pqp')
collisionChecker.SetCollisionOptions(CollisionOptions.Contacts)
env.SetCollisionChecker(collisionChecker)

with env:
    with robot:
        # Invoke the planner.
        print 'Calling the OMPL_RRTConnect planner.'
        traj = RaveCreateTrajectory(env, '')
        planner.InitPlan(robot, params)
        result = planner.PlanPath(traj)
        IPython.embed()

        # assert result == PlannerStatus.HasSolution  


        # # Shortcut the path.
        print 'Calling the OMPL_Simplifier for shortcutting.'
        simplifier.InitPlan(robot, Planner.PlannerParameters())
        result = simplifier.PlanPath(traj)
        assert result == PlannerStatus.HasSolution

        # # Time the trajectory.
        # print 'Timing trajectory'
        # result = planningutils.RetimeTrajectory(traj)
        # assert result == PlannerStatus.HasSolution

env.SetViewer('qtcoin')
viewer = env.GetViewer()
viewer.SetBkgndColor([.8, .85, .9])

robot.SetDOFValues(joint_start1, robot.GetManipulator('left_arm').GetArmIndices())
robot.SetDOFValues(start_config, robot.GetManipulator('right_arm').GetArmIndices())

trajectory = [traj.GetWaypoint(i).tolist() for i in range(traj.GetNumWaypoints())]

def simulate():
    kin = env.GetKinBody('dvrk')
    collisionChecker.SetCollisionOptions(CollisionOptions.Contacts)

    for t in trajectory:
        robot.SetDOFValues(t, robot.GetManipulator("right_arm").GetArmIndices())
        time.sleep(0.1)
        flag = env.CheckCollision(kin.GetLinks()[3], kin.GetLinks()[6])        # Checks for collisions between 2 cylinder arms of the robot
        print(flag)

IPython.embed()
