import rospy
import actionlib
from moveit_msgs.msg import ExecuteTrajectoryAction
from moveit_msgs.msg import ExecuteTrajectoryActionGoal
from moveit_msgs.msg import RobotTrajectory
from commander import MoveitCommander

'''
This is a utility script for messing around with the end effector.
Take example input "z -.05", the robot ee will move 5 cm downwards
'''


if __name__ == '__main__':
	rospy.init_node('execute_trajectory_client')

	# Simple action client executes trajectories which you can create using the MoveItCommander
	client = actionlib.SimpleActionClient('deoxys_trajectory_executor', ExecuteTrajectoryAction)
	client.wait_for_server()
	commander = MoveitCommander()

	print("Welcome to the interactive client, actions are input in the shape [axis distance]")
	print("There are 3 axis: x,y,z, and distances are measured in meters")
	print("An example is 'z .1', which would move the end effector 0.1m in the z direction")


	user_in = "none"
	while user_in != "q":
		user_in = input("Please select an action of the shape [axis distance], q to quit:\n")
		if(user_in == "q"): break
		axis,distance = user_in.split(" ")
		distance = float(distance)

		# Get the post stamped of the end effector
		target_pose = commander.group.get_current_pose()

		if axis == 'x':
			target_pose.pose.position.x += distance
		if axis == 'y':
			target_pose.pose.position.y += distance
		if axis == 'z':	
			target_pose.pose.position.z += distance

		# Creating the plan from the new target pose
		# See documentation about actions: http://wiki.ros.org/actionlib
		plan = commander.plan_to_pose_goal(target_pose)
		goal = ExecuteTrajectoryAction().action_goal.goal
		goal.trajectory = plan[1]
		
		client.send_goal(goal)
		client.wait_for_result(rospy.Duration.from_sec(5.0))
		print("Trajectory complete")
