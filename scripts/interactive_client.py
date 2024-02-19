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
	client = actionlib.SimpleActionClient('deoxys_trajectory_executor', ExecuteTrajectoryAction)
	client.wait_for_server()
	commander = MoveitCommander()


	user_in = "none"
	while user_in != "q":
		user_in = input("Please select an action of the shape [axis distance], q to quit:\n")
		if(user_in == "q"): break
		axis,distance = user_in.split(" ")
		distance = float(distance)
		target_pose = commander.group.get_current_pose()

		if axis == 'x':
			target_pose.pose.position.x += distance
		if axis == 'y':
			target_pose.pose.position.y += distance
		if axis == 'z':	
			target_pose.pose.position.z += distance


		plan = commander.plan_to_pose_goal(target_pose)
		goal = ExecuteTrajectoryAction().action_goal.goal
		goal.trajectory = plan[1]
		
		client.send_goal(goal)
		client.wait_for_result(rospy.Duration.from_sec(5.0))
		print("Trajectory complete")
