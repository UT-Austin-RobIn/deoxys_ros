import rospy
import actionlib
from moveit_msgs.msg import ExecuteTrajectoryAction
from moveit_msgs.msg import ExecuteTrajectoryActionGoal
from moveit_msgs.msg import RobotTrajectory
from commander import MoveitCommander

WORLD_FRAME = "base"
EE_FRAME = "right_gripper_tip"
GROUP_NAME = "right_arm"

if __name__ == '__main__':
	rospy.init_node('execute_trajectory_client')
	client = actionlib.SimpleActionClient('deoxys_trajectory_executor', ExecuteTrajectoryAction)
	client.wait_for_server()
	commander = MoveitCommander()

	current_pose = commander.group.get_current_pose()
	target_pose = current_pose
	target_pose.pose.position.z += 0.2

	plan = commander.plan_to_pose_goal(target_pose)
	goal = ExecuteTrajectoryAction().action_goal.goal
	goal.trajectory = plan[1]
	
	client.send_goal(goal)
	client.wait_for_result(rospy.Duration.from_sec(5.0))
