import rospy
import actionlib
from moveit_msgs.msg import ExecuteTrajectoryAction
from moveit_msgs.msg import ExecuteTrajectoryActionGoal
from moveit_msgs.msg import RobotTrajectory
from commander import MoveitCommander
'''
ExecuteTrajectoryAction', '_ExecuteTrajectoryActionFeedback', '_ExecuteTrajectoryActionGoal', '_ExecuteTrajectoryActionResult', '_ExecuteTrajectoryFeedback', '_ExecuteTrajectoryGoal', '_ExecuteTrajectoryResult'
'''


if __name__ == '__main__':
	rospy.init_node('execute_trajectory_client')
	client = actionlib.SimpleActionClient('traj_exeww', ExecuteTrajectoryAction)
	client.wait_for_server()
	commander = MoveitCommander()
	plan = commander.plan_to_pose_goal()
	# breakpoint()
	goal = ExecuteTrajectoryAction().action_goal.goal
	goal.trajectory = plan[1]
	
	# breakpoint()
	# goal = RobotTrajectory()
	client.send_goal(goal)
	client.wait_for_result(rospy.Duration.from_sec(5.0))
