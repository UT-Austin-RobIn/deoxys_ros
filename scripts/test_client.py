import rospy
import actionlib
from moveit_msgs.msg import ExecuteTrajectoryAction
from moveit_msgs.msg import ExecuteTrajectoryActionGoal
from moveit_msgs.msg import RobotTrajectory
from commander import MoveitCommander
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import time
from geometry_msgs.msg import *


JOINTS = [
  "panda_joint1",
  "panda_joint2",
  "panda_joint3",
  "panda_joint4",
  "panda_joint5",
  "panda_joint6",
  "panda_joint7",
  ]

def test_joint():
	rospy.init_node('test_client')
	# client = actionlib.SimpleActionClient('deoxys_trajectory_executor', ExecuteTrajectoryAction)
	# client.wait_for_server()
	commander = MoveitCommander()

	joint_target_publisher = rospy.Publisher("deoxys_bridge/goto_joint", JointState, queue_size=10)
	while joint_target_publisher.get_num_connections() < 1:
		continue

	goal = JointState()
	goal.name = JOINTS
	goal.position = commander.group.get_current_joint_values()
	goal.position[0] += 0.1
	print(goal)
	joint_target_publisher.publish(goal)

def test_pose():
	rospy.init_node('test_client')
	# client = actionlib.SimpleActionClient('deoxys_trajectory_executor', ExecuteTrajectoryAction)
	# client.wait_for_server()
	commander = MoveitCommander()

	pose_target_publisher = rospy.Publisher("deoxys_bridge/goto_pose", PoseStamped, queue_size=10)
	while pose_target_publisher.get_num_connections() < 1:
		continue

	goal  = commander.group.get_current_pose()
	goal.pose.position.z += 0.1
	print(goal)
	pose_target_publisher.publish(goal)
	

	rospy.spin()

if __name__ == '__main__':
	test_joint()
	time.sleep(1)
	test_pose()