#!/usr/bin/env python
import argparse
import pickle
import threading
import time
from operator import truediv
from pathlib import Path

import numpy as np

from deoxys import config_root
from deoxys.franka_interface import FrankaInterface
from deoxys.utils import YamlConfig
from deoxys.utils.log_utils import get_deoxys_example_logger
from std_msgs.msg import String


import rospy
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotTrajectory
import actionlib

#import the action framework
from moveit_msgs.msg import ExecuteTrajectoryAction
import time
from commander import MoveitCommander
from geometry_msgs.msg import *

logger = get_deoxys_example_logger()


JOINTS = [
  "panda_joint1",
  "panda_joint2",
  "panda_joint3",
  "panda_joint4",
  "panda_joint5",
  "panda_joint6",
  "panda_joint7",
  "panda_finger_joint1",
  "panda_finger_joint2"
  ]

# NUM_INTERP_STEPS = 80


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--interface-cfg", type=str, default="charmander.yml")
    parser.add_argument(
        "--controller-cfg", type=str, default="joint-impedance-controller.yml"
    )

    # args = parser.parse_args()
    args,unknown = parser.parse_known_args()
    return args



class ExecuteTrajectoryServer:
    def __init__(self,args, robot_interface):
        self.server = actionlib.SimpleActionServer('deoxys_trajectory_executor', ExecuteTrajectoryAction, self.execute, False)
        self.server.start()
        self.robot_interface = robot_interface
        self.controller_cfg = YamlConfig(config_root + f"/{args.controller_cfg}").as_easydict()
        self.commander = MoveitCommander()
        self.plan = []

        # Joint State Goal Planner
        self.s1 = rospy.Subscriber("deoxys_bridge/plan_joint_goal", JointState, self.plan_to_joint_state)

        # Pose Goal Planner
        self.s2 = rospy.Subscriber("deoxys_bridge/plan_pose_goal", PoseStamped, self.plan_to_pose)

        # Execute Plan
        self.s3 = rospy.Subscriber("deoxys_bridge/execute_plan", String, self.execute_plan)

        # Joint State Goal Goto
        self.s4 = rospy.Subscriber("deoxys_bridge/goto_joint_goal", JointState, self.goto_joint_state)

        # Pose Goal Goto
        self.s5 = rospy.Subscriber("deoxys_bridge/goto_pose_goal", PoseStamped, self.goto_pose)

    def plan_to_pose(self,msg):
        robot_trajectory = self.commander.plan_to_pose_goal(msg)
        self.plan = robot_trajectory
        print("planned to pose")

    def plan_to_joint_state(self,msg):
        print("planning to joint state")
        robot_trajectory = self.commander.plan_to_joint_goal(msg)
        self.plan = robot_trajectory
        print("planned to joint state")

    def execute_plan(self,msg):
        print("... Executing Trajectory ")
        joint_trajectory = self.plan[1].joint_trajectory
        trajectory = []
        for joint_trajectory_point in joint_trajectory.points:
            trajectory.append(joint_trajectory_point.positions)
        self.move_to(trajectory)
        self.server.set_succeeded()
        print("Trajectory Successfully Executed")

    def goto_pose(self,msg):
        robot_trajectory = self.commander.plan_to_pose_goal(msg)
        self.plan = robot_trajectory
        self.execute_plan(self.plan)

    def goto_joint_state(self,msg):
        print("planning to joint state")
        robot_trajectory = self.commander.plan_to_joint_goal(msg)
        self.plan = robot_trajectory
        self.execute_plan(self.plan)



        


    def execute(self, goal):
        print("... Executing Trajectory ")
        joint_trajectory = goal.trajectory.joint_trajectory
        trajectory = []
        for joint_trajectory_point in joint_trajectory.points:
            trajectory.append(joint_trajectory_point.positions)
        self.move_to(trajectory)
        self.server.set_succeeded()
        print("Trajectory Successfully Executed")


    def move_to(self, trajectory):
        """
        Use joint impedance controller to move to a new joint position using interpolation.
        """
        robot_interface = self.robot_interface
        controller_cfg = self.controller_cfg

        while True:
            if len(robot_interface._state_buffer) > 0:
                if np.max(np.abs(np.array(robot_interface._state_buffer[-1].q))) < 1e-3:
                    print(
                        len(robot_interface._state_buffer),
                        np.array(robot_interface._state_buffer[-1].q),
                    )
                    continue
                else:
                    break


        current_q = robot_interface.last_q
        logger.info(f"Initial: {current_q}")
        jpos_steps = trajectory

        logger.info(f"Trajectory Length: {len(jpos_steps)}")
        logger.info(f"Current joints: {np.round(current_q, 3)}")
        # try to follow path

        controller_type = "JOINT_IMPEDANCE"
        for i, jpos_t in enumerate(jpos_steps):
            # print("target: ",np.round(jpos_t,4))
            # print("curret: ",np.round(robot_interface.last_q,4), "\n")

            # input("continue to next waypoint")
            if(self.server.is_preempt_requested()):
                print("trajectory cancelled")
                break
            
            action = list(jpos_t) + [-1.0] 
            logger.debug("step {}, action {}".format(i, np.round(action, 2)))
            robot_interface.control(
                controller_type=controller_type,
                action=action,
                controller_cfg=controller_cfg,
            )



def main():
    args = parse_args()
    rospy.init_node('trajectory_executor')
    robot_interface = FrankaInterface(
            config_root + f"/{args.interface_cfg}", use_visualizer=False
        )
    pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    rate = rospy.Rate(20) # 20hz
    server = ExecuteTrajectoryServer(args, robot_interface)

    
    

    while not rospy.is_shutdown():
        try:
            state_msg = JointState()
            state_msg.header.stamp = rospy.Time.now()
            position = [0.]*9
            position[:7] = robot_interface.last_q
            state_msg.name = JOINTS
            state_msg.position = position
            pub.publish(state_msg)
            

        except Exception as e:
            print(e)

        rate.sleep()

    

if __name__ == "__main__":
    main()
