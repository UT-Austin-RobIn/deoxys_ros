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
from deoxys.utils import transform_utils
from deoxys.experimental.motion_utils import reset_joints_to
from receive_hit_start import HitReceiver
from std_msgs.msg import String, Float64, Bool
from deoxys.utils.config_utils import (get_default_controller_config,
                                       verify_controller_config)



import rospy
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotTrajectory
from geometry_msgs.msg import Pose
import actionlib

#import the action framework
from moveit_msgs.msg import ExecuteTrajectoryAction, ExecuteTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped
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
# Send a pose delta, it will not path to the pose you send but rather interpret the position as an offset in world frame
def osc_move(robot_interface, controller_type, controller_cfg, target_pose, num_steps):
    target_pos, target_quat = target_pose
    target_axis_angle = transform_utils.quat2axisangle(target_quat)

    for _ in range(num_steps):
        current_rot, current_pos = robot_interface.last_eef_rot_and_pos
        current_pose = robot_interface.last_eef_pose
        current_pos = current_pose[:3, 3:]
        current_rot = current_pose[:3, :3]
        current_quat = transform_utils.mat2quat(current_rot)
        if np.dot(target_quat, current_quat) < 0.0:
            current_quat = -current_quat
        quat_diff = transform_utils.quat_distance(target_quat, current_quat)
        current_axis_angle = transform_utils.quat2axisangle(current_quat)
        axis_angle_diff = transform_utils.quat2axisangle(quat_diff)
        action_pos = (target_pos - current_pos).flatten() * 100
        action_axis_angle = axis_angle_diff.flatten() * 1
        action_pos = np.clip(action_pos, -1.0, 1.0)
        action_axis_angle = np.clip(action_axis_angle, -0.5, 0.5)

        action = action_pos.tolist() + action_axis_angle.tolist() + [-1.0]
        # logger.info(f"Axis angle action {action_axis_angle.tolist()}")
        # print(np.round(action, 2))
        robot_interface.control(
            controller_type=controller_type,
            action=action,
            controller_cfg=controller_cfg,
        )
        # pos_error_norm = np.linalg.norm((target_pos - current_pos).flatten())
        # ori_error_norm = np.linalg.norm((action_axis_angle).flatten())
        #Format the float to 5 decimal places
        # pos_error_norm = "{:.5f}".format(pos_error_norm)
        # ori_error_norm = "{:.5f}".format(ori_error_norm)
        # print(f"Error norm: {pos_error_norm}, {ori_error_norm}") 
    # logger.info(f'pos errors:{(target_pos - current_pos).flatten()}')
    logger.info(f'pos error norm:{np.linalg.norm((target_pos - current_pos).flatten())}')
    # logger.info(f'ori errors:{action_axis_angle}')
    # logger.info(f'ori error norm:{np.linalg.norm((action_axis_angle).flatten())}')
    return action

def move_to_target_pose(
    robot_interface,
    controller_type,
    controller_cfg,
    target_delta_pose,
    num_steps,
    num_additional_steps,
    interpolation_method,
    ):
    while robot_interface.state_buffer_size == 0:
        logger.warn("Robot state not received")
        time.sleep(0.5)

    target_delta_pos, target_delta_axis_angle = (
        target_delta_pose[:3],
        target_delta_pose[3:],
    )
    current_ee_pose = robot_interface.last_eef_pose
    current_pos = current_ee_pose[:3, 3:]
    current_rot = current_ee_pose[:3, :3]
    current_quat = transform_utils.mat2quat(current_rot)
    current_axis_angle = transform_utils.quat2axisangle(current_quat)

    target_pos = np.array(target_delta_pos).reshape(3, 1) + current_pos

    target_axis_angle = np.array(target_delta_axis_angle) + current_axis_angle

    logger.info(f"Before conversion {target_axis_angle}")
    target_quat = transform_utils.axisangle2quat(target_axis_angle)
    target_pose = target_pos.flatten().tolist() + target_quat.flatten().tolist()

    if np.dot(target_quat, current_quat) < 0.0:
        current_quat = -current_quat
    target_axis_angle = transform_utils.quat2axisangle(target_quat)
    logger.info(f"After conversion {target_axis_angle}")
    current_axis_angle = transform_utils.quat2axisangle(current_quat)

    start_pose = current_pos.flatten().tolist() + current_quat.flatten().tolist()

    osc_move(
        robot_interface,
        controller_type,
        controller_cfg,
        (target_pos, target_quat),
        num_steps,
    )
    osc_move(
        robot_interface,
        controller_type,
        controller_cfg,
        (target_pos, target_quat),
        num_additional_steps,
    )
    


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
        self.client = actionlib.SimpleActionClient('deoxys_trajectory_executor', ExecuteTrajectoryAction)
        self.osc_done_pub = rospy.Publisher("deoxys_bridge/osc_done", Bool, queue_size=2)
        # Create a PoseStamped server
        self.server.start()
        self.robot_interface = robot_interface
        self.controller_cfg = YamlConfig(config_root + f"/{args.controller_cfg}").as_easydict()
        self.osc_controller_cfg = YamlConfig(config_root + "/osc-pose-controller.yml").as_easydict()
        self.compliant_cfg = YamlConfig(config_root + "/compliant-joint-impedance-controller.yml").as_easydict()
        
        self.commander = MoveitCommander()
        self.target_grip = 0.0
        self.compliant = False
        self.plan = []
        self.prev_control_mode = "JOINT_IMPEDANCE"

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
        
        # Grip Goal Goto
        self.s6 = rospy.Subscriber("deoxys_bridge/goto_grip_goal", Float64, self.set_target_grip)
        
        # Activate Compliance
        self.s7 = rospy.Subscriber("deoxys_bridge/set_compliance", Bool, self.set_compliance)
        
        # Perform OSC Move
        # self.s8 = rospy.Subscriber("deoxys_bridge/osc_move", PoseStamped, self.osc_move_to) 
        
        # Perform OSC Move Delta (cartesian move only)
        #TODO make this interactive as a service or by publishing some status when finished
        self.s9 = rospy.Subscriber("deoxys_bridge/osc_move_delta", PoseStamped, self.osc_move_delta) 

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
        print("ExecutePlan")
        print("... Executing Trajectory ")
        joint_trajectory = self.plan[1].joint_trajectory
        trajectory = []
        for joint_trajectory_point in joint_trajectory.points:
            trajectory.append(joint_trajectory_point.positions)
        self.move_to(trajectory)
        self.server.set_succeeded()
        self.osc_done_pub.publish(True)
        print("Trajectory Successfully Executed (execute_plan)")

    def goto_pose(self,msg):
        print("Going to Pose")
        print(msg)
        robot_trajectory = self.commander.plan_to_pose_goal(msg)
        self.plan = robot_trajectory
        self.execute_plan(self.plan)

    def goto_joint_state(self,msg):
        print("planning to joint state")
        robot_trajectory = self.commander.plan_to_joint_goal(msg)
        self.plan = robot_trajectory
        self.execute_plan(self.plan)

    def set_target_grip(self, msg: Float64):
        print("Target grip set to: ", msg.data)
        prev_grip = self.target_grip
        self.target_grip = abs(msg.data)

    def set_compliance(self, msg: Bool):
        print("Compliance set to: ", msg.data)
        self.compliant = msg.data
        self.move_to(RobotTrajectory())
        


    def execute(self, goal: ExecuteTrajectoryGoal):
        print("... Executing Trajectory ")
        joint_trajectory = goal.trajectory.joint_trajectory
        trajectory = []
        for joint_trajectory_point in joint_trajectory.points:
            trajectory.append(joint_trajectory_point.positions)
        self.move_to(trajectory)
        self.server.set_succeeded()
        self.osc_done_pub.publish(True)
        print("Trajectory Successfully Executed (execute)")


    def move_to(self, trajectory: RobotTrajectory):
        """
        Use joint impedance controller to move to a new joint position using interpolation.
        """
        print("MoveTo called")
        robot_interface = self.robot_interface
        controller_cfg = self.controller_cfg
    
        if(self.compliant):
            rate = rospy.Rate(6)
            controller_cfg = self.compliant_cfg
            for i in range(5):
                robot_interface.control(
                    controller_type="JOINT_IMPEDANCE",
                    action=list(robot_interface.last_q) + [self.target_grip],
                    controller_cfg=controller_cfg,
                )
                rate.sleep()
            return

        while not rospy.is_shutdown():
            if len(robot_interface._state_buffer) > 0:
                if np.max(np.abs(np.array(robot_interface._state_buffer[-1].q))) < 1e-3:
                    print(
                        len(robot_interface._state_buffer),
                        np.array(robot_interface._state_buffer[-1].q),
                    )
                    continue
                else:
                    break

        controller_type = "JOINT_IMPEDANCE"
        current_q = robot_interface.last_q
        logger.info(f"Initial: {current_q}")
        jpos_steps = trajectory

        logger.info(f"Trajectory Length: {len(jpos_steps)}")
        logger.info(f"Current joints: {np.round(current_q, 3)}")
        
        print(f"Trajectory Length: {len(jpos_steps)}")
        print(f"Current joints: {np.round(current_q, 3)}")
        
        # when switching controller, command current pose briefly
        if self.prev_control_mode == "OSC_POSE":
            for i in range(20):
                action = list(current_q) + [self.target_grip]
                robot_interface.control(
                    controller_type=controller_type,
                    action=action,
                    controller_cfg=controller_cfg,
                )
                print('switching mode')
                time.sleep(0.05)     
            
                
        
        # try to follow path
        # rate = rospy.Rate(20)
        for i, jpos_t in enumerate(jpos_steps):
            # print("target: ",np.round(jpos_t,4))
            # print("curret: ",np.round(robot_interface.last_q,4), "\n")

            # input("continue to next waypoint")
            if(self.server.is_preempt_requested()):
                print("trajectory cancelled")
                break
            
            action = list(jpos_t) + [self.target_grip]
            # print(self.target_grip)
            # logger.debug("step {}, action {}".format(i, np.round(action, 2)))
            # print("step {}, action {}".format(i, np.round(action, 2)))
            robot_interface.control(
                controller_type=controller_type,
                action=action,
                controller_cfg=controller_cfg,
            )
            
        # jpos_t = jpos_steps[-1]      
        # for i in range(10):
            
        #     if(self.server.is_preempt_requested()):
        #         print("trajectory cancelled")
        #         break
            
        #     action = list(jpos_t) + [self.target_grip]
        #     robot_interface.control(
        #         controller_type=controller_type,
        #         action=action,
        #         controller_cfg=controller_cfg,
        #     )     
        self.prev_control_mode = "JOINT_IMPEDANCE"
    # def osc_move_to(self, msg: PoseStamped):
    #     print("oscmoveto called")
    #     controller_type = "OSC_POSE"
    #     target_pose = msg.pose
    #     num_steps = 1
    #     robot_interface = self.robot_interface
    #     controller_cfg = self.osc_controller_cfg
    #     target_pos = np.array([target_pose.position.x, target_pose.position.y, target_pose.position.z])
    #     target_quat = np.array([target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w])
    #     target_axis_angle = transform_utils.quat2axisangle(target_quat)

        
    #     for _ in range(num_steps):
            
            
    #         action_pos = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
    #         action_quat = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
    #         action_axis_angle = transform_utils.quat2axisangle(action_quat)
    #         action_pos = np.clip(action_pos, -1.0, 1.0)
    #         action_axis_angle = np.clip(action_axis_angle, -0.5, 0.5)

    #         action = action_pos.tolist() + action_axis_angle.tolist() + [-1.0]
    #         # logger.info(f"OSC Axis angle action {action_axis_angle.tolist()}")
    #         # logger.info(f"OSC Pos action {action_pos.tolist()}")
    #         print(f"OSC Axis angle action {action_axis_angle.tolist()}")
    #         print(f"OSC Pos action {action_pos.tolist()}")
    #         # print(np.round(action, 2))
    #         robot_interface.control(
    #             controller_type=controller_type,
    #             action=action,
    #             controller_cfg=controller_cfg,
    #         )
    #     print("OSC Move Complete")
    #     self.osc_done_pub.publish(True)
        
    def osc_move_delta(self, msg: PoseStamped):
        print("osc_move_delta called")
        controller_type = "OSC_POSE"
        robot_interface = self.robot_interface
        controller_cfg = get_default_controller_config(controller_type)
        controller_cfg['action_scale']['translation'] = 0.01
        controller_cfg['Kp']['translation'] = [200.]*3
        # breakpoint()
        
        dx = msg.pose.position.x
        dy = msg.pose.position.y
        dz = msg.pose.position.z
        dqx = msg.pose.orientation.x
        dqy = msg.pose.orientation.y
        dqz = msg.pose.orientation.z
        dqw = msg.pose.orientation.w
        
        axis_angle_delta = transform_utils.quat2axisangle(np.array([dqx, dqy, dqz, dqw]))
        
        # drx = axis_angle_delta[0]
        # dry = axis_angle_delta[1]
        # drz = axis_angle_delta[2]
        
        move_to_target_pose(
            robot_interface,
            controller_type,
            controller_cfg,
            # target_delta_pose=[dx, dy, dz, drx, dry, drz],
            target_delta_pose=[dx, dy, dz, 0, 0, 0],
            num_steps=80,
            num_additional_steps=40,
            interpolation_method="linear",
        )
        
        self.prev_control_mode = "OSC_POSE"
        self.osc_done_pub.publish(True)

    def test_local_cartesian(self):
        R_y = [1.414,1.414,0.0] # moves in between x and y direction
        controller_type = "OSC_POSE"
        robot_interface = self.robot_interface
        controller_cfg = get_default_controller_config(controller_type)

        reset_joint_positions = [
            0.09162008114028396,
            -0.19826458111314524,
            -0.01990020486871322,
            -2.4732269941140346,
            -0.01307073642274261,
            2.30396583422025,
            0.8480939705504309,
        ]
        
        reset_joints_to(
            robot_interface,
            reset_joint_positions,)

        osc_delta_forward = PoseStamped()
        osc_delta_forward.pose.position.x = R_y[0] * 0.065
        osc_delta_forward.pose.position.y = R_y[1] * 0.065
        osc_delta_forward.pose.position.z = R_y[2] * 0.065
        
        osc_delta_backward = PoseStamped()
        osc_delta_backward.pose.position.x = -R_y[0] * 0.065
        osc_delta_backward.pose.position.y = -R_y[1] * 0.065
        osc_delta_backward.pose.position.z = -R_y[2] * 0.065
        
        self.osc_move_delta(osc_delta_forward)

        self.osc_move_delta(osc_delta_backward)

        robot_interface.close()
        print("OSC Move Complete")
        self.osc_done_pub.publish(True)


def main():
    args = parse_args()
    print("\n\n\n\nStarting Deoxys_Server")
    print(args)
    #TODO: UNDO ARGS OVERRIDE
    args.controller_cfg='joint-impedance-controller.yml'
    args.interface_cfg='charmander.yml'
    
    
    rospy.init_node('trajectory_executor')
    robot_interface = FrankaInterface(
            config_root + f"/{args.interface_cfg}", use_visualizer=False, has_gripper=True
        )
    pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    rate = rospy.Rate(40) # 20hz
    server = ExecuteTrajectoryServer(args, robot_interface)

    receiver = HitReceiver(robot_interface)

    while not rospy.is_shutdown():
        try:
            state_msg = JointState()
            state_msg.header.stamp = rospy.Time.now()
            position = [0.]*9
            position[:7] = robot_interface.last_q
            position[7:] = [0 if robot_interface.last_gripper_q == None else robot_interface.last_gripper_q] * 2
            # print(robot_interface.last_q)
            # print(robot_interface.last_gripper_q)
            # print("\n\n")
            state_msg.name = JOINTS
            state_msg.position = position
            pub.publish(state_msg)
            

        except Exception as e:
            print(e)

        rate.sleep()

    
if __name__ == "__main__":
    main()
    


# print(len(robot_interface._state_buffer))
# # print(len(robot_interface._state_buffer))
# if(len(robot_interface._state_buffer) > 0):
#     # print(robot_interface._state_buffer[-1])
    
    
    # tau_ext_hat_filtered = np.array(robot_interface._state_buffer[-1].tau_ext_hat_filtered, dtype=float)
    # tau_ext_hat_filtered_scaled = int(tau_ext_hat_filtered[6] * 30)
    # tau_ext_hat_filtered_decimal = str(int(tau_ext_hat_filtered[6] * 100))
    # tau_ext_hat_filtered_decimal = " " * (5 - len(tau_ext_hat_filtered_decimal)) + tau_ext_hat_filtered_decimal
    # if(tau_ext_hat_filtered_scaled > 0):
    #     print(tau_ext_hat_filtered_decimal, ", ", "#" * tau_ext_hat_filtered_scaled)
    # else:
    #     print(tau_ext_hat_filtered_decimal, ", ", "." * (tau_ext_hat_filtered_scaled * -1))
    # if robot_interface._state_buffer[-1].tau_ext_hat_filtered[6] < -.4:
    #     print("HIT")
    

    # tau_J = np.array(robot_interface._state_buffer[-1].tau_J, dtype=float)
    # tau_J_scaled = int(tau_J[6] * 30)
    # tau_J_decimal = str(int(tau_J[6] * 100))
    # tau_J_decimal = " " * (5 - len(tau_J_decimal)) + tau_J_decimal
    
    # tau_J_d = np.array(robot_interface._state_buffer[-1].tau_J_d, dtype=float)
    # tau_J_d_scaled = int(tau_J_d[6] * 30)
    # tau_J_d_decimal = str(int(tau_J_d[6] * 100))
    # tau_J_d_decimal = " " * (5 - len(tau_J_d_decimal)) + tau_J_d_decimal
    # if(tau_J_d_scaled > 0):
    #     print(tau_J_d_decimal, ", ", "H" * tau_J_d_scaled)
    # else:
    #     print(tau_J_d_decimal, ", ", "O" * (tau_J_d_scaled * -1))
    # if(tau_J_scaled > 0):
    #     print(tau_J_decimal, ", ", "H" * tau_J_scaled)
    # else:
    #     print(tau_J_decimal, ", ", "O" * (tau_J_scaled * -1))
