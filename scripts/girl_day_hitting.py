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

NUM_INTERP_STEPS = 80


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--interface-cfg", type=str, default="charmander.yml")
    parser.add_argument(
        "--controller-cfg", type=str, default="joint-impedance-controller.yml"
    )
    parser.add_argument(
        "--direction", type=int, default=0
    )

    args = parser.parse_args()
    return args


if __name__ == '__main__':
    args = parse_args()
    direction = args.direction

    robot_interface = FrankaInterface(
                config_root + f"/{args.interface_cfg}", use_visualizer=False
            )

    controller_cfg = YamlConfig(config_root + f"/{args.controller_cfg}").as_easydict()


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

    trajectory = [current_q]
    steps = 20
    for i in range(steps):
        next_target = trajectory[-1].copy()
        if direction == 0:
            next_target[-1] -= np.pi/2 /steps
        elif direction == 1:
            next_target[-1] += np.pi/2 /steps
        trajectory.append(next_target)
    print("current_q: ", current_q)
    print("trajectory: ",trajectory)
    logger.info(f"Initial: {current_q}")
    jpos_steps = trajectory

    logger.info(f"Trajectory Length: {len(jpos_steps)}")
    logger.info(f"Current joints: {np.round(current_q, 3)}")
    # try to follow path

    controller_type = "JOINT_IMPEDANCE"
    action_history = []
    state_history = []
    prev_action = list(current_q)
    for i, jpos_t in enumerate(jpos_steps):
        action = list(jpos_t) + [-1.0]
        logger.debug("step {}, action {}".format(i, np.round(action, 2)))
        robot_interface.control(
            controller_type=controller_type,
            action=action,
            controller_cfg=controller_cfg,
        )