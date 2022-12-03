#!/usr/bin/env python3

"""
This is to be used instead of MotorManager whenever the arm is not being controlled (ie for
    simulations). It is a drop-in replacement for MotorManager in launch files.
Its main job is to subscribe to the motor commands and update the joint positions in
    /arm_state accordingly--normally, this is done using the hardware sensors in MotorManager.
"""

import parameters as p
import rospy as ros
import numpy as np
from sensor_msgs.msg import JointState
from control_msgs.msg import JointJog


class SimManager:
    def __init__(self):
        self.motor_cmds = None
        self.joint_state = JointState()
        self.joint_state.position = p.INITIAL_Q

        # SUBSCRIBERS
        self.cmd_sub = ros.Subscriber("/motor_commands", JointJog, self.save_cmds)

        # PUBLISHERS
        self.pos_pub = ros.Publisher("/arm_state", JointState, queue_size=1)
        self.pos_pub.publish(self.joint_state)

    def save_cmds(self, motor_cmds):
        self.motor_cmds = motor_cmds

    def update(self):
        if self.motor_cmds is None:
            return
        self.joint_state.position = [d for d in self.motor_cmds.displacements]
        self.pos_pub.publish(self.joint_state)
        self.motor_cmds = None


if __name__ == "__main__":
    ros.init_node("Simulation Manager")
    rate = ros.Rate(p.WORKING_FREQ)
    manager = SimManager()

    while not ros.is_shutdown():
        manager.update()

        rate.sleep()
