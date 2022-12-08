#!/usr/bin/env python3

"""
This is to be used instead of MotorManager whenever the arm is not being controlled (ie for
    simulations). It is a drop-in replacement for MotorManager in launch files.
Its main job is to subscribe to the motor commands and update the joint positions in
    /joint_state accordingly--normally, this is done using the hardware sensors in MotorManager.
"""

import parameters as p
import numpy as np
import rospy
import roboticstoolbox as rtb
from sensor_msgs.msg import JointState
from control_msgs.msg import JointJog


class SimManager:
    def __init__(self):
        self.motor_cmds = None
        self.joint_state = JointState()
        self.joint_state.position = [p.angle_to_cmd(d) for d in p.INITIAL_Q]
        self.joint_state_rad = JointState()
        self.joint_state_rad.position = p.INITIAL_Q
        # self.plot = rtb.backends.PyPlot.PyPlot()
        # self.plot.launch()
        # self.robot = p.PlanarArm()
        # self.plot.add(self.robot)
        # self.robot.q = np.array(p.INITIAL_Q)

        # SUBSCRIBERS
        self.cmd_sub = rospy.Subscriber("/motor_commands", JointJog, self.save_cmds)

        # PUBLISHERS
        self.pos_pub = rospy.Publisher("/joint_state_inc", JointState, queue_size=1)
        self.pos_pub_rad = rospy.Publisher("/joint_state_rad", JointState, queue_size=1)
        self.pos_pub.publish(self.joint_state)

    def save_cmds(self, motor_cmds):
        self.motor_cmds = motor_cmds

    def update(self):
        # TODO jointlims
        if self.motor_cmds is not None:
            self.joint_state.position = [int(d) for d in self.motor_cmds.displacements]
        self.pos_pub.publish(self.joint_state)
        self.joint_state_rad.position = [
            p.cmd_to_angle(d) for d in self.joint_state.position
        ]
        self.pos_pub_rad.publish(self.joint_state_rad)
        # self.robot.q = np.array(self.joint_state_rad.position)
        # self.plot.step(dt=0.001)
        self.motor_cmds = None


if __name__ == "__main__":
    rospy.init_node("Simulation Manager")
    rate = rospy.Rate(p.WORKING_FREQ)
    manager = SimManager()

    while not rospy.is_shutdown():
        manager.update()

        rate.sleep()
