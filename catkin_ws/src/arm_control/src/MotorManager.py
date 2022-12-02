#!/usr/bin/env python3

"""
ROS node that connects to the hardware

Responsibilities:
- receives commands on the /motor_commands topic and accordingly commands the motors.
- periodically reads the positions of the motors and publishes them
"""

import rospy
from control_msgs import JointJog
from sensor_msgs import JointState
import parameters as p
import Ax12 as Ax12


class MotorManager:
    def __init__(self):
        # Connect to motors
        Ax12.DEVICENAME = p.DEVICE_NAME
        Ax12.BAUDRATE = p.BAUDRATE
        Ax12.connect()

        # Set up motors
        m1 = Ax12(1)
        m2 = Ax12(2)
        m3 = Ax12(3)
        self.motors = [m1, m2, m3]
        for m in self.motors:
            m.set_moving_speed(p.SPEED)

        # SUBSCRIBERS
        self.last_cmds = None
        self.sub_cmds = rospy.Subscriber("/motor_commands", JointJog, self.save_cmds)

        # PUBLISHERS
        self.pub_pos = rospy.Publisher("/joint_state", JointState)

    def save_cmds(self, motor_cmd_msg):
        self.last_cmds = [p.angles_to_cmds(d) for d in motor_cmd_msg.displacements]

    def send_cmds(self):
        if self.last_cmds is None:
            return
        for m, cmd in zip(self.motors, self.last_cmds):
            m.set_goal_position(cmd)

    def update_positions(self):
        msg = JointState()
        msg.positions = [p.cmd_to_angle(m.get_present_position()) for m in self.motors]
        self.pub_pos(msg)

    def disconnect(self):
        [m.set_torque_enable(0) for m in self.motors]
        Ax12.disconnect()


if __name__ == "__main__":
    rospy.init_node("Motor Manager")
    rate = rospy.Rate(p.WORKING_FREQ)
    manager = MotorManager()

    while not rospy.is_shutdown():
        manager.update_positions()
        manager.send_cmds()

        rate.sleep()
