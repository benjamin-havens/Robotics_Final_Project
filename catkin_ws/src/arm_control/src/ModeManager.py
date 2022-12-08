#!/usr/bin/env python3

"""
Description
-----------
Monitors the input stream from the XBOX controller and the current mode, and passes
the message to either the IK node or the joint by joint node.
"""

import rospy
import time
from sensor_msgs.msg import Joy
import parameters as p


class ModeManager:
    def __init__(self):
        # State variables
        self.xbox_msg = Joy()
        self.mode = p.ArmControlMode.JOINT_CONTROL

        # SUBSCRIBERS
        self.sub_xbox = rospy.Subscriber("/xbox_arm", Joy, self.xbox_callback)

        # PUBLISHERS
        self.pub_jointbyjoint = rospy.Publisher("/xbox_jointbyjoint", Joy, queue_size=1)
        self.pub_IK = rospy.Publisher("/xbox_IK", Joy, queue_size=1)
        self.pub_DLT = rospy.Publisher("/xbox_DLT", Joy, queue_size=1)

    def xbox_callback(self, msg):
        """
        Just saves the msg. This will be processed in other functions.
        """
        self.xbox_msg = msg

    def check_mode(self):
        """
        Reads saved msg, decides what mode we are in, other needed processing.
        """

        # If no message received yet
        if self.xbox_msg is None or not self.xbox_msg.buttons:
            return

        button_dict = {
            name: value
            for name, value in zip(p.BUTTON_NAMES_IN_ORDER, self.xbox_msg.buttons)
        }
        axes_dict = {
            name: value
            for name, value in zip(p.AXES_NAMES_IN_ORDER, self.xbox_msg.axes)
        }  # Here in case needed in future but not used now

        # Some buttons do different things in IK mode vs Joint by Joint
        if self.mode == p.ArmControlMode.IK_BASE_FRAME:

            # XBOX button switches to joint control
            if button_dict["power"]:
                manager.mode = p.ArmControlMode.JOINT_CONTROL
                print("Joint by Joint Control Initiated")
                time.sleep(0.25)
                return

            if button_dict["back"]:
                manager.mode = p.ArmControlMode.DLT
                print("DLT Initiatated")
                time.sleep(0.25)
                return

        elif self.mode == p.ArmControlMode.JOINT_CONTROL:

            # XBOX button switches to IK
            if button_dict["power"]:
                manager.mode = p.ArmControlMode.IK_BASE_FRAME
                print("IK Base Control Initiated")
                time.sleep(0.25)
                return

            if button_dict["back"]:
                manager.mode = p.ArmControlMode.DLT
                print("DLT Initiatated")
                time.sleep(0.25)
                return

        elif self.mode == p.ArmControlMode.DLT:

            if button_dict["power"]:
                manager.mode = p.ArmControlMode.JOINT_CONTROL
                print("Joint by Joint Control Initiated")
                time.sleep(0.25)
                return

            if button_dict["back"]:
                manager.mode = p.ArmControlMode.IK_BASE_FRAME
                print("DLT Initiatated")
                time.sleep(0.25)
                return

        else:
            raise ValueError(f"In unknown/unimplemented mode {self.state.mode}")

        self.pass_msg()

    def pass_msg(self):
        if self.mode == p.ArmControlMode.JOINT_CONTROL:
            self.pub_jointbyjoint.publish(self.xbox_msg)
        elif self.mode == p.ArmControlMode.IK_BASE_FRAME:
            self.pub_IK.publish(self.xbox_msg)
        elif self.mode == p.ArmControlMode.DLT:
            self.pub_DLT.publish(self.xbox_msg)
        else:
            raise ValueError(f"Unknown/unimplemented mode {self.mode}.")
        self.xbox_msg = Joy()


if __name__ == "__main__":
    rospy.init_node("Mode Manager")
    rate = rospy.Rate(p.WORKING_FREQ)
    manager = ModeManager()

    while not rospy.is_shutdown():
        manager.check_mode()
        rate.sleep()
