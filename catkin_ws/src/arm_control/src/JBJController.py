#!/usr/bin/env python3

"""
Description
-----------
When ModeManager passes an xbox input to this node, it processes it and 
    controls the joints accordingly.
You control one node at a time with this control scheme. Different buttons select
    the node.
"""

import rospy
from sensor_msgs.msg import Joy, JointState
from control_msgs.msg import JointJog
from numpy import set_printoptions
import parameters as p
from enum import Enum, auto

set_printoptions(precision=3)


class Controllable(Enum):
    NODE_1 = 1
    NODE_2 = 2
    NODE_3 = 3
    NONE_SELECTED = auto()


class JBJController:
    def __init__(self):
        # State variables
        self.xbox_msg = Joy()
        self.joints = JointState()
        self.selected_node = Controllable.NONE_SELECTED

        # Initial conditions
        self.joints.position = p.INITIAL_Q

        # SUBSCRIBERS
        self.sub_joy = rospy.Subscriber("/xbox_jointbyjoint", Joy, self.save_xbox_msg)
        self.sub_pos = rospy.Subscriber("/joint_state", JointState, self.update_pos)

        # PUBLISHERS
        self.pub_commands = rospy.Publisher("/motor_commands", JointJog, queue_size=1)

    def save_xbox_msg(self, msg):
        """
        Just save the msg. This will be processed in other messages, like
            check_method and arm IK base tool
        """
        self.xbox_msg = msg

    def update_pos(self, measured_joint_pos):
        """
        Upon receiving a joint position measured by the Hall effect sensors
        internal to the motors, uses this as new estimate of joint positions
        """
        self.joints.position = measured_joint_pos.position

    def process_msg(self):
        """
        Handles changing the speed, laser and solenoid?, etc
            then passes to self.joint_by_joint to control the arm.
        """
        button_dict = {
            name: value
            for name, value in zip(p.BUTTON_NAMES_IN_ORDER, self.xbox_msg.buttons)
        }
        axes_dict = {
            name: value
            for name, value in zip(p.AXES_NAMES_IN_ORDER, self.xbox_msg.axes)
        }  # Here in case needed in future but not used now

        self.joint_by_joint(button_dict, axes_dict)

    def joint_by_joint(self, button_dict, axes_dict):
        """
        Determines what node is selected, sends motor commands to control it.
        Also handles joint limits.

        Passing in button_dict and axes_dict avoids redundant code.
        """

        # Handle node selection button.
        if button_dict["D pad LR"] > 0.05:
            self.selected_node = Controllable.NODE_1
            return
        if button_dict["D pad UD"] > 0.05:
            self.selected_node = Controllable.NODE_2
            return
        if button_dict["D pad LR"] < -0.05:
            self.selected_node = Controllable.NODE_3
            return

        # Triggers do motion. When triggers are unpressed, msg has value 1.
        #   The 0.95 margin allows for trigger noise
        # If both are pressed, do not move (hence the xor)
        if button_dict["R trigger"] < 0.95 ^ button_dict["L trigger"] < 0.95:
            # If they don't have anything selected, warn them
            if self.selected_node == Controllable.NONE_SELECTED:
                rospy.logwarn(
                    "Trying to move in joint by joint but no node selected."
                    " Defaulting to node 1"
                )
                self.selected_node = Controllable.NODE_1
            # Move the node (if we ever update to a newer python,
            #   replace if chain with match statement)
            # TODO: move nodes
            # TODO: joint lims


if __name__ == "__main__":
    rospy.init_node("Joint by joint control")
    rate = rospy.Rate(p.WORKING_FREQ)
    controller = JBJController()

    while not rospy.is_shutdown():
        controller.process_msg()

        rate.sleep()
