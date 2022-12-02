#!/usr/bin/env python3

"""
Description
-----------
When ModeManager passes an xbox input to this node, it processes it in a IK
method and passes the resulting commands to the MotorManager.
"""

import rospy
from sensor_msgs.msg import Joy, JointState
from control_msgs.msg import JointJog
from numpy import array, eye, set_printoptions
from numpy.linalg import inv
import parameters as p

set_printoptions(precision=3)


class IKController:
    def __init__(self):
        # State variables
        self.xbox_msg = Joy()
        self.joints = JointState()

        # Arm DH model for IK
        self.arm_dh_model = p.PlanarArm()

        # Initial conditions
        self.joints.position = p.INITIAL_Q

        # SUBSCRIBERS
        self.sub_joy = rospy.Subscriber("/xbox_IK", Joy, self.save_xbox_msg)
        self.sub_pos = rospy.Subscriber("/joint_state", JointState, self.update_pos)

        # PUBLISHERS
        self.pub_commands = rospy.Publisher("/motor_commands", JointJog, queue_size=1)

    def save_xbox_msg(self, msg):
        """
        Just save the msg. This will be processed in other functions.
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
        Handles changing the speed and resetting positions, then calls IK method.
        """
        # If the buttons aren't there, we haven't received a msg yet
        if not self.xbox_msg.buttons:
            return

        button_dict = {
            name: value
            for name, value in zip(p.BUTTON_NAMES_IN_ORDER, self.xbox_msg.buttons)
        }
        axes_dict = {
            name: value
            for name, value in zip(p.AXES_NAMES_IN_ORDER, self.xbox_msg.axes)
        }  # Here in case needed in future but not used now

        self.IK_base_frame(button_dict, axes_dict)

    def IK_base_frame(self, button_dict, axes_dict):
        """
        Handles controls other than moving speed or resetting positions.
        Passing in the button_dict and axes_dict from process_msg avoids redundant code.
        """

        # Note: ee == end_effector in these variable names
        des_y_vel = axes_dict["LR Left Stick"]
        des_x_vel = axes_dict["UD Left Stick"]
        des_z_vel = 0.0

        des_ee_twist = array(
            [[des_x_vel], [des_y_vel], [des_z_vel], [0.0], [0.0], [0.0]]
        )

        q_current = array([[p] for p in self.joints.position])

        # Do the IK with the pseudoinverse method
        qdot = self.get_qdot(q_current, des_ee_twist)

        # Construct message and scale by speed
        msg = JointJog()
        speed_scale = p.SPEED
        qdot *= speed_scale
        msg.velocities = qdot[:, 0]

        # Publish the commands
        self.pub_commands.publish(msg)

    def get_qdot(self, q_current, des_twist):
        """
        For a given configuration q_current and desired twist,
            return the new joint positions and joint velocities
            to move the end effector in the desired direction.
        """
        # psuedo-inverse method
        J = self.arm_dh_model.jacob0(q_current)
        J_sword = J.T @ inv(J @ J.T + p.KD**2 * eye(len(J)))

        # TODO: enforce joint limits

        qdot_des = J_sword @ des_twist

        return qdot_des


if __name__ == "__main__":
    rospy.init_node("IK Control")
    rate = rospy.Rate(p.WORKING_FREQ)
    controller = IKController()

    while not rospy.is_shutdown():
        controller.process_msg()

        rate.sleep()
