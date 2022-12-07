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
        self.speed_idx = p.STARTING_SPEED_IDX

        # Arm DH model for IK
        self.arm_dh_model = p.PlanarArm()

        # Initial conditions
        self.joints.position = p.INITIAL_Q

        # SUBSCRIBERS
        self.sub_joy = rospy.Subscriber("/xbox_IK", Joy, self.save_xbox_msg)
        self.sub_pos = rospy.Subscriber("/joint_state_inc", JointState, self.update_pos)

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

        # TODO speeds

        self.IK_with_xbox(button_dict, axes_dict)

    def IK_with_xbox(self, button_dict, axes_dict):
        """
        Handles controls other than moving speed or resetting positions.
        Passing in the button_dict and axes_dict from process_msg avoids redundant code.
        """

        speed_scale = p.IK_SPEEDS[self.speed_idx]

        # Note: ee == end_effector in these variable names
        q_current = array([p.cmd_to_angle(d) for d in self.joints.position])
        ee_current = self.arm_dh_model.fkine(q_current)

        # rospy.logwarn(f"Currently at \nq: {q_current}\nee: {ee_current}.")

        # Plot transforms in RVIZ for this to make sense.
        delta_x = speed_scale * axes_dict["L stick UD"]
        delta_y = speed_scale * axes_dict["L stick LR"]
        delta_z = speed_scale * axes_dict["R stick UD"]

        des_ee = ee_current
        for i, delta in enumerate([delta_x, delta_y, delta_z]):
            des_ee.t[i] += delta
        # rospy.logwarn(f"Trying to get to \nee: {des_ee}.")

        # Use robotics toolbox IK method
        # Returns tuple(q, success, reason, iterations, residual)
        ik_res = self.arm_dh_model.ikine_LMS(des_ee, tol=1e-5, mask=[1, 1, 1, 0, 0, 0])

        if not ik_res[1]:
            rospy.logwarn(f"Failed to find IK sol with tol={1e-5}")
            rospy.logwarn(ik_res)
            # ik_res = self.arm_dh_model.ikine_LMS(
            #     des_ee, tol=1e-2, mask=[1, 1, 1, 0, 0, 0]
            # )
            # if not ik_res[1]:
            #     rospy.logwarn(f"Failed to find IK sol again with tol={1e-2}")
            #     rospy.logwarn(str(ik_res) + "\n\n")

        # Construct message and scale by speed
        msg = JointJog()
        msg.displacements = list(int(p.angle_to_cmd(d)) for d in ik_res[0])

        # Publish the commands
        self.pub_commands.publish(msg)


if __name__ == "__main__":
    rospy.init_node("IK Control")
    rate = rospy.Rate(p.WORKING_FREQ)
    controller = IKController()

    while not rospy.is_shutdown():
        controller.process_msg()

        rate.sleep()
