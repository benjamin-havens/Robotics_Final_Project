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
import parameters as p
import numpy as np
from DLT import DLT


class DLTController:
    def __init__(self):
        # State variables
        self.xbox_msg = Joy()
        self.joints = JointState()
        self.speed_idx = p.STARTING_SPEED_IDX

        # DLT stuff
        self.dlt = DLT(
            "/home/benjaminhavens/Robotics_Final_Project/catkin_ws/src/DLT/left_with_robot.jpg",
            "/home/benjaminhavens/Robotics_Final_Project/catkin_ws/src/DLT/right_with_robot.jpg",
            numPoints=10,
        )
        self.dlt.DLT_Load_From_File(
            "/home/benjaminhavens/Robotics_Final_Project/DLT_Data/left_with_robot_right_with_robot.txt"
        )

        # Arm DH model for IK
        self.arm_dh_model = p.PlanarArm()

        # Initial conditions
        self.joints.position = p.INITIAL_Q

        # SUBSCRIBERS
        self.sub_joy = rospy.Subscriber("/xbox_DLT", Joy, self.save_xbox_msg)
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
        if not self.xbox_msg or not self.xbox_msg.buttons:
            return

        button_dict = {
            name: value
            for name, value in zip(p.BUTTON_NAMES_IN_ORDER, self.xbox_msg.buttons)
        }
        axes_dict = {
            name: value
            for name, value in zip(p.AXES_NAMES_IN_ORDER, self.xbox_msg.axes)
        }

        if button_dict["A"]:
            x, y, z = self.dlt.getXYZ()
            x = 16 * 2.54 - x
            y = 16 * 2.54 - y
            z = z - 4.5 * 2.54
            x, y, z = x / 100, y / 100, z / 100
            rospy.logwarn(f"x, y, z = {x}, {y}, {z}")

            q_curr = np.array([p.cmd_to_angle(d) for d in self.joints.position])
            ee_curr = self.arm_dh_model.fkine(q_curr)

            ee_des = ee_curr
            for i, val in enumerate([x, y, z]):
                ee_des.t[i] = val

            ik_res = self.arm_dh_model.ikine_LMS(
                ee_des, mask=[1, 1, 1, 0, 0, 0], tol=1e-3
            )

            if not ik_res[1]:
                rospy.logwarn("Failed to find solution in DLT")
                found = False
                tol = 1e-2
                while not found:
                    ik_res = self.arm_dh_model.ikine_LMS(
                        ee_des, mask=[1, 1, 1, 0, 0, 0], tol=tol
                    )
                    if not ik_res[1]:
                        rospy.logwarn(f"Failed with tolerance {tol}")
                        tol += 1e-2
                    else:
                        found = True
                        rospy.logwarn(f"Found solution with tolerance {tol}")

            msg = JointJog()
            msg.displacements = list(int(p.angle_to_cmd(d)) for d in ik_res[0])

            self.pub_commands.publish(msg)


if __name__ == "__main__":
    rospy.init_node("DLT Control")
    rate = rospy.Rate(p.WORKING_FREQ)
    controller = DLTController()

    while not rospy.is_shutdown():
        controller.process_msg()

        rate.sleep()
