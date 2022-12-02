"""
ROS node that connects to the hardware

Responsibilities:
- receives commands on the /motor_command topic and accordingly commands the motors.
- periodically reads the positions of the motors and publishes them
"""

import rospy
from control_msgs import JointJog
from sensor_msgs import JointState
import parameters as p


class MotorManager:
    def __init__(self):
        # SUBSCRIBERS

        # PUBLISHERS

        pass

    def save_cmds(self, motor_cmd_msg):
        pass

    def send_cmds(self):
        pass

    def update_positions(self):
        pass


if __name__ == "__main__":
    rospy.init_node("Motor Manager")
    rate = rospy.Rate(p.WORKING_FREQ)
    manager = MotorManager()

    while not rospy.is_shutdown():
        manager.update_positions()
        manager.send_cmds()

        rate.sleep()
