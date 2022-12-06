"""
Contains the parameters for final project for ME 537, a simple planar arm.

Isaac Shaw, Joey LeCheminant, and Benjamin Havens
"""

###########
# IMPORTS #
###########
import roboticstoolbox as rtb
import matplotlib.pyplot as plt
from enum import Enum, auto
from numpy import pi


#########
# ENUMS #
#########
class ArmControlMode(Enum):
    JOINT_CONTROL = auto()  # Joint by joint
    IK_BASE_FRAME = auto()  # IK in the base frame


##########
# SPEEDS #
##########
JBJ_SPEEDS = [4, 8, 16]
IK_SPEEDS = [0.01, 0.02, 0.04]
STARTING_SPEED_IDX = 1


#################
# MISCELLANEOUS #
#################
WORKING_FREQ = 20  # hz
CENTER_ANGLE = 512  # Corresponds to 0 position
HALF_RANGE = 308  # Corresponds to 90deg rotation
QLIM_BOTTOM = CENTER_ANGLE - HALF_RANGE
QLIM_TOP = CENTER_ANGLE + HALF_RANGE
INITIAL_Q = [0, 0, 0]
KD = 0.01


def angle_to_cmd(angle, radians=True):
    if radians:
        angle *= 180 / pi
    return int(CENTER_ANGLE + (HALF_RANGE / 90) * angle)


def cmd_to_angle(cmd, radians=True):
    angle = float((cmd - CENTER_ANGLE) * 90 / HALF_RANGE)
    if radians:
        angle *= pi / 180
    return angle


####################
# MOTOR CONNECTION #
####################
DEVICE_NAME = "/dev/ttyUSB0"
BAUDRATE = int(1e6)
SPEED = 100


############
# DH MODEL #
############
LINK_LENGTH = 0.15  # 15cm
CLAW_LENGTH = 0.094

PLANAR_DH = [
    rtb.RevoluteDH(a=LINK_LENGTH),
    rtb.RevoluteDH(a=LINK_LENGTH, alpha=pi / 2),
    rtb.RevoluteDH(a=CLAW_LENGTH, alpha=-pi / 2),
]
N = len(PLANAR_DH)

# Constructor that builds a DHRobot with our parameters
class PlanarArm(rtb.DHRobot):
    def __init__(self):
        super().__init__(PLANAR_DH, name="Mini Rover Arm")


###################
# XBOX PARAMETERS #
###################
# See http://wiki.ros.org/joy

BUTTON_NAMES_IN_ORDER = [
    "A",
    "B",
    "X",
    "Y",
    "L bumper",
    "R bumper",
    "back",
    "start",
    "power",  # xbox button
    "L stick press",  # Pressing the stick in like a button
    "R stick press",
]
AXES_NAMES_IN_ORDER = [
    "L stick LR",  # Moving stick left or right (left is positive)
    "L stick UD",  # Moving stick up and down (up is positive)
    "L trigger",
    "R stick LR",
    "R stick UD",
    "R trigger",
    "D pad LR",
    "D pad UD",
]


#################
# EXAMPLE USAGE #
#################
if __name__ == "__main__":
    # Print the arm parameters
    arm = PlanarArm()
    print("\n", arm, "\n", sep="")

    # Show the arm in all negative configuration
    q0 = [-pi / 4] * N
    T0 = arm.fkine(q0)
    print(f"When q=[0, 0, 0], the end effector is at\n{T0}\n")
    fig_1 = plt.figure(1)
    arm.plot(q0, fig=fig_1)

    # Show the arm in all positive configuration
    q1 = [pi / 4] * N
    T1 = arm.fkine(q1)
    print(f"When q=[pi/4, pi/4, pi/4], the end effector is at\n{T1}\n")
    fig_2 = plt.figure(2)
    arm.plot(q1, fig=fig_2)

    # Show the arm moving from the first to the second configuration
    trajectory = arm.jtraj(T0, T1, t=200)  # use t steps
    fig_3 = plt.figure(3)
    arm.plot(trajectory.q, fig=fig_3, block=True)
