import roboticstoolbox as rtb

PI = 3.14159265358979323846
LINK_LENGTH = 0.15
CLAW_LENGTH = 0.05

PLANAR_DH = [
    rtb.RevoluteDH(a=LINK_LENGTH),
    rtb.RevoluteDH(a=LINK_LENGTH, alpha=PI / 2),
    rtb.RevoluteDH(a=CLAW_LENGTH, alpha=-PI / 2),
]

N = len(PLANAR_DH)


class PlanarArm(rtb.DHRobot):
    def __init__(self):
        super().__init__(PLANAR_DH, name="Mini Rover Arm")


if __name__ == "__main__":
    # Print the arm parameters
    arm = PlanarArm()
    print(arm, "\n")

    # Show the arm in all 0 configuration
    q0 = [0] * N
    T0 = arm.fkine(q0)
    print(f"The end effector is at\n{T0}\n")
    arm.plot(q0, block=True)

    # Show the arm in another configuration
    q1 = [PI / 4] * N
    T1 = arm.fkine(q1)
    print(f"The end effector is at\n{T1}\n")
    arm.plot(q1, block=True)

    # Show the arm moving from 0 to the second configuration
    trajectory = arm.jtraj(T0, T1, t=200)  # use t steps
    arm.plot(trajectory.q, block=True)
