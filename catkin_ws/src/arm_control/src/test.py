
import roboticstoolbox as rtb
import numpy as np
import spatialmath

LINK_LENGTH = 0.15  # 15cm
CLAW_LENGTH = 0.094

PLANAR_DH = [
    rtb.RevoluteDH(a=LINK_LENGTH),
    rtb.RevoluteDH(a=LINK_LENGTH, alpha=np.pi / 2),
    rtb.RevoluteDH(a=CLAW_LENGTH, alpha=-np.pi / 2),
]
N = len(PLANAR_DH)

robot = rtb.DHRobot(PLANAR_DH)
fk = robot.fkine([0, 0, 0])
print(fk)
print(type(fk))
des = spatialmath.SE3(LINK_LENGTH + 0.001, LINK_LENGTH, 0)

print(des)

sol = robot.ikine_LMS(des, q0=[0, 0, 0], mask = [1,1,1,0,0,0],ilimit=1000)



robot.plot(sol.q, block=True)


