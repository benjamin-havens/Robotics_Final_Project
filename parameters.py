import roboticstoolbox as rtb
from numpy import pi

planar_dh = [
    rtb.RevoluteDH(a=0.15),
    rtb.RevoluteDH(a=0.15, alpha=pi / 2),
    rtb.RevoluteDH(a=0.05, alpha=-pi / 2),
]
