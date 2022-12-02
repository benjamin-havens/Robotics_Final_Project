import parameters as p
import numpy as np

pi = np.pi


def IK_test():
    arm = p.PlanarArm()

    # TRIVIAL
    test_0_ee = np.array([[0.35], [0], [0]])
    test_0_q = np.array([])

    # EASILY ACHIEVABLE
    test_1_q = np.array([[pi / 4] for _ in range(p.N)])

    arm.ikine_LMS()


if __name__ == "__main__":
    IK_test()
