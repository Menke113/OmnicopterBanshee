import numpy as np

def get_thrusts(T):

    a = 322.58

    c = (np.sqrt(2)/2) * a

    d = 1e3

    T1 = T[2:6]
    A = np.array([[1, 1, 1, 1], [c, -c, c, -c], [c, c, -c, -c], [d, -d, -d, d]])

    # print(A)

    # print(T1)

    thrusts1_4 = np.array(np.linalg.solve(A, T1))

    # print(thrusts1_4)

    thrusts5_6 = np.array([T[0]/2] * 2)
    thrusts7_8 = np.array([T[1]/2] * 2)

    # print(thrusts5_6)
    # print(thrusts7_8)

    thrusts = np.concatenate((thrusts1_4, thrusts5_6, thrusts7_8))

    return thrusts
