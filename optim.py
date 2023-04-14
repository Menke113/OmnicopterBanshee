from scipy.optimize import minimize, NonlinearConstraint
import numpy as np

# get python PID library

w_max = 100000; # max angular rate^2 of motors, this is a guess


rng = np.random.default_rng()
rng.random()
xp = rng.random(16) * w_max
# thrust_desired = [rng.random() * 3500, rng.random() * 3500, rng.random() * 6000]
# torque_desired = [rng.random() * 350, rng.random() * 350, rng.random() * 600]
thrust_desired = rng.random() * 6000
torque_desired = rng.random() * 600
# print(xp)


# print(xp)

states = np.concatenate((np.ones(16), xp))

# print(states)

k_t = 0.01
k_r = 10
k_e = 1

def objective(states): # x is new 16D vector of w's, x_p is prev 16D vector of w's
    # print(states)
    x = states[0:16]
    xp = states[16:]
    # print(x)
    # print(xp)
    # print(len(x))
    # print(len(xp))

    calc = k_e * np.sum(x) + k_r * (np.sum(x - xp))

    # print(calc)

    return calc

    # return k_e * np.sum(x) + k_r * (x[0:15] - xp[0:15]) # this is the cost function

def thrust_constraint(k_t):
    thrust = (lambda x: (np.sum(k_t*(x[::2] - x[1::2]))))
    print(thrust)
    return thrust


def torque_constraint(k_t): # torque from just the thrust forces, not including moments from the props
    torque = (lambda x: (np.sum(k_t*0.1*(x[::2] - x[1::2]))))
    print(torque)
    return torque


b = (0.0, w_max) # bounds are 0 to max angular rate^2 of motors
bnds_xp = [(xp[0], xp[0]), (xp[1], xp[1]), (xp[2], xp[2]), (xp[3], xp[3]), (xp[4], xp[4]), (xp[5], xp[5]), (xp[6], xp[6]), (xp[7], xp[7]), (xp[8], xp[8]), (xp[9], xp[9]), (xp[10], xp[10]), (xp[11], xp[11]), (xp[12], xp[12]), (xp[13], xp[13]), (xp[14], xp[14]), (xp[15], xp[15])]
# print(bnds_xp)
bnds_x = [b] * 16
# print(bnds_x)
# bnds_xp = [b_xp]
bnds = np.concatenate((bnds_x, bnds_xp))
# print(bnds)
constraint1 = LinearConstraint(thrust_constraint, -thrust_desired, thrust_desired)
constraint2 = LinearConstraint(torque_constraint, -torque_desired, torque_desired)
constraints = [constraint1, constraint2]
sol = minimize(objective, states, method='trust-constr', bounds=bnds, constraints=constraints)

print(sol)


