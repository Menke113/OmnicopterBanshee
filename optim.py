from scipy.optimize import minimize, NonlinearConstraint
import numpy as np

# get python PID library

w_max = 100000; # max angular rate^2 of motors, this is a guess

rng = np.random.default_rng()
rng.random()
xp = rng.random(16) * w_max

print(xp)

states = np.concatenate(np.ones(16), xp)

k_t = 0.01
k_r = 10
k_e = 1

def objective(states): # x is new 16D vector of w's, x_p is prev 16D vector of w's
    x = states[1:16]
    xp = states[16:]
    return k_e * np.sum(x) + k_r * (x[1:16] - xp[1:16]) # this is the cost function

def constraint1(x):
    return np.sum(x) - 16

def constraint2(x):
    return -np.sum(x)


b = (0.0, w_max) # bounds are 0 to max angular rate^2 of motors
bnds_X = [b] * 16
nonlinear_constraint1 = NonlinearConstraint(constraint1, -np.inf, 0)
nonlinear_constraint2 = NonlinearConstraint(constraint2, -np.inf, 0)
constraints = [nonlinear_constraint1, nonlinear_constraint2]
sol = minimize(objective, xp, method='trust-constr', bounds=bnds, constraints=constraints)

print(sol)


