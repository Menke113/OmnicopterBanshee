from scipy.optimize import minimize, NonlinearConstraint
import numpy as np

def objective(x):
    return np.prod(x) # this is the cost function

def constraint1(x):
    return np.sum(x) - 16

def constraint2(x):
    return -np.sum(x)

w_max = 100000; # max angular rate^2 of motors, this is a guess

x0 = np.ones(16) # will want this to be previous soluation
b = (0.0, w_max) # bounds are 0 to max angular rate^2 of motors
bnds = [b -b b -b b -b b -b b -b b -b b -b b -b] # must be in format [w1_pos w1_neg w2_pos w2_neg ...]
nonlinear_constraint1 = NonlinearConstraint(constraint1, -np.inf, 0)
nonlinear_constraint2 = NonlinearConstraint(constraint2, -np.inf, 0)
constraints = [nonlinear_constraint1, nonlinear_constraint2]
sol = minimize(objective, x0, method='trust-constr', bounds=bnds, constraints=constraints)

print(sol)


