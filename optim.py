from scipy.optimize import minimize, NonlinearConstraint, LinearConstraint
import numpy as np
import time

# get python PID library

w_max = 100000 # max angular rate^2 of motors, this is a guess
max_thrust_motor = 1500 # max thrust of an individual motor in grams
max_thrust_matrix = np.full((1,8), 1500)

rng = np.random.default_rng()
rng.random()
xp = rng.random(16) * w_max
x = np.zeros(16)
# thrust_desired = [rng.random() * 3500, rng.random() * 3500, rng.random() * 6000]
# torque_desired = [rng.random() * 350, rng.random() * 350, rng.random() * 600]
thrust_desired = rng.random() * 6000
torque_desired = rng.random() * 600

k_t = 0.01
k_r = 10
k_e = 1


def objective(x): # x is new 16D vector of w's, xp is prev 16D vector of w's
    return k_e * np.sum(x) + k_r * (np.sum(x - xp))


def thrust_constraint(x): # this only does total thrust and total torque, must break up into three dimensions
    thrust = (np.sum(k_t*(x[::2] - x[1::2]))) - thrust_desired
    print(thrust)
    return thrust

def torque_constraint(x): # torque from just the thrust forces, not including moments from the props
    torque = (np.sum(k_t*0.1*(x[::2] - x[1::2]))) - torque_desired
    return torque

# def omega_constraint_neg(x):
#     return x[::2] - x[1::2] + w_max

# def omega_constraint_pos(x):
#     return w_max - (x[::2] - x[1::2])

# def thrust_constraint_motors_pos(x): # this seemingly has to be defined on each motor individually, it doesn't like a vector output
#     return max_thrust_matrix - ((k_t*(x[::2] - x[1::2])))

# def thrust_constraint_motors_neg(x): 
#     return ((k_t*(x[::2] - x[1::2]))) - max_thrust_matrix

def thrust_constraint_motor1_pos(x):
    return max_thrust_motor - ((k_t*(x[1] - x[2])))

def thrust_constraint_motor1_neg(x): 
    return ((k_t*(x[1] - x[2]))) - max_thrust_motor

def thrust_constraint_motor2_pos(x):
    return max_thrust_motor - ((k_t*(x[3] - x[4])))

def thrust_constraint_motor2_neg(x): 
    return ((k_t*(x[3] - x[4]))) - max_thrust_motor

b = (0.0, w_max) # bounds are 0 to max angular rate^2 of motors
bnds = [b] * 16
# print(bnds)
# thrust_constraint = LinearConstraint((np.sum(k_t*(x[::2] - x[1::2]))), thrust_desired, thrust_desired)
# torque_constraint = LinearConstraint((np.sum(k_t*0.1*(x[::2] - x[1::2]))), torque_desired, torque_desired)

# constraint1 = NonlinearConstraint(thrust_constraint)
# constraint2 = NonlinearConstraint(torque_constraint)

constraints = ({'type':'eq', 'fun':thrust_constraint, \
                 'type':'eq', 'fun':torque_constraint, \
                 'type':'ineq', 'fun':thrust_constraint_motors_pos, \
                 'type':'ineq', 'fun':thrust_constraint_motors_neg})
# hess = lambda x: np.zeros(16)

def hess(x):
    return np.zeros((len(x),len(x)))

start = time.time()

sol = minimize(objective, x, method='trust-constr', bounds=bnds, constraints=constraints, hess=hess)

end = time.time()

# print(sol)
# print(end-start)

