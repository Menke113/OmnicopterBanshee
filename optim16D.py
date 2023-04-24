from scipy.optimize import minimize, NonlinearConstraint, LinearConstraint
import numpy as np
import time

# get python PID library

w_max = 100000 # max angular rate^2 of motors, this is a guess
max_thrust_motor = 1500 * 0.009806652 # max thrust of an individual motor in grams to Newtons
rng = np.random.default_rng()
rng.random()
xp = rng.random(16) * w_max
x = np.zeros(16)
# thrust_desired = [rng.random() * 3500, rng.random() * 3500, rng.random() * 6000]
# torque_desired = [rng.random() * 350, rng.random() * 350, rng.random() * 600]
thrust_desired_x = rng.random() * 6000 * 0.009806652
torque_desired_x = rng.random() * 600 * 0.009806652
thrust_desired_y = rng.random() * 6000 * 0.009806652
torque_desired_y = rng.random() * 600 * 0.009806652
thrust_desired_z = rng.random() * 6000 * 0.009806652
torque_desired_z = rng.random() * 600 * 0.009806652

k_t = 0.01
k_r = 10
k_e = 1

timestep = 0.05 # 20Hz update rate, must be enforced
alpha = (x[0:2:9] - x[1:2:10]) - (xp[:2:9] - xp[1:2:10])

print(x[0:2:9] - x[1:2:10])

def objective(x): # x is new 16D vector of w's, xp is prev 16D vector of w's
    return k_e * np.sum(x) + k_r * (np.sum(x - xp))


def thrust_constraint_z(x): # this only does total thrust and total torque, must break up into three dimensions
    thrust_x = (np.sum(x[0:2:9] - x[1:2:10]))) - thrust_desired_z
    return thrust

def torque_constraint_z(x): # torque from just the thrust forces, not including moments from the props
    torque = (np.sum(k_t*0.1*(x[::2] - x[1::2]))) - torque_desired_z
    return torque

def thrust_constraint(x): # this only does total thrust and total torque, must break up into three dimensions
    thrust = (np.sum(k_t*(x[::2] - x[1::2]))) - thrust_desired
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

def thrust_constraint_motor3_pos(x):
    return max_thrust_motor - ((k_t*(x[5] - x[6])))

def thrust_constraint_motor3_neg(x): 
    return ((k_t*(x[5] - x[6]))) - max_thrust_motor

def thrust_constraint_motor4_pos(x):
    return max_thrust_motor - ((k_t*(x[7] - x[8])))

def thrust_constraint_motor4_neg(x): 
    return ((k_t*(x[7] - x[8]))) - max_thrust_motor

def thrust_constraint_motor5_pos(x):
    return max_thrust_motor - ((k_t*(x[9] - x[10])))

def thrust_constraint_motor5_neg(x): 
    return ((k_t*(x[9] - x[10]))) - max_thrust_motor

def thrust_constraint_motor6_pos(x):
    return max_thrust_motor - ((k_t*(x[11] - x[12])))

def thrust_constraint_motor6_neg(x): 
    return ((k_t*(x[11] - x[12]))) - max_thrust_motor

def thrust_constraint_motor7_pos(x):
    return max_thrust_motor - ((k_t*(x[13] - x[14])))

def thrust_constraint_motor7_neg(x): 
    return ((k_t*(x[13] - x[14]))) - max_thrust_motor

def thrust_constraint_motor8_pos(x):
    return max_thrust_motor - ((k_t*(x[15] - x[16])))

def thrust_constraint_motor8_neg(x): 
    return ((k_t*(x[15] - x[16]))) - max_thrust_motor

b = (0.0, w_max) # bounds are 0 to max angular rate^2 of motors
bnds = [b] * 16
# print(bnds)
# thrust_constraint = LinearConstraint((np.sum(k_t*(x[::2] - x[1::2]))), thrust_desired, thrust_desired)
# torque_constraint = LinearConstraint((np.sum(k_t*0.1*(x[::2] - x[1::2]))), torque_desired, torque_desired)

# constraint1 = NonlinearConstraint(thrust_constraint)
# constraint2 = NonlinearConstraint(torque_constraint)

constraints = ({'type':'eq', 'fun':thrust_constraint, \
                 'type':'eq', 'fun':torque_constraint, \
                 'type':'ineq', 'fun':thrust_constraint_motor1_pos, \
                 'type':'ineq', 'fun':thrust_constraint_motor1_neg, \
                 'type':'ineq', 'fun':thrust_constraint_motor2_pos, \
                 'type':'ineq', 'fun':thrust_constraint_motor2_neg, \
                 'type':'ineq', 'fun':thrust_constraint_motor3_pos, \
                 'type':'ineq', 'fun':thrust_constraint_motor3_neg, \
                 'type':'ineq', 'fun':thrust_constraint_motor4_pos, \
                 'type':'ineq', 'fun':thrust_constraint_motor4_neg, \
                 'type':'ineq', 'fun':thrust_constraint_motor5_pos, \
                 'type':'ineq', 'fun':thrust_constraint_motor5_neg, \
                 'type':'ineq', 'fun':thrust_constraint_motor6_pos, \
                 'type':'ineq', 'fun':thrust_constraint_motor6_neg, \
                 'type':'ineq', 'fun':thrust_constraint_motor7_pos, \
                 'type':'ineq', 'fun':thrust_constraint_motor7_neg, \
                 'type':'ineq', 'fun':thrust_constraint_motor8_pos, \
                 'type':'ineq', 'fun':thrust_constraint_motor8_neg})

# hess = lambda x: np.zeros(16)

def hess(x):
    return np.zeros((len(x),len(x)))

start = time.time()

sol = minimize(objective, x, method='trust-constr', bounds=bnds, constraints=constraints, hess=hess)

end = time.time()

# print(sol)
# print(end-start)

