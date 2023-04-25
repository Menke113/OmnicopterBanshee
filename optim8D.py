from scipy.optimize import minimize, NonlinearConstraint, LinearConstraint
import numpy as np
import time

# get python PID library

w_max_2306 = 2000 # max angular rate of 2306 motors, rad/s
w_max_2806 = 2900 # max angular rate of 2806 motors, rad/s

max_thrust_motor_2306 = 1200 * 0.009806652 # max thrust of an individual 2306 motor in grams to Newtons - should be scaled by throttle input (Rx_chan[2])
max_thrust_motor_2806 = 1600 * 0.009806652 # max thrust of an individual 2806 motor in grams to Newtons - this is an estimate - should be scaled by throttle input (Rx_chan[2])
max_torque_motor_2306 = 2e7 # torque, N * mm
max_torque_motor_2806 = 2.9e7 # torque, N * mm

rng = np.random.default_rng()
rng.random()
xp = rng.random(8) * w_max_2306
x = np.zeros(8)

thrust_desired_x = rng.random() * 2000 * 0.009806652 # get these from from the dynamics function
torque_desired_x = rng.random() * 200 * 0.009806652
thrust_desired_y = rng.random() * 2000 * 0.009806652
torque_desired_y = rng.random() * 200 * 0.009806652
thrust_desired_z = rng.random() * 2000 * 0.009806652
torque_desired_z = rng.random() * 200 * 0.009806652


print(thrust_desired_x)
print(thrust_desired_y)
print(thrust_desired_z)

print(torque_desired_x)
print(torque_desired_y)
print(torque_desired_z)


# k_t = 0.006 # thrust coefficient, using N and rad/s
k_t = max_thrust_motor_2306/(w_max_2306**2)
k_r = 1
k_e = 1

I = 1.77e-8 * 0.009806652  # propeller moment of inertia, in g/mm^2 to Newtons/mm^2
a = 322.58 # the diagonal distance from the quadrotor motors to the center of the frame in mm

timestep = 0.1 # 10Hz update rate, must be enforced (change if code can't run fast enough)

def objective(x): # x is new 16D vector of w's, xp is prev 16D vector of w's
    
    return np.abs(k_e * np.sum(x) + k_r * (np.sum(x - xp)))

# constraints to achieve the correct thrust and torque components

def thrust_constraint_x(x): # this only does total thrust and total torque, must break up into three dimensions
    return ((k_t * x[4]**2) * np.sign(x[4])) + ((k_t * x[5]**2) * np.sign(x[5])) - thrust_desired_x

def torque_constraint_x(x): # torque from just the thrust forces, not including moments from the props
    alpha = (x - xp)/timestep
    return - (I * alpha[4]) - (I * alpha[5]) \
            + np.sqrt(2)/2 * a * ((k_t * x[0]**2) * np.sign(x[0])) + np.sqrt(2)/2 * a * ((k_t * x[2]**2) * np.sign(x[2])) \
            - np.sqrt(2)/2 * a * ((k_t * x[1]**2) * np.sign(x[1])) - np.sqrt(2)/2 * a * ((k_t * x[3]**2) * np.sign(x[3])) - torque_desired_x

def thrust_constraint_y(x): # this only does total thrust and total torque, must break up into three dimensions
    return ((k_t * x[6]**2) * np.sign(x[6])) + ((k_t * x[7]**2) * np.sign(x[7])) - thrust_desired_x

def torque_constraint_y(x): # torque from just the thrust forces, not including moments from the props
    alpha = (x - xp)/timestep
    return - (I * alpha[6]) - (I * alpha[7]) \
            + np.sqrt(2)/2 * a * ((k_t * x[0]**2) * np.sign(x[0])) + np.sqrt(2)/2 * a * ((k_t * x[1]**2) * np.sign(x[1])) \
            - np.sqrt(2)/2 * a * ((k_t * x[2]**2) * np.sign(x[2])) - np.sqrt(2)/2 * a * ((k_t * x[3]**2) * np.sign(x[3])) - torque_desired_y


def thrust_constraint_z(x): # this only does total thrust and total torque, must break up into three dimensions
    return ((k_t * x[0]**2) * np.sign(x[0])) + ((k_t * x[1]**2) * np.sign(x[1])) + ((k_t * x[2]**2) * np.sign(x[2])) + ((k_t * x[3]**2) * np.sign(x[3])) - thrust_desired_z

def torque_constraint_z(x): # torque from just the thrust forces, not including moments from the props
    alpha = (x - xp)/timestep
    return - (I * alpha[0]) - (I * alpha[1]) - (I * alpha[2]) - (I * alpha[3]) - torque_desired_z


# per-motor thrust constraints

def thrust_constraint_motor1(x):
    return max_thrust_motor_2306 - (k_t*(x[0]**2))

def thrust_constraint_motor2(x):
    return max_thrust_motor_2306 - (k_t*(x[1]**2))

def thrust_constraint_motor3(x):
    return max_thrust_motor_2306 - (k_t*(x[2]**2))

def thrust_constraint_motor4(x):
    return max_thrust_motor_2306 - (k_t*(x[3]**2))

def thrust_constraint_motor5(x):
    return max_thrust_motor_2806 - (k_t*(x[4]**2))

def thrust_constraint_motor6(x):
    return max_thrust_motor_2806 - (k_t*(x[5]**2))

def thrust_constraint_motor7(x):
    return max_thrust_motor_2806 - (k_t*(x[6]**2))

def thrust_constraint_motor8(x):
    return max_thrust_motor_2806 - (k_t*(x[7]**2))

# per-motor torque constraints

def torque_constraint_motor1(x):
    return max_torque_motor_2306 - (k_t*(x[0]**2))

def torque_constraint_motor2(x):
    return max_torque_motor_2306 - (k_t*(x[1]**2))

def torque_constraint_motor3(x):
    return max_torque_motor_2306 - (k_t*(x[2]**2))

def torque_constraint_motor4(x):
    return max_torque_motor_2306 - (k_t*(x[3]**2))

def torque_constraint_motor5(x):
    return max_torque_motor_2806 - (k_t*(x[4]**2))

def torque_constraint_motor6(x):
    return max_torque_motor_2806 - (k_t*(x[5]**2))

def torque_constraint_motor7(x):
    return max_torque_motor_2806 - (k_t*(x[6]**2))

def torque_constraint_motor8(x):
    return max_torque_motor_2806 - (k_t*(x[7]**2))



b = (-w_max_2306, w_max_2306) # bounds are 0 to max angular rate of motors
bnds1 = [b] * 4


b = (-w_max_2806, w_max_2806) # bounds are 0 to max angular rate of motors
bnds2 = [b] * 4

bnds = np.concatenate((bnds1, bnds2))

bnds = None

# print(bnds)
# thrust_constraint = LinearConstraint((np.sum(k_t*(x[::2] - x[1::2]))), thrust_desired, thrust_desired)
# torque_constraint = LinearConstraint((np.sum(k_t*0.1*(x[::2] - x[1::2]))), torque_desired, torque_desired)

# constraint1 = NonlinearConstraint(thrust_constraint)
# constraint2 = NonlinearConstraint(torque_constraint)

constraints = [{'type':'eq', 'fun':thrust_constraint_x},
                {'type':'eq', 'fun':thrust_constraint_y},
                {'type':'eq', 'fun':thrust_constraint_z},
                {'type':'eq', 'fun':torque_constraint_x},
                {'type':'eq', 'fun':torque_constraint_y},
                {'type':'eq', 'fun':torque_constraint_z},
                {'type':'ineq', 'fun':thrust_constraint_motor1},
                {'type':'ineq', 'fun':thrust_constraint_motor2},          
                {'type':'ineq', 'fun':thrust_constraint_motor3}, 
                {'type':'ineq', 'fun':thrust_constraint_motor4},
                {'type':'ineq', 'fun':thrust_constraint_motor5},
                {'type':'ineq', 'fun':thrust_constraint_motor6},
                {'type':'ineq', 'fun':thrust_constraint_motor7},
                {'type':'ineq', 'fun':thrust_constraint_motor8}, 
                {'type':'ineq', 'fun':torque_constraint_motor1},
                {'type':'ineq', 'fun':torque_constraint_motor2},            
                {'type':'ineq', 'fun':torque_constraint_motor3}, 
                {'type':'ineq', 'fun':torque_constraint_motor4},
                {'type':'ineq', 'fun':torque_constraint_motor5},
                {'type':'ineq', 'fun':torque_constraint_motor6},
                {'type':'ineq', 'fun':torque_constraint_motor7},
                {'type':'ineq', 'fun':torque_constraint_motor8}]

# hess = lambda x: np.zeros(16)

start = time.time()

sol = minimize(objective, x, method='trust-constr', bounds=bnds, constraints=constraints, options={'gtol': 1e-8})

end = time.time()

print(sol)
print(end-start)

# time.sleep(timestep - (end-start))

print('\n')

print(thrust_constraint_x(sol.x))
print(thrust_constraint_y(sol.x))
print(thrust_constraint_z(sol.x))

print(torque_constraint_x(sol.x))
print(torque_constraint_y(sol.x))
print(torque_constraint_z(sol.x))

# print(objective(sol.x))


