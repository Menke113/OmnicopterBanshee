from scipy.optimize import minimize, NonlinearConstraint, LinearConstraint, HessianUpdateStrategy, BFGS
import numpy as np
import time
from optim8D import *
import dynamicsModel
from dynamicsModel import *
# get python PID library

print('\n \n \n')

w_max_2306 = 2000 # max angular rate of 2306 motors, rad/s
w_max_2806 = 2900 # max angular rate of 2806 motors, rad/s

# w_max_2306 = 200000000 # max angular rate of 2306 motors, rad/s
# w_max_2806 = 290000000 # max angular rate of 2806 motors, rad/s

max_thrust_motor_2306 = 1200 * 0.009806652 # max thrust of an individual 2306 motor in grams to Newtons - should be scaled by throttle input (Rx_chan[2])
max_thrust_motor_2806 = 1600 * 0.009806652 # max thrust of an individual 2806 motor in grams to Newtons - this is an estimate - should be scaled by throttle input (Rx_chan[2])
max_torque_motor_2306 = 40 # torque, N * mm
max_torque_motor_2806 = 60 # torque, N * mm

# rng = np.random.uniform(-1.0,1.0)
# print(rng)
# rng.random()
# xp_quadratic = [np.random.uniform(-0.1,0.1) * xp[1], np.random.uniform(-1.0,1.0) * w_max_2306, np.random.uniform(-1.0,1.0) * w_max_2306, np.random.uniform(-1.0,1.0) * w_max_2306, 
# np.random.uniform(-1.0,1.0) * w_max_2806, np.random.uniform(-1.0,1.0) * w_max_2806, np.random.uniform(-1.0,1.0) * w_max_2806, np.random.uniform(-1.0,1.0) * w_max_2806]

x = np.zeros(8)
print("here")
T = asyncio.run(get_T())
print(T)
"""
thrust_desired_x = np.random.uniform(-1.0,1.0) * 2000 * 0.009806652 # get these from the dynamics function
torque_desired_x = np.random.uniform(-1.0,1.0) * 16
thrust_desired_y = np.random.uniform(-1.0,1.0) * 2000 * 0.009806652
torque_desired_y = np.random.uniform(-1.0,1.0) * 16
thrust_desired_z = np.random.uniform(-1.0,1.0) * 2000 * 0.009806652
torque_desired_z = np.random.uniform(-1.0,1.0) * 16

T = [thrust_desired_x, thrust_desired_y, thrust_desired_z, torque_desired_x, torque_desired_y, torque_desired_z]
"""
#xp_quadratic = optim_quadratic_8D(T)

k_t = max_thrust_motor_2306/(w_max_2306**2)

xp_quadratic_squared_times_kt = [x**2 * np.sign(x) * k_t for x in xp_quadratic]

print(T)
T = [t + t * np.random.uniform(-0.1,0.1) for t in T]
print(T)


print('desired thrusts: ')
print(thrust_desired_x)
print(thrust_desired_y)
print(thrust_desired_z)

print(torque_desired_x)
print(torque_desired_y)
print(torque_desired_z)

# k_t = 0.006 # thrust coefficient, using N and rad/s

print('k_t is:')
print(k_t)


k_r = 1
k_e = 1

# I = 2.03e-2 * 0.009806652  # propeller + motor bell moment of inertia, in g/mm^2 to Newtons/mm^2
a = 322.58 # the diagonal distance from the quadrotor motors to the center of the frame in mm

timestep = 0.1 # 10Hz update rate, must be enforced (change if code can't run fast enough)

def objective(x): # x is new 8D vector of w's, xp_quadratic is prev 8D vector of w's
    
    return (np.sum(x - (xp_quadratic**2 * k_t)))

# constraints to achieve the correct thrust and torque componen


def thrust_constraint_x(x): # this only does total thrust and total torque, must break up into three dimensions
    return  (x[4] + x[5]) - T[0]

def torque_constraint_x(x): # torque from just the thrust forces, not including moments from the props
    return (np.sqrt(2)/2 * a * x[0]) + (np.sqrt(2)/2 * a * x[2]) \
            - (np.sqrt(2)/2 * a * x[1]) - (np.sqrt(2)/2 * a * x[3]) - T[3]

def thrust_constraint_y(x): # this only does total thrust and total torque, must break up into three dimensions
    return (x[6] + x[7]) - T[1]

def torque_constraint_y(x): # torque from just the thrust forces, not including moments from the props
    return (np.sqrt(2)/2 * a * x[0]) + (np.sqrt(2)/2 * a * x[1]) \
            - (np.sqrt(2)/2 * a * x[2]) - (np.sqrt(2)/2 * a * x[3]) - T[4]


def thrust_constraint_z(x): # this only does total thrust and total torque, must break up into three dimensions
    return (x[0] + x[1] + x[2] + x[3] - T[2])


# bnds = [(-w_max_2306, w_max_2306), (-w_max_2306, w_max_2306), (-w_max_2306, w_max_2306), (-w_max_2306, w_max_2306), (-w_max_2806, w_max_2806), (-w_max_2806, w_max_2806), (-w_max_2806, w_max_2806), (-w_max_2806, w_max_2806)] # bounds are 0 to max angular rate of motors
#bnds1 = b * 4

bnds = [(-np.inf, np.inf)] * 8
print('bnds: ')
print(bnds)

#b = [(-w_max_2806, w_max_2806)] # bounds are 0 to max angular rate of motors
#bnds2 = b * 4

#bnds = np.concatenate((bnds1, bnds2))

# print(bnds)

#bnds = None

# print(bnds)
# thrust_constraint = LinearConstraint((np.sum(k_t*(x[::2] - x[1::2]))), thrust_desired, thrust_desired)
# torque_constraint = LinearConstraint((np.sum(k_t*0.1*(x[::2] - x[1::2]))), torque_desired, torque_desired)

# constraint1 = NonlinearConstraint(thrust_constraint)
# constraint2 = NonlinearConstraint(torque_constraint)

# constraints = [{'type':'eq', 'fun':thrust_constraint_x},
#                {'type':'eq', 'fun':thrust_constraint_y},
#                {'type':'eq', 'fun':thrust_constraint_z},
#                {'type':'eq', 'fun':torque_constraint_x},
#                {'type':'eq', 'fun':torque_constraint_y},
#                {'type':'eq', 'fun':torque_constraint_z},
#                {'type':'ineq', 'fun':thrust_constraint_motor1},
#                {'type':'ineq', 'fun':thrust_constraint_motor2},
#                {'type':'ineq', 'fun':thrust_constraint_motor3},
#                {'type':'ineq', 'fun':thrust_constraint_motor4},
#                {'type':'ineq', 'fun':thrust_constraint_motor5},
#                {'type':'ineq', 'fun':thrust_constraint_motor6},
#                {'type':'ineq', 'fun':thrust_constraint_motor7},
#                {'type':'ineq', 'fun':thrust_constraint_motor8},
#                {'type':'ineq', 'fun':torque_constraint_motor1},
#                {'type':'ineq', 'fun':torque_constraint_motor2},
#                {'type':'ineq', 'fun':torque_constraint_motor3},
#                {'type':'ineq', 'fun':torque_constraint_motor4},
#                {'type':'ineq', 'fun':torque_constraint_motor5},
#                {'type':'ineq', 'fun':torque_constraint_motor6},
#                {'type':'ineq', 'fun':torque_constraint_motor7},
#                {'type':'ineq', 'fun':torque_constraint_motor8}]

               # thrust constraint in x axis: 
# constraints = [{'type':'eq', 'fun':lambda x: (x[4] + x[5]) - T[0]},

constraints = [{NonlinearConstraint(lambda x: (x[4] + x[5]) - T[0], [0], [0])},
               
               # thrust constraint in y axis:
               {'type':'eq', 'fun':lambda x: (x[6] + x[7]) - T[1]},

               # thrust constraint in z axis:
               {'type':'eq', 'fun':lambda x: (x[0] + x[1] + x[2] + x[3] - T[2])},

               # torque constraint in x axis:
               {'type':'eq', 'fun':lambda x:
                + (np.sqrt(2)/2 * a * x[0]) + (np.sqrt(2)/2 * a * x[2]) \
                - (np.sqrt(2)/2 * a * x[1]) - (np.sqrt(2)/2 * a * x[3]) - T[3]},

               # torque constraint in y axis:
               {'type':'eq', 'fun':lambda x:
                (np.sqrt(2)/2 * a * x[0]) + (np.sqrt(2)/2 * a * x[1]) \
                - (np.sqrt(2)/2 * a * x[2]) - (np.sqrt(2)/2 * a * x[3]) - T[4]},

               # max thrust constraints for all motors:
               {'type':'ineq', 'fun':lambda x: max_thrust_motor_2306 - np.abs(x[0])},
               {'type':'ineq', 'fun':lambda x: max_thrust_motor_2306 - np.abs(x[1])},
               {'type':'ineq', 'fun':lambda x: max_thrust_motor_2306 - np.abs(x[2])},
               {'type':'ineq', 'fun':lambda x: max_thrust_motor_2306 - np.abs(x[3])},
               {'type':'ineq', 'fun':lambda x: max_thrust_motor_2806 - np.abs(x[4])},
               {'type':'ineq', 'fun':lambda x: max_thrust_motor_2806 - np.abs(x[5])},
               {'type':'ineq', 'fun':lambda x: max_thrust_motor_2806 - np.abs(x[6])},
               {'type':'ineq', 'fun':lambda x: max_thrust_motor_2806 - np.abs(x[7])}]

print(constraints)

            #    ub = inf
            #    lb = 0`


# print(constraints)

#hess = lambda x: [0] * 8

#hess = BFGS()

#print(hess(x))

start = time.time()

# print('xp_quadratic: ')
# print(xp_quadratic)

# print('xp_quadratic_squared:')
# print(xp_quadratic_squared)


print(k_t)

sol = minimize(lambda x: (np.sum(x - (xp_quadratic_squared_times_kt))), xp_quadratic_squared_times_kt, method='trust-constr', bounds=bnds, constraints=constraints, options={'gtol': 1e-2, 'maxiter' : 1000}) #, jac = '2-point', hess = hess)

end = time.time()

# print(sol)
# print(end-start)f

# time.sleep(timestep - (end-start))

print(' \n RESULT INFORMATION - OPTIMTHRUST: \n')


def thrust_result_x(x): # this only does total thrust and total torque, must break up into three dimensions
    return  (x[4] + x[5])

def torque_result_x(x): # torque from just the thrust forces, not including moments from the props
    return (np.sqrt(2)/2 * a * x[0]) + (np.sqrt(2)/2 * a * x[2]) \
            - (np.sqrt(2)/2 * a * x[1]) - (np.sqrt(2)/2 * a * x[3])

def thrust_result_y(x): # this only does total thrust and total torque, must break up into three dimensions
    return (x[6] + x[7])

def torque_result_y(x): # torque from just the thrust forces, not including moments from the props
    return (np.sqrt(2)/2 * a * x[0]) + (np.sqrt(2)/2 * a * x[1]) \
            - (np.sqrt(2)/2 * a * x[2]) - (np.sqrt(2)/2 * a * x[3])

def thrust_result_z(x): # this only does total thrust and total torque, must break up into three dimensions
    return x[0] + x[1] + x[2] + x[3]



print('success value of the optimization:')
print(sol.success)
# print('\n')

print('solution of the optimization:')
print(sol.x)
# print('\n')

print(xp_quadratic_squared_times_kt)

print('runtime of the optimization:')
print(sol.execution_time)
# print('\n')

print('thrust constraint functions with sol.x passed in, result should be zero if the constraint is satisfied by the solution:')

print(thrust_constraint_x(sol.x))
print(thrust_constraint_y(sol.x))
print(thrust_constraint_z(sol.x))

# print('\n')

print('torque constraint functions with sol.x passed in, result should be zero if the constraint is satisfied by the solution:')

print(torque_constraint_x(sol.x))
print(torque_constraint_y(sol.x))
# print(torque_constraint_z(sol.x))

# print('\n')

print('thrust result functions with sol.x passed in, result is the thrust created in each axis by the calculated solution:')

print(thrust_result_x(sol.x))
print(thrust_result_y(sol.x))
# print(thrust_result_z(sol.x))

# print('\n')

print('torque result functions with sol.x passed in, result is the torque created in each axis by the calculated solution:')

print(torque_result_x(sol.x))
print(torque_result_y(sol.x))


print('\n COMPARISON OF RESULTS: \n')


print('thrusts of motors via quadratic, omega-based optimization: ')
print(xp_quadratic_squared_times_kt)

print('thrusts of motors via linear, thrust-based optimization: ')
print(sol.x)

