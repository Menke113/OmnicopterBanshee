from scipy.optimize import minimize, NonlinearConstraint, LinearConstraint, HessianUpdateStrategy, BFGS
import numpy as np
import time
from optimThrust_BBB import *

thrust_desired_x = np.random.uniform(-1.0,1.0) * 2000 * 0.009806652 # get these from from the dynamics function
torque_desired_x = np.random.uniform(-1.0,1.0) * 16
thrust_desired_y = np.random.uniform(-1.0,1.0) * 2000 * 0.009806652
torque_desired_y = np.random.uniform(-1.0,1.0) * 16
thrust_desired_z = np.random.uniform(-1.0,1.0) * 2000 * 0.009806652
torque_desired_z = np.random.uniform(-1.0,1.0) * 16

T = [thrust_desired_x, thrust_desired_y, thrust_desired_z, torque_desired_x, torque_desired_y, torque_desired_z]

max_thrust_motor_2306 = 1200 * 0.009806652 # grams to Newtons
max_thrust_motor_2806 = 1600 * 0.009806652 # grams to Newtons

max_thrusts = [max_thrust_motor_2306, max_thrust_motor_2306, max_thrust_motor_2306, max_thrust_motor_2306, max_thrust_motor_2806, max_thrust_motor_2806, max_thrust_motor_2806, max_thrust_motor_2806]

optim_thrust(T, max_thrusts, [0] * 8)