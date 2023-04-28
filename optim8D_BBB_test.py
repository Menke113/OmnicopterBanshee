from scipy.optimize import minimize, NonlinearConstraint, LinearConstraint, HessianUpdateStrategy, BFGS
import numpy as np
import time
from optim8D_BBB import *

thrust_desired_x = np.random.uniform(-1.0,1.0) * 2000 * 0.009806652 # get these from from the dynamics function
torque_desired_x = np.random.uniform(-1.0,1.0) * 1e7 * 0.009806652
thrust_desired_y = np.random.uniform(-1.0,1.0) * 2000 * 0.009806652
torque_desired_y = np.random.uniform(-1.0,1.0) * 1e7 * 0.009806652
thrust_desired_z = np.random.uniform(-1.0,1.0) * 2000 * 0.009806652
torque_desired_z = np.random.uniform(-1.0,1.0) * 1e7 * 0.009806652

T = [thrust_desired_x, thrust_desired_y, thrust_desired_z, torque_desired_x, torque_desired_y, torque_desired_z]

optim_quadratic_8D(T)