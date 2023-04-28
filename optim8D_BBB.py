from scipy.optimize import minimize, NonlinearConstraint, LinearConstraint, HessianUpdateStrategy, BFGS
import numpy as np
import time

def optim_quadratic_8D(T):

    # print('Optim8D Ran')

    w_max_2306 = 2000 # max angular rate of 2306 motors, rad/s
    w_max_2806 = 2900 # max angular rate of 2806 motors, rad/s

    # w_max_2306 = 200000000 # max angular rate of 2306 motors, rad/s
    # w_max_2806 = 290000000 # max angular rate of 2806 motors, rad/s

    max_thrust_motor_2306 = 1200 * 0.009806652 # max thrust of an individual 2306 motor in grams to Newtons - should be scaled by throttle input (Rx_chan[2])
    max_thrust_motor_2806 = 1600 * 0.009806652 # max thrust of an individual 2806 motor in grams to Newtons - this is an estimate - should be scaled by throttle input (Rx_chan[2])
    max_torque_motor_2306 = 40 # torque, N * mm
    max_torque_motor_2806 = 60 # torque, N * mm

    rng = np.random.uniform(-1.0,1.0)
    # print(rng)
    # rng.random()
    xp = [np.random.uniform(-1.0,1.0) * w_max_2306, np.random.uniform(-1.0,1.0) * w_max_2306, np.random.uniform(-1.0,1.0) * w_max_2306, np.random.uniform(-1.0,1.0) * w_max_2306, 
    np.random.uniform(-1.0,1.0) * w_max_2806, np.random.uniform(-1.0,1.0) * w_max_2806, np.random.uniform(-1.0,1.0) * w_max_2806, np.random.uniform(-1.0,1.0) * w_max_2806]

    x = np.zeros(8)

    # thrust_desired_x = np.random.uniform(-1.0,1.0) * 2000 * 0.009806652 # get these from from the dynamics function
    # torque_desired_x = np.random.uniform(-1.0,1.0) * 1e7 * 0.009806652
    # thrust_desired_y = np.random.uniform(-1.0,1.0) * 2000 * 0.009806652
    # torque_desired_y = np.random.uniform(-1.0,1.0) * 1e7 * 0.009806652
    # thrust_desired_z = np.random.uniform(-1.0,1.0) * 2000 * 0.009806652
    # torque_desired_z = np.random.uniform(-1.0,1.0) * 1e7 * 0.009806652

    # k_t = 0.006 # thrust coefficient, using N and rad/s
    k_t = max_thrust_motor_2306/(w_max_2306**2)
    k_r = 1
    k_e = 1

    I = 2.03e-14 * 0.009806652  # propeller moment of inertia, in g/mm^2 to Newtons/mm^2
    a = 322.58 # the diagonal distance from the quadrotor motors to the center of the frame in mm

    timestep = 0.1 # 10Hz update rate, must be enforced (change if code can't run fast enough)

    # bnds = [(-w_max_2306, w_max_2306), (-w_max_2306, w_max_2306), (-w_max_2306, w_max_2306), (-w_max_2306, w_max_2306), (-w_max_2806, w_max_2806), (-w_max_2806, w_max_2806), (-w_max_2806, w_max_2806), (-w_max_2806, w_max_2806)] # bounds are 0 to max angular rate of motors
    #bnds1 = b * 4

    bnds = [(-np.inf, np.inf)] * 8

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
    constraints = [{'type':'eq', 'fun':lambda x: ((k_t * x[4]**2) * np.sign(x[4])) + ((k_t * x[5]**2) * np.sign(x[5])) - T[0]},
                
                # thrust constraint in y axis:
                {'type':'eq', 'fun':lambda x: ((k_t * x[6]**2) * np.sign(x[6])) + ((k_t * x[7]**2) * np.sign(x[7])) - T[1]},

                # thrust constraint in z axis:
                {'type':'eq', 'fun':lambda x: ((k_t * x[0]**2) * np.sign(x[0])) + ((k_t * x[1]**2) * np.sign(x[1])) + ((k_t * x[2]**2) * np.sign(x[2])) + ((k_t * x[3]**2) * np.sign(x[3])) - T[2]},

                # torque constraint in x axis:
                {'type':'eq', 'fun':lambda x: - (I * ((x[4] - xp[4])/timestep)) - (I * ((x[5] - xp[5])/timestep)) \
                    + (np.sqrt(2)/2 * a * ((k_t * x[0]**2) * np.sign(x[0]))) + (np.sqrt(2)/2 * a * ((k_t * x[2]**2) * np.sign(x[2]))) \
                    - (np.sqrt(2)/2 * a * ((k_t * x[1]**2) * np.sign(x[1]))) - (np.sqrt(2)/2 * a * ((k_t * x[3]**2) * np.sign(x[3]))) - T[3]},

                # torque constraint in y axis:
                {'type':'eq', 'fun':lambda x: - (I * ((x[6] - xp[6])/timestep)) - (I * ((x[7] - xp[7])/timestep)) \
                    + (np.sqrt(2)/2 * a * ((k_t * x[0]**2) * np.sign(x[0]))) + (np.sqrt(2)/2 * a * ((k_t * x[1]**2) * np.sign(x[1]))) \
                    - (np.sqrt(2)/2 * a * ((k_t * x[2]**2) * np.sign(x[2]))) - (np.sqrt(2)/2 * a * ((k_t * x[3]**2) * np.sign(x[3]))) - T[4]},

                # torque constraint in z axis:
                {'type':'eq', 'fun':lambda x: - (I * ((x[0] - xp[0])/timestep)) - (I * ((x[1] - xp[1])/timestep)) - (I * ((x[2] - xp[2])/timestep)) - (I * ((x[3] - xp[3])/timestep)) - T[5]},

                # max thrust constraints for all motors:
                {'type':'ineq', 'fun':lambda x: max_thrust_motor_2306 - (k_t*(x[0]**2))},
                {'type':'ineq', 'fun':lambda x: max_thrust_motor_2306 - (k_t*(x[1]**2))},
                {'type':'ineq', 'fun':lambda x: max_thrust_motor_2306 - (k_t*(x[2]**2))},
                {'type':'ineq', 'fun':lambda x: max_thrust_motor_2306 - (k_t*(x[3]**2))},
                {'type':'ineq', 'fun':lambda x: max_thrust_motor_2306 - (k_t*(x[4]**2))},
                {'type':'ineq', 'fun':lambda x: max_thrust_motor_2306 - (k_t*(x[5]**2))},
                {'type':'ineq', 'fun':lambda x: max_thrust_motor_2306 - (k_t*(x[6]**2))},
                {'type':'ineq', 'fun':lambda x: max_thrust_motor_2306 - (k_t*(x[7]**2))},

                # max torque constraint for all motors:
                {'type':'ineq', 'fun':lambda x: max_torque_motor_2306 - (I * ((x[0] - xp[0])/timestep))},
                {'type':'ineq', 'fun':lambda x: max_torque_motor_2306 - (I * ((x[1] - xp[1])/timestep))},
                {'type':'ineq', 'fun':lambda x: max_torque_motor_2306 - (I * ((x[2] - xp[2])/timestep))},
                {'type':'ineq', 'fun':lambda x: max_torque_motor_2306 - (I * ((x[3] - xp[3])/timestep))},
                {'type':'ineq', 'fun':lambda x: max_torque_motor_2306 - (I * ((x[4] - xp[4])/timestep))},
                {'type':'ineq', 'fun':lambda x: max_torque_motor_2306 - (I * ((x[5] - xp[5])/timestep))},
                {'type':'ineq', 'fun':lambda x: max_torque_motor_2306 - (I * ((x[6] - xp[6])/timestep))},
                {'type':'ineq', 'fun':lambda x: max_torque_motor_2306 - (I * ((x[7] - xp[7])/timestep))}]

    #hess = lambda x: [0] * 8

    #hess = BFGS()

    sol = minimize(lambda x: np.abs(k_e * np.sum(x) + k_r * (np.sum(x - xp))), xp, method='trust-constr', bounds=bnds, constraints=constraints, options={'gtol': 1, 'maxiter' : 1000}) #, jac = '2-point', hess = hess)

    print(' \n RESULT INFORMATION - OPTIM8D: \n')


    def thrust_constraint_x(x): # this only does total thrust and total torque, must break up into three dimensions
        return ((k_t * x[4]**2) * np.sign(x[4])) + ((k_t * x[5]**2) * np.sign(x[5])) - T[0]

    def torque_constraint_x(x): # torque from just the thrust forces, not including moments from the props
        return - (I * ((x[4] - xp[4])/timestep)) - (I * ((x[5] - xp[5])/timestep)) \
                + (np.sqrt(2)/2 * a * ((k_t * x[0]**2) * np.sign(x[0]))) + (np.sqrt(2)/2 * a * ((k_t * x[2]**2) * np.sign(x[2]))) \
                - (np.sqrt(2)/2 * a * ((k_t * x[1]**2) * np.sign(x[1]))) - (np.sqrt(2)/2 * a * ((k_t * x[3]**2) * np.sign(x[3]))) - T[3]

    def thrust_constraint_y(x): # this only does total thrust and total torque, must break up into three dimensions
        return ((k_t * x[6]**2) * np.sign(x[6])) + ((k_t * x[7]**2) * np.sign(x[7])) - T[1]

    def torque_constraint_y(x): # torque from just the thrust forces, not including moments from the props
        return - (I * ((x[6] - xp[6])/timestep)) - (I * ((x[7] - xp[7])/timestep)) \
                + (np.sqrt(2)/2 * a * ((k_t * x[0]**2) * np.sign(x[0]))) + (np.sqrt(2)/2 * a * ((k_t * x[1]**2) * np.sign(x[1]))) \
                - (np.sqrt(2)/2 * a * ((k_t * x[2]**2) * np.sign(x[2]))) - (np.sqrt(2)/2 * a * ((k_t * x[3]**2) * np.sign(x[3]))) - T[4]


    def thrust_constraint_z(x): # this only does total thrust and total torque, must break up into three dimensions
        return ((k_t * x[0]**2) * np.sign(x[0])) + ((k_t * x[1]**2) * np.sign(x[1])) + ((k_t * x[2]**2) * np.sign(x[2])) + ((k_t * x[3]**2) * np.sign(x[3])) - T[2]

    def torque_constraint_z(x): # torque from just the thrust forces, not including moments from the props
        return - (I * ((x[0] - xp[0])/timestep)) - (I * ((x[1] - xp[1])/timestep)) - (I * ((x[2] - xp[2])/timestep)) - (I * ((x[3] - xp[3])/timestep)) - T[5]
    
    def thrust_result_x(x): # this only does total thrust and total torque, must break up into three dimensions
        return ((k_t * x[4]**2) * np.sign(x[4])) + ((k_t * x[5]**2) * np.sign(x[5]))

    def torque_result_x(x): # torque from just the thrust forces, not including moments from the props
        return - (I * ((x[4] - xp[4])/timestep)) - (I * ((x[5] - xp[5])/timestep)) \
                + (np.sqrt(2)/2 * a * ((k_t * x[0]**2) * np.sign(x[0]))) + (np.sqrt(2)/2 * a * ((k_t * x[2]**2) * np.sign(x[2]))) \
                - (np.sqrt(2)/2 * a * ((k_t * x[1]**2) * np.sign(x[1]))) - (np.sqrt(2)/2 * a * ((k_t * x[3]**2) * np.sign(x[3])))

    def thrust_result_y(x): # this only does total thrust and total torque, must break up into three dimensions
        return ((k_t * x[6]**2) * np.sign(x[6])) + ((k_t * x[7]**2) * np.sign(x[7]))

    def torque_result_y(x): # torque from just the thrust forces, not including moments from the props
        return - (I * ((x[6] - xp[6])/timestep)) - (I * ((x[7] - xp[7])/timestep)) \
                + (np.sqrt(2)/2 * a * ((k_t * x[0]**2) * np.sign(x[0]))) + (np.sqrt(2)/2 * a * ((k_t * x[1]**2) * np.sign(x[1]))) \
                - (np.sqrt(2)/2 * a * ((k_t * x[2]**2) * np.sign(x[2]))) - (np.sqrt(2)/2 * a * ((k_t * x[3]**2) * np.sign(x[3])))


    def thrust_result_z(x): # this only does total thrust and total torque, must break up into three dimensions
        return ((k_t * x[0]**2) * np.sign(x[0])) + ((k_t * x[1]**2) * np.sign(x[1])) + ((k_t * x[2]**2) * np.sign(x[2])) + ((k_t * x[3]**2) * np.sign(x[3]))

    def torque_result_z(x): # torque from just the thrust forces, not including moments from the props
        return - (I * ((x[0] - xp[0])/timestep)) - (I * ((x[1] - xp[1])/timestep)) - (I * ((x[2] - xp[2])/timestep)) - (I * ((x[3] - xp[3])/timestep))


    print('success value of the optimization:')
    print(sol.success)
    # print('\n')

    print('solution of the optimization:')
    print(sol.x)
    # print('\n')

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
    print(torque_constraint_z(sol.x))

    # print('\n')

    print('thrust result functions with sol.x passed in, result is the thrust created in each axis by the calculated solution:')

    print(thrust_result_x(sol.x))
    print(thrust_result_y(sol.x))
    print(thrust_result_z(sol.x))

    # print('\n')

    print('torque result functions with sol.x passed in, result is the torque created in each axis by the calculated solution:')

    print(torque_result_x(sol.x))
    print(torque_result_y(sol.x))
    print(torque_result_z(sol.x))

    # print('\n')

    # print(objective(sol.x))

    # must map from omegas to throttle to command (duty cycle)

    return sol.x
