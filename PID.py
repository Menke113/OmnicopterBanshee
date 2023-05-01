import numpy as np

# Assume we have a drone object with methods for getting its current velocity
# and setting its acceleration

def PID(cur_val, des_val, prev_error, error_sum, Kp, Ki, Kd):

    # Calculate the error between the setpoint and the current velocity
    error = np.abs(des_val - cur_val)

    # Calculate the integral term
    error_sum += error

    # Calculate the derivative term
    derivative = error - prev_error

    # Compute the new acceleration using the PID controller
    return [Kp * error + Ki * error_sum + Kd * derivative, error, error_sum]



    # Set the acceleration of the drone
    # drone.set_acceleration(acceleration)

    # Update the previous error
    previous_error = error
