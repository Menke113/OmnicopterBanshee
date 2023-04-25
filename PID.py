import numpy as np

# Assume we have a drone object with methods for getting its current velocity
# and setting its acceleration

def PID(cur_val, des_val, prev_error, Kp, Ki, Kd):

    # Get the current velocity of the drone
    current_state = drone.get_velocity()

    # Calculate the error between the setpoint and the current velocity
    error = np.abs(des_val - cur_val)

    # Calculate the integral term
    integral += error

    # Calculate the derivative term
    derivative = error - prev_error

    # Compute the new acceleration using the PID controller
    return Kp * error + Ki * integral + Kd * derivative



    # Set the acceleration of the drone
    # drone.set_acceleration(acceleration)

    # Update the previous error
    previous_error = error