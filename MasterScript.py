import adafruit_bno055
import Adafruit_BBIO.PWM as PWM
import time
# import dataParser
from dataParser import *
import asyncio
#import adafruit_bno055
# import IMU_Code
from IMU_Code import *
from PID import *
from optim8D import *
from optimThrust import *
from dynamicsModel import *


# initial arming loop:
while 1:

    prev_switch = 2000
    Rx_chan = asyncio.run(get_frame())

    if Rx_chan[0] < 250 and Rx_chan[1] < 250 and Rx_chan[2] < 250 and Rx_chan[3] < 250 and Rx_chan[4] > 1800 and prev_switch < 1800:
        break
    prev_switch = Rx_chan[4]

# Rx_chan[0]  -> yaw

# Rx_chan[1]  -> throttle

# Rx_chan[2]  -> pitch

# Rx_chan[3]  -> roll

# these pins now correspond to the motor number assignments:
motorPin1 = "P8_34"
motorPin2 = "P9_29"
motorPin3 = "P8_13"
motorPin4 = "P9_31"
motorPin5 = "P8_36"
motorPin6 = "P8_46"
motorPin7 = "P8_19"
motorPin8 = "P9_14"

# make sure this works, starting duty cycle was previously 50, changed to 75 to see if this will arm them w/o moving throttle stick to middle
PWM.start(motorPin1,75,500)
PWM.start(motorPin2,75,500)
PWM.start(motorPin3,75,500)
PWM.start(motorPin4,75,500)
PWM.start(motorPin5,75,500)
PWM.start(motorPin6,75,500)
PWM.start(motorPin7,75,500)
PWM.start(motorPin8,75,500)

duty_cycle = 75

# should initialize the ESCs:
PWM.set_duty_cycle(motorPin1, duty_cycle)
PWM.set_duty_cycle(motorPin2, duty_cycle)
PWM.set_duty_cycle(motorPin3, duty_cycle)
PWM.set_duty_cycle(motorPin4, duty_cycle)
PWM.set_duty_cycle(motorPin5, duty_cycle)
PWM.set_duty_cycle(motorPin6, duty_cycle)
PWM.set_duty_cycle(motorPin7, duty_cycle)
PWM.set_duty_cycle(motorPin8, duty_cycle)

time.sleep(2)

prev_switch4 = Rx_chan[4]

prev_error_x = 0
prev_error_y = 0
prev_error_z = 0

prev_error_yaw = 0
prev_error_roll = 0
prev_error_pitch = 0

k_t = 0.006

while(1):
    if Rx_chan[4] < 1800:
        duty_cycle = 75
        PWM.set_duty_cycle(motorPin1, duty_cycle)
        PWM.set_duty_cycle(motorPin2, duty_cycle)
        PWM.set_duty_cycle(motorPin3, duty_cycle)
        PWM.set_duty_cycle(motorPin4, duty_cycle)
        PWM.set_duty_cycle(motorPin5, duty_cycle)
        PWM.set_duty_cycle(motorPin6, duty_cycle)
        PWM.set_duty_cycle(motorPin7, duty_cycle)
        PWM.set_duty_cycle(motorPin8, duty_cycle)
        break
    Rx_chan = asyncio.run(get_frame())
    s0 = (Rx_chan[0]-173)/(1810-173) # yaw input, 0-1
    s1 = (Rx_chan[1]-173)/(1810-173) # throttle input, 0-1
    s2 = (Rx_chan[2]-173)/(1810-173) # pitch input, 0-1
    s3 = (Rx_chan[3]-173)/(1810-173) # roll input, 0-1

    max_yaw = 1 # max yaw rate, rad/s
    max_roll = 1 # max roll rate, rad/s
    max_pitch = 1 # max pitch rate, rad/s

    des_yaw = s0 * max_yaw
    des_roll = s3 * max_roll
    des_pitch = s2 * max_pitch

    # set desired x velocity:
    if Rx_chan[5] > 1800 and Rx_chan[4] != prev_switch4:

        if Rx_chan[4] > 1800 and Rx_chan[4]:
            des_x = 100 # 100 mm/s

        if Rx_chan[4] < 1800 and Rx_chan[4] > 250:
            des_x = 0 # 100 mm/s

        if Rx_chan[4] < 250:
            des_x = -100 # 100 mm/s

    # set desired y velocity:
    if Rx_chan[5] < 1800 and Rx_chan[5] > 250 and Rx_chan[4] != prev_switch4:

        if Rx_chan[4] > 1800:
            des_y = 100 # 100 mm/s

        if Rx_chan[4] < 1800 and Rx_chan[4] > 250:
            des_y = 0 # 100 mm/s

        if Rx_chan[4] < 250:
            des_y = -100 # 100 mm/s

    # set desired z velocity:
    if Rx_chan[5] < 250 and Rx_chan[4] != prev_switch4:

        if Rx_chan[4] > 1800:
            des_z = 100 # 100 mm/s

        if Rx_chan[4] < 1800 and Rx_chan[4] > 250:
            des_z = 0 # 100 mm/s

        if Rx_chan[4] < 250:
            des_z = -100 # 100 mm/s

    prev_switch4 = Rx_chan[4]
     
    linvel = get_linearVelocity
    
    # are these rates?
    phi = get_euler()[0]
    theta = get_euler()[1]
    psi = get_euler()[2]

    # run through a PID loop for each DOF:

    PID_x = PID(linvel[0], des_x, prev_error_x, 1, 1, 1)
    xlinc = PID_x[0] # controller output for linear x DOF
    prev_error_x = PID_x[1]

    PID_y = PID(linvel[1], des_y, prev_error_y, 1, 1, 1)
    ylinc = PID_y[0] # controller output for linear y DOF
    prev_error_y = PID_y[1]

    PID_z = PID(linvel[2], des_z, prev_error_z, 1, 1, 1)
    zlinc = PID_z[0] # controller output for linear z DOF
    prev_error_z = PID_z[1]

    PID_yaw = PID(psi, des_yaw, prev_error_yaw, 1, 1, 1)
    yawc = PID_yaw[0] # controller output for yaw DOF
    prev_error_yaw = PID_yaw[1]

    PID_roll = PID(phi, des_roll, prev_error_roll, 1, 1, 1)
    rollc = PID_roll[0] # controller output for roll DOF
    prev_error_roll = PID_roll[1]

    PID_pitch = PID(theta, des_pitch, prev_error_pitch, 1, 1, 1)
    pitchc = PID_pitch[0] # controller output for pitch DOF
    prev_error_pitch = PID_pitch[1]

    accel_mods = [xlinc, ylinc, zlinc, yawc, rollc, pitchc]

    # now that we have controller outputs, call dynamics function with the inputted desired accelerations scaled by the controller outputs:
    T = dynamicsModel(accel_mods)

    # call optimization to get desired angular velocities of motors
    omegas = optim_quadratic_8D(T)

    # now back out throttle level from desired omegas of the motors and command this throttle level to the motors
    
    thrust = k_t * omega**2

    PWM.set_duty_cycle(motorPin1, duty_cycle)
    PWM.set_duty_cycle(motorPin2, duty_cycle)
    PWM.set_duty_cycle(motorPin3, duty_cycle)
    PWM.set_duty_cycle(motorPin4, duty_cycle)
    PWM.set_duty_cycle(motorPin5, duty_cycle)
    PWM.set_duty_cycle(motorPin6, duty_cycle)
    PWM.set_duty_cycle(motorPin7, duty_cycle)
    PWM.set_duty_cycle(motorPin8, duty_cycle)
