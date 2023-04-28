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
from optimThrust_BBB import *
from dynamicsModel import *
from arm_condition import *


def control_loop():

    loop_timestep_optimThrust = 0.1

    motorPin1 = "P8_34"
    motorPin2 = "P9_29"
    motorPin3 = "P8_13"
    motorPin4 = "P9_31"
    motorPin5 = "P8_36"
    motorPin6 = "P8_46"
    motorPin7 = "P8_19"
    motorPin8 = "P9_14"

    max_yaw = 100 # max yaw rate, rad/s
    max_roll = 100 # max roll rate, rad/s
    max_pitch = 100 # max pitch rate, rad/s

    des_x = 0
    des_y = 0
    des_z = 0

    Rx_chan = asyncio.run(get_frame())
    prev_switch4 = Rx_chan[4]

    prev_error_x = 0
    prev_error_y = 0
    prev_error_z = 0

    prev_error_yaw = 0
    prev_error_roll = 0
    prev_error_pitch = 0

    error_sum_x = 0
    error_sum_y = 0
    error_sum_z = 0

    error_sum_yaw = 0
    error_sum_roll = 0
    error_sum_pitch = 0

    i = 0

    max_thrust_motor_2306 = 1200 * 0.009806652 # grams to Newtons
    max_thrust_motor_2806 = 1600 * 0.009806652 # grams to Newtons

    max_thrusts = [max_thrust_motor_2306, max_thrust_motor_2306, max_thrust_motor_2306, max_thrust_motor_2306, max_thrust_motor_2806, max_thrust_motor_2806, max_thrust_motor_2806, max_thrust_motor_2806]

    omegas = [0] * 8
    thrusts = [0] * 8

    begin = time.time()

    prev_switch5 = 0

    while(1):

        # print('Control Loop Ran')

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
            arm_loop()

        Rx_chan = asyncio.run(get_frame())
        s0 = (Rx_chan[0]-173)/(1810-173) # yaw input, 0-1
        s1 = (Rx_chan[1]-173)/(1810-173) # throttle input, 0-1
        s2 = (Rx_chan[2]-173)/(1810-173) # pitch input, 0-1
        s3 = (Rx_chan[3]-173)/(1810-173) # roll input, 0-1

        des_yaw = s0 * max_yaw
        des_roll = s3 * max_roll
        des_pitch = s2 * max_pitch

        prev_switch5 = 

        # set desired x velocity:
        if Rx_chan[6] > 1800 and Rx_chan[5] != prev_switch5 and i != 0:

            if Rx_chan[5] > 1800 and Rx_chan[5]:
                print('Set des_x to 100')
                des_x = 100 # 100 mm/s

            if Rx_chan[5] < 1800 and Rx_chan[5] > 250:
                print('Set des_x to 0')
                des_x = 0 # 100 mm/s

            if Rx_chan[5] < 250:
                print('Set des_x to -100')
                des_x = -100 # 100 mm/s

        # set desired y velocity:
        if Rx_chan[6] < 1800 and Rx_chan[6] > 250 and Rx_chan[5] != prev_switch5 and i != 0:

            if Rx_chan[5] > 1800:
                print('Set des_y to 100')
                des_y = 100 # 100 mm/s

            if Rx_chan[5] < 1800 and Rx_chan[4] > 250:
                print('Set des_y to 0')
                des_y = 0 # 100 mm/s

            if Rx_chan[5] < 250:
                print('Set des_y to -100')
                des_y = -100 # 100 mm/s

        # set desired z velocity:
        if Rx_chan[6] < 250 and Rx_chan[5] != prev_switch5 and i != 0:

            if Rx_chan[5] > 1800:
                print('Set des_z to 100')
                des_z = 100 # 100 mm/s

            if Rx_chan[5] < 1800 and Rx_chan[5] > 250:
                print('Set des_z to 0')
                des_z = 0 # 100 mm/s

            if Rx_chan[5] < 250:
                print('Set des_z to -100')
                des_z = -100 # 100 mm/s

        # prev_switch4 = Rx_chan[4]
        prev_switch5 = Rx_chan[5]
        
        linvel = get_linearVelocity()
        
        phi = get_euler()[0]
        theta = get_euler()[1]
        psi = get_euler()[2]

        # run through a PID loop for each DOF:

        PID_x = PID(linvel[0], des_x, prev_error_x, error_sum_x, 8.5, 1.1, 1)
        xlinc = PID_x[0] # controller output for linear x DOF
        prev_error_x = PID_x[1]
        error_sum_x = PID_x[2]

        PID_y = PID(linvel[1], des_y, prev_error_y, error_sum_y, 8.5, 1.1, 1)
        ylinc = PID_y[0] # controller output for linear y DOF
        prev_error_y = PID_y[1]
        error_sum_y = PID_y[2]

        PID_z = PID(linvel[2], des_z, prev_error_z, error_sum_z, 8.5, 1.1, 1)
        zlinc = PID_z[0] # controller output for linear z DOF
        prev_error_z = PID_z[1]
        error_sum_z = PID_z[2]

        PID_yaw = PID(psi, des_yaw, prev_error_yaw, error_sum_yaw, 3, 2, 0.75)
        yawc = PID_yaw[0] # controller output for yaw DOF
        prev_error_yaw = PID_yaw[1]
        error_sum_yaw = PID_yaw[2]

        PID_roll = PID(phi, des_roll, prev_error_roll, error_sum_roll, 3, 2, 0.75)
        rollc = PID_roll[0] # controller output for roll DOF
        prev_error_roll = PID_roll[1]
        error_sum_roll = PID_roll[2]

        PID_pitch = PID(theta, des_pitch, prev_error_pitch, error_sum_pitch, 3, 2, 0.75)
        pitchc = PID_pitch[0] # controller output for pitch DOF
        prev_error_pitch = PID_pitch[1]
        error_sum_pitch = PID_pitch[2]

        accel_mods = [xlinc, ylinc, zlinc, yawc, rollc, pitchc]

        # now that we have controller outputs, call dynamics function with the inputted desired accelerations scaled by the controller outputs:
        # print("here")
        T = asyncio.run(get_T())
        # print(T)

        # call optimization to get desired angular velocities of motors

        thrusts = optim_thrust(T, max_thrusts, thrusts)
        print('thrusts: ')
        print(thrusts)
        print('max_thrusts: ')
        print(max_thrusts)
        throttles = thrusts/max_thrusts
        duty_cycles = throttles * 25 + 75
        for dc in duty_cycles:
            if dc < 50:
                dc = 50
            if dc > 100:
                dc = 100

        print('throttle levels: ')
        print(throttles)
        print('duty_cycles: ')
        print(duty_cycles)

        # now back out throttle level from desired omegas of the motors and command this throttle level to the motors
        
        if i != 0:

            PWM.set_duty_cycle(motorPin1, duty_cycles[0])
            PWM.set_duty_cycle(motorPin2, duty_cycles[1])
            PWM.set_duty_cycle(motorPin3, duty_cycles[2])
            PWM.set_duty_cycle(motorPin4, duty_cycles[3])
            PWM.set_duty_cycle(motorPin5, duty_cycles[4])
            PWM.set_duty_cycle(motorPin6, duty_cycles[5])
            PWM.set_duty_cycle(motorPin7, duty_cycles[6])
            PWM.set_duty_cycle(motorPin8, duty_cycles[7])
        
        end = time.time()

        i += 1

        if i % 100 == 0:
            i = 1
            print(end - begin)
