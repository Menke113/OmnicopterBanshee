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
from optim8D_BBB import *
from optimThrust_BBB import *
from dynamicsModel import *
from arm_condition import *
from control_loop import *


# initial arming loop:
arm_loop()

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

# k_t = 0.

control_loop()
