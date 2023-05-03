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

motorPin = "P8_19"
PWM.start(motorPin,75,500)

PWM.set_duty_cycle(motorPin, 75)
time.sleep(2)

while 1:

    Rx_chan = asyncio.run(get_frame())
    s1 = (Rx_chan[1]-173)/(1810-173) # throttle input, 0-1

    duty_cycle = (((s1 - 0.5) * 2) * 25) + 75

    PWM.set_duty_cycle(motorPin, duty_cycle)


