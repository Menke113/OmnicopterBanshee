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

def arm_loop():

    while 1:

        prev_switch = 2000
        Rx_chan = asyncio.run(get_frame())

        if Rx_chan[0] < 250 and Rx_chan[1] < 250 and Rx_chan[2] < 250 and Rx_chan[3] < 250 and Rx_chan[4] > 1800: #  and prev_switch < 1800:
            break
        prev_switch = Rx_chan[4]
