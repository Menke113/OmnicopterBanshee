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
from SOE_solver import *



for i in range(100):

    # T = np.array([np.random.uniform(-1,1) * 25, np.random.uniform(-1,1) * 25, np.random.uniform(-1,1) * 24, np.random.uniform(-1,1) * 2500, 
    # np.random.uniform(-1,1) * 2500, np.random.uniform(-1,1) * 0])

    T = np.array([np.random.uniform(-1,0) * 25, np.random.uniform(-1,0) * 25, np.random.uniform(-1,0) * 24, np.random.uniform(-1,0) * 2500, 
    np.random.uniform(-1,0) * 2500, np.random.uniform(-1,0) * 0])

    thrusts = get_thrusts(T)

    print(thrusts)
