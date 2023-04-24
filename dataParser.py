# Data Parser
import IMU_Code
from IMU_Code import *
import HCSR04
import HCSR04_2
from HCSR04 import distance_measurement as d_m1
from HCSR04 import distance_measurement as d_m2
import time
import python_sbus_modified_HarryVersion
from python_sbus_modified_HarryVersion import *
import asyncio


def get_dis1():
	TRIG = "P8_16"
	ECHO = "P8_15"
	return d_m1(TRIG,ECHO)
def get_dis2():
	TRIG = "P8_18"
	ECHO = "P8_17"
	return d_m2(TRIG,ECHO)
#print(getdis1)
#print(getdis2)


## To get IMU use these functions
#get_euler()
#get_quaternion()
#get_acceleration()
#get_linear_acceleration()
#get_gyroscope()
#get_gravity()

#q = get_quaternion()
#print(q[1])
#a = get_acceleration()
#print(a[1])
#la = get_linear_acceleration()
#print(la[2])


## To get frame data use this function
# asyncio.run(get_frame())


async def get_frame():
	frame = await main()
	f = SBUSReceiver.SBUSFrame.get_rx_channels(frame)
	return [f[0],f[1],f[2],f[3],f[4],f[5],f[6]]
#print(asyncio.run(get_frame()))


while 1:
	print(asyncio.run(get_frame()))
