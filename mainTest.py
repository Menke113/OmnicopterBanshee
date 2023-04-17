import time
t1 = time.time()
import IMU_Code
from IMU_Code import *
import HCSR04
from HCSR04 import distance_measurement
t2 = time.time()


def get_distance():
	TRIG = "P8_16"
	ECHO = "P8_15"
	return distance_measurement(TRIG,ECHO)

## To get IMU use these functions
#get_euler()
#get_quaternion()
#get_acceleration()
#get_linear_acceleration()
#get_gyroscope()
#get_gravity()


#print(get_distance())


print(get_euler())
print(get_quaternion())
print(get_acceleration())
print(get_linear_acceleration())
print(get_gyroscope())
print(get_gravity())

#print(get_distance())
#print(get_distance())

#print(get_distance())

t3 = time.time()

t12 = t2-t1
t13 = t3-t1
print(t12)
print(t13)
