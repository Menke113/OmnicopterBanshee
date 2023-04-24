#IMU
import adafruit_bno055
import board
import time
import math
import numpy as np

i2c = board.I2C()
sensor = adafruit_bno055.BNO055_I2C(i2c)


def get_euler():
	out = np.deg2rad(sensor.euler)
	return out
def get_quaternion():
	out = np.deg2rad(sensor.quaternion)
	return out
def get_acceleration():
	a = sensor.acceleration
	out = np.array([[a[0]],[a[1]],[a[2]]])
	return out
def get_gravity():
	return sensor.gravity
def get_gyro():
	out = np.deg2rad(sensor.gyro)
	return out
def get_linear_acceleration():
	return sensor.linear_acceleration

#Linear Velocity Calculation
SampleRateDelay_ms = 10 # how often to read data from the board
Accel_vel_transition = SampleRateDelay_ms / 1000 #
Deg2Rad = 0.01745329251 #trig functions require radians, BNO055 outputs$

def get_linearVelocity():
	headingVelx = Accel_vel_transition * sensor.linear_acceleration[0]/math.cos(Deg2Rad*sensor.euler[0])
	headingVely = Accel_vel_transition * sensor.linear_acceleration[1]/math.cos(Deg2Rad*sensor.euler[1])
	headingVelz = Accel_vel_transition * sensor.linear_acceleration[2]/math.cos(Deg2Rad*sensor.euler[2])
	headingVel = [headingVelx,headingVely,headingVelz]
	return headingVel


#quaternionData = sensor.quaternion
#eulerData = sensor.euler
#gravityData = sensor.gravity
#print("Euler: {}".format(get_euler()))
#print("Gravity: {}".format(gravityData))
#print("Acceleration: {}".format(sensor.acceleration))
#print("Gyroscope: {}".format(sensor.gyro))
#print("Quaternion: {}".format(quaternionData))
#print("Linear Acceleration: {}".format(sensor.linear_acceleration))


