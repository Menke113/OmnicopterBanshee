#IMU
import adafruit_bno055
import board
import time

i2c = board.I2C()
sensor = adafruit_bno055.BNO055_I2C(i2c)


def get_euler():
	return sensor.euler
def get_quaternion():
	return sensor.quaternion
def get_acceleration():
	return sensor.acceleration
def get_gravity():
	return sensor.gravity
def get_gyroscope():
	return sensor.gyro
def get_linear_acceleration():
	return sensor.linear_acceleration


#quaternionData = sensor.quaternion
#eulerData = sensor.euler
#gravityData = sensor.gravity
#print("Euler: {}".format(eulerData))
#print("Gravity: {}".format(gravityData))
#print("Acceleration: {}".format(sensor.acceleration))
#print("Gyroscope: {}".format(sensor.gyro))
#print("Quaternion: {}".format(quaternionData))
#print("Linear Acceleration: {}".format(sensor.linear_acceleration))

