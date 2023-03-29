#IMU

import adafruit_bno055
import board
import time

i2c = board.I2C()

sensor = adafruit_bno055.BNO055_I2C(i2c)


while(1):
	print("Euler: {}".format(sensor.euler))
	print("Gravity: {}".format(sensor.gravity))
	print("Acceleration: {}".format(sensor.acceleration))
	print("Gyroscope: {}".format(sensor.gyro))
	print("Quaternion: {}".format(sensor.quaternion))
	print("Linear Acceleration: {}".format(sensor.linear_acceleration))
	time.sleep(0.25) #only give reading every quarter second
