#IMU

import adafruit_bno055
import board
import time

i2c = board.I2C()

sensor = adafruit_bno055.BNO055_I2C(i2c)




#while(count<15):
#	quaternionData = sensor.quaternion
#	eulerData = sensor.euler
#	gravityData = sensor.gravity
#	print("Euler: {}".format(eulerData))
#	print("Gravity: {}".format(gravityData))
#	print("Acceleration: {}".format(sensor.acceleration))
#	print("Gyroscope: {}".format(sensor.gyro))
#	print("Quaternion: {}".format(quaternionData))
#	print("Linear Acceleration: {}".format(sensor.linear_acceleration))
#	time.sleep(0.25) #only give reading every quarter second
#	count = count + 1


#def set_e_data():
#	eulerData = sensor.euler
#i = 1
#while(i<10):

#	quaternionData = sensor.quaternion
#	eulerData = sensor.euler
#	i =i +1
