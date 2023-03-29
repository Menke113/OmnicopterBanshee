
#IMU

import adafruit_bno055
import board

i2c = board.I2C()

sensor = adafruit_bno055.BNO055_I2C(i2c)

count = 0

while(count<10):
	print('Euler: {}'.format(sensor.euler))
	print("Gravity: {}".format(sensor.gravity))
	print("Acceleration: {}".format(sensor.acceleration))
	print("Gyroscope: {}".format(sensor.gyro))
	print('Quaternion: {}'.format(sensor.quaternion))

	count += 1
