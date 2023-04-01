import Adafruit_BBIO.PWM as PWM
import time

motorPin = "P9_14"
PWM.start(motorPin,100,500) #50Hz PWM frequency, will want to ramp this up later - ESCs can go up to 500Hz

duty_cycle = 55

PWM.set_duty_cycle(motorPin, duty_cycle)

while(duty_cycle > 0):

	time.sleep(0.1)
	PWM.set_duty_cycle(motorPin, duty_cycle)
	duty_cycle = duty_cycle - 5;

