import Adafruit_BBIO.PWM as PWM
import time

motorPin = "P9_14"
PWM.start(motorPin,50,500) #50Hz PWM frequency, will want to ramp this up later - ESCs can go up to 500Hz

duty_cycle = 50

print("plug in motor now")

time.sleep(5)

PWM.set_duty_cycle(motorPin, duty_cycle)

time.sleep(2)

while(1):

	duty_cycle_input = input("Enter desired thrust level (50 is 0%, 100 is 100%: ")
	duty_cycle = int(duty_cycle_input)
	#print(duty_cycle)
	#permission = input("Proceed? (y/n): ")

	#if permission == "y":
	PWM.set_duty_cycle(motorPin, duty_cycle)
	#else:
		#break

