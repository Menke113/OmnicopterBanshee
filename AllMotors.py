import Adafruit_BBIO.PWM as PWM
import time

motorPin1 = "P9_14"

motorPin2 = "P8_46"

motorPin3 = "P9_29"
motorPin4 = "P9_31"
motorPin5 = "P8_13"
motorPin6 = "P8_19"
motorPin7 = "P8_34"
motorPin8 = "P8_36"


PWM.start(motorPin1,50,500) #50Hz PWM frequency, will want to ramp this up later - ESCs can go up to 500Hz
PWM.start(motorPin2,50,500)
PWM.start(motorPin3,50,500)
PWM.start(motorPin4,50,500)
PWM.start(motorPin5,50,500)
PWM.start(motorPin6,50,500)
PWM.start(motorPin7,50,500)
PWM.start(motorPin8,50,500)

duty_cycle = 50

print("plug in now")

time.sleep(5)

PWM.set_duty_cycle(motorPin1, duty_cycle)
PWM.set_duty_cycle(motorPin2, duty_cycle)
PWM.set_duty_cycle(motorPin3, duty_cycle)
PWM.set_duty_cycle(motorPin4, duty_cycle)
PWM.set_duty_cycle(motorPin5, duty_cycle)
PWM.set_duty_cycle(motorPin6, duty_cycle)
PWM.set_duty_cycle(motorPin7, duty_cycle)
PWM.set_duty_cycle(motorPin8, duty_cycle)


time.sleep(2)

while(1):

	duty_cycle_input = input("Enter desired thrust level (50 is 0%, 100 is 100%: ")
	duty_cycle = int(duty_cycle_input)
	#print(duty_cycle)
	#permission = input("Proceed? (y/n): ")

	#if permission == "y":
	#PWM.set_duty_cycle(motorPin, duty_cycle)
	PWM.set_duty_cycle(motorPin1, duty_cycle)
	PWM.set_duty_cycle(motorPin2, duty_cycle)
	PWM.set_duty_cycle(motorPin3, duty_cycle)
	PWM.set_duty_cycle(motorPin4, duty_cycle)
	PWM.set_duty_cycle(motorPin5, duty_cycle)
	PWM.set_duty_cycle(motorPin6, duty_cycle)
	PWM.set_duty_cycle(motorPin7, duty_cycle)
	PWM.set_duty_cycle(motorPin8, duty_cycle)

	#else:
		#break
