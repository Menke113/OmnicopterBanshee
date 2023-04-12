
import Adafruit_BBIO.PWM as PWM
import time

motorPin1 = "P9_14" # not actually used # need to find
motorPin3 = "P9_29"
motorPin4 = "P9_31"
motorPin5 = "P9_28"
motorPin6 = "P9_42"

motorPin7 = "P8_13"
motorPin8 = "P9_34"
motorPin = "P9_36"
motorPin = "P8_19"

PWM.start(motorPin7,50,500)

duty_cycle = 50

print("plug in motor now")

time.sleep(5)

PWM.set_duty_cycle(motorPin7, duty_cycle)

time.sleep(2)

while(1):

	duty_cycle_input = input("Enter desired thrust level (50 is 0%, 100 is 100%: ")
	duty_cycle = int(duty_cycle_input)
	#print(duty_cycle)
	#permission = input("Proceed? (y/n): ")

	#if permission == "y":
	PWM.set_duty_cycle(motorPin7, duty_cycle)
	#else:
		#break
