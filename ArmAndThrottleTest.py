import adafruit_bno055
import Adafruit_BBIO.PWM as PWM
import time
import dataParser
from dataParser import *
import asyncio
#import adafruit_bno055

while 1:

    Rx_chan = asyncio.run(get_frame())
    print(asyncio.run(get_frame()))
    if Rx_chan[0] < 250 and Rx_chan[1] < 250 and Rx_chan[2] < 250 and Rx_chan[3] < 250 and Rx_chan[4] > 1800:
        break

# Rx_chanp[1]  -> yaw

# Rx_chanp[2]  -> throttle

# Rx_chanp[3]  -> pitch

# Rx_chanp[4]  -> roll


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

duty_cycle = 75

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
    if Rx_chan[4] < 1800:
        duty_cycle = 75
        PWM.set_duty_cycle(motorPin1, duty_cycle)
        PWM.set_duty_cycle(motorPin2, duty_cycle)
        PWM.set_duty_cycle(motorPin3, duty_cycle)
        PWM.set_duty_cycle(motorPin4, duty_cycle)
        PWM.set_duty_cycle(motorPin5, duty_cycle)
        PWM.set_duty_cycle(motorPin6, duty_cycle)
        PWM.set_duty_cycle(motorPin7, duty_cycle)
        PWM.set_duty_cycle(motorPin8, duty_cycle)
        break
    Rx_chan = asyncio.run(get_frame())
   # duty_cycle_input = input("Enter value from 173 to 1810: ")
    duty_cycle = (Rx_chan[1]-173)/(1810-173) * (50) + 50 
   # duty_cycle = (duty_cycle_input-173)/(1810-173) * (50) + 50 
    print(duty_cycle)
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