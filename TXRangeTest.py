# while 1:
#     if Rx_chan[0] < 200 and Rx_chan[1] < 200 and Rx_chan[2] < 200 and Rx_chan[3] < 200 and Rx_chan[4] > 1800:
#         break

# Rx_chanp[1]  -> yaw

# Rx_chanp[2]  -> throttle

# Rx_chanp[3]  -> pitch

# Rx_chanp[4]  -> roll


# import Adafruit_BBIO.PWM as PWM
# import time

# motorPin1 = "P9_14"

# motorPin2 = "P8_46"

# motorPin3 = "P9_29"
# motorPin4 = "P9_31"
# motorPin5 = "P8_13"
# motorPin6 = "P8_19"
# motorPin7 = "P8_34"
# motorPin8 = "P8_36"


# PWM.start(motorPin1,75,500) #50Hz PWM frequency, will want to ramp this up later - ESCs can go up to 500Hz
# PWM.start(motorPin2,75,500)
# PWM.start(motorPin3,75,500)
# PWM.start(motorPin4,75,500)
# PWM.start(motorPin5,75,500)
# PWM.start(motorPin6,75,500)
# PWM.start(motorPin7,75,500)
# PWM.start(motorPin8,75,500)

# duty_cycle = 75

# print("plug in now")

# time.sleep(5)

# PWM.set_duty_cycle(motorPin1, duty_cycle)
# PWM.set_duty_cycle(motorPin2, duty_cycle)
# PWM.set_duty_cycle(motorPin3, duty_cycle)
# PWM.set_duty_cycle(motorPin4, duty_cycle)
# PWM.set_duty_cycle(motorPin5, duty_cycle)
# PWM.set_duty_cycle(motorPin6, duty_cycle)
# PWM.set_duty_cycle(motorPin7, duty_cycle)
# PWM.set_duty_cycle(motorPin8, duty_cycle)


# time.sleep(2)

while(1):

    duty_cycle_input = input("Enter value from 173 to 1810: ")
    num = int(duty_cycle_input)
	#duty_cycle = (Rx_chan[2]-173)/(1810-173) * (50) + 50 
    duty_cycle = (num-173)/(1810-173) * (50) + 50 
    print(duty_cycle)

	#if permission == "y":
	#PWM.set_duty_cycle(motorPin, duty_cycle)
	# PWM.set_duty_cycle(motorPin1, duty_cycle)
	# PWM.set_duty_cycle(motorPin2, duty_cycle)
	# PWM.set_duty_cycle(motorPin3, duty_cycle)
	# PWM.set_duty_cycle(motorPin4, duty_cycle)
	# PWM.set_duty_cycle(motorPin5, duty_cycle)
	# PWM.set_duty_cycle(motorPin6, duty_cycle)
	# PWM.set_duty_cycle(motorPin7, duty_cycle)
	# PWM.set_duty_cycle(motorPin8, duty_cycle)

	#else:
		#break
