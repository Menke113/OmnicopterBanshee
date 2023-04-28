import Adafruit_BBIO.GPIO as GPIO
import time
#Distance Sensor 1
vcc = "5v"
trigger = "P8_16"
echo = "P8_15"
gnd = "GND"


GPIO.cleanup()


def distance_measurement(TRIG,ECHO):
	GPIO.output(TRIG, True)
	time.sleep(0.00001)
	GPIO.output(TRIG, False)
	pulseStart = time.time()
	pulseEnd = time.time()
	count = 0
	while GPIO.input(ECHO) == 0:
		pulseStart = time.time()
		count += 1
	while GPIO.input(ECHO) == 1:
		pulseEnd = time.time()

	pulseDuration = pulseEnd - pulseStart
	distance = pulseDuration * 17150 / 100
	distaance = round(distance, 2)
	return distance


#Configuration
#print("trigger: [{}]".format(trigger))
GPIO.setup(trigger, GPIO.OUT) #Trigger
#print("echo: [{}]".format(echo))
GPIO.setup(echo, GPIO.IN) #Echo
GPIO.output(trigger, False)
#print("Setup completed!")

# Security
GPIO.output(trigger, False)
time.sleep(0.5)

#distance = distance_measurement(trigger,echo)
#while True:
#	text = "Distance: [{}] m.".format(distance)
#	print(text)
#	time.sleep(2)
#	distance = distance_measurement(trigger, echo)

GPIO.cleanup()
#print("Done")
