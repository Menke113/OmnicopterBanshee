import serial
import Adafruit_BBIO.UART as UART
import time
import asyncio

UART.setup("UART1")

xfire = serial.Serial('/dev/ttyO1',100000)



while(1):

#	while xfire.inWaiting() == 0:
#		pass
#	time.sleep(0.000003)
	NMEA = xfire.readline()
	print(NMEA)

	
