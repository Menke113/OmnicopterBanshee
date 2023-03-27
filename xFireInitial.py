import serial
import Adafruit_BBIO.UART as UART
import sbus
import pyb
from receiver.sbus_receiver import SBUSReceiver

UART.setup("UART1")


def update_rx_data(timRx):
        global update_rx
        update_rx = True

def status_led(tim1):
        global updateLed
        updateLed = True
        led.toggle()

updateLed = False
update_rx = False
led = pyb.LED(4)

xfire = sbus.Serial('/dev/ttyO1', 100000)
xfire = SBUSReceiver(1)

timRx = pyb.Timer(1)

timRx.init(freq = 2778)
timRx.callback(update_rx_data)

tim1.init(freq=1)
tim1.callback(status_led)

tim1 = pyb.Timer(1)

while(1):

        if update_rx:
                xfire.get_new_data()
                update_rx = False

        if updateLed:
                print((xfire.get_rx_channels())
                updateLed = False #syntax error

