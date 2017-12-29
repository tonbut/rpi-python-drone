from AirMainLoop import AirMainLoop
import sys, time
#from setRadio import SetRadio

#SetRadio("/dev/ttyS0")

loop=AirMainLoop(
	multiwii="/dev/ttyUSB0",
	radio="/dev/ttyS0" )
loop.start()

while True:
	time.sleep(1)