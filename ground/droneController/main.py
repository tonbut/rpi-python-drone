from GroundMainLoop import GroundMainLoop
import sys, time
#from setRadio import SetRadio

#SetRadio("/dev/ttyUSB0")

loop=GroundMainLoop(
	joystick="/dev/input/js0",
	radio="/dev/ttyUSB0" )
loop.start()

while True:
	time.sleep(1)
	