# Simple demo of of the PCA9685 PWM servo/LED controller library.
# This will move channel 0 from min to max position repeatedly.
# Author: Tony DiCola
# License: Public Domain
from __future__ import division
import time, sys
#import paho.mqtt.client as mqtt
from pyMultiwii import MultiWii
from soundEnvelope import SoundEnvelope, Envelope
sys.path.append('/home/pi/dev/mavlink/')
from pymavlink import mavutil
import pymavlink.dialects.v10.ardupilotmega as mavlink


se=SoundEnvelope()

throttle=1000
yaw=1500
roll=1500
pitch=1500

def updateRC():
	global throttle,yaw,roll,pitch,board
	data = [pitch,roll,throttle,yaw,1000,1000,1000,1000]
	board.sendCMD(16,MultiWii.SET_RAW_RC,data)

	
def on_connect(client, userdata, flags, rc):
	print("Connected to MQTT broker "+str(rc))
	client.subscribe("test")

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
	global throttle,yaw,roll,pitch
	pl=str(msg.payload)
	i=pl.index(":")
	channel=pl[:i]
	value=int(pl[i+1:])
	print (channel,value)
	if channel=="throttle":
		throttle=value
	elif channel=="yaw":
		yaw=value
	elif channel=="pitch":
		pitch=value
	elif channel=="roll":
		roll=value
	#updateRC()
	


#client = mqtt.Client("", True, None, mqtt.MQTTv31)
#client.on_connect = on_connect
#client.on_message = on_message

#client.connect("192.168.1.1", 1883, 60)
#client.loop_start()

mav = mavutil.mavlink_connection("/dev/ttyS0", baud="57600", autoreconnect = True )

board = MultiWii("/dev/ttyUSB0")
try:
	board.disarm()
	
	e3=Envelope(8,48,0,0,5,16,16,127,0,0,-127,127,8)
	se.play([[e3,0,25]],pitchOffset=0,speed=1,legato=1.0)
	
	
	
	#print "arming..."
	#board.arm()
	lastHeartbeat=0
	
	while True:
	
		now=time.time()
		if now-lastHeartbeat>1:
			#print "heartbeat"
			lastHeartbeat=now
			mav.mav.heartbeat_send(0,0,0,0,0)
			time.sleep(0.05)
			#board.getData(MultiWii.MOTOR)
			#time.sleep(0.05)
			
			board.getData(MultiWii.ANALOG)
			#print board.vbat
			# def battery_status_send(self, id, battery_function, type, temperature, voltages, current_battery, current_consumed, energy_consumed, battery_remaining, force_mavlink1=False):
			millivolt=board.vbat*1000
			mav.mav.battery_status_send(0,0,0,0,[millivolt,0,0,0,0,0,0,0,0,0],0,0,0,-1);
		
		msg = mav.recv_msg()
		if msg!=None:
			#print msg.get_type()
			if msg.get_type()=='RC_CHANNELS_OVERRIDE':
				throttle=msg.chan1_raw
				yaw=msg.chan2_raw
		
		time.sleep(0.02)
		updateRC()
finally:
	#e3=Envelope(4,-4,0,0,4,3,1,127,0,0,-127,127,8)
	e3=Envelope(4,48,0,-38,4,0,4,127,0,0,-127,127,8)
	se.play([[e3,0,25]],pitchOffset=0,speed=2,legato=1.0)
	print "disarming..."
	board.disarm()
	#time.sleep(1)

