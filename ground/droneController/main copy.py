from joystick import Joystick
from threading import Thread
import sys, time
#import paho.mqtt.client as mqtt
#import paho.mqtt.publish as publish
from soundEnvelope import SoundEnvelope, Envelope

sys.path.append('/home/pi/dev/mavlink/')
from pymavlink import mavutil
import pymavlink.dialects.v10.ardupilotmega as mavlink

joystick=None

def joystickLoop():
	global joystick
	while True:
		try:
			if joystick==None:
				joystick=Joystick("/dev/input/js0",onButton)
			joystick.poll()
			
		except:
			print "Unexpected error:", sys.exc_info()[0]
			joystick=None
			time.sleep(1)

def onButton(button, press):
	print button,press
	#if button=='circle':
	#	client.publish("test","kill: 0");

previousAxisValues = { 'throttle': 1000, 'yaw': 1500 }
lastRCSend=0
dirtyRC=False

def updateAxis(name,value):
	global previousAxisValues, lastRCSend, dirtyRC
	
	if not(previousAxisValues.has_key(name)) or previousAxisValues[name]!=value:
		previousAxisValues[name]=value
		dirtyRC=True
		#print "dirty"
		#s="{}: {:>6.0f}".format(name,value)
		#client.publish("test",s);
		#print s
		

def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))


#client = mqtt.Client("", True, None, mqtt.MQTTv31)
#client.on_connect = on_connect
#client.connect("192.168.1.1", 1883, 60)
#client.loop_start()

mav = mavutil.mavlink_connection("/dev/ttyUSB0", baud="57600", autoreconnect = True )
se=SoundEnvelope()

t = Thread(target=joystickLoop)
t.setDaemon(True)
t.start()

lastHeartbeat=0
lastRemoteHeartbeat=time.time()

while True:

	now=time.time()
	if now-lastHeartbeat>1:
		#print "heartbeat"
		lastHeartbeat=now
		mav.mav.heartbeat_send(0,0,0,0,0)
		
	msg = mav.recv_msg()
	if msg!=None:
		msgType=msg.get_type()
		#print msgType
		if msgType=='RADIO_STATUS':
			localSignal=(msg.rssi/1.9-127)
			localNoise=(msg.noise/1.9-127)
			localSNR=localSignal-localNoise
			remoteSignal=(msg.remrssi/1.9-127)
			remoteNoise=(msg.remnoise/1.9-127)
			remoteSNR=remoteSignal-remoteNoise
			print localSNR,  remoteSNR
			if now-lastRemoteHeartbeat>2:
				e3=Envelope(8,48,0,0,5,16,16,127,0,0,-127,127,8)
				se.play([[e3,0,25]],pitchOffset=0,speed=1,legato=1.0)
		elif msgType=='HEARTBEAT':
			lastRemoteHeartbeat=now
		elif msgType=='BATTERY_STATUS':
			vbat=msg.voltages[0]/1000.0
			print "battery",vbat
		
	time.sleep(0.02)
	js=joystick
	if js!=None:
		#print js.getAxis('x1'),js.getAxis('y1')
		throttle = -joystick.getAxis( 'y1' )
		ym=0
		if throttle<0:
			throttle=0
			ym=500
		updateAxis("throttle",1000+throttle*1000);
		updateAxis("yaw", 1500+ym*js.getAxis('x1') );    
		updateAxis("pitch", 1500+100*js.getAxis('y2') );
		updateAxis("roll", 1500+100*js.getAxis('x2') );
	
	now=time.time()
	if dirtyRC and now-lastRCSend>0.05:
		#print "send"
		dirtyRC=False
		lastRCSend=now
		mav.mav.rc_channels_override_send(0,0,previousAxisValues['throttle'],previousAxisValues['yaw'],1003,1004,1005,1006,1007,1008)

	