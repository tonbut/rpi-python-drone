from joystick import Joystick
from threading import Thread
import sys, time
import paho.mqtt.client as mqtt
import paho.mqtt.publish as publish

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
	if button=='circle':
		client.publish("test","kill: 0");

previousAxisValues = { }
def updateAxis(name,value):
	global previousAxisValues
	
	if not(previousAxisValues.has_key(name)) or previousAxisValues[name]!=value:
		previousAxisValues[name]=value
		s="{}: {:>6.0f}".format(name,value)
		client.publish("test",s);
		print s


def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))


client = mqtt.Client("", True, None, mqtt.MQTTv31)
client.on_connect = on_connect
client.connect("192.168.1.1", 1883, 60)
client.loop_start()

t = Thread(target=joystickLoop)
t.setDaemon(True)
t.start()


while True:


	time.sleep(0.05)
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
	
	