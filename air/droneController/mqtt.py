# Simple demo of of the PCA9685 PWM servo/LED controller library.
# This will move channel 0 from min to max position repeatedly.
# Author: Tony DiCola
# License: Public Domain
from __future__ import division
import time
import paho.mqtt.client as mqtt
import Adafruit_PCA9685
import serial
from lightTelemetry import LightTelemetry

pwm = Adafruit_PCA9685.PCA9685()

PIN_THROTTLE=10
PIN_YAW=11
PIN_PITCH=8
PIN_ROLL=9
PIN_AUX1=12

# Helper function to make setting a servo pulse width simpler.
def set_servo_pulse(channel, pulse):
	if pulse<1000:
		pulse=1000
	elif pulse>2000:
		pulse=2000
	t=0.2114*(pulse)
	pwm.set_pwm(channel, 0, int(t))
	#print t
	
def on_connect(client, userdata, flags, rc):
	print("Connected to MQTT broker "+str(rc))
	client.subscribe("test")

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
	#print(msg.topic+" "+str(msg.payload))	
	pl=str(msg.payload)
	i=pl.index(":")
	channel=pl[:i]
	value=int(pl[i+1:])
	print (channel,value)
	c=-1
	if channel=="throttle":
		c=PIN_THROTTLE
	elif channel=="yaw":
		c=PIN_YAW
	elif channel=="pitch":
		c=PIN_PITCH
	elif channel=="roll":
		c=PIN_ROLL
	if c>=0:
		set_servo_pulse(c,value)
	if channel=="kill":
		pwm.set_pwm(2, 0, 0)

pwm.set_pwm_freq(50)
set_servo_pulse(PIN_THROTTLE,1000)

set_servo_pulse(PIN_ROLL,1500)
set_servo_pulse(PIN_PITCH,1500)
set_servo_pulse(PIN_YAW,1500)
set_servo_pulse(PIN_AUX1,1500)

client = mqtt.Client("", True, None, mqtt.MQTTv31)
client.on_connect = on_connect
client.on_message = on_message

client.connect("192.168.1.1", 1883, 60)
client.loop_start()

ser = serial.Serial(
              
               port='/dev/ttyUSB0',
               baudrate = 19200,
               parity=serial.PARITY_NONE,
               stopbits=serial.STOPBITS_ONE,
               bytesize=serial.EIGHTBITS,
               timeout=1
           )

lt=LightTelemetry()

try:
	while True:

		b=ser.read();
		#print "serial",b
		if b:
			lt.processByte(b)
finally:
	pwm.set_pwm(PIN_THROTTLE, 0, 10)
	set_servo_pulse(PIN_YAW,1500)
	set_servo_pulse(PIN_ROLL,1500)
	set_servo_pulse(PIN_PITCH,1500)
	#time.sleep(0.01)    

