from joystick import Joystick
from threading import Thread
import threading
import sys, time, math
from soundEnvelope import SoundEnvelope, Envelope
sys.path.append('/home/pi/dev/mavlink/')
from pymavlink import mavutil
import pymavlink.dialects.v10.ardupilotmega as mavlink
from hd44780 import hd44780

flatTone=Envelope(16,0,0,0,1,1,0,127,0,0,-127,127,8)
bellTone=Envelope(1,-48,48,0,2,2,255,127,-3,0,-127,127,0)
errorTone=Envelope(8,48,0,0,5,16,16,127,0,0,-127,127,8)

class GroundMainLoop:

	previousAxisValues = { 'throttle': 1000, 'yaw': 1500, 'pitch': 1500, 'roll': 1500 }
	lastRCSend=0
	lastHeartbeatSent=0
	dirtyRC=False
	threadInterrupted=False
	
	lastRemoteHeartbeat=time.time()
	localSNR=0
	remoteSNR=0
	batteryVoltage=0
	remoteConnection=False
	lastPoll=0
	errorToggle=0
	lastError=False
	#yaw=0.0
	#pitch=0.0
	#roll=0.0
	highLatency=None
	startPressed=None
	armed=False
	failsafe=False
	selectMode=-1
	pidTuneIndex=0
	trimRoll=0
	trimPitch=0
	trimPressed=0
	remoteError=None
	trimPressed=None
	modeDisplay=None
	lastErrorHash=0
	errorCountDown=0
	hover=False
	tiltControl=False
	
	def __init__(self,joystick,radio):
		self.joystick=Joystick(joystick,self.onButton)
		self.mavconnection = mavutil.mavlink_connection(radio, baud="57600", autoreconnect = True )
		self.mav=self.mavconnection.mav
		self.se=SoundEnvelope()
		self.lcd=hd44780()
		self.lcdLock=threading.Lock()
		
	def setLCDStatus(self,msg):
		self.lcdLock.acquire()
		self.lcd.line2(msg)
		self.lcdLock.release()
		
	def setLCDMain(self,msg):
		self.lcdLock.acquire()
		self.lcd.line1(msg)
		self.lcdLock.release()
	
	def setModeDisplay(self,msg):
		self.modeDisplay=msg
		self.setLCDStatus(msg)
	
	def onStartButton(self, press):
		if press:
			self.startPressed=time.time()
		else:
			self.startPressed=None
			
	def onSelectButton(self,press):
		if not press:
			self.selectMode=(self.selectMode+1)%2
			print "selectMode",self.selectMode
			if self.selectMode==0:
				self.setModeDisplay(self.getTrimMessage())
			elif self.selectMode==1:
				#self.mav.param_request_read_send(0,mavlink.MAV_MODE_PREFLIGHT,0)
				#param_request_read_send(self, target_system, target_component, param_id, param_index, force_mavlink1=False):
				self.mav.param_request_read_send(0,0,"",self.pidTuneIndex)
			self.se.play([[36,4,flatTone]])
			
	def onPadButton(self,button,press):
		if self.selectMode==0:
			self.onTrim(button,press)
		elif self.selectMode==1:
			self.onPIDTune(button,press)
	
	def getTrimMessage(self):
		return "Trim {:3.0f} {:3.0f}".format(self.trimPitch,self.trimRoll)
				
	def onTrim(self,button,press):
		if self.selectMode==0:
			now=time.time()
			if press:
				self.trimPressed=now
			else:
				if self.trimPressed!=None:
					if now-self.trimPressed>0.5:
						self.trimPitch=0
						self.trimRoll=0
						self.se.play([[12,16,flatTone]])
					else:
						correction=10
						if button=="dpad_up":
							self.trimPitch+=correction
						elif button=="dpad_down":
							self.trimPitch-=correction
						elif button=="dpad_right":
							self.trimRoll+=correction
						elif button=="dpad_left":
							self.trimRoll-=correction
						self.se.play([[36,4,flatTone]])
					
					#print "trim correction",self.trimPitch,self.trimRoll
					self.setModeDisplay(self.getTrimMessage())
					self.trimPressed=None
			
	def toggleTiltMode(self):
		if self.tiltControl:
			self.tiltControl=False
		else:
			self.tiltControl=True
			js=self.joystick
			js.poll()
			pitchOffset=js.getAxis( 'ry' )
			rollOffset=js.getAxis( 'rx' )
			print "tilt",pitchOffset,rollOffset
					
			
	def onButton(self,button, press):
		#print button,press
		
		if button=="start":
			self.onStartButton(press)
		elif button.startswith("dpad_"):
			self.onPadButton(button,press)
		elif button=="cross":
			self.hover=press
			self.dirtyRC=True
		elif button=="select":
			self.onSelectButton(press)
		elif button=="triangle" and press:
			#set altitude diff in cm not metres!
			self.mav.command_long_send(0,0,mavlink.MAV_CMD_DO_CHANGE_ALTITUDE,0,
					25,0,0,0,0,0,0,0)
		elif button=="square" and press:
			#set altitude diff in cm not metres!
			self.mav.command_long_send(0,0,mavlink.MAV_CMD_DO_CHANGE_ALTITUDE,0,
					-25,0,0,0,0,0,0,0)
		elif button=="R2" and press:
			self.toggleTiltMode()
		else:
			print button

		#else:
		#	print buttonsudo 
		
	def onPIDUpdate(self):
		self.mav.command_long_send(0,0,mavlink.MAV_CMD_DO_SET_PARAMETER,0,
					self.pidTuneIndex,self.pidTuneValue,0,0,0,0,0,0)
		self.setModeDisplay(self.getPidMessage())
		
	def onPIDTune(self,button,press):
		if press:
			if button=="dpad_up":
				self.pidTuneIndex=(self.pidTuneIndex+1)%35
				print self.pidTuneIndex
				self.mav.param_request_read_send(0,0,"",self.pidTuneIndex)
			elif button=="dpad_down":
				self.pidTuneIndex=(self.pidTuneIndex-1)%35
				self.mav.param_request_read_send(0,0,"",self.pidTuneIndex)
			elif button=="dpad_right":
				self.pidTuneValue+=1
				self.onPIDUpdate()
			elif button=="dpad_left":
				self.pidTuneValue-=1
				self.onPIDUpdate()
			self.se.play([[36,4,flatTone]])
	
	def getPidMessage(self):
		return "{:12s}{:4.0f}".format(self.pidTuneMode,self.pidTuneValue);
	
	def onPIDTuneValueReceived(self,paramId,paramValue):
		self.pidTuneMode=paramId
		self.pidTuneValue=paramValue
		self.setModeDisplay(self.getPidMessage())
		
	def doHeartbeat(self,now):
		self.mav.heartbeat_send(0,0,0,0,0)
		
	def doArmCheck(self,now):
		sp=self.startPressed
		if sp!=None:
			d=int(now-sp)
			if self.armCheck():
				if d<3:
					self.se.play([[36,16,flatTone]])
				elif d==3:
					self.se.play([[48,32,flatTone]])
					self.startPressed=None
					self.setArmed()
					print "armed"
			elif self.disarmCheck():
				if d<1:
					self.se.play([[36,16,flatTone]])
				elif d==1:
					self.se.play([[24,32,flatTone]])
					self.startPressed=None
					self.setDisarmed()
					print "disarmed"
			else:
				self.se.play([[12,32,flatTone],[0,64]],legato=0.8,speed=1.5,pitchOffset=0)
					
	def armCheck(self):
		return not self.armed and self.remoteConnection # and self.highLatency.gps_fix_type>0
	def disarmCheck(self):
		return self.armed
				
	def setArmed(self):
		self.mav.set_mode_send(0,mavlink.MAV_MODE_STABILIZE_ARMED,0)
	def setDisarmed(self):
		self.mav.set_mode_send(0,mavlink.MAV_MODE_PREFLIGHT,0)
		
	def doPoll(self,now):
	
		#print self.remoteSNR,self.localSNR
		status="?"
		if self.failsafe:
			status="F"
		elif self.armed:
			status="A"
		else:
			status="0"
		
		#line1="{:s} {:>4.1f}v {:3.0f} {:2.0f} {:2.0f}".format(status,self.batteryVoltage, self.highLatency.heading/100,self.remoteSNR,self.localSNR);
		try:
			line1="{:s} {:>4.1f}v{:3.0f} {:4.0f}".format(
				status,
				self.batteryVoltage,
				self.previousAxisValues['throttle'],
				self.highLatency.altitude_sp
				)
		
			self.setLCDMain(line1)
		except Exception, error:
			print error
	
		errors=[]
		hashcode=0
		
		if not self.joystick.isReady:
			errors.append("Joystick lost");
			hashcode+=1
		if now-self.lastRemoteHeartbeat>2:
			errors.append("No heartbeat");
			self.remoteConnection=False
			hashcode+=2
		if self.remoteSNR<10:
			errors.append("Low R-SNR {:2.0f}".format(self.remoteSNR))
			hashcode+=3
		if self.localSNR<10:
			errors.append("Low L-SNR {:2.0f}".format(self.localSNR))
			hashcode+=4
		if self.remoteConnection:
			if self.batteryVoltage!=None and self.batteryVoltage<10:
				errors.append("Bat danger "+str(self.batteryVoltage)+"v")
				hashcode+=5
			elif self.remoteConnection and self.batteryVoltage!=None and self.batteryVoltage<10.8:
				errors.append("Bat low "+str(self.batteryVoltage)+"v")
				hashcode+=6
			#print self.highLatency
			if self.highLatency!=None and self.highLatency.gps_fix_type==0:
				errors.append("No GPS Fix {:1.0f}sat".format(self.highLatency.gps_nsat))
				hashcode+=7
		if self.remoteError!=None:
			errors.append(self.remoteError)
			
		if len(errors)>0:
			if hashcode!=self.lastErrorHash:
				self.lastErrorHash=hashcode
				self.errorCountDown=1 #10
			if self.errorCountDown>0:
				self.errorCountDown-=1
				self.se.play([[0,25,errorTone]],pitchOffset=0,speed=1,legato=1.0)
			print errors
			
			if self.modeDisplay==None:
				self.setLCDStatus(errors[self.errorToggle%len(errors)])
				self.errorToggle=self.errorToggle+1
			else:
				self.modeDisplay=None
			self.lastError=True
		elif self.lastError:
			if self.modeDisplay==None:
				self.setLCDStatus("                ");
			else:
				self.modeDisplay=None
			self.lastError=False
		
	def processMessage(self,msg,now):
		msgType=msg.get_type()
		#print msgType
		
		if msgType=='RADIO_STATUS':
			localSignal=(msg.rssi/1.9-127)
			localNoise=(msg.noise/1.9-127)
			self.localSNR=localSignal-localNoise
			remoteSignal=(msg.remrssi/1.9-127)
			remoteNoise=(msg.remnoise/1.9-127)
			self.remoteSNR=remoteSignal-remoteNoise
			
		elif msgType=='HEARTBEAT':
			self.lastRemoteHeartbeat=now
			if not(self.remoteConnection):
				self.remoteConnection=True
				self.se.play([[60,128,bellTone]])
		
		elif msgType=='BATTERY_STATUS':
			self.batteryVoltage=msg.voltages[0]/1000.0
			#print "battery",self.batteryVoltage
		elif msgType=='HIGH_LATENCY':
			self.lastRemoteHeartbeat=now
			self.highLatency=msg
			
			#print "mode=",self.highLatency.custom_mode
			self.armed=self.highLatency.custom_mode&1>0
			self.failsafe=self.highLatency.custom_mode&4096>0
		elif msgType=='STATUSTEXT':
			if len(msg.text)>0:
				self.remoteError=msg.text
			else:
				self.remoteError=None
		elif msgType=='PARAM_VALUE':
			#print "param_value", msg.param_id, msg.param_value
			self.onPIDTuneValueReceived(msg.param_id,msg.param_value)
		#else:
		#	print msgType, msg
		
	def updateAxis(self,name,value):
		if not(self.previousAxisValues.has_key(name)) or self.previousAxisValues[name]!=value:
			self.previousAxisValues[name]=value
			self.dirtyRC=True
		
	def joystickLoop(self):
		while not self.threadInterrupted:
			js=self.joystick
			js.poll()
			throttle = -js.getAxis( 'y1' )
			ym=500
			ypr=150
			if not self.armed: # to allow joystick commands to be sent
				ypr=500
			
			self.updateAxis("throttle",1000+throttle*1000);
			#print "throttle",1000+throttle*1000
			self.updateAxis("yaw", 1500+ym*js.getAxis('x1') );    
			self.updateAxis("pitch", 1500-ypr*js.getAxis('y2') );
			self.updateAxis("roll", 1500+ypr*js.getAxis('x2') );
			
			#print "rot",js.getAxis('rx'),js.getAxis('ry')
		
	def updateJoystickAxes(self,now):
		if self.dirtyRC and now-self.lastRCSend>0.05:
			self.dirtyRC=False
			self.lastRCSend=now
			aux1=1000
			if self.hover:
				aux1=1100
			self.mav.rc_channels_override_send(0,0,
				self.previousAxisValues['throttle'],
				self.previousAxisValues['yaw'],
				self.previousAxisValues['pitch']+self.trimPitch,
				self.previousAxisValues['roll']+self.trimRoll,
				aux1,1000,1000,1000 )
				
	def processErrors(self,now):
		print "processErrors"
		
	def loop(self):
	
		now=time.time()
		self.se.play([[0,16,flatTone],[4,16],[7,16],[12,48]],legato=0.9,speed=1.5,pitchOffset=24)
		self.lastPoll=now
		
		while not self.threadInterrupted:
			try:
				t=time.time();
				#print (t-now)
				now=t
			
				if now-self.lastHeartbeatSent>1:
					self.lastHeartbeatSent=now
					self.doHeartbeat(now)
					self.doArmCheck(now)
				
				msg = self.mavconnection.recv_msg()
				if msg!=None:
					self.processMessage(msg,now)
			
				self.updateJoystickAxes(now)
				
				if now-self.lastPoll>2:
					self.lastPoll=now
					self.doPoll(now)
			
				time.sleep(0.02)
			except Exception, err:
				print Exception, err
				time.sleep(0.1)
		
	def start(self):
		self.threadInterrupted=False
		
		t = Thread(target=self.loop)
		t.setDaemon(True)
		t.start()
		self.threadMain=t
		
		t = Thread(target=self.joystickLoop)
		t.setDaemon(True)
		t.start()
		self.threadJoystick=t
		
