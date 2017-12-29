from threading import Thread
import sys, time, math, traceback, serial, os
from soundEnvelope import SoundEnvelope, Envelope
sys.path.append('/home/pi/dev/mavlink/')
from pymavlink import mavutil
from pyMultiwii import MultiWii
import pymavlink.dialects.v10.ardupilotmega as mavlink
from Logger import Logger
from sonar import Sonar
import configparser
from altitude import Altitude
from windowfilter import WindowFilter

flatTone=Envelope(16,0,0,0,1,1,0,127,0,0,-127,127,8)
bellTone=Envelope(1,-48,48,0,2,2,255,127,-3,0,-127,127,0)
errorTone=Envelope(8,48,0,0,5,16,16,127,0,0,-127,127,8)
warningTone=Envelope(1,2,-2,0,12,12,0,127,0,0,-127,127,8)
sirenTone=Envelope(1,2,-2,0,12,12,0,127,0,0,-127,127,8)

def constrain(v,min,max):
	if v<min:
		r=min
	elif v>max:
		r=max
	else:
		r=v
	return r	




class AirMainLoop:

	lastHeartbeatSent=0
	threadInterrupted=False
	lastRemoteHeartbeat=time.time()
	localSNR=0
	remoteSNR=0
	remoteConnection=False
	rcDirty=False
	lastRCSend=0
	board=None
	inhibitMultiWii=False
	homeAltitude=0
	onGround=False
	onGroundCountDown=5
	theta=1.0 # ratio of thrust pointing upward
	
	throttleIn=1000
	throttle=1000
	yaw=1500
	roll=1500
	pitch=1500
	aux1=1000
	pidLookup= ["Roll P", "Roll D", "Roll I",
				"Pitch P", "Pitch D", "Pitch I",
				"Yaw P", "Yaw D", "Yaw I",
				"Alt P", "Alt D", "Alt I",
				"Pos P", "Pos D", "Pos I",
				"PosR P", "PosR D", "PosR I",
				"NavR P", "NavR D", "NavR I",
				"Level Angle", "Lev Horizon", "Tran Horizon",
				"Mag P", "---", "---",
				"Vel P", "Vel D", "Vel I",
				"AltitudeA","AltitudeP","AltitudeD","AltitudeI","Alt Throttle"
				]
	
	altHoldA=0
	altHoldP=1.0
	altHoldD=0
	altHoldI=0
	
	def __init__(self,radio,multiwii):
		os.chdir(sys.path[0])
		self.se=SoundEnvelope()
		try:
			self.loadInitialisation()
			
			self.mavconnection = mavutil.mavlink_connection(radio, baud="57600", autoreconnect = True )
			self.mav=self.mavconnection.mav
			self.board = MultiWii(multiwii)
			#self.board.PRINT=0
			#print "init logger"
			self.logger = Logger()
			self.sonar = Sonar()
			self.altitude = Altitude(self)
			#self.logger.logTelemetry(self,True)
		except Exception, error:
			self.se.play([[12,32,flatTone],[0,64]],legato=0.8,speed=1.5,pitchOffset=0)
			raise error
			
	def loadInitialisation(self):
		self.config = configparser.ConfigParser()
		self.config.read("air.ini")
		default=self.config['DEFAULT']
		self.altHoldA=default.getfloat('altHoldA')
		self.altHoldP=default.getfloat('altHoldP')
		self.altHoldI=default.getfloat('altHoldI')
		self.altHoldD=default.getfloat('altHoldD')
		self.altHoldOffset=default.getfloat('altHoldOffset')
		self.altHoldThrottleMultiplier=default.getfloat('altHoldThrottleMult')
		
	def saveConfig(self):
		default=self.config['DEFAULT']
		default['altHoldA']=str(self.altHoldA)
		default['altHoldP']=str(self.altHoldP)
		default['altHoldI']=str(self.altHoldI)
		default['altHoldD']=str(self.altHoldD)
		default['altHoldOffset']=str(self.altHoldOffset)
		default['altHoldThrottleMult']=str(self.altHoldThrottleMultiplier)
		with open('air.ini', 'w') as configfile:
			self.config.write(configfile)
		
	def updateRC(self,now):
		if (self.rcDirty or now-self.lastRCSend>0.05) and self.remoteConnection:
			self.rcDirty=False
			self.lastRCSend=now
			data = [self.roll,self.pitch,self.throttle,self.yaw,1000,1000,1000,1000]
			self.board.sendCMD(16,MultiWii.SET_RAW_RC,data)
		
	def doHeartbeat(self,now):
		self.mav.heartbeat_send(0,0,0,0,0)
		
	def arm(self,now):
		self.inhibitMultiWii=True
		self.se.play([[48,128,sirenTone]])
		time.sleep(0.05)
		self.homeAltitude=self.filterAlt.get()
		self.board.arm()
		self.inhibitMultiWii=False
	def disarm(self,now):
		self.inhibitMultiWii=True
		time.sleep(0.05)
		self.board.disarm()
		self.inhibitMultiWii=False
		
	
	def setPIDTuning(self,tuningId,tuningValue):
		if tuningId>=0 and tuningId<30:
			self.tuningId=tuningId
			self.tuningValue=tuningValue
		elif tuningId==30:
			self.altHoldA=float(tuningValue)
		elif tuningId==31:
			self.altHoldP=float(tuningValue)
		elif tuningId==32:
			self.altHoldD=float(tuningValue)
		elif tuningId==33:
			self.altHoldI=float(tuningValue)
		elif tuningId==34:
			self.altHoldThrottleMultiplier=float(tuningValue)
			
		if tuningId>=30:
			self.saveConfig()
			
	def getPIDTuning(self,tuningId):
		if tuningId>=0 and tuningId<30:
			return self.board.PIDcoef[tuningId]
		elif tuningId==30:
			return self.altHoldA
		elif tuningId==31:
			return self.altHoldP
		elif tuningId==32:
			return self.altHoldD
		elif tuningId==33:
			return self.altHoldI
		elif tuningId==34:
			return self.altHoldThrottleMultiplier
	
	def multiWiiLoop(self):
		now=time.time()
		self.lastSensorPoll=now
		self.filterAlt=WindowFilter(4,0.2)
		self.sensorIndex=0
		self.multiWiiFailure=False
		self.tuningId=None
		
		loopPeriod=0.02
		loopExpectedTime=time.time()
		
		while not self.threadInterrupted:
		
			now=time.time()
			
			try:
			
				if self.aux1==1100:
					self.altitude.doAltitudeHold(now)
					self.rcDirty=True
				else:
					self.altitude.setpoint=0
					if self.throttleIn<1000:
						self.throttleIn=1000
					self.throttle=self.throttleIn
			
				if not self.inhibitMultiWii:
					
					self.updateRC(now)
					self.sensorPoll(now)
					self.multiWiiFailure=False
				
				#keep loop synchronized to clock period
				tdiff=now-loopExpectedTime
				loopExpectedTime+=loopPeriod
				if tdiff<0: # too quick
					time.sleep(-tdiff)
				elif tdiff>0.1: #much too slow - skip frames
					loopExpectedTime=now+loopPeriod
					print "multiWiiLoop skipping ",tdiff
				
			except (serial.SerialException, serial.portNotOpenError, ValueError) as se:
				#deal with disconnect of flight controller from Pi
				print se
				self.multiWiiFailure=True
				time.sleep(0.5)
				try:
					try:
						self.board.ser.close()
					except:
						pass
					multiwii=self.board.ser.port
					self.board = MultiWii(multiwii)
				except:
					pass
				
			except Exception, err:
				print err
				traceback.print_exc(file=sys.stdout)
				time.sleep(0.1)
	
	def updateSonarDistance(self,now):
		distance=self.sonar.getDistance()
		
		if distance>0 and abs(self.board.attitude['angx'])<15 and abs(self.board.attitude['angy'])<15:
			#print self.board.attitude['angx']
			if now-self.sonarLastReading>1:
				self.sonarDistance=distance
			else:
				self.sonarDistance=distance #self.sonarDistance*0.25+distance*0.75
			self.sonarLastReading=now	
				
		if now-self.sonarLastReading>0.5:
			self.sonarDistance=-1
			
		
		#print distance,"cm"
		
		

	
	#lastSensorPoll=0

	#called ~50/sec - each step called 5/sec
	def sensorPoll(self,now):
		#td=now-self.lastSensorPoll
		#self.lastSensorPoll=now
		#print "sensorPoll",td
	
		#self.board.flushIn()
		while self.board.ser.in_waiting>0:
			code=self.board.doRead()
			
			if code==MultiWii.RAW_IMU:
				self.altitude.updateAccAltitude()
			elif code==MultiWii.ALTITUDE:
				self.altitude.updateBaroAltitude()
			elif code==MultiWii.ATTITUDE:
				f=math.pi/180;
				angx=self.board.attitude['angx']*f
				angy=self.board.attitude['angy']*f
				self.theta=math.cos(angx)*math.cos(angy)
				if self.theta<0: # 90 degrees off level
					self.disarm(now)
			elif code==MultiWii.RAW_GPS:
				print self.board.gps
				
		self.sensorIndex=(self.sensorIndex+1)%10
		index=self.sensorIndex
	
		if self.aux1==1100: # if in althold
			a=index%2
			if a==0:
				self.board.getData(MultiWii.ALTITUDE)
			else:
				self.board.getData(MultiWii.RAW_IMU)
		else:
			if index==4:
				self.board.getData(MultiWii.ALTITUDE)
			a=index%2
			if a==1:
				self.board.getData(MultiWii.RAW_IMU)
				
	
		
		if index==0:
			self.sonar.doTrigger()
			self.board.getData(MultiWii.ANALOG)
		elif index==1:
			self.board.getData(MultiWii.STATUS)
			flags=self.board.status['flag']
			self.armed=flags&1!=0
			self.failsafe=flags&4096!=0
		elif index==2:
			self.board.getData(MultiWii.RAW_GPS)
		elif index==3:
			#self.board.getData(MultiWii.RAW_IMU)
			if self.armed:
				self.logger.logTelemetry(self)
		elif index==4:
			#self.board.getData(MultiWii.ALTITUDE)
			
			#test if on ground
			onGround=False
			if self.throttle==1000:
				gx=self.board.rawIMU['gx']
				gy=self.board.rawIMU['gy']
				gz=self.board.rawIMU['gz']
				gyroMag=math.sqrt(gx*gx+gy*gy+gz*gz)
				if gyroMag<100:
					onGround=True
			if onGround:
				if self.onGroundCountDown>0:
					self.onGroundCountDown-=1
				else:
					self.onGround=True
					self.homeAltitude=self.filterAlt.get()
			else:
				self.onGround=False
				self.onGroundCountDown=5
			self.altitude.updateSonarDistance(now)
		elif index==5:
			self.board.getData(MultiWii.MOTOR)
			self.sonar.doTrigger()
		elif index==6:
			self.board.getData(MultiWii.ATTITUDE)
		elif index==7:
			self.board.getData(MultiWii.PID)
			if self.tuningId!=None:
				self.board.PIDcoef[self.tuningId]=self.tuningValue
				self.tuningId=None
				self.board.setPID2()
				print "changing PID",self.tuningId,self.tuningValue
		elif index==8:
			pass
			#self.board.getData(MultiWii.RAW_IMU)
			#self.updateAccAltitude(now)
		elif index==9:
			#self.board.getData(MultiWii.ALTITUDE)
			self.altitude.updateSonarDistance(now)
			
		
	def checkForErrors(self,now):
		
		errors=[]
		warnings=[]
		
		if now-self.lastRemoteHeartbeat>4:
			errors.append("No heartbeat received");
			self.remoteConnection=False
		if self.remoteSNR<10:
			errors.append("low remoteSNR "+str(self.remoteSNR))
		if self.localSNR<10:
			errors.append("low localSNR "+str(self.localSNR))
		if self.board.vbat!=None and self.board.vbat<10:
			errors.append("battery critical "+str(self.board.vbat))
		elif self.board.vbat!=None and self.board.vbat<10.8:
			warnings.append("battery low "+str(self.board.vbat))
		if self.board.gps['fix']==0:
			warnings.append("No GPS Fix {:1.0f}sat".format(self.board.gps['satellites']))
			#self.board.arm()
		if self.multiWiiFailure:
			errors.append("Lost MultiWii")
			
		if len(errors)>0:
			print errors
			#self.se.play([[0,25,errorTone]],pitchOffset=0,speed=1,legato=1.0)
		elif len(warnings)>0:
			#print warnings
			#self.se.play([[0,12,warningTone]],pitchOffset=0,speed=1,legato=1.0)
			pass
			
	def telemetrySend(self,now):
	
		#send data to ground
		self.existingRemoteError=None
		remoteError=None
		if self.multiWiiFailure:
			remoteError="Lost MultiWii"
		
		if remoteError and self.existingRemoteError!=remoteError:
			self.mav.statustext_send(mavlink.MAV_SEVERITY_EMERGENCY,remoteError)
			self.existingRemoteError=remoteError
		else:
			self.mav.statustext_send(mavlink.MAV_SEVERITY_INFO,"")
			self.existingRemoteError=None
		
		batteryPercent=(self.board.vbat-9.6)/(12.6-9.6)*100
		if batteryPercent>100:
			batteryPercent=100
		elif batteryPercent<0:
			batteryPercent=0
		self.mav.battery_status_send(0,0,0,0,[self.board.vbat*1000,0,0,0,0,0,0,0,0,0],0,0,0,batteryPercent);
		'''
		#MAVLINK messages ATTITUDE GLOBAL_POSITION_INT
		#attitude_send(self, time_boot_ms, roll, pitch, yaw, rollspeed, pitchspeed, yawspeed, force_mavlink1=False):
		degToRad=math.pi/180
		att=self.board.attitude
		pitch=att['angy']*degToRad
		roll=att['angx']*degToRad
		yaw=att['heading']*degToRad
		#print pitch,roll,yaw
		self.mav.attitude_send(0,roll,pitch,yaw,0,0,0)
		
		#def gps_raw_int_send(self, time_usec, fix_type, lat, lon, alt, eph, epv, vel, cog, satellites_visible, force_mavlink1=False):
		self.mav.gps_raw_int_send(0,
			self.board.gps['fix'],
			self.board.gps['lat']*10000000,
			self.board.gps['lon']*10000000,
			self.board.gps['altitude']*1000,
			0,0,
			self.board.gps['speed'],
			self.board.gps['course']*100,
			self.board.gps['satellites']);
		
		altitude #141
		'''
		#def high_latency_send(self, base_mode, custom_mode, landed_state, roll, pitch, heading, throttle, heading_sp, latitude, longitude, altitude_amsl, altitude_sp, airspeed, airspeed_sp, groundspeed, climb_rate, gps_nsat, gps_fix_type, battery_remaining, temperature, temperature_air, failsafe, wp_num, wp_distance, force_mavlink1=False):
		heading=self.board.attitude['heading']
		if heading>180.0:
			heading=360.0-heading
		gpsAltitude=self.board.gps['altitude']
		if gpsAltitude>32767:
			gpsAltitude=32767
			
		baroAltitude=self.altitude.getFusionAltitude() #self.filterAlt.get()-self.homeAltitude
		#print heading, altitude
		self.mav.high_latency_send(0,
			self.board.status['flag'],
			0,
			self.board.attitude['angx']*100,
			self.board.attitude['angy']*100,
			heading*100,
			0,0,
			self.board.gps['lat']*10000000,
			self.board.gps['lon']*10000000,
			gpsAltitude,
			baroAltitude,
			0,0,
			self.board.gps['speed'],
			0,
			self.board.gps['satellites'],
			self.board.gps['fix'],
			batteryPercent,
			0,0,0,0,0)
		#possible problem fields:  heading, altitude x2, 
		
		
	def processMessage(self,msg,now):
		msgType=msg.get_type()
		#print msgType
		
		if msg.get_type()=='RC_CHANNELS_OVERRIDE':
			self.lastRemoteHeartbeat=now
			self.throttleIn=msg.chan1_raw
			self.yaw=msg.chan2_raw
			self.pitch=msg.chan3_raw
			self.roll=msg.chan4_raw
			self.aux1=msg.chan5_raw
			self.rcDirty=True
		elif msgType=='RADIO_STATUS' or msgType=='RADIO':
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
		elif msgType=='SET_MODE':
			#print "mode",msg
			mode=msg.base_mode
			if mode==mavlink.MAV_MODE_STABILIZE_ARMED:
				self.arm(now)
			elif mode==mavlink.MAV_MODE_PREFLIGHT:
				self.disarm(now)
		elif msgType=='PARAM_REQUEST_READ':
			paramId=msg.param_index
			paramValue=int(self.getPIDTuning(paramId))
			#print "PARAM_REQUEST_READ",paramId,paramValue
			self.mav.param_value_send(self.pidLookup[paramId],paramValue,0,0,paramId)
		elif msgType=='COMMAND_LONG':
			command=msg.command
			if command==mavlink.MAV_CMD_DO_SET_PARAMETER:
				#print "MAV_CMD_DO_SET_PARAMETER",msg.param1,msg.param2
				self.setPIDTuning(int(msg.param1),int(msg.param2))
				self.se.play([[0,16,flatTone],[None,16],[0,16]],legato=0.9,speed=1.5,pitchOffset=24)
			if command==mavlink.MAV_CMD_DO_CHANGE_ALTITUDE:
				altitude=msg.param1
				self.altitude.altHoldSetpoint+=altitude
				print "set altHoldSetpoint",altitude
		else:
			print msgType
		
	def loop(self):
		now=time.time();
		self.se.play([[0,16,flatTone],[4,16],[7,16],[12,48]],legato=0.9,speed=1.5,pitchOffset=24)
		
		self.lastTelemetrySend=now
		
		while not self.threadInterrupted:
			try:
				now=time.time();
				
				if now-self.lastHeartbeatSent>1:
					self.lastHeartbeatSent=now
					self.doHeartbeat(now)
					self.checkForErrors(now)
				
				msg = self.mavconnection.recv_msg()
				if msg!=None:
					self.processMessage(msg,now)
			
				if now-self.lastTelemetrySend>1:
					self.lastTelemetrySend=now
					try:
						self.telemetrySend(now)
					except Exception, err:
						print err
				
				time.sleep(0.02)
			except Exception, err:
				print "here3",Exception, err
				traceback.print_exc(file=sys.stdout)
				time.sleep(0.1)
				
		#on termination
		self.board.disarm()
		
	def start(self):
		self.threadInterrupted=False
		
		t = Thread(target=self.loop)
		t.setDaemon(True)
		t.start()
		self.threadMain=t
		
		t = Thread(target=self.multiWiiLoop)
		t.setDaemon(True)
		t.start()
		self.threadMultiWii=t
		