#import time,datetime
from datetime import datetime
import os, os.path, time, random

class Logger:

	def __init__(self):
		while True:
			r = random.randint(0,65536)
			filename="/home/pi/dev/droneController/log/log_{:4x}.csv".format(r);
			if not os.path.isfile(filename):
				break;
				
		#filenameBase=datetime.now().strftime("/home/pi/dev/droneController/log/log_%y-%m-%d_%H-%M.txt")
		#while os.path.isfile(filename):
		#	filename
		self.file=open(filename,mode="w+",buffering=1)
		self.first=True
		print filename
		self.lastFlush=time.time()
		
	def logRaw(self,msg):
		timeString=datetime.now().strftime("%H:%M:%S:%f")
		line=timeString+","+msg+"\r\n"
		#print msg
		self.file.write(line)
		
		now=time.time()
		if now-self.lastFlush>10:
			self.lastFlush=now
			self.file.flush()
			os.fsync(self.file.fileno())
		
	def logSmart(self,logdef,src):
		for d in logdef:
			ref=src
			for d2 in d:
				if d2.endswith("[]"):
					#print "array",d2
					ref=ref[d2[:-2]]
				elif d2.endswith("()"):
					#print "function",d2
					m=getattr(ref,d2[:-2])
					ref=m()
				else:
					ref=getattr(ref,d2)
			print "value=",str(ref),"for",d2
			
	def logTelemetry(self,src):
		
		defn=[
			[src.localSNR,"{:.2f}","localSNR"],
			[src.remoteSNR,"{:.2f}","remoteSNR"],
			[src.remoteConnection,"{0}","groundConnection"],
			[not src.multiWiiFailure,"{0}","nazeConnection"],
			[src.board.vbat,"{:.1f}","batteryVoltage"],
			[src.board.current,"{:.2f}","batteryCurrent"],
			[src.failsafe,"{0}","failsafe"],
			
			[src.armed,"{0}","armed"],
			#[src.throttle,"{:.0f}","throttleRC"],
			#[src.yaw,"{:.0f}","yawRC"],
			#[src.pitch,"{:.0f}","pitchRC"],
			#[src.roll,"{:.0f}","rollRC"],
			#[src.aux1,"{:.0f}","aux1RC"],
			[src.aux1==1100,"{0}","altHold"],
			
			[src.board.attitude['angx'],"{:.1f}","pitch"],
			[src.board.attitude['angy'],"{:.1f}","roll"],
			#[src.board.attitude['heading'],"{:.1f}","heading"],
			
			#[src.altitude.getBaroAltitude(),"{:.0f}","baroAltitude"],
			#[src.altitude.getSonarAltitude(),"{:.0f}","sonarAltitude"],
			[(src.throttle-1000)/10,"{:.1f}","throttlePercent"],
			[src.altitude.altHoldSetpoint,"{:.0f}","setpoint"],
			[src.altitude.getFusionAltitude(),"{:.0f}","fusionAltitude"],
			#[src.altitude.getAltitudeVelocity(),"{:.1f}","altitudeVelocityAcc"],
			#[src.altitude.baroVelocityFilter.get(),"{:.1f}","altitudeVelocityBaro"],
			#[src.altitude.altHoldSetpoint,"{:.0f}","altHoldSetpoint"],
			
			[src.altitude.hoverpoint,"{:.0f}","hoverpoint"],
			[src.altitude.hoverpointLowerBound,"{:.0f}","hpLowerBound"],
			[src.altitude.hoverpointUpperBound,"{:.0f}","hpUpperBound"],
			[src.altitude.hoverpointLast,"{:.0f}","hoverpointLast"],
			[src.altitude.hoverpointCountdown,"{:.0f}","hpCount"],
			
			
			#[src.board.gps['lat'],"{:.7f}","latitude"],
			#[src.board.gps['lon'],"{:.7f}","longitude"],
			#[src.board.gps['altitude'],"{:.0f}","gpsAltitude"],
			#[src.board.gps['speed'],"{:.2f}","speed"],
			#[src.board.gps['satellites'],"{:.0f}","satellites"],
			#[src.board.gps['fix'],"{:.0f}","fix"],
			
			#[src.board.motor['m1'],"{:.0f}","motor1-SE-CW"],
			#[src.board.motor['m2'],"{:.0f}","motor2-NE-CCW"],
			#[src.board.motor['m3'],"{:.0f}","motor3-SW-CCW"],
			#[src.board.motor['m4'],"{:.0f}","motor4-NW-CW"],
			
			#[src.board.rawIMU['ax'],"{:.0f}","ax"],
			#[src.board.rawIMU['ay'],"{:.0f}","ay"],
			[src.board.rawIMU['az'],"{:.0f}","az"],
			#[src.board.rawIMU['gx'],"{:.0f}","gx"],
			#[src.board.rawIMU['gy'],"{:.0f}","gy"],
			#[src.board.rawIMU['gz'],"{:.0f}","gz"],
			#[src.board.rawIMU['mx'],"{:.0f}","mx"],
			#[src.board.rawIMU['my'],"{:.0f}","my"],
			#[src.board.rawIMU['mz'],"{:.0f}","mz"],
			
			#[src.altitude.pidAltP,"{:.0f}","altHoldP"],
			#[src.altitude.pidAltI,"{:.0f}","altHoldI"],
			#[src.altitude.pidAltD,"{:.0f}","altHoldD"],
			
		]
		
		if self.first:
			self.first=False
			parts=[]
			for def1 in defn:
				parts.append(def1[2])
			line = ",".join(parts )
			self.logRaw(line)
		
		parts=[]
		for def1 in defn:
			v=def1[0]
			parts.append(def1[1].format(v))
		line = ", ".join(parts )
		self.logRaw(line)