import math, random, os, time
from windowfilter import WindowFilter
from hoverCalc import HoverCalc

def constrain(v,min,max):
	if v<min:
		r=min
	elif v>max:
		r=max
	else:
		r=v
	return r	

class Altitude:

	lastBaroReading=None
	lastAccReading=None
	lastSonarReading=None
	#accZVelocity=0.0
	#accHeight=0.0
	#velocityDiffIntegral=0.0
	sonarAlt=0.0
	accZ=0.0
	
	
	baroThrottleOffset=0.0 # -0.1 #how thottle causes pressure drop and perceived altitude to be higher cm/servoVal
	
	hoverpointInitial=1550
	hoverpointAnnealDistrupt=25
	hoverpointAnnealPeriod=50
	hoverpointAnnealK1=0.04
	hoverpointAnnealK2=0.02
	hoverpointAnnealK3=0.025

	def __init__(self,airmainloop):
		self.airmainloop=airmainloop
		self.board=airmainloop.board
		self.baroAltFilter=WindowFilter(5,0.2)
		self.baroVelocityFilter=WindowFilter(10,0.1)
		#self.accVelocityLongTermAverage=WindowFilter(100,0)
		
		self.hoverpointUpperBound=self.hoverpointInitial+self.hoverpointAnnealDistrupt*4;
		self.hoverpointLowerBound=self.hoverpointInitial-self.hoverpointAnnealDistrupt*4;
		self.hoverpoint=self.hoverpointInitial
		self.hoverpointLast=self.hoverpointInitial
		self.hoverpointCountdown=self.hoverpointAnnealPeriod/2;
		self.hoverpointInitialRamp=4.0

		self.hoverpointInitial=self.airmainloop.altHoldOffset
	
		self.hoverpoint2=self.hoverpointInitial
		
		'''
		filename="/home/pi/dev/droneController/log/hover.csv"
		self.file=open(filename,mode="w+",buffering=1)
		self.lastFlush=time.time()
		'''
		
		self.hoverCalc=HoverCalc(airmainloop)
		
		

	
	#lhc=0
	def calculateHoverpoint2(self,accZ,throttle):
		#print accZ,throttle
		#t=time.time()
		#print t-self.lhc
		#self.lhc=t;
		#self.calculateHoverpoint2Count+=1
		#print self.calculateHoverpoint2Count
		#pass
		'''
		line="{:.2f},{:.0f}\r\n".format(accZ,throttle)
		self.file.write(line)
		
		now=time.time()
		if now-self.lastFlush>1:
			self.lastFlush=now
			self.file.flush()
			os.fsync(self.file.fileno())
		'''
	

	def calculateHoverpoint(self,accZ,throttle):
	
		#only attempt to calculate when we have potential hover situations
		if throttle<self.hoverpointInitial-self.hoverpointAnnealDistrupt*4 or throttle>self.hoverpointInitial+self.hoverpointAnnealDistrupt*4:
			return
		if self.getFusionAltitude()<10:
			return
	
		if accZ>0:
			k1=self.hoverpointAnnealK1*self.hoverpointInitialRamp
			k2=self.hoverpointAnnealK2*self.hoverpointInitialRamp
		else:
			k1=self.hoverpointAnnealK2*self.hoverpointInitialRamp
			k2=self.hoverpointAnnealK1*self.hoverpointInitialRamp
		
		
		
		self.hoverpointUpperBound=self.hoverpointUpperBound*(1-k1)+throttle*k1;
		self.hoverpointLowerBound=self.hoverpointLowerBound*(1-k2)+throttle*k2;
		
		self.hoverpointCountdown-=1
		if self.hoverpointCountdown<=0 or self.hoverpointLowerBound>self.hoverpointUpperBound:
			#process anneal phase
			if self.hoverpointUpperBound>=self.hoverpointLast+self.hoverpointAnnealDistrupt*0.9:
				#we have maxed out and need to move up fast
				self.hoverpointCountdown=self.hoverpointAnnealPeriod/2
				self.hoverpointLast=self.hoverpointUpperBound
				self.hoverpointUpperBound=self.hoverpointLast+self.hoverpointAnnealDistrupt*4
				self.hoverpointLowerBound=self.hoverpointLast-self.hoverpointAnnealDistrupt*0.5
			elif self.hoverpointLowerBound<=self.hoverpointLast-self.hoverpointAnnealDistrupt*0.9:
				#we have mined out and need to move down fast
				self.hoverpointCountdown=self.hoverpointAnnealPeriod/2
				self.hoverpointLast=self.hoverpointLowerBound
				self.hoverpointUpperBound=self.hoverpointLast+self.hoverpointAnnealDistrupt*0.5
				self.hoverpointLowerBound=self.hoverpointLast-self.hoverpointAnnealDistrupt*4
			else:
				#normal anneal within tolerances
				self.hoverpointCountdown=self.hoverpointAnnealPeriod
				self.hoverpointLast=(self.hoverpointUpperBound+self.hoverpointLowerBound)*0.5
				self.hoverpointUpperBound+=self.hoverpointAnnealDistrupt
				self.hoverpointLowerBound-=self.hoverpointAnnealDistrupt
			print "hoverpoint",self.hoverpointLast,self.hoverpointInitialRamp
			
			#ramp down scaling to unity
			if self.hoverpointInitialRamp>1.0:
				self.hoverpointCountdown=self.hoverpointAnnealPeriod/2
				self.hoverpointInitialRamp*=0.8
			else:
				self.hoverpointInitialRamp=1.0
		else:
			self.hoverpoint=self.hoverpoint*(1-self.hoverpointAnnealK3)+self.hoverpointLast*self.hoverpointAnnealK3;


	def updateAccAltitude(self):
		az=self.board.rawIMU['az']
	
		#correct for pitch/roll
		theta=self.airmainloop.theta
		az=az/theta
		#accZ acceleration in cm/s^2
		self.accZ=(az-512)*1.916015625 # /512.0*981
		
		
		if theta>0.96 and self.getFusionAltitude()>10: # only recalculate when tilting from level by <15degrees and over 10cm 
			#self.hoverCalc.update(self.accZ,self.airmainloop.throttle*theta)
			self.calculateHoverpoint(self.accZ,self.airmainloop.throttle*theta)
			#self.calculateHoverpoint2(self.accZ,self.airmainloop.throttle*theta)
	
		'''	
		t=self.board.rawIMU['timestamp']
		if self.lastAccReading!=None:
			td=t-self.lastAccReading
			if td<0 or td>1: #happens if clock changes
				self.lastAccReading=t
				return
			self.accZVelocity+=self.accZ*td
		'''
			#pull accZVelocity drift back to baroVelocity
			#velocityDiff=self.accZVelocity-self.baroVelocityFilter.get()
			#self.accZVelocity-=velocityDiff*0.2
			
			
			#self.accZVelocity=constrain(self.accZVelocity,-200,200)
			#self.accZVelocity-=self.accVelocityLongTermAverage.get()*0.1
			#self.accVelocityLongTermAverage.add(self.accZVelocity)
			
			#use integral to fix persistent offset
			#self.velocityDiffIntegral=constrain(self.velocityDiffIntegral+velocityDiff,-200,200)
			#self.accZVelocity-=self.velocityDiffIntegral*0.002
			
			#pull accHeight drift back to baroHeight
			#heightDiff=self.accHeight-self.baroAltFilter.get()
			#self.accHeight-=heightDiff*0.04
		
		#self.lastAccReading=t
		
		#print "acc velocity",self.accZVelocity, self.baroVelocity#self.accHeight,self.baroFilter.get()
		#print "altitude {:.2f}cm/s {:.2f}cm {:.2f}cm".format(self.accZVelocity,self.baroVelocityFilter.get(),0)

	def updateBaroAltitude(self):
		t=self.board.altitude['timestamp']
		alt=self.board.altitude['estalt']
		offset=(self.airmainloop.throttle-1000)*self.baroThrottleOffset
		#alt+=offset
		#print "baro",alt,alt+offset
		
		
		if self.lastBaroReading!=None:
			self.baroAltFilter.add(alt-self.baroOffset)
			td=t-self.lastBaroReading
			if td<0 or td>1: #happens if clock changes
				self.lastBaroReading=t
				return
			self.baroVelocityFilter.add((alt-self.lastBaroAlt)/td)
			
			#pull baroAlt drift back to sonar
			if self.sonarAlt>0 and self.baroAltFilter.get()<300:
				heightDiff=self.baroAltFilter.get()-self.sonarAlt
				self.baroOffset+=heightDiff*0.4
			
		else:
			self.baroOffset=alt
			
			
		self.lastBaroAlt=alt
		self.lastBaroReading=t
	
	
	def updateSonarDistance(self,now):
		distance=self.airmainloop.sonar.getDistance()
		
		if distance>0 and abs(self.board.attitude['angx'])<15 and abs(self.board.attitude['angy'])<15:
			#print self.board.attitude['angx']
			if self.lastSonarReading==None or now-self.lastSonarReading>1:
				self.sonarAlt=distance
			else:
				self.sonarAlt=distance #self.sonarDistance*0.25+distance*0.75
			self.lastSonarReading=now	
				
		if self.lastSonarReading!=None and now-self.lastSonarReading>0.5:
			self.sonarAlt=-1
			
		#print "sonar {:.2f}cm {:.2f}cm".format(self.sonarAlt,self.baroAltFilter.get())
	
	def getFusionAltitude(self):
		baroHeight=self.baroAltFilter.get()
		if self.sonarAlt>0 and baroHeight<300:
			altitude=self.sonarAlt
		else:
			altitude=baroHeight
		return altitude
		
	def getBaroAltitude(self):
		return self.baroAltFilter.get()
	
	def getSonarAltitude(self):
		return self.sonarAlt
		
	def getAltitudeVelocity(self):
		return self.accZVelocity;
		
		
	altHoldLastTime=0
	altHoldOffset=1500
	altHoldThrottleMultiplier=1.0 #how throttle alters altitude hold
	altHoldSetpoint=0.0
	
	pidAltP=0
	pidAltI=0
	pidAltD=0
	
	def doAltitudeHold(self,now):
		if now-self.altHoldLastTime>0.5:
			self.altHoldPreviousError=0 # self.throttle/self.altHoldD*dt
			self.altHoldIntegral=0
			if self.airmainloop.throttleIn<1150:
				self.altHoldSetpoint=0
			else:
				self.altHoldSetpoint=self.getFusionAltitude()
			self.altHoldControl=False
		else:
		
			alt=self.getFusionAltitude()
			error=constrain(self.altHoldSetpoint-alt,-100,100)
			p=constrain(self.airmainloop.altHoldP*0.05*error, -200,200)
			vel=self.baroVelocityFilter.get()
			d=constrain(self.airmainloop.altHoldD*0.005*vel,-150,150)
			
			#scale hoverpoint throttle for tilt from level (more tilt - more thrust needed)
			throttleHoverpoint=self.hoverpoint-1000.0
			throttleHoverpoint/=self.airmainloop.theta
			hoverpoint=throttleHoverpoint+1000
			
			output=constrain(hoverpoint/self.airmainloop.theta+p+d,1350,1650)
			self.airmainloop.throttle=output
			self.pidAltP=p
			self.pidAltD=d
		
			'''
			alt=self.getFusionAltitude()
			altError=constrain(self.altHoldSetpoint-alt,-100,100)
			setVel=constrain(self.airmainloop.altHoldA*0.01*altError,-50,50)
		
			#altitude P control
			error=constrain(self.altHoldSetpoint-alt,-100,100)
			

			p=constrain(self.airmainloop.altHoldP*0.1*error, -200,200)
			
			#velocity I control
			dt=now-self.altHoldLastTime
			self.altHoldIntegral=constrain(self.altHoldIntegral+self.airmainloop.altHoldI*0.01*error*dt, -200,200)
			i=self.altHoldIntegral
			
			#velocity D control
			vel=setVel-self.baroVelocityFilter.get()
			d=constrain(self.airmainloop.altHoldD*0.01*vel,-150,150)
			
			output=constrain(self.altHoldOffset+p+i+d,1200,1650)
			self.airmainloop.throttle=output
			
			#print "altHold", altError,output
			#print "altitudeHold {:.0f}cm {:.1f}cm/s {:.0f} P{:.0f} I{:.0f} D{:.0f}".format(altError,setVel,output,p,i,d)
			self.pidAltP=p
			self.pidAltI=i
			self.pidAltD=d
			'''
			
			
		# throttle must return to center before control can commence
			if abs(self.airmainloop.throttleIn-1000.0)<10 and not self.altHoldControl:
				self.altHoldControl=True
				
				
			if self.altHoldControl:
				#allow throttle to alter altitude
				throttleOffset=(self.airmainloop.throttleIn-1000)/1000.0;
				self.altHoldSetpoint+=throttleOffset*self.altHoldThrottleMultiplier*0.1
				if self.altHoldSetpoint<0.0:
					self.altHoldSetpoint=0.0
		
		self.altHoldLastTime=now	
		
	def doAltitudeHold2(self,now):
		if now-self.altHoldLastTime>0.5:
			self.altHoldPreviousError=0 # self.throttle/self.altHoldD*dt
			self.altHoldIntegral=0
			self.altHoldSetpoint=self.getFusionAltitude()
			self.altHoldControl=False
		else:
		
			#altitude P control
			alt=self.getFusionAltitude()
			altError=constrain(self.altHoldSetpoint-alt,-100,100)
			setVel=constrain(self.airmainloop.altHoldA*0.01*altError,-50,50)
			
			#velocity P control
			#error=setVel-self.accZVelocity
			error=setVel-self.baroVelocityFilter.get()
			 
			p=constrain(self.airmainloop.altHoldP*0.1*error, -200,200)
			
			#velocity I control
			dt=now-self.altHoldLastTime
			self.altHoldIntegral=constrain(self.altHoldIntegral+self.airmainloop.altHoldI*0.01*error*dt, -200,200)
			i=self.altHoldIntegral
			
			#velocity D control
			d=constrain(self.airmainloop.altHoldD*0.01*self.accZ,-150,150)
			
			output=constrain(self.altHoldOffset+p+i+d,1200,1650)
			self.airmainloop.throttle=output
			
			#print "altHold", altError,output
			#print "altitudeHold {:.0f}cm {:.1f}cm/s {:.0f} P{:.0f} I{:.0f} D{:.0f}".format(altError,setVel,output,p,i,d)
			self.pidAltP=p
			self.pidAltI=i
			self.pidAltD=d
			
		# throttle must return to center before control can commence
			if abs(self.airmainloop.throttleIn-1000.0)<10 and not self.altHoldControl:
				self.altHoldControl=True
				
				
			if self.altHoldControl:
				#allow throttle to alter altitude
				throttleOffset=(self.airmainloop.throttleIn-1000)/1000.0;
				self.altHoldSetpoint+=throttleOffset*self.altHoldThrottleMultiplier*0.1
				if self.altHoldSetpoint<0.0:
					self.altHoldSetpoint=0.0
		
		self.altHoldLastTime=now		
		