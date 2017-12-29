import wiringpi
import time
from threading import Thread

twelthRootOf2=2**(1.0/12)

class SoundEnvelope:

	thread=None;
	threadInterrupted=False

	def __init__(self,clock=76, pin=1):
		self.clock=clock
		self.pin=pin
		
		wiringpi.wiringPiSetup()
		wiringpi.pinMode(self.pin,wiringpi.OUTPUT)
		wiringpi.pinMode(self.pin,wiringpi.PWM_OUTPUT)
		wiringpi.pwmSetMode(wiringpi.PWM_MODE_BAL)
		wiringpi.pwmSetMode(wiringpi.PWM_MODE_MS)
		wiringpi.pwmSetClock(clock)
		self.wiringpi=wiringpi
		
	def getRangeForNote2(self,note):
		freq=65.41*(twelthRootOf2**note)
		rg=int(19.2e6/self.clock/freq)
		return rg
	
	def getRangeForNote(self, note, subnote):
		scaleVal=scale[int(note%12)]
		if subnote!=0:
			scaleVal2=scale[1+int(note%12)]
			scaleVal=scaleVal2*subnote+scaleVal*(1-subnote)
	
		octave=1+int(note/12)
		freq=65.41*(2**octave)*scaleVal
		rg=int(19.2e6/clock/freq)
		#print note,freq,range, octave, int(note%12)
		return rg

	def setSound(self,note,volume):
		rg=self.getRangeForNote2(note)
		mr=int(rg*0.5*volume)
		self.wiringpi.pwmSetRange(rg)
		self.wiringpi.pwmWrite(self.pin,mr)
		
	def playRun(self,list,pitchOffset,speed,legato):
		env=None
		for sound in list:
			if len(sound)==3:
				env=sound[2]
			duration=int(sound[1]/speed)
			pitch=sound[0]
			if pitch==None:
				for i in range(0,duration):
					time.sleep(0.01)
					if self.threadInterrupted:
						break
			else:
				env.start((pitch+pitchOffset)*4,duration*legato)
				for i in range(0,duration):
					env.process(self)
					time.sleep(0.01)
					if self.threadInterrupted:
						break
			if self.threadInterrupted:
				break
		self.setSound(0,0)
		
	def play(self,list,pitchOffset=0,speed=1,legato=0.8):
		if self.thread!=None:
			self.threadInterrupted=True
			self.thread.join()
			self.threadInterrupted=False

		thread = Thread(target=self.playRun, args=(list,pitchOffset,speed,legato,))
		thread.start()
		self.thread=thread
		
class Envelope:

	def __init__(self,s,pi1,pi2,pi3,pn1,pn2,pn3,aa,ad,asus,ar,ala,ald):
		self.s=s
		self.pi=(pi1,pi2,pi3)
		self.pn=(pn1,pn2,pn3)
		self.aa=aa
		self.ad=ad
		self.asus=asus
		self.ar=ar
		self.ala=ala
		self.ald=ald
		
	def start(self,pitch,duration):
		self.pitch=pitch
		self.volume=0.0
		self.div=0
		self.phase=-1
		self.phaseStepCount=0
		self.ampPhase=0
		self.duration=duration
	
	def process(self,soundEnvelope):

		self.div=(self.div+1)%self.s
		if self.div==0:
			while self.phaseStepCount==0:
				self.phase=(self.phase+1)%3
				self.phaseStepCount=self.pn[self.phase]
			self.pitch+=self.pi[self.phase]
			self.phaseStepCount=self.phaseStepCount-1
			
			
			if self.pitch>255:
				self.pitch=self.pitch-256
			elif self.pitch<0:
				self.pitch=self.pitch+256
			
		if self.duration==0:
			self.ampPhase=3
		else:
			self.duration=self.duration-1
			
		if self.ampPhase==0: #attach
			self.volume+=self.aa
			if self.volume>=self.ala:
				self.ampPhase=1
		elif self.ampPhase==1: #decay
			self.volume+=self.ad
			if self.volume<=self.ald:
				self.ampPhase=2
		elif self.ampPhase==2: #sustain
			self.volume+=self.asus
			if self.volume<=0:
				self.ampPhase=4
		elif self.ampPhase==3:
			self.volume+=self.ar
			if self.volume<=0:
				self.ampPhase=4
	
		#print self.ampPhase, self.volume
		
		
		if self.volume<=0:
			soundEnvelope.setSound(12+self.pitch/4.0,0)
			return True
		else:
			soundEnvelope.setSound(12+self.pitch/4.0,self.volume/128.0)
			return False