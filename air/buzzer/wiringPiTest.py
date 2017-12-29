# Pulsates an LED connected to GPIO pin 1 with a suitable resistor 4 times using softPwm
# softPwm uses a fixed frequency
import wiringpi
import time


PIN_TO_PWM = 1
clock=76

wiringpi.wiringPiSetup()

wiringpi.pinMode(PIN_TO_PWM,wiringpi.PWM_OUTPUT)
wiringpi.pwmSetMode(wiringpi.PWM_MODE_MS)
wiringpi.pwmSetClock(clock)
#wiringpi.pwmSetRange(1024)
#wiringpi.pwmWrite(PIN_TO_PWM,512)


twelthRootOf2=2**(1.0/12)


scale=[1,1.05946,1.12246,1.18921,1.25992,1.33483,1.41421,1.49831,1.58740,1.68179,1.78180,1.88775,2]

def getRangeForNote2(note):
	freq=65.41*(twelthRootOf2**note)
	rg=int(19.2e6/clock/freq)
	#print note,freq,rg, int(note%12)
	return rg
	
def getRangeForNote(note, subnote):
	global clock
	scaleVal=scale[int(note%12)]
	if subnote!=0:
		scaleVal2=scale[1+int(note%12)]
		scaleVal=scaleVal2*subnote+scaleVal*(1-subnote)
	
	octave=1+int(note/12)
	freq=65.41*(2**octave)*scaleVal
	rg=int(19.2e6/clock/freq)
	#print note,freq,range, octave, int(note%12)
	return rg

def setSound(note,volume):
	rg=getRangeForNote2(note)
	mr=int(rg*0.5*volume)
	wiringpi.pwmSetRange(rg)
	wiringpi.pwmWrite(PIN_TO_PWM,mr)
	#wiringpi.pwmSetRange(2048)
	#wiringpi.pwmWrite(PIN_TO_PWM,1024)
	#print rg,mr

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
	
	def process(self):

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
			setSound(12+self.pitch/4.0,0)
			return True
		else:
			setSound(12+self.pitch/4.0,self.volume/128.0)
			return False
		
	def play(self):
		while True:
			if self.process():
				break
			time.sleep(0.01)
	
#e=Envelope(1,70,16,2,2,0,0,126,0,0,-126,126,126)
#e=Envelope(4,-1,1,0,2,2,0,126,0,0,-126,126,126)
e=Envelope(2,1,-1,0,4,4,0,20,-1,0,-1,126,110)
e2=Envelope(4,48,-48,0,2,2,0,127,-4,0,-1,127,0)
#e=Envelope(1,-12,0,0,12,12,0,127,-2,0,-1,127,0)
#e=Envelope(1,0,0,0,3,3,2,32,-2,0,-16,127,8)
e=Envelope(1,0,0,0,3,3,2,127,0,0,-127,127,8)
e2=Envelope(2,-1,-1,-1,2,2,2,127,0,0,-1,127,0)
e3=Envelope(4,48,-48,0,2,2,0,127,-4,0,-1,127,0)

def play(list,pitchOffset=0,speed=1,legato=0.8):
	for sound in list:
		print sound
		env=sound[0]
		duration=int(sound[2]/speed)
		env.start((sound[1]+pitchOffset)*4,duration*legato)
		for i in range(0,duration):
			env.process()
			time.sleep(0.01)
	setSound(0,0)


play([[e3,12,25],[e3,16,25],[e3,19,50]],pitchOffset=12,speed=1,legato=0.5)

#e=Envelope(1,4,-4,0,12,12,0,2,0,0,-127,127,0)
#play([[e,120,200]])

'''
e.start(0,200)
e.play()
e.start(48,200)
e.play()
e.start(96,200)
e.play()
e.start(96+48,200)
e.play()
e.start(96+96,200)
e.play()
e.start(96+96+48,200)
e.play()
'''

'''
time.sleep(0.1)
e.start(48,100)
e.play()
time.sleep(0.1)
e.start(96,100)
e.play()
time.sleep(0.1)
e.start(0,100)
e.play()
'''
'''	
volume=1.0
for i in range(0,20):
	setSound(32+i/4.0,volume)
	#volume=volume*0.5
	if volume<0.01:
		volume=1.0
	time.sleep(0.1)
'''



#wiringpi.pinMode(PIN_TO_PWM,wiringpi.INPUT)

