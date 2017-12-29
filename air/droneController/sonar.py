import wiringpi
import time

class Sonar:

	triggerPin=0 # gpio17
	echoPin=2 # gpio27

		
	def onEcho2(self):
		self.p.append(time.time())

	def __init__(self):
		#print "init"
		#wiringpi.wiringPiSetup()
		wiringpi.pinMode(self.triggerPin,wiringpi.OUTPUT)
		wiringpi.pinMode(self.echoPin,wiringpi.INPUT)
		wiringpi.digitalWrite(self.triggerPin,False)
		wiringpi.wiringPiISR(self.echoPin,wiringpi.INT_EDGE_BOTH,self.onEcho2)
		self.p=[]
		
	def doTrigger(self):
		self.p=[]
		wiringpi.digitalWrite(self.triggerPin,True)
		time.sleep(0.00001)
		wiringpi.digitalWrite(self.triggerPin,False)
		
	def getDistance(self):
		#print self.p
		dist=None
		if len(self.p)>=2:
			d=self.p[1]-self.p[0]
			dist=d*17150
			if dist>400:
				dist=-1
		return dist

'''		
sonar=Sonar()
time.sleep(0.5)
while True:
	sonar.doTrigger()
	time.sleep(0.25)
	dist=sonar.getDistance()
	time.sleep(0.25)
	print dist
'''