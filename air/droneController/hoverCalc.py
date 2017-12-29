import math, random, os, time
from windowfilter import WindowAverage

class HoverCalc:

	def __init__(self,airmainloop):
		self.airmainloop=airmainloop
		self.throttleAverage=WindowAverage(32)
		self.accAverage=WindowAverage(32)
		self.G=-981 #cm/s^2
		self.minThrottle=1150
		

	def update(self,acc,throttle):
		self.accAverage.add(acc)
		self.throttleAverage.add(throttle)
		
		#calculate hoverpoint throttle
		avgAcc=self.accAverage.get()
		avgThrottle=self.throttleAverage.get()
		c=(avgAcc-self.G)/(avgThrottle-self.minThrottle)
		b=self.G-self.minThrottle*c
		hoverpoint=-b/c
		print "hoverpoint2",hoverpoint,avgThrottle,avgAcc,self.airmainloop.altitude.getFusionAltitude()
		
	