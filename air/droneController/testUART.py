import time
from MultiUART import MultiUART

muart=MultiUART(1,64)
muart.setBaud(6)

#muart=MultiUART(0,64)
#muart.setBaud(4)

muart.flushRx()
for i in range(1,1000):
	c=muart.checkRx()
	c2=muart.checkTx()
	if c>0:
		ch=muart.receiveBytes(c)
		print "c=",c,c2,ch
	else:
		#print "nothing",c,c2
		#ch=muart.receiveByte(0)
		time.sleep(0.1)
		muart.transmitBytes([65,66,67,68,69,70,71,72,73,10,13] )
muart.cleanup()
