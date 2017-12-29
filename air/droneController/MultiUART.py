import spidev
import time


class MultiUART:

	spi=None
	uart=None
	
	def __init__(self,UART,SPIDivider):
		self.uart=UART
		spi = spidev.SpiDev()
		spi.open(0,0)
		self.spi=spi
		spi.lsbfirst=False
		#div64 = ??? 250Mhz/64 = 4000000
		spi.max_speed_hz=250000000/SPIDivider
		self.spi.cshigh=False
		spi.loop=False
		#spi.bits_per_word=8
		return
		
	def cleanup(self):
		self.spi.close()
		return
		
	# Returns the number of received bytes held in queue for the selected channel.	
	def checkRx(self):
		result=0
		self.spi.xfer2( [0x10 | self.uart ]);
		time.sleep(0.00250)
		result=self.spi.readbytes(1)[0]
		time.sleep(0.00250)
		return result
		
	def checkTx(self):
		result=0
		self.spi.xfer2( [0x30 | self.uart ]);
		time.sleep(0.00250)
		result=self.spi.readbytes(1)[0]
		time.sleep(0.00250)
		return result
		
	def receiveByte(self):
		self.spi.xfer2( [0x20 | self.uart ]);
		time.sleep(0.001)
		self.spi.xfer2( [1]);
		time.sleep(0.001)
		result=self.spi.xfer2([0xFF])[0]
		time.sleep(0.001)
		return result
		
	def flushRx(self):
		c=self.checkRx()
		if c>0:
			self.receiveBytes(c)
			c=self.checkRx()
			if c>0:
				self.receiveBytes(c)
		
	def receiveBytes(self, NUMBYTES):
		self.spi.xfer2( [0x20 | self.uart ]);
		time.sleep(0.001)
		self.spi.xfer2( [NUMBYTES]);
		result=[]
		for i in range(0,NUMBYTES):
			time.sleep(0.0005)
			v=self.spi.xfer2([0xFF])[0]
			result.append(v)
		time.sleep(0.001)
		return result
		
	def transmitByte(self, DATA):
		self.spi.xfer2( [0x40 | self.uart ]);
		time.sleep(0.001)
		self.spi.xfer2( [1]);
		time.sleep(0.001)
		result=self.spi.xfer2([DATA])[0]
		time.sleep(0.001)
		return
		
	def transmitBytes(self, DATA):
		self.spi.xfer2( [0x40 | self.uart ]);
		time.sleep(0.001)
		length=len(DATA)
		self.spi.xfer2( [length]);
		for i in range(0,length):
			time.sleep(0.0005)
			self.spi.xfer2([DATA[i]])
		time.sleep(0.001)
		return
		
	# Configures the baud rate of the selected channel.
	# Baud : 0=1200, 1=2400, 2=4800, 3=9600, 4=19200, 5=38400, 6=57600, 7=115200
	def setBaud(self, BAUD):
		self.spi.xfer2( [0x80 | self.uart ]);
		time.sleep(0.00250)
		result=self.spi.xfer2([ BAUD ]);
		time.sleep(0.1)
		return
		