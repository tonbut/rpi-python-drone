import serial, time,sys

class SetRadio:

	def __init__(self,serPort):
		self.ser = serial.Serial()
		self.ser.port = serPort
		self.ser.baudrate = 57600
		self.ser.bytesize = serial.EIGHTBITS
		self.ser.parity = serial.PARITY_NONE
		self.ser.stopbits = serial.STOPBITS_ONE
		self.ser.timeout = 0
		self.ser.xonxoff = False
		self.ser.rtscts = False
		self.ser.dsrdtr = False
		self.ser.writeTimeout = 2
		self.ser.timeout=0.5

		self.ser.open()
		self.ser.flushInput()
		self.ser.flushOutput()
		self.ser.write("ATO\n\r")
		time.sleep(1)
		self.ser.write("+++")
		time.sleep(2)
		self.ser.write("ATS4=14\n\r") # 25mW
		time.sleep(0.1)
		self.ser.write("ATS11=100\n\r") # 100% duty
		time.sleep(0.1)
		self.ser.write("ATS6=1\n\r") # mavlink with RC_OVERRIDE
		time.sleep(0.1)
		self.ser.write("ATS2=64\n\r") #64kbps
		time.sleep(0.1)
		self.ser.write("ATI5\n\r")
		time.sleep(0.5)
		self.ser.write("AT&W\n\r")
		time.sleep(1.0)
		try:
			while True:
				c=self.ser.read();
				if len(c)==0:
					break;
				#sys.stdout.write('.')
				sys.stdout.write(c)
		finally:
			
			self.ser.write("ATZ\n\r")
			time.sleep(0.5)
			#self.ser.write("ATO\n\r")
			#time.sleep(0.5)
			self.ser.close()
	
sr=SetRadio("/dev/ttyS0")