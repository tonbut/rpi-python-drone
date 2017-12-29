#!/usr/bin/env python

"""multiwii.py: Handles Multiwii Serial Protocol."""

__author__ = "Aldo Vargas"
__copyright__ = "Copyright 2017 Altax.net"

__license__ = "GPL"
__version__ = "1.6"
__maintainer__ = "Aldo Vargas"
__email__ = "alduxvm@gmail.com"
__status__ = "Development"


import serial, time, struct


class MultiWii:

    """Multiwii Serial Protocol message ID"""
    """ notice: just attitude, rc channels and raw imu, set raw rc are implemented at the moment """
    IDENT = 100
    STATUS = 101
    RAW_IMU = 102
    SERVO = 103
    MOTOR = 104
    RC = 105
    RAW_GPS = 106
    COMP_GPS = 107
    ATTITUDE = 108
    ALTITUDE = 109
    ANALOG = 110
    RC_TUNING = 111
    PID = 112
    BOX = 113
    MISC = 114
    MOTOR_PINS = 115
    BOXNAMES = 116
    PIDNAMES = 117
    WP = 118
    BOXIDS = 119
    RC_RAW_IMU = 121
    SET_RAW_RC = 200
    SET_RAW_GPS = 201
    SET_PID = 202
    SET_BOX = 203
    SET_RC_TUNING = 204
    ACC_CALIBRATION = 205
    MAG_CALIBRATION = 206
    SET_MISC = 207
    RESET_CONF = 208
    SET_WP = 209
    SWITCH_RC_SERIAL = 210
    IS_SERIAL = 211
    EEPROM_WRITE = 250
    DEBUG = 254


    """Class initialization"""
    def __init__(self, serPort):

        """Global variables of data"""
        self.PIDcoef = [] #{'rp':0,'ri':0,'rd':0,'pp':0,'pi':0,'pd':0,'yp':0,'yi':0,'yd':0}
        self.rcChannels = {'roll':0,'pitch':0,'yaw':0,'throttle':0,'elapsed':0,'timestamp':0}
        self.rawIMU = {'ax':0,'ay':0,'az':0,'gx':0,'gy':0,'gz':0,'mx':0,'my':0,'mz':0,'elapsed':0,'timestamp':0}
        self.motor = {'m1':0,'m2':0,'m3':0,'m4':0,'elapsed':0,'timestamp':0}
        self.attitude = {'angx':0,'angy':0,'heading':0,'elapsed':0,'timestamp':0}
        self.altitude = {'estalt':0,'vario':0,'elapsed':0,'timestamp':0}
        self.message = {'angx':0,'angy':0,'heading':0,'roll':0,'pitch':0,'yaw':0,'throttle':0,'elapsed':0,'timestamp':0}
        self.gps = {'fix':0,'satellites':0,'lat':0,'lon':0,'altitude':0,'speed':0,'course':0}
        self.status = { 'cycleTime':0, 'i2c_errors_count':0, 'sensor':0, 'flag':0, 'currentSet':0, 'unknown1':0, 'unknown2':0 }
        self.vbat = 0.0;
        self.current = 0.0;
        self.temp = ();
        self.temp2 = ();
        self.elapsed = 0
        self.PRINT = 0

        self.ser = serial.Serial()
        self.ser.port = serPort
        self.ser.baudrate = 115200
        self.ser.bytesize = serial.EIGHTBITS
        self.ser.parity = serial.PARITY_NONE
        self.ser.stopbits = serial.STOPBITS_ONE
        self.ser.timeout = 0
        self.ser.xonxoff = False
        self.ser.rtscts = False
        self.ser.dsrdtr = False
        self.ser.writeTimeout = 2
        
        """Time to wait until the board becomes operational"""
        wakeup = 2
        try:
            self.ser.open()
            if self.PRINT:
                print "Waking up board on "+self.ser.port+"..."
            for i in range(1,wakeup):
                if self.PRINT:
                    print wakeup-i
                    time.sleep(1)
                else:
                    time.sleep(1)
        except Exception, error:
            print "\n\nError opening "+self.ser.port+" port.\n"+str(error)+"\n\n"
            raise error

	

    """Function for sending a command to the board"""
    def sendCMD(self, data_length, code, data):
        checksum = 0
        total_data = ['$', 'M', '<', data_length, code] + data
        for i in struct.pack('<2B%dH' % len(data), *total_data[3:len(total_data)]):
            checksum = checksum ^ ord(i)
        total_data.append(checksum)
        try:
            b = None
            s=struct.pack('<3c2B%dHB' % len(data), *total_data)
            b = self.ser.write(s)
        except Exception, error:
            #print "\n\nError in sendCMD."
            #print "("+str(error)+")\n\n"
            pass

    """Function for sending a command to the board and receive attitude"""
    """
    Modification required on Multiwii firmware to Protocol.cpp in evaluateCommand:

    case MSP_SET_RAW_RC:
      s_struct_w((uint8_t*)&rcSerial,16);
      rcSerialCount = 50; // 1s transition 
      s_struct((uint8_t*)&att,6);
      break;

    """
    def sendCMDreceiveATT(self, data_length, code, data):
        checksum = 0
        total_data = ['$', 'M', '<', data_length, code] + data
        for i in struct.pack('<2B%dH' % len(data), *total_data[3:len(total_data)]):
            checksum = checksum ^ ord(i)
        total_data.append(checksum)
        try:
            start = time.time()
            b = None
            b = self.ser.write(struct.pack('<3c2B%dHB' % len(data), *total_data))
            while True:
                header = self.ser.read()
                if header == '$':
                    header = header+self.ser.read(2)
                    break
            datalength = struct.unpack('<b', self.ser.read())[0]
            code = struct.unpack('<b', self.ser.read())
            data = self.ser.read(datalength)
            temp = struct.unpack('<'+'h'*(datalength/2),data)
            self.ser.flushInput()
            self.ser.flushOutput()
            elapsed = time.time() - start
            self.attitude['angx']=float(temp[0]/10.0)
            self.attitude['angy']=float(temp[1]/10.0)
            self.attitude['heading']=float(temp[2])
            self.attitude['elapsed']=round(elapsed,3)
            self.attitude['timestamp']="%0.2f" % (time.time(),) 
            return self.attitude
        except Exception, error:
            #print "\n\nError in sendCMDreceiveATT."
            #print "("+str(error)+")\n\n"
            pass

    """Function to arm / disarm """
    """
    Modification required on Multiwii firmware to Protocol.cpp in evaluateCommand:

    case MSP_SET_RAW_RC:
      s_struct_w((uint8_t*)&rcSerial,16);
      rcSerialCount = 50; // 1s transition 
      s_struct((uint8_t*)&att,6);
      break;

    """
    def arm(self):
        timer = 0
        start = time.time()
        yaw=1500
        while timer < 1.1:
            data = [1500,1500,1000,yaw,1000,1000,1000,1000]
            if yaw<2000:
            	yaw=yaw+250;
            #print yaw
            self.sendCMD(16,MultiWii.SET_RAW_RC,data)
            time.sleep(0.05)
            timer = timer + (time.time() - start)
            start =  time.time()

    def disarm(self):
        timer = 0
        start = time.time()
        yaw=1500
        while timer < 1.1:
            data = [1500,1500,1000,yaw,0,0,0,0]
            if yaw>1000:
            	yaw=yaw-250;
            #print yaw
            self.sendCMD(16,MultiWii.SET_RAW_RC,data)
            time.sleep(0.05)
            timer = timer + (time.time() - start)
            start =  time.time()
    
    
    
    
    
    def setPID(self,pd):
        nd=[]
        for i in np.arange(1,len(pd),2):
            nd.append(pd[i]+pd[i+1]*256)
        data = pd
        print "PID sending:",data
        self.sendCMD(30,MultiWii.SET_PID,data)
        self.sendCMD(0,MultiWii.EEPROM_WRITE,[])
        
        
    def sendCMDRaw(self,code,string):
    	data_length=len(string)
    	checksum = data_length ^ code
    	for i in string:
    		checksum = checksum ^ ord(i)
    	encoded = struct.pack('<3c2B','$', 'M', '<', data_length, code)
    	encoded += string
    	encoded += struct.pack('<B',checksum)
    	#print encoded
    	self.ser.write(encoded)
        
    def setPID2(self):
    	data = struct.pack('<'+'B'*(len(self.PIDcoef)),*(self.PIDcoef))
    	#da=[]
    	#for c in data:
    	#	da.append(c)
    	self.sendCMDRaw(MultiWii.SET_PID,data)
    	self.sendCMD(0,MultiWii.EEPROM_WRITE,[])

    """Function to receive a data packet from the board"""
    def getData(self, cmd):
        self.sendCMD(0,cmd,[])
        #self.ser.flushOutput()
        #self.ser.flushInput()
        #print ">",cmd
        #while self.ser.in_waiting>0:
        #	self.doRead()
        
    def flushIn(self):
    	self.ser.flushInput()
    def flushOut(self):
    	self.ser.flushOutput()
        
    def doRead(self):
		code=None
		inwaiting=self.ser.in_waiting
		#print "inwaiting",inwaiting
		if inwaiting>0:
			
			now=time.time()
			header = self.ser.read()
			if header != '$':
				#print "header!",header
				pass
			else:
				header = header+self.ser.read(2)
				if len(header)!=3:
					return
				b=self.ser.read()
				#print "b=",len(b),len(header)
				datalength = struct.unpack('<b', b)[0]
				#print datalength
				codeA = struct.unpack('<b', self.ser.read())
				code=codeA[0]
				data = self.ser.read(datalength)
				#print data
				
		
				elapsed = 0#time.time() - start
				#print "<",code
				#print cmd,code#,datalength,len(data),header
		
				#print temp
				if code == MultiWii.ATTITUDE:
					temp = struct.unpack('<'+'h'*(datalength/2),data)
					self.attitude['angx']=float(temp[0]/10.0)
					self.attitude['angy']=float(temp[1]/10.0)
					self.attitude['heading']=float(temp[2])
					#self.attitude['elapsed']=round(elapsed,3)
					self.attitude['timestamp']=now
					#return self.attitude
				elif code == MultiWii.ALTITUDE:
					temp = struct.unpack('<'+'h'*(datalength/2),data)
					self.altitude['estalt']=float(temp[0])
					self.altitude['vario']=float(temp[1])
					#self.altitude['elapsed']=round(elapsed,3)
					self.altitude['timestamp']=now
					#return self.rcChannels
				elif code == MultiWii.RC:
					temp = struct.unpack('<'+'h'*(datalength/2),data)
					self.rcChannels['roll']=temp[0]
					self.rcChannels['pitch']=temp[1]
					self.rcChannels['yaw']=temp[2]
					self.rcChannels['throttle']=temp[3]
					#self.rcChannels['elapsed']=round(elapsed,3)
					self.rcChannels['timestamp']=now
					#return self.rcChannels
				elif code == MultiWii.RAW_IMU:
					temp = struct.unpack('<'+'h'*(datalength/2),data)
					self.rawIMU['ax']=float(temp[0])
					self.rawIMU['ay']=float(temp[1])
					self.rawIMU['az']=float(temp[2])
					self.rawIMU['gx']=float(temp[3])
					self.rawIMU['gy']=float(temp[4])
					self.rawIMU['gz']=float(temp[5])
					self.rawIMU['mx']=float(temp[6])
					self.rawIMU['my']=float(temp[7])
					self.rawIMU['mz']=float(temp[8])
					#self.rawIMU['elapsed']=round(elapsed,3)
					self.rawIMU['timestamp']=now
					#return self.rawIMU
				elif code == MultiWii.MOTOR:
					#print datalength
					#print data
					temp = struct.unpack('<'+'h'*(datalength/2),data) #+'b'
					self.motor['m1']=float(temp[0])
					self.motor['m2']=float(temp[1])
					self.motor['m3']=float(temp[2])
					self.motor['m4']=float(temp[3])
					#print temp[0],temp[1],temp[2],temp[3]
					#self.motor['elapsed']="%0.3f" % (elapsed,)
					self.motor['timestamp']=now
					#return self.motor
				elif code == MultiWii.PID:
					temp = struct.unpack('<'+'B'*(datalength),data)
					self.PIDcoef=list(temp)
					#return self.PIDcoef
				elif code == MultiWii.ANALOG:
					if len(data)==7:
						temp = struct.unpack('<Bhhh',data)
						#print temp
						self.current=temp[3]/50.0;
						self.vbat=temp[0]/10.0
				elif code == MultiWii.STATUS:
						if len(data)==15:
							temp = struct.unpack('<HHHiBHH',data)
							#print temp
							self.status['cycleTime']=temp[0]
							self.status['i2c_errors_count']=temp[1]
							self.status['sensor']=temp[2]
							self.status['flag']=temp[3]
							self.status['currentSet']=temp[4]
							self.status['unknown1']=temp[5]
							self.status['unknown2']=temp[6]
							self.status['timestamp']=now
				elif code == MultiWii.RAW_GPS:
					if len(data)==16:
						temp = struct.unpack('<BBiiHHH',data)
						#print temp
						#{'fix':0,'satellites':0,'lat':0,'lon':0,'altitude':0,'speed':0,'course':0}
						self.gps['fix']=temp[0]
						self.gps['satellites']=temp[1]
						self.gps['lat']=float(temp[2])/10000000.0
						self.gps['lon']=float(temp[3])/10000000.0
						self.gps['altitude']=temp[4]
						self.gps['speed']=temp[5]
						self.gps['course']=float(temp[6])/10.0
						self.gps['timestamp']=now
						#print self.gps
		return code		


    """Function to receive a data packet from the board. Note: easier to use on threads"""
    def getDataInf(self, cmd):
        while True:
            try:
                start = time.clock()
                self.sendCMD(0,cmd,[])
                while True:
                    header = self.ser.read()
                    if header == '$':
                        header = header+self.ser.read(2)
                        break
                datalength = struct.unpack('<b', self.ser.read())[0]
                code = struct.unpack('<b', self.ser.read())
                data = self.ser.read(datalength)
                temp = struct.unpack('<'+'h'*(datalength/2),data)
                elapsed = time.clock() - start
                self.ser.flushInput()
                self.ser.flushOutput()
                if cmd == MultiWii.ATTITUDE:
                    self.attitude['angx']=float(temp[0]/10.0)
                    self.attitude['angy']=float(temp[1]/10.0)
                    self.attitude['heading']=float(temp[2])
                    self.attitude['elapsed']="%0.3f" % (elapsed,)
                    self.attitude['timestamp']="%0.2f" % (time.time(),)
                elif cmd == MultiWii.RC:
                    self.rcChannels['roll']=temp[0]
                    self.rcChannels['pitch']=temp[1]
                    self.rcChannels['yaw']=temp[2]
                    self.rcChannels['throttle']=temp[3]
                    self.rcChannels['elapsed']="%0.3f" % (elapsed,)
                    self.rcChannels['timestamp']="%0.2f" % (time.time(),)
                elif cmd == MultiWii.RAW_IMU:
                    self.rawIMU['ax']=float(temp[0])
                    self.rawIMU['ay']=float(temp[1])
                    self.rawIMU['az']=float(temp[2])
                    self.rawIMU['gx']=float(temp[3])
                    self.rawIMU['gy']=float(temp[4])
                    self.rawIMU['gz']=float(temp[5])
                    self.rawIMU['elapsed']="%0.3f" % (elapsed,)
                    self.rawIMU['timestamp']="%0.2f" % (time.time(),)
                elif cmd == MultiWii.MOTOR:
                    self.motor['m1']=float(temp[0])
                    self.motor['m2']=float(temp[1])
                    self.motor['m3']=float(temp[2])
                    self.motor['m4']=float(temp[3])
                    self.motor['elapsed']="%0.3f" % (elapsed,)
                    self.motor['timestamp']="%0.2f" % (time.time(),)
            except Exception, error:
                pass

    """Function to ask for 2 fixed cmds, attitude and rc channels, and receive them. Note: is a bit slower than others"""
    def getData2cmd(self, cmd):
        try:
            start = time.time()
            self.sendCMD(0,self.ATTITUDE,[])
            while True:
                header = self.ser.read()
                if header == '$':
                    header = header+self.ser.read(2)
                    break
            datalength = struct.unpack('<b', self.ser.read())[0]
            code = struct.unpack('<b', self.ser.read())
            data = self.ser.read(datalength)
            temp = struct.unpack('<'+'h'*(datalength/2),data)
            self.ser.flushInput()
            self.ser.flushOutput()

            self.sendCMD(0,self.RC,[])
            while True:
                header = self.ser.read()
                if header == '$':
                    header = header+self.ser.read(2)
                    break
            datalength = struct.unpack('<b', self.ser.read())[0]
            code = struct.unpack('<b', self.ser.read())
            data = self.ser.read(datalength)
            temp2 = struct.unpack('<'+'h'*(datalength/2),data)
            elapsed = time.time() - start
            self.ser.flushInput()
            self.ser.flushOutput()

            if cmd == MultiWii.ATTITUDE:
                self.message['angx']=float(temp[0]/10.0)
                self.message['angy']=float(temp[1]/10.0)
                self.message['heading']=float(temp[2])
                self.message['roll']=temp2[0]
                self.message['pitch']=temp2[1]
                self.message['yaw']=temp2[2]
                self.message['throttle']=temp2[3]
                self.message['elapsed']=round(elapsed,3)
                self.message['timestamp']="%0.2f" % (time.time(),) 
                return self.message
            else:
                return "No return error!"
        except Exception, error:
            print error
