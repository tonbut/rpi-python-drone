import struct

class LightTelemetry:

	mBuffer=[]
	mPitch=None
	mRoll=None
	mHeading=None

	def processByte(self,b):
		
		result=None
		if b=='$':
			result=self.processBuffer()
			self.mBuffer=[]
		if len(self.mBuffer)<30:
			self.mBuffer.append(b)
		else:
			print "overflow";
			self.mBuffer=[]
		return result
	
	def verifyChecksum(self,length):
		result=False;
		if len(self.mBuffer)==4+length:
			actualChecksum=0;
			for i in range(3,3+length):
				actualChecksum^=ord(self.mBuffer[i])
			expectedChecksum=ord(self.mBuffer[3+length]);
			result=expectedChecksum==actualChecksum;
		return result;
		
	def getInt16(self,p):
		s="".join(self.mBuffer[p:p+2])
		t=struct.unpack( "<h", s )
		return t[0]
	def getInt32(self,p):
		s="".join(self.mBuffer[p:p+4])
		t=struct.unpack( "<i", s )
		return t[0]
	
	def processBuffer(self):
		#print self.mBuffer
		result=None
		if len(self.mBuffer)>3 and self.mBuffer[0]=='$' and self.mBuffer[1]=='T':
			type=self.mBuffer[2]
			if type=='A':
				if self.verifyChecksum(6):
					result=type
					self.mPitch=self.getInt16(3)
					self.mRoll=self.getInt16(5)
					self.mHeading=self.getInt16(7)
					#print "A",self.mPitch, self.mRoll, self.mHeading
			elif type=='S':
				if self.verifyChecksum(7):
					result=type
					self.mBattery=self.getInt16(3)
					self.mCurrent=self.getInt16(5)
					self.mRSSI=ord(self.mBuffer[7])
					self.mAirSpeed=ord(self.mBuffer[8])
					status=ord(self.mBuffer[9])
					self.mArmed=status&0x01
					self.mFailsafe=status&0x02
					self.mFlightMode=status>>2
					print "S",self.mBattery
			elif type=='G':
				if self.verifyChecksum(14):
					result=type
					self.mLatitude=self.getInt32(3)
					self.mLongitude=self.getInt32(7)
					self.mGroundSpeed=ord(self.mBuffer[11])
					self.mAltitude=self.getInt32(12)
					satellites=ord(self.mBuffer[16])
					self.mSatellites=satellites>>2
					self.mFix=satellites&0x03
					print "GPS",self.mLatitude, self.mLongitude, self.mAltitude, self.mGroundSpeed, self.mSatellites
		return result;