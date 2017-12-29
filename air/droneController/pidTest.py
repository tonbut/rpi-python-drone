from pyMultiwii import MultiWii
import struct, time

multiwii="/dev/ttyUSB0"
board = MultiWii(multiwii)


pids=board.getData(MultiWii.PID)
time.sleep(0.1)
print pids
board.PIDcoef[1]=40
board.setPID2()
time.sleep(1.0)
pids=board.getData(MultiWii.PID)
pids=board.getData(MultiWii.PID)
'''
for i in range(0,2):
	try:
		pids=board.getData(MultiWii.PID)
		time.sleep(0.1)
	except Error, e:
		print e
		pass
'''
print pids


'''
data = [1511,1588,1000,1500,1000,1044,1000,1033]
board.sendCMD(16,MultiWii.SET_RAW_RC,data)


dataEncoded=struct.pack('<8H', *data)
board.sendCMDRaw(MultiWii.SET_RAW_RC,dataEncoded)
'''

#encoded = struct.pack('<'+'B'*(len(board.PIDcoef)),*board.PIDcoef)
#print encoded
#decoded=struct.unpack('<'+'B'*(len(board.PIDcoef)),encoded)
#print decoded