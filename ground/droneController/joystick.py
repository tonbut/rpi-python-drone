# Released by rdb under the Unlicense (unlicense.org)
# Based on information from:
# https://www.kernel.org/doc/Documentation/input/joystick-api.txt

import os, struct, array
from fcntl import ioctl


class Joystick:
	
	mFile=None
	mOnButton=None
	mJSDev=None
	isReady=False
	axis_map = []
	axis_states = {}
	button_map = []
	button_states = {}
	
	def __init__(self, file, onButton):
		self.mFile=file
		self.mOnButton=onButton
		self.openJS()
		
	def openJS(self):
		self.mJSDev = open(self.mFile, 'rb')

		# Get the device name.
		#buf = bytearray(63)
		buf = array.array('c', ['\0'] * 64)
		ioctl(self.mJSDev, 0x80006a13 + (0x10000 * len(buf)), buf) # JSIOCGNAME(len)
		js_name = buf.tostring()
		print('Device name: %s' % js_name)

		# Get number of axes and buttons.
		buf = array.array('B', [0])
		ioctl(self.mJSDev, 0x80016a11, buf) # JSIOCGAXES
		num_axes = buf[0]

		buf = array.array('B', [0])
		ioctl(self.mJSDev, 0x80016a12, buf) # JSIOCGBUTTONS
		num_buttons = buf[0]

		# Get the axis map.
		buf = array.array('B', [0] * 0x40)
		ioctl(self.mJSDev, 0x80406a32, buf) # JSIOCGAXMAP

		for axis in buf[:num_axes]:
	  		axis_name = axis_names.get(axis, 'unknown(0x%02x)' % axis)
	   		self.axis_map.append(axis_name)
	   		self.axis_states[axis_name] = 0.0

		# Get the button map.
		buf = array.array('H', [0] * 200)
		ioctl(self.mJSDev, 0x80406a34, buf) # JSIOCGBTNMAP

		for btn in buf[:num_buttons]:
	   		btn_name = button_names.get(btn, 'unknown(0x%03x)' % btn)
	   		self.button_map.append(btn_name)
	   		self.button_states[btn_name] = 0

		print '%d axes found: %s' % (num_axes, ', '.join(self.axis_map))
		#print '%d buttons found: %s' % (num_buttons, ', '.join(self.button_map))
		self.isReady=True
		
	def poll(self):
		try:
			evbuf = self.mJSDev.read(8)
			#print "js", len(evbuf)
			if evbuf:
				time, value, type, number = struct.unpack('IhBB', evbuf)

				#if type & 0x80:
				#	 print "(Press PS Button)",

				if type & 0x01:
					button = self.button_map[number]
					if button:
						self.button_states[button] = value
						if value:
							if self.mOnButton!=None:
								self.mOnButton(button,True)
						else:
							if self.mOnButton!=None:
								self.mOnButton(button,False)

				if type & 0x02:
					axis = self.axis_map[number]
					if axis:
						fvalue = value / 32767.0
						self.axis_states[axis] = fvalue
						print "%s: %.3f" % (axis, fvalue)
				if type>2:
					print "jstype:",type
		except IOError, e:
			#lost joystick
			self.isReady=False
			try:
				self.openJS()
			except:
				pass

	def getAxis(self,name):
		try:
			return self.axis_states[name]
		except:
			return 0

	def isReady(self):
		return self.isReady



# These constants were borrowed from linux/input.h
axis_names = {
    0x00 : 'x1',
    0x01 : 'y1',
    0x02 : 'x2',
    0x05 : 'y2',
    
    0x3d : 'rx',
    0x3c : 'ry',
    
    #0x3d : 'rx',
    #0x3c : 'ry',
}

button_names = {
	0x12e : 'cross',
	0x12d : 'circle',
	0x12c : 'triangle',
	0x12f : 'square',
	
    0x120 : 'select',
    0x2c0 : 'PS',
    0x123 : 'start',
    0x121 : 'thumbL',
    0x122 : 'thumbR',
    
    0x127 : 'dpad_left',
    0x125 : 'dpad_right',
    0x126 : 'dpad_down',
    0x124 : 'dpad_up',
    
    0x128 : 'L2',
    0x129 : 'R2',
    0x12a : 'L1',
    0x12b : 'R1',
    
    
}


