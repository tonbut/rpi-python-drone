#!/usr/bin/env python

"""test-send.py: Test script to send RC commands to a MultiWii Board."""

__author__ = "Aldo Vargas"
__copyright__ = "Copyright 2016 Altax.net"

__license__ = "GPL"
__version__ = "1"
__maintainer__ = "Aldo Vargas"
__email__ = "alduxvm@gmail.com"
__status__ = "Development"

from pyMultiwii import MultiWii
import time

if __name__ == "__main__":

    #board = MultiWii("/dev/tty.usbserial-AM016WP4")
    #board = MultiWii("/dev/tty.SLAB_USBtoUART")
    board = MultiWii("/dev/ttyUSB0")
    try:
        board.arm()
        #print "attitude"
        #board.getData(MultiWii.ATTITUDE)
        #board.getData(MultiWii.ATTITUDE)
        #print "rc"
        #d=board.getData(MultiWii.RC);
        #d=board.getData(MultiWii.RC);
        #d=board.getData(MultiWii.RC);
        #print board.rcChannels['throttle'],board.rcChannels['yaw']
        #print "motor"
        board.getData(MultiWii.MOTOR)
        
        print "Board is armed now!"
        print "In 3 seconds it will disarm..."
        
        timer = 0
        start = time.time()
        throttle=1400
        while timer < 1:
            data = [1500,1500,throttle,1500,1000,1000,1000,1000]
            throttle=throttle-50;
            board.sendCMD(16,MultiWii.SET_RAW_RC,data)
            time.sleep(0.05)
            timer = timer + (time.time() - start)
            start =  time.time()
            board.getData(MultiWii.MOTOR)
        
        time.sleep(0.1)
        board.disarm()
        print "Disarmed."
        time.sleep(3)

    except Exception,error:
        print "Error on Main: "+str(error)
