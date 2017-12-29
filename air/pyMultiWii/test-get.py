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
        
        board.getData(MultiWii.PID)

    except Exception,error:
        print "Error on Main: "+str(error)
