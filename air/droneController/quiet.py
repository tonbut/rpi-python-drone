from pyMultiwii import MultiWii

multiwii="/dev/ttyUSB0"
board = MultiWii(multiwii)
board.arm()
board.disarm()