import spidev
import time


spi = spidev.SpiDev()
spi.open(0,0)
spi.cshigh=False
spi.loop=False
spi.bits_per_word=8
spi.max_speed_hz=800000
for i in range(0,10):

	spi.xfer2( [0x40] );
	time.sleep(0.001)
	spi.xfer2( [0x55] );
	time.sleep(0.001)

	spi.xfer2( [0x11] );
	time.sleep(0.001)
	c=spi.readbytes(1)
	print c
	time.sleep(0.5)
spi.close()