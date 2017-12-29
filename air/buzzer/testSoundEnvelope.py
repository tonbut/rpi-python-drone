from soundEnvelope import SoundEnvelope, Envelope
import time


def test1():
    print "test1()"
    for i in range(0,2):
        e3=Envelope(2,1,-1,0,4,4,0,20,-1,0,-1,126,110)
        se.play([[24,200,e3]])
        time.sleep(1)

        e3=Envelope(4,48,-48,0,1,1,0,127,0,0,-127,127,8)
        se.play([[0,25,e3],[2,25],[4,25],[5,25],[7,25],[9,25],[11,25],[12,50]],pitchOffset=24,speed=2,legato=1.0)

        time.sleep(2)
        
def test2():
    print "test2()"
    #random
    e3=Envelope(4,121,17,171,3,1,4,127,0,0,-127,127,8)
    se.play([[0,200,e3]],legato=1)
    time.sleep(2.5)

    #updown1
    e3=Envelope(4,48,-48,0,5,5,100,127,0,0,-127,127,8)
    se.play([[0,10*4,e3]],legato=1)
    time.sleep(2)
    
    #updown2
    e3=Envelope(2,24,-24,0,10,10,100,127,0,0,-127,127,8)
    se.play([[0,20*2,e3]],legato=1)
    time.sleep(2)
    
    #down
    e3=Envelope(8,-48,0,0,5,16,16,127,0,0,-127,127,8)
    se.play([[12*5,8*6,e3]])
    
def test3():
    print "test3()"
    #no
    e3=Envelope(16,0,0,0,1,1,0,127,0,0,-127,127,8)
    se.play([[12,32,e3],[0,64]],legato=0.8,speed=1.5,pitchOffset=0)
    time.sleep(2)
    #yes
    se.play([[0,16,e3],[4,16],[7,16],[12,48]],legato=0.9,speed=1.5,pitchOffset=24)
    time.sleep(2)
    
    #beep-beep
    se.play([[0,16,e3],[None,16],[0,16]],legato=0.9,speed=1.5,pitchOffset=24)
    time.sleep(2)
    se.play([[0,16,e3],[None,16],[0,16],[None,16],[0,16]],legato=0.9,speed=1.5,pitchOffset=24)
    time.sleep(2)
    
def test4():
    print "test4()"
    #slow siren
    e3=Envelope(1,1,-1,0,48,48,0,127,0,0,-127,127,8)
    se.play([[36,256,e3]])
    time.sleep(2)
    
    #fast siren
    e3=Envelope(1,2,-2,0,12,12,0,127,0,0,-127,127,8)
    se.play([[48,128,e3]])
    
def test5():
    print "test5()"
    #bell
    e3=Envelope(1,-48,48,0,2,2,255,127,-3,0,-127,127,0)
    e4=Envelope(4,1,-1,0,2,2,0,127,-3,0,-127,127,0)
    se.play([[60,128,e3],[54,128],[None,64],[60,128,e4]])
    time.sleep(2)
    
def test6():
    print "test6()"
    #click
    e3=Envelope(1,0,0,0,12,12,0,127,-127,0,-127,127,8)
    se.play([[48,4,e3]])
    
def test7():
    print "test7()"
    brass=Envelope(8,0,0,0,2,2,4,4,-1,0,-16,127,126)
    fill=Envelope(4,48,-48,0,1,1,0,127,0,0,-127,127,8)
    
    se.play([
    	[24,32,brass],[22,8],[None,8],[21,8],[None,8],
    	[24,32],[22,8],[None,8],[21,8],[None,8],
    	[24,8],[None,8],[19,16],[7,8,fill],[None,8],[12,8],[None,8],
    	[16,24],[None,8],[12,24],[None,8],
    	[26,16,brass],[24,8],[None,8],[26,8],[None,8],[28,32],
    	[26,8],[None,8],[24,8],[None,8],[22,8],[None,8],
    	[17,24,fill],[None,8],[16,24],[None,8],
    	[14,24],[None,8],[12,24],[None,8],
    	
    	[24,32,brass],[22,8],[None,8],[21,8],[None,8],
    	[24,32],[22,8],[None,8],[21,8],[None,8],
    	[24,8],[None,8],[19,16],[7,8,fill],[None,8],[12,8],[None,8],
    	[16,24],[None,8],[12,24],[None,8],
    	[26,16,brass],[24,8],[None,8],[26,8],[None,8],[28,24],
    	
    	[None,8],[26,32],[28,8],[None,8],
    	[29,8],[None,24],
    	[24,16,fill],[26,8],[28,8],[29,16]
    	
    	],legato=0.8,speed=0.75,pitchOffset=12)    
    
se=SoundEnvelope()
test1()
test2()
test3()
test4()
test5()
test6()
test7()