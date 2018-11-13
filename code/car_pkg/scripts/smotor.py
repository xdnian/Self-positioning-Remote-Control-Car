#!/usr/bin/env python
import sys
import time
import RPi.GPIO as GPIO

def servo(steps,direction):
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)	
    direction=-direction
    IOpin = [25,8,7,12]

    for pin in IOpin:
        GPIO.setup(pin,GPIO.OUT)
        GPIO.output(pin, False)

    seq = [[1,0,0,1],              # 4 phase half stepping 
        [1,0,0,0],
        [1,1,0,0],
        [0,1,0,0],
        [0,1,1,0],
        [0,0,1,0],
        [0,0,1,1],
        [0,0,0,1]]
        
    # steps = int(sys.argv[1])        # 128 with directon +-1 for 90 degree (total 1024)
    # direction = int(sys.argv[2])    # +1 +2 clock, minus anti
    # sleeptime = float(sys.argv[3])  # in seconds, usually 0.005-0.01
    sleeptime=0.005
    counter = 0

    for x in xrange(steps):
        for s in seq[::direction]:
            for pin in xrange(4):
                GPIO.output(IOpin[pin], False if s[pin]==0 else True)
            time.sleep(sleeptime)
        
    for pin in xrange(4):
    	GPIO.output(IOpin[pin], False) 

    
if __name__ == "__main__":
    servo(128,1)
