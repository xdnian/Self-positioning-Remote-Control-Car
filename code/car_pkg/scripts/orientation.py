import motor
import sys
import time
import threading

global target  
global current

def rotate():
    global target
    global current
    if ((target+360-current)%360)<=180:
        print ('right')
        while True:
            motor.act(command='RIGHT', dutycycle=50)
        
    elif ((target+360-current)%360)>180:
        print ('left')
        while True:
            motor.act(command='RIGHT', dutycycle=50)
            
current = int(sys.argv[1])        
target = int(sys.argv[2]) 

def change():
    global target
    global current
    while True:
        if ((target+360-current)%360)<=180:
            current=(current+1)%360
        elif ((target+360-current)%360)>180:
            current=(current-1)%360
        time.sleep(0.02)

t_detect=threading.Thread(target=change)
t_detect.start()

rotate()