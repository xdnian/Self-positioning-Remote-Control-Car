import RPi.GPIO as GPIO
import time

LEFT_1=24
LEFT_2=23
RIGHT_1=20
RIGHT_2=21

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

GPIO.setup(LEFT_1, GPIO.OUT) 
GPIO.setup(LEFT_2, GPIO.OUT) 
GPIO.setup(RIGHT_1, GPIO.OUT) 
GPIO.setup(RIGHT_2, GPIO.OUT)

l1 = GPIO.PWM(LEFT_1, 1000)
l2 = GPIO.PWM(LEFT_2, 1000)   
r1 = GPIO.PWM(RIGHT_1, 1000)
r2 = GPIO.PWM(RIGHT_2, 1000)

LEFT=1
RIGHT=0.93

def fire(direction):
    para=[0,0,0,0]

    if direction=='FORWARD':
        para=[100,0,100,0]
      
    elif direction=='BACKWARD':
        para=[0,100,0,100]

    elif direction=='RIGHT':
        para=[100,0,0,100]

    elif direction=='LEFT':
        para=[0,100,100,0]

    else:
        para=[0,0,0,0]

    l1.start(para[0])
    l2.start(para[1])
    r1.start(para[2])
    r2.start(para[3])
    time.sleep(0.05)


def act(command, dutycycle=50):
    para=[0,0,0,0,0]

    fire(command)

    if command == 'FORWARD':
        para=[dutycycle,0,dutycycle,0]

    elif command == 'BACKWARD':
        para=[0,dutycycle,0,dutycycle*1.1]

    elif command == 'LEFT':
        para=[0,dutycycle,dutycycle,0]

    elif command == 'RIGHT':
        para=[dutycycle,0,0,dutycycle]

    else:
        para=[0,0,0,0]
    
    l1.start(para[0]*LEFT)
    l2.start(para[1]*LEFT)
    r1.start(para[2]*RIGHT)
    r2.start(para[3]*RIGHT)
    return

