#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from multiprocessing import Process, Value, Array
import RPi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
#distance threshold(cm)
THRESHOLD_1 = 40
THRESHOLD_2 = 15
last_sonar = "0 0 0 0"
interface = [
    {
        #front
        "TRIG":10,
        "ECHO":9
    },
    {
        #right
        "TRIG":4,
        "ECHO":17
    },
    {
        #back
        "TRIG":19,
        "ECHO":26
    },
    {
        #left
        "TRIG":27,
        "ECHO":22,
    },
]
for i in range(0,len(interface)):
    GPIO.setup(interface[i]["TRIG"],GPIO.OUT)
    GPIO.setup(interface[i]["ECHO"],GPIO.IN)
    GPIO.output(interface[i]["TRIG"], False)

#result of sonar

def checkDistance(TRIG,ECHO,detected,index):
    print "Sensor #",index,"started"
    while True:
        def sample():
            try:
                GPIO.output(TRIG, True)
                time.sleep(0.00001)
                GPIO.output(TRIG, False)
                watchdog = time.time()
                while GPIO.input(ECHO)==0:
                    pulse_start = time.time()
                    if pulse_start-watchdog>1:
                        rospy.logwarn("Ending loop 1 for sensor #"+str(index))
                        return 0
                while GPIO.input(ECHO)==1:
                    pulse_end = time.time()
                    if pulse_end-watchdog>1:
                        rospy.logwarn("Ending loop 2 for sensor #"+str(index))
                        return 0
                pulse_duration = pulse_end - pulse_start
                distance = pulse_duration*17150
                distance = round(distance, 2)
                # print "Sensor #",index," Distance:",distance,"cm""
                if distance>THRESHOLD_1:
                    return 0
                elif distance>THRESHOLD_2:
                    return 1
                else:
                    return 2
                    #print "Close detected at sonar #",index
            except Exception as e:
                rospy.logwarn("Sensor #"+str(index)+" "+str(e))
                return 0
        detected[index]=sample()
        time.sleep(0.4)

def talker():
    global last_sonar
    print "Starting sonar system"
    pub = rospy.Publisher('sonar', String, queue_size=10)
    rospy.init_node('sonar', anonymous=True)
    rate = rospy.Rate(5) # 5hz

    #initialize cross process variable
    raw_detected=[]
    for i in range(0,len(interface)):
        raw_detected.append(0)
    detected = Array("i",raw_detected)

    #create detect threads and start
    p = [None]*len(interface)
    for i in range(0,len(interface)):
        p[i] = Process(target=checkDistance,args=(interface[i]["TRIG"],interface[i]["ECHO"],detected,i))
        p[i].daemon = True
        p[i].start()

    #main loop
    while not rospy.is_shutdown():
        output = ""
        for i in range(0,len(detected[:])):
            output=output+str(detected[i])+" "
        output=output[:(len(output)-1)]
        if(output!=last_sonar):
            rospy.loginfo("outputing sonar")
            rospy.loginfo(output)
            pub.publish(output)
            last_sonar=output
        rate.sleep()
        
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
