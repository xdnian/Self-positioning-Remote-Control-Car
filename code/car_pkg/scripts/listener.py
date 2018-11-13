#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic

import rospy
from std_msgs.msg import String
import motor
import numpy as np
import time
import math
#12 16 20 21
dutycycle = 90
#coordinate
coordinate=[0,0]
#angle
angle=0
#target
target=[0,0]
#is_map
is_map=False
#
state="align"
next_state="align"
#
direction=None
STOP_FLAG=True
def stop(command):
    global is_map,STOP_FLAG
    initialize_sonar_handlers()
    is_map=False
    STOP_FLAG=True
    rospy.loginfo("Stoping motor")
    motor.act("STOP",dutycycle)

def force_stop():
    global is_map,STOP_FLAG
    initialize_sonar_handlers()
    is_map=False
    STOP_FLAG=True
    rospy.loginfo("Stoping motor")
    motor.act("STOP",dutycycle)

def normal_forward(command):
    global sonar_difference_handlers
    global STOP_FLAG
    STOP_FLAG=False
    #stop when front sensor return 1
    sonar_difference_handlers[0][2]=force_stop
    rospy.loginfo("Normal forward")
    motor.act("FORWARD",dutycycle)

def normal_left(command):
    global STOP_FLAG
    STOP_FLAG=False
    rospy.loginfo("Normal turn left")
    motor.act("LEFT",dutycycle)

def normal_right(command):
    global STOP_FLAG
    STOP_FLAG=False
    rospy.loginfo("Normal turn right")
    motor.act("RIGHT",dutycycle)

def normal_backward(command):
    global STOP_FLAG
    STOP_FLAG=False
    global sonar_difference_handlers
    #stop when back sensor return 1
    sonar_difference_handlers[2][2]=force_stop
    rospy.loginfo("Normal backward")
    motor.act("BACKWARD",dutycycle)

def direct_forward(command):
    global STOP_FLAG
    STOP_FLAG=False
    global sonar_difference_handlers
    #stop when front sensor return 1
    sonar_difference_handlers[0][2]=stop
    rospy.loginfo("Direct forward")
    turnto(0)
    if not STOP_FLAG:
        motor.act("FORWARD",dutycycle)

def direct_left(command):
    global STOP_FLAG
    STOP_FLAG=False
    global sonar_difference_handlers
    #stop when front sensor return 1
    sonar_difference_handlers[0][2]=stop
    rospy.loginfo("Direct left")
    turnto(270)
    if not STOP_FLAG:
        motor.act("FORWARD",dutycycle)

def direct_right(command):
    global STOP_FLAG
    STOP_FLAG=False
    global sonar_difference_handlers
    #stop when front sensor return 1
    sonar_difference_handlers[0][2]=stop
    rospy.loginfo("Direct right")
    turnto(90)
    if not STOP_FLAG:
        motor.act("FORWARD",dutycycle)

def direct_backward(command):
    global STOP_FLAG
    STOP_FLAG=False
    global sonar_difference_handlers
    #stop when front sensor return 1
    sonar_difference_handlers[0][2]=stop
    rospy.loginfo("Direct backward")
    turnto(180)
    if not STOP_FLAG:
        motor.act("FORWARD",dutycycle)

def turnto(target_angle):
    direction = None
    while difference(target_angle,angle)>15 and not STOP_FLAG:
        error = difference(target_angle,angle)
        if is_right(target_angle,angle):
            motor.act(command='RIGHT',dutycycle=50)
            rospy.loginfo("Aligning, turning right, error: "+str(error))
            time.sleep(error*0.005)
        elif not is_right(target_angle,angle):
            motor.act(command='LEFT',dutycycle=50)
            rospy.loginfo("Aligning, turning left, error: "+str(error))
            time.sleep(error*0.005)
        motor.act(command="STOP",dutycycle=90)
        time.sleep(0.4)
        

def goto(command):
    global STOP_FLAG,is_map
    STOP_FLAG=False
    global target,state,next_state
    x=command[1]
    y=command[2]
    rospy.loginfo("Go to coordinate %s %s",x,y)
    target = [x,y]
    is_map = True
    state="align"
    next_state="align"
    map_goto()

#utility
def getangle(dx,dy):
    target_angle = np.arctan2(abs(dy), abs(dx))
    target_angle = math.ceil(target_angle *180/ 3.14)
    if dx>0 and dy>0:
        target_angle = target_angle
    elif dx>0 and dy<0:
        target_angle = 90 + target_angle
    elif dx<0 and dy<0:
        target_angle = 180 + target_angle
    elif dx==0 and dy>0:
        target_angle = 0
    elif dx==0 and dy<0:
        target_angle = 180
    elif dy==0 and dx>0:
        target_angle = 90
    elif dy==0 and dx<0:
        target_angle = 270
    else:
        return False
    return target_angle

def map_goto():
    global is_map,state
    direction = None
    target_angle = None
    while is_map:
        dx=int(target[0])-int(coordinate[0])
        dy=int(target[1])-int(coordinate[1])
        dy = -dy
        if dx==0 and dy==0:
            rospy.loginfo("Goto command done")
            stop("")
        if state=="align":
            target_angle=getangle(dx,dy)
            rospy.loginfo("dx: "+str(dx)+" dy: "+str(dy))
            rospy.loginfo("Turning to angle "+str(target_angle))
            turnto(target_angle)
            update_state_forward()
        if state=="forward":
            target_angle=getangle(dx,dy)
            rospy.loginfo("Going forward dx: "+str(dx)+" dy: "+str(dy))
            if dx==0 and dy==0:
                rospy.loginfo("Goto command done")
                stop("")
            elif dx==0 or dy==0:
                rospy.loginfo("Re-aligning due to coordinate half reach")
                update_state_align()
            elif difference(target_angle,angle)>10:
                rospy.loginfo("Re-aligning due to large error")
                update_state_align()
        state=next_state
        time.sleep(0.1)

def update_state_forward():
    global next_state,sonar_difference_handlers
    initialize_sonar_handlers()
    rospy.loginfo("Going forward")
    sonar_difference_handlers[0][2]=update_state_obstacle
    motor.act(command="FORWARD",dutycycle=60)
    next_state="forward"

def update_state_obstacle():
    global next_state
    initialize_sonar_handlers()
    rospy.loginfo("Encountered obstacle")
    sonar_difference_handlers[0][0]=update_state_evade
    sonar_difference_handlers[0][2]=update_state_obstacle
    motor.act(command='LEFT',dutycycle=50)
    next_state="evade"

def update_state_evade():
    global next_state,sonar_difference_handlers
    initialize_sonar_handlers()
    rospy.loginfo("Evading obstacle")
    sonar_difference_handlers[1][0]=update_state_align
    motor.act(command='FORWARD',dutycycle=50)
    next_state="evade"

def update_state_align():
    global next_state
    initialize_sonar_handlers()
    motor.act(command='STOP',dutycycle=50)
    next_state="align"

def is_right(target_angle,angle):
    if ((target_angle+360-angle)%360)-180<=0:
        return True
    else:
        return False

def difference(target_angle,angle):
    difference = 180-angle
    target_angle = target_angle+difference
    if target_angle<0:
        rospy.loginfo("Target<0,+360")
        target_angle=target_angle+360
    elif target_angle>360:
        rospy.loginfo("Target>360,-360")
        target_angle=target_angle-360
    return abs(target_angle - 180)

remote_commands = {
    "0":stop,
    "1":normal_forward,
    "2":normal_right,
    "3":normal_backward,
    "4":normal_left,
    "5":direct_forward,
    "6":direct_right,
    "7":direct_backward,
    "8":direct_left,
    "9":goto
}


def empty_handler():
    pass

sonar_difference_handlers = []
#initialize the handlers

def initialize_sonar_handlers():
    global sonar_difference_handlers
    sonar_difference_handlers=[]
    for i in range(4):
        new = [empty_handler,empty_handler,empty_handler]
        sonar_difference_handlers.append(new)

#obstacle difference record
last_detected = [0,0,0,0]

def remote_handler(data):
    rospy.loginfo(rospy.get_caller_id() + ' Remote command received %s', data.data)
    command = str(data.data).split(",")
    remote_commands[command[0]](command)

def sonar_handler(data):
    global last_detected
    #rospy.loginfo(rospy.get_caller_id() + ' Sonar info received %s', data.data)
    #detected = str(data.data).split()
    detected = list(map((lambda x: int(x)), str(data.data).split()))
    for i in range(0,len(detected)):
        if last_detected[i]!=detected[i]:
            print "Calling handler #",detected[i]," for sensor #",i
            sonar_difference_handlers[i][detected[i]]()
    last_detected=detected

def coord_handler(data):
    global coordinate,angle
    raw = str(data.data).split()
    coordinate=[int(raw[0]),int(raw[1])]
    angle=int(raw[2])
    if is_map and coordinate==target:
        stop("")
    # rospy.loginfo("Angle: "+str(angle))

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('car', anonymous=True)
    rospy.Subscriber('sonar', String, sonar_handler)
    rospy.Subscriber('remote', String, remote_handler)
    rospy.Subscriber('coord', String, coord_handler)
    rospy.loginfo("Car module running")
    initialize_sonar_handlers()
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
