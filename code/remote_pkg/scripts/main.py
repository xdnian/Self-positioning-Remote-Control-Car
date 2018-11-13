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

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
from std_msgs.msg import String
from sense_hat import SenseHat

sense = SenseHat()
sense.clear()
#main state
state = "normal" #normal,direct,map
#sub state
direction = "stop"
#next state
next_state = state
#timer
timer = 0
#timer flag
is_timer = False
#wait for timer
wait = False
#current coordinate
coord = [0,0]
#selected coordinate
selected = [0,0]
#before_map
before_map = "normal"
#warning sonar
warning = False
#utility function


w=[150,150,150]
g=[0,255,0]
r=[255,0,0]
b=[0,0,255]
e=[0,0,0]
y=[255,255,0]
coord_color = g
warning_color = r
map_color = y

def make_blank():
    return [e,e,e,e,e,e,e,e,
            e,e,e,e,e,e,e,e,
            e,e,e,e,e,e,e,e,
            e,e,e,e,e,e,e,e,
            e,e,e,e,e,e,e,e,
            e,e,e,e,e,e,e,e,
            e,e,e,e,e,e,e,e,
            e,e,e,e,e,e,e,e]

def add_point(x,y,c,pixels):
    pixels[int(x)+8*int(y)] = c
    return pixels


#callback function for coordinator
def coord_handler(data):
    global coord
    rospy.loginfo(rospy.get_caller_id() + 'Coordinate received %s', data.data)
    coord = data.data.split(" ")
    x = int(coord[0])
    y = int(coord[1])
    rospy.loginfo('Coordinate: %d %d',x,y)
    coord = [x,y]

def sonar_handler(data):
    global warning
    rospy.loginfo(rospy.get_caller_id() + 'Sonar received %s', data.data)
    detected = data.data.split(" ")
    setWarning = False
    for i in range(0,len(detected)):
        if int(detected[i])>0:
            setWarning = True
    warning = setWarning

def run_timer():
    global timer,is_timer
    #time expired
    if(timer<0):
        is_timer = False
    #timer not expired, let it -1
    if(is_timer):
        timer=timer-1

def use_timer(interval):
    global timer,is_timer
    is_timer = True
    timer = interval

def render_UI():
    pixels = make_blank()
    if warning:
        pixels = add_point(coord[0],coord[1],warning_color,pixels)
    else:
        pixels = add_point(coord[0],coord[1],coord_color,pixels)
    if(state=="map"):
        pixels = add_point(selected[0],selected[1],y,pixels)
    sense.set_pixels(pixels)

def move_selected(axis,increased):
    global selected
    if axis=="x":
        index = 0
    else:
        index = 1
    if increased:
        selected[index] = selected[index]+1
    else:
        selected[index] = selected[index]-1
    if selected[index]>7:
        selected[index]=0
    if selected[index]<0:
        selected[index]=7

def main():
    global state,next_state,direction,selected,is_timer,wait,coord_color
    pub = rospy.Publisher('remote', String, queue_size=10)
    rospy.init_node('remote', anonymous=True)
    rospy.Subscriber('coord', String, coord_handler)
    rospy.Subscriber('sonar', String, sonar_handler)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        # run timer
        run_timer()
        # render UI
        render_UI()
        state = next_state
        # Get Input
        events = sense.stick.get_events()
        # there is input
        if len(events) > 0:
            # event is the last event logged
            event = events[len(events)-1]
            if state=="normal":
                if direction=="stop":
                    if event.action=="pressed" and event.direction!="middle":
                        direction = event.direction
                        if direction=="up":
                            pub.publish("1")
                        if direction=="right":
                            pub.publish("2")
                        if direction=="down":
                            pub.publish("3")
                        if direction=="left":
                            pub.publish("4")
                        rospy.loginfo("Normal go "+direction)
                    if event.action=="pressed" and event.direction=="middle":
                        wait = True
                        use_timer(10)
                    if wait and event.action=="held" and event.direction=="middle":
                        if not is_timer:
                            wait = False
                            coord_color = b
                            next_state = "direct"
                            rospy.loginfo("Switch to direct mode")
                    if wait and event.action=="released" and event.direction=="middle":
                        next_state = "map"
                        before_map = state
                        wait = False
                        rospy.loginfo("Switch to map mode")
                if direction!="stop":
                    if event.action=="released":
                        direction="stop"
                        pub.publish("0")
                        rospy.loginfo("stop")

            if state=="direct":
                if direction=="stop":
                    if event.action=="pressed" and event.direction!="middle":
                        direction = event.direction
                        if direction=="up":
                            pub.publish("5")
                        if direction=="right":
                            pub.publish("6")
                        if direction=="down":
                            pub.publish("7")
                        if direction=="left":
                            pub.publish("8")
                        rospy.loginfo("Direct go "+direction)
                    if event.action=="pressed" and event.direction=="middle":
                        wait = True
                        use_timer(10)
                    if wait and event.action=="held" and event.direction=="middle":
                        if not is_timer:
                            wait = False
                            coord_color = g
                            next_state = "normal"
                            rospy.loginfo("Switch to normal mode")
                    if wait and event.action=="released" and event.direction=="middle":
                        next_state = "map"
                        before_map = state
                        wait = False
                        rospy.loginfo("Switch to map mode")
                if direction!="stop":
                    if event.action=="released":
                        direction="stop"
                        pub.publish("0")
                        rospy.loginfo("stop")

            if state=="map":
                if direction=="stop":
                    if event.action=="pressed" and event.direction!="middle":
                        if(direction=="left"):
                            move_selected("x",False)
                        if(direction=="right"):
                            move_selected("x",True)
                        if(direction=="down"):
                            move_selected("y",True)
                        if(direction=="up"):
                            move_selected("y",False)
                        direction = event.direction
                    if event.action=="pressed" and event.direction=="middle":
                        wait = True
                        use_timer(10)
                    if wait and event.action=="held" and event.direction=="middle":
                        if not is_timer:
                            wait = False
                            next_state = before_map
                            rospy.loginfo("Switch to previous mode")
                    if wait and event.action=="released" and event.direction=="middle":
                        direction = "stop"
                        pub.publish("9"+","+str(selected[0])+","+str(selected[1]))
                        rospy.loginfo("Go to "+str(selected))
                        wait = False
                if direction!="stop":
                    if event.direction==direction:
                        if not is_timer:
                            if(direction=="left"):
                                move_selected("x",False)
                            if(direction=="right"):
                                move_selected("x",True)
                            if(direction=="down"):
                                move_selected("y",True)
                            if(direction=="up"):
                                move_selected("y",False)
                            use_timer(2)
                    if event.action=="released" or event.direction!=direction:
                        direction="stop"
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
