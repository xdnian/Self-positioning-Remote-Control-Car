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
idle = True
last_event = ""

#utility function

w=[150,150,150]
g=[0,255,0]
r=[255,0,0]
b=[0,0,255]
e=[0,0,0]

def make_blank():
    return [e,e,e,e,e,e,e,e,
            e,e,e,e,e,e,e,e,
            e,e,e,e,e,e,e,e,
            e,e,e,e,e,e,e,e,
            e,e,e,e,e,e,e,e,
            e,e,e,e,e,e,e,e,
            e,e,e,e,e,e,e,e,
            e,e,e,e,e,e,e,e]

def make_point(x,y):
    map = make_blank()
    map[x+8*y] = g
    return map

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + 'Data received %s', data.data)
    coord = data.data.split(" ")
    x = int(coord[0])
    y = int(coord[1])
    rospy.loginfo('Coordinate: %d %d',x,y)
    sense.set_pixels(make_point(x,y))

def main():
    global idle
    pub = rospy.Publisher('remote', String, queue_size=10)
    rospy.init_node('remote', anonymous=True)
    rospy.Subscriber('coord', String, callback)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        # Get Input
        events = sense.stick.get_events()
        if len(events) > 0:
            event = events[len(events)-1]
            if idle and event.action=="pressed":
                idle = False
                pub.publish(event.direction)
                rospy.loginfo(event.direction)
            if (not idle) and event.action=="released":
                idle = True
                pub.publish("stop")
                rospy.loginfo("released")

        #hello_str = "hello world %s" % rospy.get_time()
        #rospy.loginfo(hello_str)
        #pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
