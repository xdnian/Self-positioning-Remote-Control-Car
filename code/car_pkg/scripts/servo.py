#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import smotor
from multiprocessing import Process, Value
import time
#-1 left 0 middle 1 right
direction = Value("i",0)
left_boundary = -90.0
right_boundary = 90.0
step = 5

def controller(direction):
    angle = 0.0
    rospy.loginfo("Controller process started")
    while True:
        if direction.value==1 and angle<right_boundary:
            rospy.loginfo("Turning right, current angle "+str(angle))
            smotor.servo(step*128/90,1)
            angle=angle+step
        elif direction.value==-1 and angle>left_boundary:
            rospy.loginfo("Turning left, current angle "+str(angle))
            smotor.servo(step*128/90,-1)
            angle=angle-step
        elif direction.value==3 and angle!=0:
            if angle<0:
                rospy.loginfo("Resetting turning right, current angle "+str(angle))
                smotor.servo(step*128/90,1)
                angle=angle+step
            elif angle>0:
                rospy.loginfo("Resetting turning left, current angle "+str(angle))
                smotor.servo(step*128/90,-1)
                angle=angle-step
        else:
            time.sleep(0.5)

def callback(data):
    global direction
    rospy.loginfo("Servo command received "+data.data)
    if data.data=="left":
        direction.value=-1
    elif data.data=="right":
        direction.value=1
    elif data.data=="reset":
        direction.value=3
    else:
        direction.value=0

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('camera_servo', anonymous=True)
    rospy.Subscriber('cam_rotate', String, callback)
    rospy.loginfo("Camera_servo module running")
    p = Process(target=controller, args=(direction,) )
    p.start()
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
    # listener()
