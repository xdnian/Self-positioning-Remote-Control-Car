#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from apriltags_ros.msg import AprilTagDetectionArray
import tf
from math import pi, cos, sin

CAR_TAG_ID = 0
COR_TAG_ID = 10
SIDE_LEN = 1.2

class location():
    def __init__(self):
        self.image_sub = rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.callback)
        self.coord_pub = rospy.Publisher("coord", String, queue_size=10)
        self.origin_x = 0
        self.origin_y = 0
        self.origin_z = 0
        self.t = 0

    def callback(self, data):
        if not rospy.is_shutdown():
            dects = data.detections
            for i in range(len(dects)):
                if dects[i].id == COR_TAG_ID:
                    self.origin_x = dects[i].pose.pose.position.x
                    self.origin_y = dects[i].pose.pose.position.y
                    self.origin_z = dects[i].pose.pose.position.z
                    quaternion = (
                        dects[i].pose.pose.orientation.x,
                        dects[i].pose.pose.orientation.y,
                        dects[i].pose.pose.orientation.z,
                        dects[i].pose.pose.orientation.w
                    )
                    roll = tf.transformations.euler_from_quaternion(quaternion)[0]
                    self.t = pi - roll
            for i in range(len(dects)):
                if dects[i].id == CAR_TAG_ID:
                    quaternion = (
                        dects[i].pose.pose.orientation.x,
                        dects[i].pose.pose.orientation.y,
                        dects[i].pose.pose.orientation.z,
                        dects[i].pose.pose.orientation.w
                    )
                    yaw = tf.transformations.euler_from_quaternion(quaternion)[2]
                    x = dects[i].pose.pose.position.x - self.origin_x
                    y = dects[i].pose.pose.position.y - self.origin_y
                    z = dects[i].pose.pose.position.z - self.origin_z
                    coord = (x, y*cos(self.t)-z*sin(self.t), y*sin(self.t)+z*cos(self.t))
                    # print coord, round(self.t,2), (round(x,2), round(y,2), round(z,2)),
                    angle = int(round((yaw/pi * 180))) if yaw >= 0 else int(round(((yaw/pi+2) * 180)))
                    grid_x = int(round(coord[0]/SIDE_LEN*7)) if 0<=coord[0]<=SIDE_LEN else (0 if coord[0]<0 else 7)
                    grid_y = int(round(coord[1]/SIDE_LEN*7)) if 0<=coord[1]<=SIDE_LEN else (0 if coord[1]<0 else 7)
                    # print(coord, grid_x, grid_x)
                    self.coord_pub.publish(str(grid_x) + ' ' + str(grid_y) + ' ' + str(angle))

if __name__ == '__main__':
    loc = location()
    rospy.init_node('coordinator', anonymous=True)
    print("Start")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
