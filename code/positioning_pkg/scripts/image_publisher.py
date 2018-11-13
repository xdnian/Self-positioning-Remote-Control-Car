#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
# from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import cv2

class image_publisher():
    def __init__(self):
        self.cap = cv2.VideoCapture(0)
        self.image_pub = rospy.Publisher("image_topic",Image, queue_size=10)
        self.bridge = CvBridge()

    def publish(self):
        while self.cap.isOpened() and not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if ret==True:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                try:
                    self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "passthrough"))
                except CvBridgeError as e:
                    print(e)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            else:
                break

if __name__ == '__main__':
    ic = image_publisher()
    rospy.init_node('image_publisher', anonymous=True)
    try:
        ic.publish()
    except KeyboardInterrupt:
        print("Shutting down")
    ic.cap.release()
