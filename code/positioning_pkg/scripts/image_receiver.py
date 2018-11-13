#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import cv2

class image_receiver():
    def __init__(self):
        self.image_sub = rospy.Subscriber("/tag_detections_image",Image,self.callback)
##        self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback)
        self.bridge = CvBridge()

    def callback(self, data):
        if not rospy.is_shutdown():
            try:
                cv_image = self.bridge.imgmsg_to_cv2(data, "passthrough")
                cv2.imshow("Image window", cv_image)
                cv2.waitKey(1)
            except CvBridgeError as e:
                print(e)

if __name__ == '__main__':
    ic = image_receiver()
    rospy.init_node('image_receiver', anonymous=True)
    print("Start")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
