#!/usr/bin/env python

import cv2
import rospy
from cv_bridge import CvBridge
import sensor_msgs

def onGrayscaleMessage(grayscale):
    rospy.loginfo("Gray image arrived")

if __name__ == '__main__':
   rospy.init_node('image_subscriber_node_py', anonymous=True)
   rospy.Subscriber("/camera/grayscale/image_raw", sensor_msgs.msg.Image, onGrayscaleMessage, queue_size=1)
   rospy.spin()