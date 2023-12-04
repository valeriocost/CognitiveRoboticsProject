#!/usr/bin/python3

import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import numpy as np
import rospy


if __name__=='__main__':
    rospy.init_node("image_webcam")
    cap = cv2.VideoCapture(14)

    # Check if the webcam is opened correctly
    if not cap.isOpened():
        raise IOError("Cannot open webcam")

    image_publisher = rospy.Publisher('in_rgb', Image, queue_size=1)
    bridge = CvBridge()
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        frame = cv2.resize(frame,(640,480))
        if frame is not None:
            msg = bridge.cv2_to_imgmsg(frame)
            msg.header.stamp = rospy.Time.now()
            image_publisher.publish(msg)
        rate.sleep()

    cap.release()