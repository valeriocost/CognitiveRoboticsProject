#!/usr/bin/python3
import pyrealsense2 as rs
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import numpy as np
import rospy


if __name__=='__main__':
    rospy.init_node("image_webcam")

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    profile = pipeline.start(config)
    


    image_publisher = rospy.Publisher('in_rgb', Image, queue_size=1)
    bridge = CvBridge()
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        frames = pipeline.wait_for_frames()
        frame = np.asanyarray(frames.get_color_frame().get_data())
        frame = cv2.resize(frame,(640,480))
        if frame is not None:
            msg = bridge.cv2_to_imgmsg(frame)
            msg.header.stamp = rospy.Time.now()
            image_publisher.publish(msg)
        rate.sleep()

