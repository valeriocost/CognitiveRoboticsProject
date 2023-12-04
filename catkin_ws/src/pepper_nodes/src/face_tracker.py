#!/usr/bin/python3
from utils import Session
from optparse import OptionParser
import rospy
from pepper_nodes.srv import WakeUp, Rest
import qi
import argparse
import sys

'''
This class implements a ROS node used to controll the Pepper posture
'''
class TrackerNode:
    
    '''
    The costructor creates a session to Pepper and inizializes the services
    '''
    def __init__(self, ip, port, faceSize):
        self.ip = ip
        self.port = port
        self.session = Session(ip, port)
        self.faceSize = faceSize
        self.motion_proxy = self.session.get_service("ALMotion")
        self.posture_proxy = self.session.get_service("ALRobotPosture")
        self.tracker_service = self.session.get_service("ALTracker")
        self.animation_player_service = self.session.get_service("ALAnimationPlayer")
    
    '''
    This method calls the ALMotion service and sets the robot to rest position
    '''
    def stop(self, *args):
        try:
            self.motion_proxy.rest()
            self.tracker_service.stopTracker()
        except:
            self.tracker_service = self.session.get_service("ALTracker") 
            self.tracker_service.stopTracker()
        return "ACK"
    
    '''
    This method calls the ALMotion and ALRobotPosture services and it sets motors on and then it sets the robot posture to initial position
    '''
    def trackernode(self, *args):
        try:
            # Add target to track.
            targetName = "Face"
            faceWidth = self.faceSize
            self.tracker_service.registerTarget(targetName, faceWidth)

            # Then, start tracker.
            self.tracker_service.track(targetName)


        except:
            self.motion_proxy = self.session.get_service("ALMotion")
            self.posture_proxy = self.session.get_service("ALRobotPosture")
            self.tracker_service = self.session.get_service("ALTracker") 
            self.animation_player_service = self.session.get_service("ALAnimationPlayer")

        return "ACK"   
    
    def start(self):
        rospy.init_node("tracker_node")
        self.trackernode()
        rospy.Service("tracker", WakeUp, self.trackernode)
        rospy.spin()

if __name__ == "__main__":
    parser = OptionParser()
    parser.add_option("--ip", dest="ip", default="10.0.1.207")
    parser.add_option("--port", dest="port", default=9559)
    (options, args) = parser.parse_args()

    try:
        node = TrackerNode(options.ip, int(options.port), 0.1)
        node.start()
    except rospy.ROSInterruptException:
        node.stop()
