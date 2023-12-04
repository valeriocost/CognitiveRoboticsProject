#!/usr/bin/python3
import rospy
from std_msgs.msg import String, Bool
from pepper_nodes.srv import Text2Speech
import time

def main():
    print('starting tts client')
    rospy.init_node('speaking_node')
    rospy.wait_for_service('/tts')
    tts_service = rospy.ServiceProxy('/tts', Text2Speech)
    pub = rospy.Publisher('mutex_mic', Bool, queue_size=10)
    
    while not rospy.is_shutdown():
        txt = rospy.wait_for_message("bot_answer", String)
        tts_service(txt.data)
        pub.publish(False)


if __name__ == '__main__':
    try: 
        main()
    except rospy.ROSInterruptException:
        pass