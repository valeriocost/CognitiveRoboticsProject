#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Bool
from rasa_ros.srv import Dialogue, DialogueResponse
import time

class TerminalInterface:
    '''Class implementing a terminal i/o interface. 

    Methods
    - get_text(self): return a string read from the speech to text module
    - set_text(self, text): prints the text on the terminal and publish it to the bot_answer topic

    '''
    def __init__(self, pub):
        self.pub = pub

    
    def get_text(self):
        print("Waiting")
        txt = rospy.wait_for_message("voice_txt", String)
        print("[IN]: ", txt.data)
        return str(txt.data)

    def set_text(self,text):
        data_to_send = String()
        data_to_send.data = text
        self.pub.publish(data_to_send)
        print("[OUT]:",text)

def main():
    rospy.init_node('writing')
    rospy.wait_for_service('dialogue_server')
    
    pub = rospy.Publisher('mutex_mic', Bool, queue_size=10)
    time.sleep(5)
    pub.publish(False)
    
    dialogue_service = rospy.ServiceProxy('dialogue_server', Dialogue)

    pub = rospy.Publisher('bot_answer', String, queue_size=10)
    terminal = TerminalInterface(pub)

    while not rospy.is_shutdown():
        message = terminal.get_text()
        if message == 'exit': 
            break
        try:
            bot_answer = dialogue_service(message)
            terminal.set_text(bot_answer.answer)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

if __name__ == '__main__':
    try: 
        main()
    except rospy.ROSInterruptException:
        pass