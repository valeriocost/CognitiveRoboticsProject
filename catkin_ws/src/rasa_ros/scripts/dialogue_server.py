#!/usr/bin/env python3
from rasa_ros.srv import Dialogue, DialogueResponse

import rospy
import requests
from std_msgs.msg import String, Bool

id = -1

def handle_service(req):
    global id
    input_text = req.input_text

    # Get answer        
    get_answer_url = 'http://localhost:5002/webhooks/rest/webhook'
    message = {
        "sender": id,
        "message": input_text
    }

    r = requests.post(get_answer_url, json=message)
    response = DialogueResponse()
    response.answer = ""
    for i in r.json():
        response.answer += i['text'] + '\n' if 'text' in i else ''

    return response

def callback(data):
    global id
    id = data.data 

def main():
    print("Rasa Server started")
    rospy.init_node('dialogue_service', anonymous=True)

    # Server Initialization
    rospy.Service('dialogue_server',
                        Dialogue, handle_service)

    rospy.Subscriber('id', String, callback=callback)
    rospy.logdebug('Dialogue server READY.')
    rospy.spin()


if __name__ == '__main__':
    try: 
        main()
    except rospy.ROSInterruptException as e:
        pass
