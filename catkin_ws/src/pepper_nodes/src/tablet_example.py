#!/usr/bin/python3

import rospy
from pepper_nodes.srv import LoadUrl, LoadUrlRequest, LoadUrlResponse
from std_msgs.msg import String
from pepper_nodes.msg import InfoView

IP = '10.0.1.239'

class Handler:
    '''
    The constructor creates the service proxy object, which is able to display the desired URL on the tablet.
    '''
    def __init__(self):
        self.tablet_service = rospy.ServiceProxy("load_url", LoadUrl)

    '''
    This method calls the tablet service and sends it the URL of the web page to be displayed.
    '''
    def load_url(self, url):
        msg = LoadUrlRequest()
        msg.url = url
        resp = self.tablet_service(msg)
        rospy.loginfo(resp.ack)

handler = Handler()

def callback(data):
    id = data.id
    category = data.category
    date = data.date

    url=""

    if category!="" and date!="":
        url = r"http://{}:5000/view?id={}&category={}&date={}".format(IP,id,category,date)
    elif date!="":
        url = r"http://{}:5000/view?id={}&date={}".format(IP,id,date)
    elif category!="":
        url = r"http://{}:5000/view?id={}&category={}".format(IP,id,category)
    else:
        url = r"http://{}:5000/view?id={}".format(IP,id)
    
    handler.load_url(url)

    print(url)



if __name__ == "__main__":
    NODE_NAME = "table_node_example"
    rospy.init_node(NODE_NAME)
    rospy.Subscriber('info_topic', InfoView, callback=callback)

    rospy.spin()
