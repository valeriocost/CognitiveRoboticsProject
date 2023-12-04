#!/usr/bin/python3
import rospy
from std_msgs.msg import Int16MultiArray, Bool
import numpy as np

import speech_recognition as sr

pub = rospy.Publisher('mic_data', Int16MultiArray, queue_size=10)
rospy.init_node('voice_detection_node', anonymous=True)

# Initialize a Recognizer
r = sr.Recognizer()

# Audio source
m = sr.Microphone(device_index=None,
                    sample_rate=16000,
                    chunk_size=1024)
                    
# Manage time mutex mic while new user is being added
skip = False
def callback(data):
    global skip
    skip = True
sub = rospy.Subscriber('skip_mic', Bool, callback=callback)

# Calibration within the environment
# we only need to calibrate once, before we start listening
print("Calibrating...")
with m as source:
    r.adjust_for_ambient_noise(source,duration=3)  
print("Calibration finished")

while not rospy.is_shutdown():
    if not skip:
        rospy.wait_for_message('mutex_mic', Bool)
    else:
        skip = False
    print('Recording...')
    with m as source:
        audio = r.listen(source, phrase_time_limit=10)
    if not skip:
        data = np.frombuffer(audio.get_raw_data(), dtype=np.int16)
        data_to_send = Int16MultiArray()
        data_to_send.data = data
        pub.publish(data_to_send)
        print('STOP recording...')
    else:
        print('Jump')




