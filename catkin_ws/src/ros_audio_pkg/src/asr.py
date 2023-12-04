#!/usr/bin/python3
import rospy
from std_msgs.msg import Int16MultiArray, String, Bool
import numpy as np

from speech_recognition import AudioData
import speech_recognition as sr

# Initialize a Recognizer
r = sr.Recognizer()

# Init node
rospy.init_node('speech_recognition', anonymous=True)

pub2 = rospy.Publisher('voice_txt', String, queue_size=10)
pub = rospy.Publisher('mutex_mic', Bool, queue_size=10)

# this is called from the background thread
def callback(audio):
    data = np.array(audio.data,dtype=np.int16)
    audio_data = AudioData(data.tobytes(), 16000, 2)

    try:
        spoken_text= r.recognize_google(audio_data, language='en-GB')
        print("Speech To Text: " + spoken_text)
        pub2.publish(spoken_text)
    except sr.UnknownValueError:
        pub.publish(False)
        print("Google Speech Recognition non riesce a capire da questo file audio")
    except sr.RequestError as e:
        pub.publish(False)
        print("Could not request results from Google Speech Recognition service; {0}".format(e))

def listener():
    rospy.Subscriber("mic_data", Int16MultiArray, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()