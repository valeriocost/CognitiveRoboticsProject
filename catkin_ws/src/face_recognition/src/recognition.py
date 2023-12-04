#!/usr/bin/python3
import cv2
import numpy as np
import os
from glob import glob
import rospy

from keras_vggface.vggface import VGGFace
from keras_vggface.utils import preprocess_input
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool
import ros_numpy 

from scipy.spatial.distance import cosine
from sklearn.metrics import accuracy_score, confusion_matrix

### Bounding box
import numpy as np
import cv2
import tensorflow as tf

import sys
project_home = os.environ['PROJECT_HOME']
sys.path.insert(0, project_home)
from create_table import connect

import time


PATH_DB = project_home + '/toDoDB.db'


THRESHOLD = 5
DEFAULT_DISTANCE = 1000000000000
NUM_FACES_REQUIRED = 5

pub_answer = rospy.Publisher('bot_answer', String, queue_size=10)
pub_mutex = rospy.Publisher('skip_mic', Bool, queue_size=10)

# # This method takes as input the frame captured from the camera and returns
# # the bounding boxes of the detected faces in the frame
def getFaceBox(net, frame, conf_threshold=0.8):
    frameOpencvDnn = frame.copy()
    frameHeight = frameOpencvDnn.shape[0]
    frameWidth = frameOpencvDnn.shape[1]
    
    blob = cv2.dnn.blobFromImage(frameOpencvDnn, 1.0, (300, 300), [104, 117, 123], True, False)

    net.setInput(blob)
    detections = net.forward()
    bboxes = []
    for i in range(detections.shape[2]):
        confidence = detections[0, 0, i, 2]
        if confidence > conf_threshold and detections[0, 0, i, 5]<1 and detections[0, 0, i, 6]<1:
            x1 = int(detections[0, 0, i, 3] * frameWidth)
            y1 = int(detections[0, 0, i, 4] * frameHeight)
            x2 = int(detections[0, 0, i, 5] * frameWidth)
            y2 = int(detections[0, 0, i, 6] * frameHeight)
            bboxes.append([x1, y1, x2, y2])
            cv2.rectangle(frameOpencvDnn, (x1, y1), (x2, y2), (0, 255, 0), int(round(frameHeight/300)), 8)
    return frameOpencvDnn, bboxes

# # This method takes as input the face recognition model and the filename of the image and returns
# # the feature vector
def extract_features(face_reco_model, image):
    faceim = cv2.resize(image, (224,224))
    faceim = preprocess_input([faceim.astype(np.float32)], version=2)
    feature_vector = (face_reco_model.predict(faceim)).flatten()
    return feature_vector

# # This method is called when we need to add a new user and the process
# # of taking NUM_FACES_REQUIRED starts and adds the new user to the database
def add_new_user():
    global pub_answer
    global pub_mutex
    conn = connect()
    cursor = conn.cursor()

    pub_mutex.publish(True)
    data_to_send = String()
    data_to_send.data = "Welcome, I am going to take you " + str(NUM_FACES_REQUIRED) + " pictures. Don't move!"
    pub_answer.publish(data_to_send)

    feature_vectors = []
    for i in range(NUM_FACES_REQUIRED):
        resized_face = get_face_image()
        if resized_face is None:
            break
        feature_vectors.append(extract_features(face_reco_model, resized_face))
        time.sleep(3)
    
    pub_mutex.publish(True)
    data_to_send = String()
    data_to_send.data = "I addded you successfully!"
    pub_answer.publish(data_to_send)

    sqlite_insert_query = """INSERT INTO user(feature_vector) VALUES (?)"""

    cursor.execute(sqlite_insert_query, ((np.stack(feature_vectors)),))
    conn.commit()
    print(cursor.lastrowid)
    return cursor.lastrowid

# # This method takes as input the image of the face and returns
# # the ID of the closest vector saved in the database
def decision_face(image):
    rejection_threshold = 0.45
    feature_vector = extract_features(face_reco_model, image)
    min_distance = [-1, DEFAULT_DISTANCE]
    rows = get_feature_vectors()
    for id, person in rows:
        for vector in person:
            # for face in person:
            distance = cosine(feature_vector, vector)
            if distance < min_distance[1] and distance < rejection_threshold:
                min_distance[0] = id
                min_distance[1] = distance
    if min_distance[1]==DEFAULT_DISTANCE:
        return -1
    else:
        return min_distance[0]

# # This method returns all the feature vectors saved in the database
def get_feature_vectors():
    conn = connect()
    cursor = conn.cursor()

    print("Successfully Connected to SQLite")

    sqlite_search_query = """SELECT id,feature_vector
                            FROM user
                            """
    cursor.execute(sqlite_search_query)
    rows = cursor.fetchall()
    return rows

# # This method returns the resized image of a face acquired from the camera
def get_face_image():
    frame = rospy.wait_for_message("in_rgb", Image)
    frame = ros_numpy.numpify(frame)
    
    frameFace, bboxes = getFaceBox(faceNet, frame)     # Get face
    print(frameFace)
    print(bboxes)
    resized_face = None
    for i,bbox in enumerate(bboxes):
        # Adjust crop
        w = bbox[2]-bbox[0]
        h = bbox[3]-bbox[1]
        padding_px = int(padding*max(h,w))
        face = frame[max(0,bbox[1]-padding_px):min(bbox[3]+padding_px,frame.shape[0]-1),max(0,bbox[0]-padding_px):min(bbox[2]+padding_px, frame.shape[1]-1)]
        face = face[ face.shape[0]//2 - face.shape[1]//2 : face.shape[0]//2 + face.shape[1]//2, :, :]
        # Preprocess image
        resized_face = cv2.resize(face,INPUT_SIZE)
        # Predict
        # Draw
        cv2.imshow("f%d"%i, resized_face)
        cv2.moveWindow("f%d"%i, INPUT_SIZE[0]*i, 40)
    print(resized_face)
    return resized_face
        

if __name__=='__main__':
    # Load the VGG-Face model based on ResNet-50
    face_reco_model = VGGFace(model='resnet50', include_top=False, pooling='avg')
    # init_ros
    rospy.init_node('recognition_node')
    # Initialize detector
    faceProto = project_home + "/opencv_face_detector.pbtxt"
    faceModel = project_home + "/opencv_face_detector_uint8.pb"

    faceNet = cv2.dnn.readNet(faceModel, faceProto)
    
    pub = rospy.Publisher('id', String, queue_size=1) 

    padding = 0.2
    INPUT_SIZE = (224,224)
    
    database = {}
    prev_id = -1
    print('Starting face recognition module')
    counter = 0
    while True:
        resized_face = get_face_image()
        if resized_face is not None:
            id = decision_face(resized_face)
            print(id)
            if id != prev_id:
                prev_id = id
                counter = 0
            else:
                counter += 1
                if counter >= THRESHOLD:
                    if id == -1:
                        id = add_new_user()
                    counter = 0
                    pub.publish(str(id))
                