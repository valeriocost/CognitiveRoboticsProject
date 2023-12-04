import os, sys, cv2, csv, argparse
import numpy as np
import keras
import tensorflow as tf
import recognition
from scipy.spatial.distance import cosine
from keras_vggface.vggface import VGGFace
from keras_vggface.utils import preprocess_input



def decision_face(image, face_reco_model, train_data):
    rejection_threshold = 0.45
    feature_vector = recognition.extract_features(face_reco_model, image)
    min_distance = [-1, recognition.DEFAULT_DISTANCE]
    rows = test_get_feature_vectors(face_reco_model, train_data)
    for id, vector in rows:
        distance = cosine(feature_vector, vector)
        if distance < min_distance[1] and distance < rejection_threshold:
            min_distance[0] = id
            min_distance[1] = distance
    if min_distance[1]==recognition.DEFAULT_DISTANCE:
        return -1
    else:
        return min_distance[0]

def test_get_feature_vectors(face_reco_model,train_data=None):
    
    with open(train_data, mode='r') as csv_file:
        gt = csv.reader(csv_file, delimiter=',')

        rows=[]
        for row in gt:
            image=row[0]
            id=row[1]
            
            feature_vector=recognition.extract_features(face_reco_model, cv2.imread('./test_dataset/'+image))
            rows.append((id,feature_vector))
        return rows



face_reco_model = VGGFace(model='resnet50', include_top=False, pooling='avg')
def test(test_set, train_set):
    with open(test_set, mode='r') as csv_file:
        gt = csv.reader(csv_file, delimiter=',')
        rows=[]
        for row in gt:
            predict_id = decision_face(cv2.imread('./test_dataset/'+row[0]), face_reco_model, train_set)
            if int(predict_id)==int(row[1]):
                print('Test OK ' + str(predict_id))
            else:
                print('Error recognition' + str(row[1]))

print('TEST - Only 1 image for person in the train set')
test('./test_dataset/test.csv', './test_dataset/train1.csv')
print('TEST - Only 2 image for person in the train set')
test('./test_dataset/test.csv', './test_dataset/train2.csv')
print('TEST - Only 3 image for person in the train set')
test('./test_dataset/test.csv', './test_dataset/train2.csv')
print('TEST - Only 4 image for person in the train set')
test('./test_dataset/test.csv', './test_dataset/train2.csv')
print('TEST - All 5 image for person in the train set')
test('./test_dataset/test.csv', './test_dataset/train2.csv')
