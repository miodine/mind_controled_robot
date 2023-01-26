#!/usr/bin/env python3


# general imports
import matplotlib.pyplot as plt
from sklearn.metrics import classification_report, confusion_matrix
from sklearn.preprocessing import minmax_scale
import pickle
import tensorflow as tf
import numpy as np
import os
import sys
import pandas as pd

# model creation imports
from tensorflow.keras.callbacks import ModelCheckpoint, EarlyStopping
from tensorflow.keras.layers import Dense,Dropout
from tensorflow.keras.layers import Lambda, Input, Flatten
from tensorflow.keras.layers import BatchNormalization, ReLU
from tensorflow.keras import backend as K

# ROS imports
from mcr_messages.msg import mcr_datastream, mcr_predictions

os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'

MODEL_NAME = 'ffnet_in_640_cl_700'
LOAD_PATH = str(os.path.dirname(__file__)) + '/models/'+ MODEL_NAME + '/'


# PTF = path to file
PTF = str(os.path.dirname(__file__))

# load run info
__log_txt_file_handle = open(PTF + "/presentation_layer/init_msg.txt","r")
LOGINFO = __log_txt_file_handle.read()
__log_txt_file_handle.close()


def FourierFeaturesNet(nb_classes = 5, Chans = 4, Samples = 640, classification_layer_dim = 700):
    ff_net = tf.keras.Sequential()
    ff_net.add(Input(shape = (Chans,Samples)))
    ff_net.add(Lambda(lambda v: tf.math.abs(tf.signal.rfft(v))))
    ff_net.add(Flatten())
    ff_net.add(Dense(units = classification_layer_dim)) 
    ff_net.add(BatchNormalization()) 
    ff_net.add(ReLU()) 
    ff_net.add(Dropout(rate=0.2))
    ff_net.add(Dense(units = nb_classes,activation = 'softmax'))
    return ff_net



def model_preparation():
    ffnet = FourierFeaturesNet()
    model = tf.keras.models.load_model(LOAD_PATH, custom_objects={"CustomModel":ffnet})
    print(model.summary())
    return model 


class MlClassifier:
    def __init__(self):
        self.data_block = np.empty((4,640))
        self.predictions = np.empty([5, 1])
        self.classifier = model_preparation()

    def get_signals_callback(self, data):
        e1 = list(data.electrode_1)
        e2 = list(data.electrode_2)
        e3 = list(data.electrode_3)
        e4 = list(data.electrode_4)
        summaric_np_debug = np.asarray([e1, e2, e3,e4])
        self.data_block = summaric_np_debug

    def push_predictions_to_msg(self):
        msg = mcr_predictions()
        msg.meta = 'predictions'
        msg.predictions = self.predictions.tolist()[0] # tolist encapsulates vector as a list wrapped as list - our 5 predictions are at 0 index
        return msg

    def classify(self):
        self.predictions = self.classifier.predict(np.array([self.data_block]),verbose = False)
        return self.push_predictions_to_msg()





