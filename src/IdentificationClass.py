# Solution found at https://github.com/googleapis/oauth2client/issues/642
import sys
if not hasattr(sys, 'argv'):
        sys.argv  =  ['']

import numpy as np
from _collections import deque
import itertools
import tensorflow as tf


class Identification:
     
    def __init__(self):
        self.dnn_model = tf.keras.models.load_model('/home/pedrohrpbs/catkin_ws_tensorflow/src/dnn_system_identification/src/model.h5')
        self.systems = np.loadtxt('/home/pedrohrpbs/catkin_ws_tensorflow/src/dnn_system_identification/src/systems_truth_table.csv', delimiter=',')
        self.__MRFT_command = deque([], 40000) #Considering data is received at 400Hz max, 100seg of data is more than enough
        self.__MRFT_error = deque([], 40000)
        self.__MRFT_time = deque([], 40000)
        self.__MRFT_error_params = []
        self.__rise_edge_times = []
        self.__h_mrft = 0.04 #Change this depending on the defined amplitude of MRFT
        self.__T1 = -1.0; self.__T2 = -1.0; self.__tau = -1.0; self.__Kp = -1.0; self.__Kd = -1.0; self.__Ki = -1.0

    def receive_data(self, t_pv, t_u, t_time):
        print("Inside receive_data")
        self.__MRFT_command.append(t_u)
        self.__MRFT_error.append(t_pv)
        self.__MRFT_time.append(t_time)

        # print(t_time)
        # print("U:",self.__MRFT_command[-1])
        # print("PV",self.__MRFT_error[-1])

        self.detect_rise_edges(t_time)

        return (self.__Kp, self.__Kd)


    def detect_rise_edges(self, t_time):
        if len(self.__MRFT_command) > 1:
                print(self.__MRFT_command[-1], self.__MRFT_command[-2])
                if (self.__MRFT_command[-1] - self.__MRFT_command[-2]) > (1.95 * self.__h_mrft):  #Detecting rise-edge
                        self.__rise_edge_times.append((len(self.__MRFT_command), t_time)) #Tuple (index of rise-edge, time)
                        print("RISE DETECTED: ", len(self.__rise_edge_times), "at time: ", t_time)

                        #get_error_parameters()          

def return_instance():
    new_object = Identification()
    print("return_instance inside")
    return new_object

