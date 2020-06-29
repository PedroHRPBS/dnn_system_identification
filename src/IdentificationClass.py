# Solution found at https://github.com/googleapis/oauth2client/issues/642
import sys
if not hasattr(sys, 'argv'):
        sys.argv  =  ['']

import numpy as np
from _collections import deque
import itertools
import tensorflow as tf


class Identification:
     
    def __init__(self, t_h_mrft):
        self.__dnn_model_path = '/home/pedrohrpbs/catkin_ws_tensorflow/src/dnn_system_identification/src/DNNs/z/model.h5'
        self.__systems_path = '/home/pedrohrpbs/catkin_ws_tensorflow/src/dnn_system_identification/src/DNNs/z/systems_truth_table.csv'
        self.__dnn_model = tf.keras.models.load_model(self.__dnn_model_path)
        self.__systems = np.loadtxt(self.__systems_path, delimiter=',')
        self.__MRFT_command = deque([], 40000) #Considering data is received at 400Hz max, 100seg of data is more than enough
        self.__MRFT_error = deque([], 40000)
        self.__MRFT_time = deque([], 40000)
        self.__MRFT_error_params = []
        self.__rise_edge_times = []
        self.__h_mrft = t_h_mrft #Change this depending on the defined amplitude of MRFT
        self.__K = -1.0; self.__T = -1.0; self.__tau = -1.0; self.__Kp = -1.0; self.__Kd = -1.0; self.__Ki = -1.0
        self.__system_class = -1

    def update_dnn_model_and_system(self, dnn_model_path, systems_path):
        self.__dnn_model_path = dnn_model_path
        self.__systems_path = systems_path
        print("Model loaded: "+self.__dnn_model_path)
        print("Systems loaded: "+self.__systems_path)
        self.__dnn_model = tf.keras.models.load_model(self.__dnn_model_path)
        self.__systems = np.loadtxt(self.__systems_path, delimiter=',')

    def get_MRFT_amp(self):
        return self.__h_mrft

    def receive_data(self, t_pv, t_u, t_time):
        self.__MRFT_command.append(t_u)
        self.__MRFT_error.append(t_pv)
        self.__MRFT_time.append(t_time)

        # print(t_time)
        # print("U:",self.__MRFT_command[-1])
        # print("PV",self.__MRFT_error[-1])

        self.detect_rise_edges(t_time)

        return (self.__Kp, self.__Kd, self.__system_class)


    def detect_rise_edges(self, t_time):
        if len(self.__MRFT_command) > 1:
                if (self.__MRFT_command[-1] - self.__MRFT_command[-2]) > (1.95 * self.__h_mrft):  #Detecting rise-edge
                        self.__rise_edge_times.append((len(self.__MRFT_command), t_time)) #Tuple (index of rise-edge, time)
                        print("RISE DETECTED: ", len(self.__rise_edge_times), "at time: ", t_time)

                        self.get_error_parameters()          

    def get_error_parameters(self):
        if len(self.__rise_edge_times) >= 2:
                signal_start = self.__rise_edge_times[-2][0]
                signal_end = self.__rise_edge_times[-1][0]     
                max_peak = max(list(itertools.islice(self.__MRFT_error, signal_start, signal_end)))
                min_peak = min(list(itertools.islice(self.__MRFT_error, signal_start, signal_end)))
                period = self.__rise_edge_times[-1][1] - self.__rise_edge_times[-2][1]
                self.__MRFT_error_params.append((max_peak, min_peak, period)) #Params is a tuple (Max peak, Min peak, Period)

                print("self.__MRFT_error_params:", self.__MRFT_error_params[-1])
                if (max_peak - min_peak > 0.07):    #Check for at least 4deg of difference between max and min. This is to avoid flat signal.
                    self.detect_steady_state(signal_start, signal_end)

    def detect_steady_state(self, signal_start, signal_end, samples=3):
        if len(self.__MRFT_error_params) > samples: #Testing consistency of data to detect steady state, the test is done by checking the standard deviation of params
                                        
                max_peak_std = np.std([element[0] for element in self.__MRFT_error_params[-samples:]]) #Only get the standard deviation of the last samples (3) events
                min_peak_std = np.std([element[1] for element in self.__MRFT_error_params[-samples:]])
                period_std = np.std([element[2] for element in self.__MRFT_error_params[-samples:]])

                print(max_peak_std, min_peak_std, period_std)

                if max_peak_std < 0.02 and min_peak_std < 0.02 and period_std < 0.02: #0.02 came from analysis of real data example where the algorithm was successful
                        print("Steady State DETECTED")
                        control_timeseries = list(itertools.islice(self.__MRFT_command, signal_start, signal_end-1)) #-1 to remove the last rise edge
                        error_timeseries = list(itertools.islice(self.__MRFT_error, signal_start, signal_end-1))
                        timeseries = list(itertools.islice(self.__MRFT_time, signal_start, signal_end-1))

                        assert(len(control_timeseries) == len(error_timeseries) == len(timeseries))

                        # print(control_timeseries)
                        # print(error_timeseries)
                        # print(timeseries)

                        self.interpolate_data(control_timeseries, error_timeseries, timeseries)

    def interpolate_data(self, control_timeseries, error_timeseries, timeseries):        
        # Interpolate
        initial_time = timeseries[0]
        x = [(i-initial_time)*1000 for i in timeseries] #Start from 0, and multiply by 1000, so 1 = 1ms

        xvals = np.linspace(x[0], x[-1], round(x[-1]))  #1ms interpolation, e.g. data that goes from 0.0ms to 540.452ms will be sampled 540 times (first and last value are k
        error_interp = np.interp(xvals, x, error_timeseries)
        control_interp = np.interp(xvals, x, control_timeseries)

        # Removing transition values
        half_way = max(control_interp)-self.__h_mrft
        control_interp[control_interp>=half_way] = max(control_interp)
        control_interp[control_interp<half_way] = min(control_interp)

        control_interp = control_interp.tolist()
        error_interp = error_interp.tolist()

        assert(len(error_interp) == len(control_interp) == len(xvals))

        # print(control_interp)
        # print(error_interp)

        self.pre_process_data(control_interp, error_interp)

    def pre_process_data(self, control_timeseries, error_timeseries):
        sample_size = 2500 #TODO This should be different for Z
        h_mrft_control = (max(control_timeseries)-min(control_timeseries)) / 2.0
        h_mrft_error = (max(error_timeseries)-min(error_timeseries)) / 2.0

        SCALED_GAIN = h_mrft_control / h_mrft_error

        # Zero center
        offset_control = ((max(control_timeseries) + min(control_timeseries)) / 2.0)
        offset_error = ((max(error_timeseries) + min(error_timeseries)) / 2.0)
        control_timeseries_zero_center = [x-offset_control for x in control_timeseries]  
        error_timeseries_zero_center = [x-offset_error for x in error_timeseries]
        
        # Normalize
        control_timeseries_max = max(control_timeseries_zero_center)
        error_timeseries_max = max(error_timeseries_zero_center)
        control_timeseries_normalized = [x * 1.0 / control_timeseries_max for x in control_timeseries_zero_center]
        error_timeseries_normalized = [x * 1.0 / error_timeseries_max for x in error_timeseries_zero_center]

        # Zero-padding
        normalized_control_timeseries = np.zeros(sample_size)
        normalized_error_timeseries = np.zeros(sample_size)
        normalized_control_timeseries[-len(control_timeseries_normalized):] = control_timeseries_normalized
        normalized_error_timeseries[-len(error_timeseries_normalized):] = error_timeseries_normalized

        # print(normalized_control_timeseries)
        # print(normalized_error_timeseries)

        input_layer = np.concatenate((normalized_error_timeseries, normalized_control_timeseries), axis=0)

        self.dnn_classify(input_layer, SCALED_GAIN)

    def dnn_classify(self, input_layer, scaled_gain):
        print("DNN CLASSIFICATION")
        # Format input to comply with neural network
        input_data = input_layer.reshape(1, 1, 5000) #TODO This should be different for Z

        prediction = self.__dnn_model.predict(input_data)
        self.__system_class = np.argmax(prediction)
        temp_system = self.__systems[self.__system_class]
        
        # each row is a process, column are: K T tau P I D
        self.__K = temp_system[0]
        self.__T = temp_system[1]
        self.__tau = temp_system[2]
        self.__Kp = temp_system[3] * scaled_gain * 4 / np.pi
        self.__Ki = temp_system[4] * scaled_gain * 4 / np.pi
        self.__Kd = temp_system[5] * scaled_gain * 4 / np.pi
       
        # self.__Kp = temp_system[7] * scaled_gain * 4 / np.pi
        # self.__Kd = temp_system[8] * scaled_gain * 4 / np.pi #TODO This should be different for Z

        print("")
        print("CLASS: ", self.__system_class, "KP: ", self.__Kp, "KD: ", self.__Kd, "Scaled Gain: ", scaled_gain)
        print("")



def return_instance(t_h_mrft):
    new_object = Identification(t_h_mrft)
    print("new_object instantiated, with amplitude: ", new_object.get_MRFT_amp())
    return new_object

