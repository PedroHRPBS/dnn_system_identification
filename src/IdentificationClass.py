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
        self.__h_mrft = 0.04 #Change this depending on the defined amplitude of MRFT #TODO receive this value externally
        self.__T1 = -1.0; self.__T2 = -1.0; self.__tau = -1.0; self.__Kp = -1.0; self.__Kd = -1.0; self.__Ki = -1.0

    def receive_data(self, t_pv, t_u, t_time):
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
        control_interp[control_interp>=0] = max(control_interp)
        control_interp[control_interp<0] = min(control_interp)

        control_interp = control_interp.tolist()
        error_interp = error_interp.tolist()

        assert(len(error_interp) == len(control_interp) == len(xvals))

        # print(control_interp)
        # print(error_interp)

        self.pre_process_data(control_interp, error_interp)

    def pre_process_data(self, control_timeseries, error_timeseries):
        sample_size = 2260
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

        self.dnn_classify(normalized_error_timeseries, normalized_control_timeseries, SCALED_GAIN)

    def dnn_classify(self, normalized_error_timeseries, normalized_control_timeseries, scaled_gain):

        # Format input to comply with neural network
        pv_data_array = np.asarray(normalized_error_timeseries)
        command_data_array = np.asarray(normalized_control_timeseries)
        input_data = np.dstack([np.vstack(pv_data_array), np.vstack(command_data_array)])
        input_data = input_data.reshape(1, 2260, 1, 2)

        prediction = self.dnn_model.predict(input_data)
        classification = np.argmax(prediction)

        temp_system = self.systems[classification]
        self.__T1 = temp_system[1]
        self.__T2 = temp_system[2]
        self.__tau = temp_system[3]
        self.__Kp = temp_system[7] * scaled_gain
        self.__Kd = temp_system[8] * scaled_gain
        self.__Ki = 0
        print("")
        print("CLASS: ", classification, "KP: ", self.__Kp, "KD: ", self.__Kd, "Scaled Gain: ", scaled_gain)
        print("")


def return_instance():
    new_object = Identification()
    print("new_object instantiated")
    return new_object

