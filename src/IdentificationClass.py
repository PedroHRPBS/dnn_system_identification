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
        self.__first_layer_length = 0;
        self.__identification_done = 0;

    def update_dnn_model_and_system(self, dnn_model_path, systems_path):
        self.__dnn_model_path = dnn_model_path
        self.__systems_path = systems_path
        print("Model loaded: "+self.__dnn_model_path)
        print("Systems loaded: "+self.__systems_path)
        self.__dnn_model = tf.keras.models.load_model(self.__dnn_model_path)
        self.__systems = np.loadtxt(self.__systems_path, delimiter=',')
        self.__first_layer_length = np.shape(self.__dnn_model.inputs[0])[2];

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

        return (self.__Kp, self.__Kd, self.__system_class, self.__identification_done)


    def detect_rise_edges(self, t_time):
        if len(self.__MRFT_command) > 1:
                if (self.__MRFT_command[-1] - self.__MRFT_command[-2]) > (1.95 * self.__h_mrft):  #Detecting rise-edge
                        self.__rise_edge_times.append((len(self.__MRFT_command), t_time)) #Tuple (index of rise-edge, time)
                        print("CLASS ", self.__h_mrft,"RISE DETECTED: ", len(self.__rise_edge_times), "at time: ", t_time)

                        self.get_error_parameters()          

    def get_error_parameters(self):
        if len(self.__rise_edge_times) >= 2:
                signal_start = self.__rise_edge_times[-2][0]
                signal_end = self.__rise_edge_times[-1][0]     
                error_data = np.array(list(itertools.islice(self.__MRFT_error, signal_start, signal_end)))
                count_max = 0
                while True:
                    max_peak = np.amax(error_data[count_max:])
                    if np.argmax(error_data[count_max:]) != count_max:
                        break
                    count_max += 1

                count_min = -1
                while True:
                    min_peak = np.amin(error_data[:count_min])
                    if np.argmin(error_data[:count_min]) != error_data[:count_min].size - 1:
                        break
                    count_min -= 1

                period = self.__rise_edge_times[-1][1] - self.__rise_edge_times[-2][1]

                amplitude = max_peak - min_peak;
                center_point = (max_peak + min_peak) / 2;

                self.__MRFT_error_params.append((amplitude, center_point, period)) #Params is a tuple (Max peak, Min peak, Period)

                print("CLASS ", self.__h_mrft, " self.__MRFT_error_params:", self.__MRFT_error_params[-1])

                self.detect_steady_state(signal_start, signal_end)

    def detect_steady_state(self, signal_start, signal_end, samples=3):
        if len(self.__MRFT_error_params) >= samples: #Testing consistency of data to detect steady state, the test is done by checking the standard deviation of params
                
                last_three_samples = self.__MRFT_error_params[-samples:]
                last_three_amp = [element[0] for element in last_three_samples]
                last_three_cp = [element[1] + 1 for element in last_three_samples] #+ 1 to be far from 0 and generate huge numbers on division
                last_three_period = [element[2] for element in last_three_samples]

                amplitude_mean = np.mean(last_three_amp) #Only get the standard deviation of the last samples (3) events
                center_point_mean = abs(np.mean(last_three_cp)) 

                amplitude_std = np.std(last_three_amp) #Only get the standard deviation of the last samples (3) events
                center_point_std = np.std(last_three_cp)
                period_std = np.std(last_three_period)

                amp_tolerance = amplitude_std / amplitude_mean;
                center_point_tolerance = center_point_std / center_point_mean;

                print(amp_tolerance, center_point_tolerance, period_std)

                if amp_tolerance < 0.06 and center_point_tolerance < 0.015 and period_std < 0.06:
                        print("CLASS ", self.__h_mrft,"Steady State DETECTED")
                        control_timeseries = list(itertools.islice(self.__MRFT_command, signal_start, signal_end-1)) #-1 to remove the last rise edge
                        error_timeseries = list(itertools.islice(self.__MRFT_error, signal_start, signal_end-1))
                        timeseries = list(itertools.islice(self.__MRFT_time, signal_start, signal_end-1))

                        assert(len(control_timeseries) == len(error_timeseries) == len(timeseries))

                        # print(control_timeseries)
                        # print(error_timeseries)
                        # print(timeseries)
                        np.set_printoptions(threshold=sys.maxsize)
                        a = [str(x) for x in error_timeseries]
                        print(','.join(a))
                        b = [str(x) for x in control_timeseries]
                        print(','.join(b))
                        c = [str(x-timeseries[0]) for x in timeseries]
                        print(','.join(c))
                       

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
        # print("interpolate_data")


        self.pre_process_data(control_interp, error_interp)

    def pre_process_data(self, control_timeseries, error_timeseries):

        sample_size = self.__first_layer_length/2 #TODO This should be different for Z
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

        if(len(control_timeseries_normalized) < sample_size):
            normalized_control_timeseries[-len(control_timeseries_normalized):] = control_timeseries_normalized

            normalized_error_timeseries[-len(error_timeseries_normalized):] = error_timeseries_normalized

            # np.set_printoptions(threshold=sys.maxsize)
            # a = [str(x) for x in normalized_error_timeseries]
            # print(','.join(a))
            # print(normalized_error_timeseries)

            input_layer = np.concatenate((normalized_error_timeseries, normalized_control_timeseries), axis=0)

            # np.set_printoptions(threshold=sys.maxsize)
            # a = [str(x) for x in input_layer]
            # print(','.join(a))
            
            self.dnn_classify(input_layer, SCALED_GAIN)
        else:
            print("Steady State DISCARTED")


    def dnn_classify(self, input_layer, scaled_gain):
        print("DNN CLASSIFICATION")
        # Format input to comply with neural network
        input_data = input_layer.reshape(1, 1, self.__first_layer_length) #TODO This should be different for Z

        prediction = self.__dnn_model.predict(input_data)
        self.__system_class = np.argmax(prediction)
        temp_system = self.__systems[self.__system_class]
        
        if self.__first_layer_length == 4520:
            self.__Kp = temp_system[7] * scaled_gain * 4 / np.pi
            self.__Kd = temp_system[8] * scaled_gain * 4 / np.pi #TODO This should be different for Z
       
        elif self.__first_layer_length == 5000:
            # each row is a process, column are: K T tau P I D
            self.__K = temp_system[0]
            self.__T = temp_system[1]
            self.__tau = temp_system[2]
            self.__Kp = temp_system[3] * scaled_gain * 4 / np.pi
            self.__Ki = temp_system[4] * scaled_gain * 4 / np.pi
            self.__Kd = temp_system[5] * scaled_gain * 4 / np.pi

        self.__identification_done = 1;

        print("")
        print("CLASS: ", self.__system_class, "KP: ", self.__Kp, "KD: ", self.__Kd, "Scaled Gain: ", scaled_gain)
        print("")



def return_instance(t_h_mrft):
    new_object = Identification(t_h_mrft)
    print("new_object instantiated, with amplitude: ", new_object.get_MRFT_amp())
    return new_object

