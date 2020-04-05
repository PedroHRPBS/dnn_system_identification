# Solution found at https://github.com/googleapis/oauth2client/issues/642
import sys
if not hasattr(sys, 'argv'):
        sys.argv  =  ['']

import numpy as np
from _collections import deque
# Solution found at https://answers.ros.org/question/289855/import-tensorflow-in-ros-kinetic/ by Oscar Pang
import tensorflow as tf
import itertools

MRFT_command = deque([], 40000) #Considering data is received at 400Hz max, 100seg of data is more than enough
MRFT_error = deque([], 40000)
MRFT_time = deque([], 40000)
MRFT_error_params = []
rise_edge_times = []
h_mrft = 0.04 #Change this depending on the defined amplitude of MRFT

def receive_data(t_pv, t_u, t_time):

        MRFT_command.append(t_u)
        MRFT_error.append(t_pv)
        MRFT_time.append(t_time)

        # print(t_time)
        # print("U:",t_u)
        # print("PV",t_pv)

        detect_rise_edges(t_time)
                       

def detect_rise_edges(t_time):
        if len(MRFT_command) > 1:
                if (MRFT_command[-1] - MRFT_command[-2]) > (1.95 * h_mrft):  #Detecting rise-edge
                        rise_edge_times.append((len(MRFT_command), t_time)) #Tuple (index of rise-edge, time)
                        print("RISE DETECTED: ", len(rise_edge_times), "at time: ", t_time)

                        get_error_parameters()

def get_error_parameters():
        if len(rise_edge_times) >= 2:
                signal_start = rise_edge_times[-2][0]
                signal_end = rise_edge_times[-1][0]     
                max_peak = max(list(itertools.islice(MRFT_error, signal_start, signal_end)))
                min_peak = min(list(itertools.islice(MRFT_error, signal_start, signal_end)))
                period = rise_edge_times[-1][1] - rise_edge_times[-2][1]
                MRFT_error_params.append((max_peak, min_peak, period))

                print("MRFT_error_params:", MRFT_error_params[-1])

                detect_steady_state(signal_start, signal_end)

def detect_steady_state(signal_start, signal_end, samples=3):
        if len(MRFT_error_params) > 3: #Testing consistency of data, to detect steady state
                                        
                max_peak_std = np.std([element[0] for element in MRFT_error_params[-samples:]]) #Only get the std of the last 3 events
                min_peak_std = np.std([element[1] for element in MRFT_error_params[-samples:]])
                period_std = np.std([element[2] for element in MRFT_error_params[-samples:]])

                print(max_peak_std, min_peak_std, period_std)

                if max_peak_std < 0.02 and min_peak_std < 0.02 and period_std < 0.02: #0.02 came from analysis of good data example
                        print("Steady State DETECTED")
                        control_timeseries = list(itertools.islice(MRFT_command, signal_start, signal_end-1)) #-1 to remove the last rise edge
                        error_timeseries = list(itertools.islice(MRFT_error, signal_start, signal_end-1))
                        timeseries = list(itertools.islice(MRFT_time, signal_start, signal_end-1))

                        assert(len(control_timeseries) == len(error_timeseries))

                        #print(control_timeseries)
                        #print(error_timeseries)
                        #print(timeseries)

                        interpolate_data(control_timeseries, error_timeseries, timeseries)

def interpolate_data(control_timeseries, error_timeseries, timeseries):
        #Interpolate
        initial_value = timeseries[0]
        x = [(i-initial_value)*1000 for i in timeseries] 

        xvals = np.linspace(x[0], x[-1], round(x[-1]))  #1ms interpolation
        error2_interp = np.interp(xvals, x, error_timeseries)
        control2_interp = np.interp(xvals, x, control_timeseries)

        #Removing transition values
        control2_interp[control2_interp>=0] = max(control2_interp)
        control2_interp[control2_interp<0] = min(control2_interp)

        control2_interp = control2_interp.tolist()
        error2_interp = error2_interp.tolist()

        assert(len(error2_interp) == len(control2_interp) == len(xvals))

        normalize_data(control2_interp, error2_interp)


def normalize_data(control_timeseries, error_timeseries):
        sample_size = 2260
        h_mrft_control = (max(control_timeseries)-min(control_timeseries)) / 2.0
        h_mrft_error = (max(error_timeseries)-min(error_timeseries)) / 2.0

        scaled_gain =  h_mrft_control / h_mrft_error

        #Zero center
        offset_control = ((max(control_timeseries) + min(control_timeseries)) / 2.0)
        offset_error = ((max(error_timeseries) + min(error_timeseries)) / 2.0)
        control_timeseries_zero_center = [x-offset_control for x in control_timeseries]  
        error_timeseries_zero_center = [x-offset_error for x in error_timeseries]

        #Normalize
        control_timeseries_max = max(control_timeseries_zero_center)
        error_timeseries_max = max(error_timeseries_zero_center)
        control_timeseries_normalized = [x * 1.0 / control_timeseries_max for x in control_timeseries_zero_center]
        error_timeseries_normalized = [x * 1.0 / error_timeseries_max for x in error_timeseries_zero_center]


        normalized_control_timeseries = np.zeros(sample_size)
        normalized_error_timeseries = np.zeros(sample_size)

        #Zero-padding
        normalized_control_timeseries[-len(control_timeseries_normalized):] = control_timeseries_normalized
        normalized_error_timeseries[-len(error_timeseries_normalized):] = error_timeseries_normalized

        #Concatenate
        input_layer = np.concatenate((normalized_error_timeseries, normalized_control_timeseries), axis=0)

        input_layer = input_layer.tolist()

        print(input_layer)
       


        
