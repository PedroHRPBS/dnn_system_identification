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
h_mrft = 0.1 #Change this depending on the defined amplitude of MRFT

def receive_data(t_pv, t_u, t_time):

        MRFT_command.append(t_u)
        MRFT_error.append(0.0-t_pv) #Error = Reference - PV
        MRFT_time.append(t_time)

        if len(MRFT_command) > 1:
                if (MRFT_command[-1] - MRFT_command[-2]) > (1.95 * h_mrft):  #Detecting rise-edge
                        rise_edge_times.append((len(MRFT_command) - 1, t_time)) #Tuple (index of rise-edge, time)
                        print("RISE DETECTED: ", len(rise_edge_times), "at time: ", t_time)

                        if len(rise_edge_times) >= 2:
                                signal_start = rise_edge_times[-2][0]
                                signal_end = rise_edge_times[-1][0]     
                                max_peak = max(list(itertools.islice(MRFT_error, signal_start, signal_end)))
                                min_peak = min(list(itertools.islice(MRFT_error, signal_start, signal_end)))
                                period = rise_edge_times[-1][1] - rise_edge_times[-2][1]
                                MRFT_error_params.append((max_peak, min_peak, period))

                                print(MRFT_error_params[-1])


