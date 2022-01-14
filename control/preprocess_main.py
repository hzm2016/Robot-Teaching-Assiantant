import numpy as np
import math
import os
from path_planning.plot_path import *
from path_planning.path_generate import *
import time
import glob
import scipy

# filter
from scipy import signal


if __name__ == "__main__":
    flag_write_word = True
    flag_plot_result = False
    flag_demo_write = False
    write_name = 'ren'
    
    # build filter based on signals
    b, a = signal.butter(8, 0.02, 'lowpass')

    osc_velocity_list = \
        plot_velocity_path(
        root_path='./data/font_data/' + write_name + '/',
        file_name='real_angle_list_',
        stroke_num=1,
        epi_time=4,
        delimiter=',',
        skiprows=1
    )
    filtered_osc_velocity = np.zeros_like(osc_velocity_list)
    filtered_osc_velocity[:, 0] = signal.filtfilt(b, a, osc_velocity_list[:, 0])
    filtered_osc_velocity[:, 1] = signal.filtfilt(b, a, osc_velocity_list[:, 1])
    # print('shape_ori :', external_force.shape, 'shape_filter ', filtered_force.shape)
    
    plot_force(filtered_osc_velocity)

    # external_force = \
    #     plot_torque_path(
    #     root_path='./data/font_data/' + write_name + '/',
    #     file_name='real_torque_list_',
    #     stroke_num=2,
    #     epi_time=4,
    #     delimiter=',',
    #     skiprows=1
    # )
	
    # filtered_force = np.zeros_like(external_force)
    # filtered_force[:, 0] = signal.filtfilt(b, a, external_force[:, 0])
    # filtered_force[:, 1] = signal.filtfilt(b, a, external_force[:, 1])
    # print('shape_ori :', external_force.shape, 'shape_filter ', filtered_force.shape)
    
    # plot_force(filtered_force)