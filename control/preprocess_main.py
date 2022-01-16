import numpy as np
import math
import os
from path_planning.plot_path import *
from path_planning.path_generate import *
import time
import glob
import scipy
import matplotlib.pyplot as plt

# filter
from scipy import signal


if __name__ == "__main__":
    # flag_write_word = True
    # flag_plot_result = False
    # flag_demo_write = False
    # write_name = 'ren'
    #
    # # build filter based on signals
    # b, a = signal.butter(8, 0.02, 'lowpass')
    #
    # osc_velocity_list = \
    #     plot_velocity_path(
    #     root_path='./data/font_data/' + write_name + '/',
    #     file_name='real_angle_list_',
    #     stroke_num=1,
    #     epi_time=4,
    #     delimiter=',',
    #     skiprows=1
    # )
    # filtered_osc_velocity = np.zeros_like(osc_velocity_list)
    # filtered_osc_velocity[:, 0] = signal.filtfilt(b, a, osc_velocity_list[:, 0])
    # filtered_osc_velocity[:, 1] = signal.filtfilt(b, a, osc_velocity_list[:, 1])
    # # print('shape_ori :', external_force.shape, 'shape_filter ', filtered_force.shape)
    #
    # plot_force(filtered_osc_velocity)

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

	# # DTW
    # import numpy as np
    # # ## A noisy sine wave as query
    # # idx = np.linspace(0, 6.28, num=100)
    # # query = np.sin(idx) + np.random.uniform(size=100) / 10.0
    # #
    # # ## A cosine is for template; sin and cos are offset by 25 samples
    # # template = np.cos(idx)
    # idx_1 = np.linspace(1, 600, num=600)
    # idx_2 = np.linspace(1, 399, num=400)
    # query = np.cos(2 * np.pi * 0.05 * idx_1)
    # template = np.cos(2 * np.pi * 9 * idx_2)
    #
    # ## Find the best match with the canonical recursion formula
    # from dtw import *
    #
    # alignment = dtw(query, template, keep_internals=True)
    # print("alignment :", alignment.index1, alignment.index2)
    #
    # plt.figure()
    # # plt.plot(query)
    # # plt.plot(template)
    # plt.plot(query[alignment.index1])
    # plt.plot(template[alignment.index2])
    # plt.show()

    import numpy as np

    # We define two sequences x, y as numpy array
    # where y is actually a sub-sequence from x
    x = np.array([2, 0, 1, 1, 2, 4, 2, 1, 2, 0]).reshape(-1, 1)
    y = np.array([1, 1, 2, 4, 2, 1, 2, 0]).reshape(-1, 1)
    z = np.array([1, 2, 4, 4, 2, 1, 2]).reshape(-1, 1)

    from dtw import dtw

    manhattan_distance = lambda x, y: np.abs(x - y)

    d_1, cost_matrix_1, acc_cost_matrix_1, path_1 = dtw(x, y, dist=manhattan_distance)
    d_2, cost_matrix_2, acc_cost_matrix_2, path_2 = dtw(x, z, dist=manhattan_distance)

    # print(d)

    # You can also visualise the accumulated cost and the shortest path
    import matplotlib.pyplot as plt

    # plt.imshow(acc_cost_matrix.T, origin='lower', cmap='gray', interpolation='nearest')
    # plt.plot(path[0], path[1], 'w')
    print("path_0 :", path_1[0].shape, "path_1 :", path_1[1])
    print("path_0 :", path_2[0].shape, "path_1 :", path_2[1])
    # print("path_0 :", x[path[0]])
    # print("path_1 :", y[path[1]])
    # plt.show()

    # ## Display the warping curve, i.e. the alignment curve
    # alignment.plot(type="threeway")
    # alignment.plot(type="alignment")
    # ## Align and plot with the Rabiner-Juang type VI-c unsmoothed recursion
    # dtw(query, template, keep_internals=True,
    #     step_pattern=rabinerJuangStepPattern(6, "c")) \
    #     .plot(type="twoway", offset=-2)
    #
    # ## See the recursion relation, as formula and diagram
    # print(rabinerJuangStepPattern(6, "c"))
    # rabinerJuangStepPattern(6, "c").plot()

    from scipy.optimize import linear_sum_assignment

    cost = np.array([[4, 1, 3, 5], [2, 0, 5, 4], [3, 2, 2, 0]])
    row_ind, col_ind = linear_sum_assignment(cost)
    print(row_ind)  # 开销矩阵对应的行索引
    print(col_ind)  # 对应行索引的最优指派的列索引
    print(cost[row_ind, col_ind])  # 提取每个行索引的最优指派列索引所在的元素，形成数组
    print(cost[row_ind, col_ind].sum())  # 数组求和
