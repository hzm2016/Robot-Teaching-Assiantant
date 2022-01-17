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

    # import numpy as np
    #
    # # We define two sequences x, y as numpy array
    # # where y is actually a sub-sequence from x
    # x = np.array([2, 0, 1, 1, 2, 4, 2, 1, 2, 0]).reshape(-1, 1)
    # y = np.array([1, 1, 2, 4, 2, 1, 2, 0]).reshape(-1, 1)
    # z = np.array([1, 2, 4, 4, 2, 1, 2]).reshape(-1, 1)
    #
    # from dtw import dtw
    #
    # manhattan_distance = lambda x, y: np.abs(x - y)
    #
    # d_1, cost_matrix_1, acc_cost_matrix_1, path_1 = dtw(x, y, dist=manhattan_distance)
    # d_2, cost_matrix_2, acc_cost_matrix_2, path_2 = dtw(x, z, dist=manhattan_distance)
    #
    # # print(d)
    #
    # # You can also visualise the accumulated cost and the shortest path
    # import matplotlib.pyplot as plt
    #
    # # plt.imshow(acc_cost_matrix.T, origin='lower', cmap='gray', interpolation='nearest')
    # # plt.plot(path[0], path[1], 'w')
    # print("path_0 :", path_1[0].shape, "path_1 :", path_1[1])
    # print("path_0 :", path_2[0].shape, "path_1 :", path_2[1])
    # # print("path_0 :", x[path[0]])
    # # print("path_1 :", y[path[1]])
    # # plt.show()
    #
    # # ## Display the warping curve, i.e. the alignment curve
    # # alignment.plot(type="threeway")
    # # alignment.plot(type="alignment")
    # # ## Align and plot with the Rabiner-Juang type VI-c unsmoothed recursion
    # # dtw(query, template, keep_internals=True,
    # #     step_pattern=rabinerJuangStepPattern(6, "c")) \
    # #     .plot(type="twoway", offset=-2)
    # #
    # # ## See the recursion relation, as formula and diagram
    # # print(rabinerJuangStepPattern(6, "c"))
    # # rabinerJuangStepPattern(6, "c").plot()
    #
    # from scipy.optimize import linear_sum_assignment
    #
    # cost = np.array([[4, 1, 3, 5], [2, 0, 5, 4], [3, 2, 2, 0]])
    # row_ind, col_ind = linear_sum_assignment(cost)
    # print(row_ind)  # 开销矩阵对应的行索引
    # print(col_ind)  # 对应行索引的最优指派的列索引
    # print(cost[row_ind, col_ind])  # 提取每个行索引的最优指派列索引所在的元素，形成数组
    # print(cost[row_ind, col_ind].sum())  # 数组求和

    write_name = 'mu'
    stroke_index = 0
    file_fig_name = './data/predicted_images/'
    epi_times = 5
    dt = 0.001
    nb_samples = 5

    input_dim = 1
    output_dim = 2
    in_idx = [0]
    out_idx = [1, 2]
    nb_states = 5

    nb_data_sup = 10

    # plot font size
    font_size = 20

    # =====================================================
    ############# process data before prediction ##########
    # =====================================================
    x_list, y_list = cope_real_word_path(
        root_path='./data/font_data/' + write_name + '/',
        file_name='real_angle_list_',
        stroke_index=stroke_index,
        epi_times=5,
        delimiter=',',
        skiprows=1
    )
    x_index = signal.resample(np.arange(np.array(x_list).shape[1]), 100)
    
    print(np.arange(np.array(x_list).shape[1]), "x_list :", np.array(x_list).shape, "y_list :", np.array(y_list).shape, "x_index :", x_index)

    folder_name = './data/predicted_images/' + write_name
    if os.path.exists(folder_name):
        pass
    else:
        os.makedirs(folder_name)

    # down sampling ::::
    x_down_list = signal.resample(x_list, 200, axis=1)
    y_down_list = signal.resample(y_list, 200, axis=1)

    nb_data = x_down_list[0].shape[0]
    print("nb_data :", nb_data)

    # Create time data
    demos_t = [np.arange(x_down_list[i].shape[0])[:, None] for i in range(epi_times)]
    # print("demos_t :", demos_t)

    # Stack time and position data
    demos_tx = [np.hstack([demos_t[i] * dt, x_down_list[i][:, None], y_down_list[i][:, None]]) for i in
                range(epi_times)]
    # print("demos_tx :", demos_tx)

    # Stack demos
    demos_np = demos_tx[0]
    for i in range(1, epi_times):
        demos_np = np.vstack([demos_np, demos_tx[i]])

    X = demos_np[:, 0][:, None]
    Y = demos_np[:, 1:]

    plt.figure(figsize=(5, 5))
    for p in range(nb_samples):
        plt.plot(Y[p * nb_data:(p + 1) * nb_data, 0], Y[p * nb_data:(p + 1) * nb_data, 1], color=[.7, .7, .7])
        plt.plot(x_list[p], y_list[p], color=[0.20, 0.54, 0.93], linewidth=0.5)
    print("x_list :", x_list[p][0], "y_list :", y_list[p][1])
    axes = plt.gca()
    # axes.set_xlim([-14, 14.])
    # axes.set_ylim([-14., 14.])
    plt.xlabel('$y_1$', fontsize=30)
    plt.ylabel('$y_2$', fontsize=30)
    plt.locator_params(nbins=3)
    plt.tick_params(labelsize=20)
    plt.tight_layout()
    plt.show()
