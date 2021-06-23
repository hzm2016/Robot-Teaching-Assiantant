import numpy as np
import matplotlib.pyplot as plt
import math
import copy


def initial_parameter_estimate(num_demons_each_style=30):
    """ demonstration for each styles with zero impedance """
    num_demonstrations = 30

    writting_vel = 0.0
    impedance_params = np.array([])

    images_list = []
    for i in range(num_demonstrations):
        images_list.append(i.copy())

    return writting_vel, impedance_params


def generate_path(font, style, period=10, impedance_params=[], training_episodes=10):

    score = 0.0
    return score


def check_path(root_path='', font_name='J_font', type=2, period=10, Ts=0.001):
    angle_1_range = np.array([-np.pi/2, np.pi/2])
    angle_2_range = np.array([-np.pi * 3/4, np.pi * 3/4])

    plt.rcParams['font.size'] = 16
    linewidth = 3
    N = np.array(period/Ts).astype(int)
    path_data = np.loadtxt(root_path + '/' + font_name + '/2_font_' + str(type) +'.txt')
    print("path_data :::", path_data[:, 0])
    print("path_data :::", path_data[:, 1])

    # need to check which dimension can be applied for interp
    x_list = np.linspace(path_data[-1, 1], path_data[0, 1], N)
    # x_list = path(1, 2):(path(end, 2) - path(1, 2)) / (N - 1): path(end, 2)
    y_list = np.interp(x_list, path_data[:, 1][::-1], path_data[:, 0][::-1])
    print("x_list :::", x_list)
    print("y_list :::", y_list)

    # transform to Catersian space
    ratio = 128/0.6

    center_shift = np.array([0.0, 0.0])

    x_1_list = x_list / ratio + center_shift[0]
    x_2_list = y_list / ratio + center_shift[1]

    angle_1_list_e = []
    angle_2_list_e = []

    Length = [0.30, 0.150, 0.25, 0.125]
    L1 = Length[0]
    L2 = Length[2]

    x_1_list = np.hstack([x_1_list, x_1_list[::-1]])
    x_2_list = np.hstack([x_2_list, x_2_list[::-1]])

    print("x_1_list :::", x_1_list)
    print("x_2_list :::", x_2_list)
    t_list = np.linspace(0.0, 2*period, 2*N)
    for t in range(1, 2 * N):
        x1 = x_1_list[t]
        x2 = x_2_list[t]

        # Inverse kinematics
        L = x1**2 + x2**2

        gamma = math.atan2(x2, x1)

        cos_belta = (L1**2 + L - L2**2) / (2 * L1 * np.sqrt(L))

        if cos_belta > 1:
            angle_1 = gamma
        elif cos_belta < -1:
            angle_1 = gamma - np.pi
        else:
            angle_1 = gamma - math.acos(cos_belta)

        angle_1_list_e.append(angle_1)

        cos_alpha = (L1**2 - L + L2**2) / (2 * L1 * L2)

        if cos_alpha > 1:
            angle_2 = np.pi
        elif cos_alpha < -1:
            angle_2 = 0
        else:
            angle_2 = np.pi - math.acos(cos_alpha)

        angle_2_list_e.append(angle_2)

    max_angle_1 = np.max(angle_1_list_e)
    max_angle_2 = np.max(angle_2_list_e)
    if max_angle_1 < angle_1_range[0] or max_angle_1 > angle_1_range[1]:
        print("!!!!!! angle 1 is out of range !!!!!")
        print("max angle 1 :::", max_angle_1)
    if max_angle_2 < angle_2_range[0] or max_angle_2 > angle_2_range[1]:
        print("!!!!!! angle 1 is out of range !!!!!")
        print("max angle 2 :::", max_angle_2)

    fig = plt.figure(figsize=(15, 4))
    plt.subplot(1, 3, 1)
    plt.subplots_adjust(wspace=2, hspace=0)

    plt.plot(path_data[:, 1], path_data[:, 0], marker='o', linewidth=linewidth)
    plt.plot(x_list, y_list, linewidth=linewidth - 2)
    plt.xlim([0, 128])
    plt.ylim([0, 128])
    plt.xlabel('$x_1$')
    plt.ylabel('$x_2$')
    # plt.axis('equal')
    plt.tight_layout()

    plt.subplot(1, 3, 2)
    plt.subplots_adjust(wspace=0.2, hspace=0.2)

    plt.plot(x_1_list, x_2_list, linewidth=linewidth + 2, color='r')

    plt.xlim([0., 0.6])
    plt.ylim([0., 0.6])
    plt.xlabel('$x_1$(m)')
    plt.ylabel('$x_2$(m)')

    plt.subplot(1, 3, 3)
    plt.plot(t_list[1:], angle_1_list_e, linewidth=linewidth, label='$q_1$')
    plt.plot(t_list[1:], angle_2_list_e, linewidth=linewidth, label='$q_2$')
    plt.xlabel('Time (s)')
    plt.ylabel('One-loop Angle (rad)')
    plt.legend()

    plt.show()

    return angle_1_list_e, angle_2_list_e


check_path(root_path='data', font_name='third', type=3)
