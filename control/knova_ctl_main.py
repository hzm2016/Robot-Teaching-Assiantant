from matplotlib.pyplot import title
from numpy.lib import NumpyVersion
import numpy as np
import math
import os
from path_planning.plot_path import *
from path_planning.path_generate import *
import time
import glob
import scipy

sns.set(font_scale=1.5)
np.set_printoptions(precision=5)

#################################################################
########################## Hyper Parameters #####################
#################################################################
Move_Impedance_Params = np.array([40.0, 35.0, 4.0, 0.2])
Move_velocity = 0.1
Initial_point = np.array([0.0, 0.0])


def write_word(word_path, word_params=None, word_name='yi', epi_times=0):
    """
        Write a word and plot :::::
    """
    for index in range(len(word_path)):
        print("*" * 50)
        print("*" * 50)
        print("Write Stroke %d : " % index)
        stroke_points = word_path[index]

        if index < (len(word_path) - 1):
            stroke_points_next = word_path[index + 1]
            stroke_target_point = np.zeros(2)
            stroke_target_point[0] = stroke_points_next[0, 0]
            stroke_target_point[1] = stroke_points_next[0, 1]
        else:
            stroke_target_point = Initial_point

        # write a stroke
        write_stroke(
            stroke_points=stroke_points,
            stroke_params=word_params[index],
            target_point=stroke_target_point,
            epi_time=epi_times,
            word_name=word_name,
            stroke_name=str(index)
        )


def write_stroke(stroke_points=None,
                 stroke_params=None,
                 target_point=None,  # next initial point
                 epi_time=1,
                 word_name='yi',
                 stroke_name='0'
                 ):
    print("Write stroke !!!%s", word_name + '_' + str(stroke_name))
    done = False
    velocity_list = np.zeros_like(stroke_points)

    # params_list = np.tile(impedance_params, (Num_stroke_points, 1))
    if stroke_params is None:
        print("Please give impedance parameters !!!")
        exit()

    start_point = np.zeros(2)
    start_point[0] = stroke_points[0, 0]
    start_point[1] = stroke_points[0, 1]

    done = set_pen_up()
    # time.sleep(0.5)

    # move to target point
    done = move_to_target_point(start_point, Move_Impedance_Params, velocity=0.1)
    # time.sleep(0.5)

    done = set_pen_down()
    # time.sleep(0.5)

    # assistive control for one loop
    done = run_one_loop(
        stroke_points,
        stroke_params,
        velocity_list,
        epi_time,
        word_name,
        stroke_name
    )

    done = set_pen_up()
    # time.sleep(0.5)

    # move to target point
    done = move_to_target_point(target_point, Move_Impedance_Params, velocity=0.1)

    print("Write stroke once done !!!")
    print("*" * 50)

    return done


def set_pen_up():
    """
        pull pen up
    """
    done = False
    time.sleep(1.0)
    return done


def set_pen_down():
    """
        pull pen down
    """
    done = False
    time.sleep(2.0)
    return done


def move_to_target_point(desired_point, desired_impedance_params, velocity=0.1):
    """
        move to desired point
    """
    done = False
    return done


def run_one_loop(stroke_points, params_list, velocity_list,
                 eval_epi_times=1,
                 word_name='yi',
                 stroke_name='0'
                 ):
    """
        run one loop
    """
    done = False

    for epi_time in range(eval_epi_times):
        stroke_angle_name = './data/real_path_data/' + word_name + '/' + 'real_angle_list_' + stroke_name + '_' \
                            + str(epi_time) + '.txt'
        stroke_torque_name = './data/real_path_data/' + word_name + '/' + 'real_torque_list_' + stroke_name + '_' \
                             + str(epi_time) + '.txt'

        # send waypoint

    return done


def load_word_path(root_path='./data/font_data', word_name=None, writing_params=None):
    word_file = root_path + '/' + word_name + '/'
    stroke_list_file = glob.glob(word_file + 'angle_list_*txt')
    print("Load Stroke Data :", len(stroke_list_file))

    word_path = []
    word_params = []
    for i in range(len(stroke_list_file)):
        way_points = np.loadtxt(word_file + 'angle_list_' + str(i) + '.txt', delimiter=' ')

        if writing_params is not None:
            params_list = np.tile(writing_params, (way_points.shape[0], 1))
        else:
            params_list = np.loadtxt(word_file + 'params_list_' + str(i) + '.txt', delimiter=' ')

        N_way_points = way_points.shape[0]
        print("N_way_points :", N_way_points)
        word_path.append(way_points.copy())
        word_params.append(params_list.copy())
    return word_path, word_params


def load_real_path(root_path='./data/font_data', word_name=None):
    word_file = root_path + '/' + word_name + '/'
    stroke_list_file = glob.glob(word_file + 'real_angle_list_*txt')
    print("Load Stroke Data :", len(stroke_list_file))
    real_path = []
    for i in range(len(stroke_list_file)):
        real_way_points = np.loadtxt(root_path + 'real_angle_list_' + str(i) + '.txt', delimiter=' ', skiprows=1)
        real_path.append(real_way_points)

        # truth_data = scipy.signal.resample(real_way_points, 100)
        # down_sample = scipy.signal.resample(real_way_points, 100)
        # down_sample = real_way_points

        # idx = np.arange(0, down_sample.shape[0], down_sample.shape[0]/100).astype('int64')
        # real_angle_list = np.zeros((idx.shape[0], 2))
        # desired_angle_list = np.zeros((idx.shape[0], 2))
        # real_angle_list[:, 0] = down_sample[idx, 1]
        # real_angle_list[:, 1] = down_sample[idx, 4]
        # real_angle_list[:, 0] = scipy.signal.resample(down_sample[:, 1], 100)
        # real_angle_list[:, 1] = scipy.signal.resample(down_sample[:, 4], 100)

        # real_2d_path = forward_ik_path(real_angle_list)
        # print(real_2d_path)
        # real_path.append(real_2d_path.copy())
        # np.array(real_path).reshape(np.array(real_path).shape[0] * np.array(real_path).shape[1], 2)

    return np.array(real_path)


if __name__ == "__main__":
    # word_path, word_params = load_word_path(
    #     root_path='./data/font_data',
    #     word_name='mu', joint_params=np.array([1.0, 1.0])
    # )

    x_list, y_list = plot_real_word_2d_path(
        root_path='./data/font_data',
        word_name='mu',
        stroke_num=4,
        delimiter=' ',
        skiprows=0,
    )

    osc_data_list = transform_task_space(x_list, y_list, offset=np.array([0.0, 0.0]))

    plot_real_osc_2d_path(
        osc_data_list
    )
