from matplotlib.pyplot import title
from numpy.lib import NumpyVersion
from protocol.task_interface import *
import numpy as np
import math
import os
# from motor_control import motor_control
from path_planning.plot_path import *
from path_planning.path_generate import *
import ctypes
import time
import glob
import scipy

sns.set(font_scale=1.5)
np.set_printoptions(precision=5)

#################################################################
########################## Hyper Parameters #####################
#################################################################

# initial angle (rad) :::
Initial_angle = np.array([-1.31, 1.527])

Initial_point = np.array([0.32299, -0.23264])

Angle_initial = np.array([-0.344848, 0.253177, 1.981514])

# impedance params :
Move_Impedance_Params = np.array([40.0, 35.0, 4.0, 0.2])
Move_velocity = 0.1


def load_word_path(root_path='./data/font_data', word_name=None, joint_params=None):
    word_file = root_path + '/' + word_name + '/'
    stroke_list_file = glob.glob(word_file + 'angle_list_*txt')
    print("Load Stroke Data :", len(stroke_list_file))

    word_path = []
    word_params = []
    real_path = []
    for i in range(len(stroke_list_file)):
        way_points = np.loadtxt(word_file + 'angle_list_' + str(i) + '.txt', delimiter=' ')

        # real_way_points = np.loadtxt(word_file + 'real_angle_list_' + str(i) + '.txt', delimiter=' ', skiprows=1)

        if joint_params is not None:
            params_list = np.tile(joint_params, (way_points.shape[0], 1))
        else:
            params_list = np.loadtxt(word_file + 'params_list_' + str(i) + '.txt', delimiter=' ')

        N_way_points = way_points.shape[0]
        print("N_way_points :", N_way_points)
        word_path.append(way_points.copy())
        word_params.append(params_list.copy())

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

    return word_path, word_params, np.array(real_path)


def write_word(word_path, word_params=None, word_name='yi', epi_times=0):
    """
        Write a word and plot :::::
    """
    for index in range(len(word_path)):
        print("*" * 50)
        print("*" * 50)
        print("Write Stroke %d : " % index)
        stroke_points_index = word_path[index]

        if index < (len(word_path) - 1):
            next_index = index + 1
            stroke_points_next_index = word_path[next_index]

            # get target point of next
            target_angle = np.zeros(2)
            target_angle[0] = stroke_points_next_index[0, 0]
            target_angle[1] = stroke_points_next_index[0, 1]
            stroke_target_point = forward_ik(target_angle)
        else:
            stroke_target_point = Initial_point

        write_stroke(
            stroke_points=stroke_points_index,
            stroke_params=word_params[index],
            target_point=stroke_target_point,
            word_name=word_name,
            stroke_name=str(index),
            epi_time=epi_times
        )

        motor_control.motor_3_stop()


def write_stroke(stroke_points=None,
                 stroke_params=None,
                 target_point=Initial_point,
                 word_name='yi',
                 stroke_name='0',
                 epi_time=0):
    ### rewrite stroke ###
    # print("Write stroke !!!")
    done = False

    way_points = stroke_points
    Num_way_points = way_points.shape[0]
    # print("Num_way_points :", Num_way_points)

    initial_angle = np.zeros(2)
    initial_angle[0] = way_points[0, 0]
    initial_angle[1] = way_points[0, 1]
    start_point = forward_ik(initial_angle)

    # move to target point
    done = set_pen_up()
    # time.sleep(0.5)

    done = move_to_target_point(start_point, Move_Impedance_Params, velocity=0.1)
    # time.sleep(0.5)

    done = set_pen_down()
    # time.sleep(0.5)

    # params_list = np.tile(impedance_params, (Num_way_points, 1))
    if stroke_params is None:
        exit()
    else:
        params_list = stroke_params

    stroke_angle_name = './data/real_path_data/' + word_name + '/' + 'real_angle_list_' + stroke_name + '_' + str(
        epi_time) + '.txt'
    stroke_torque_name = './data/real_path_data/' + word_name + '/' + 'real_torque_list_' + stroke_name + '_' + str(
        epi_time) + '.txt'

    # assistive control for one loop
    done = motor_control.run_one_loop(
        way_points[:, 0].copy(), way_points[:, 1].copy(),
        params_list[:, 0].copy(), params_list[:, 1].copy(),
        params_list[:, 2].copy(), params_list[:, 3].copy(),
        Num_way_points,
        Angle_initial[0], Angle_initial[1], 1,
        stroke_angle_name, stroke_torque_name
    )

    # time.sleep(0.5)

    # move to target point
    done = set_pen_up()
    # time.sleep(0.5)

    done = move_to_target_point(target_point, Move_Impedance_Params, velocity=0.1)

    print("Write stroke once done !!!")
    print("*" * 50)

    return done


def set_pen_up():
    """
        pull pen up
    """
    # motor_control.motor_3_stop()
    up_angle = np.int32(9000)
    done = motor_control.set_position(0.0, up_angle)
    time.sleep(1.0)

    return done


def set_pen_down():
    """
        pull pen down
    """
    # motor_control.motor_3_stop()
    down_angle = np.int32(11200)
    done = motor_control.set_position(0.0, down_angle)
    time.sleep(2.0)

    return done


if __name__ == "__main__":
    load_word_path(root_path='./data/font_data', word_name='mu', joint_params=np.array([1.0, 1.0]))