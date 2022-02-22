import argparse
import os
from motor_control import motor_control
import time
import numpy as np

# path prediction
from utils.word_preprocess import * 

L_1 = 0.3
L_2 = 0.25
action_dim = 3
DIST_THREHOLD = 0.05

FILE_FIG_NAME = './data/predicted_images/'
FILE_FONT_NAME = './data/font_data'
FILE_TRAIN_NAME = './data/training_data'
FILE_EVAL_NAME = './data/real_path_data'

ANGLE_1_RANGE = np.array([-1.90, 1.90])
ANGLE_2_RANGE = np.array([-2.2, 2.5])
center_shift = np.array([0.15, -WIDTH / 2])

FONT_SIZE = 20

WRITING_Y = [-WIDTH / 2, WIDTH / 2]
WRITING_X = [0.13, 0.13 + WIDTH]

# initial angle (rad) :::
Initial_angle = np.array([-1.31, 1.527])

Initial_point = np.array([0.32299, -0.23264])

Angle_initial = np.array([-0.294084, -0.126821, 1.981514]) 

# impedance params :::
Move_Impedance_Params = np.array([30.0, 30.0, 4.0, 0.2])


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


def motor_stop():
    """
        sometimes need to stop motors
    """
    motor_control.motor_two_link_stop()
    motor_control.motor_3_stop()


def get_observation(angle_initial=Angle_initial):
    """
        obtain joint angles and cartesian state
    """
    # ######################################################
    # ############## get current state #####################
    # ######################################################
    angle = np.zeros(action_dim)
    point = np.zeros(action_dim)
    
    print('+' * 20)
    angle[0] = motor_control.read_angle_1(angle_initial[0])
    # print("Joint 1 angles (rad) :", angle[0])
    angle[1] = motor_control.read_angle_2(angle_initial[1], angle[0].copy())
    # print("Joint 2 angles (rad) :", angle[1])
    # angle[2] = motor_control.read_angle_3(angle_initial[2])
    print("Joint angles (rad) :", np.array(angle))
    
    point[0] = L_1 * math.cos(angle[0]) + L_2 * math.cos(angle[0] + angle[1])
    point[1] = L_1 * math.sin(angle[0]) + L_2 * math.sin(angle[0] + angle[1])
    print("Position (m) :", np.array(point))
    
    return angle, point


def reset_and_calibration():
    print("Please make sure two links are at zero position !!!")
    angle_initial = np.zeros(3)
    
    angle_initial[0] = motor_control.read_initial_angle_1()
    angle_initial[1] = motor_control.read_initial_angle_2()
    angle_initial[2] = motor_control.read_initial_angle_3()
    
    return angle_initial


def load_word_path(
    root_path='./data/font_data',
    word_name=None,
    task_params=None,
    joint_params=None
):
    # load original training way points :::
    word_file = root_path + '/' + word_name + '/'
    stroke_list_file = glob.glob(word_file + 'angle_list_*txt')
    print("Load stroke data %d", len(stroke_list_file))

    word_path = []
    word_joint_params = []
    word_task_params = []
    
    for i in range(len(stroke_list_file)):
        way_points = np.loadtxt(word_file + 'angle_list_' + str(i) + '.txt', delimiter=' ')

        if joint_params is not None:
            joint_params_list = np.tile(joint_params, (way_points.shape[0], 1))
        else:
            joint_params_list = np.loadtxt(word_file + 'params_list_' + str(i) + '.txt', delimiter=' ')
         
        N_way_points = way_points.shape[0]
        print("N_way_points :", N_way_points)

        word_path.append(way_points.copy())
        word_joint_params.append(joint_params_list.copy())
    
        # word parameters
        if task_params is not None:
            task_params_list = np.tile(task_params, (way_points.shape[0], 1))
        else:
            task_params_list = np.loadtxt(word_file + 'params_list_' + str(i) + '.txt', delimiter=' ')

        angle_list = way_points
        stiffness_list = task_params_list[:, :2]
        damping_list = task_params_list[:, 2:]
        joint_params_list = generate_stroke_stiffness_path(
            angle_list, stiffness_list, damping_list,
            save_path=False, save_root='', word_name='yi', stroke_index=0
        )
        
        word_task_params.append(joint_params_list)

    return word_path, word_joint_params, word_task_params



def load_impedance_list(
    word_name='mu', 
    stroke_index=0, 
    desired_angle_list=None, 
    current_angle_list=None, 
    joint_params=None,  
    task_params=None  
):
    print("============== {} ============".format('Load Impedance !!!'))
    way_points = desired_angle_list  
    N_way_points = way_points.shape[0]  
    
    # joint parameters
    if joint_params is not None:
        joint_params_list = np.tile(joint_params, (way_points.shape[0], 1))
    else:  
        joint_params_list = np.loadtxt(FILE_TRAIN_NAME + '/' + word_name + '/' + 'params_stroke_list_' + str(stroke_index) + '.txt', delimiter=' ')
    
    # task parameters
    if task_params is not None:
        task_params_list = np.tile(task_params, (way_points.shape[0], 1))
    else:
        task_params_list = np.loadtxt(FILE_TRAIN_NAME + '/' + word_name + '/' + 'params_stroke_list_' + str(stroke_index) + '.txt', delimiter=' ')
    
    # stiffness_list = task_params_list[:, :2] 
    # damping_list = task_params_list[:, 2:] 
    # joint_params_list = generate_stroke_stiffness_path(
    #     desired_angle_list, stiffness_list, damping_list,
    #     save_path=False, save_root=FILE_TRAIN_NAME, word_name=word_name, stroke_index=stroke_index
    # )
    
    return joint_params_list


def get_demo_writting():
    """
        zero impedance control
    """
    buff_size = np.zeros((100, 2))
    impedance_params = np.array([35.0, 25.0, 0.4, 0.1])

    set_pen_down()
    motor_control.get_demonstration(Angle_initial[0], Angle_initial[1],
    2.0, 2.0, 0.0, 0.0, buff_size)
