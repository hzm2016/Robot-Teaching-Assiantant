# from matplotlib.pyplot import title
# from numpy.lib import NumpyVersion
from protocol.task_interface import *
import numpy as np
import math
import os
from motor_control import motor_control
from path_planning.plot_path import * 
from path_planning.path_generate import * 
import ctypes 
import time 
import glob 
import scipy

sns.set(font_scale=1.5)
np.set_printoptions(precision=5)

L_1 = 0.3
L_2 = 0.25
action_dim = 3
DIST_THREHOLD = 0.05

# initial angle (rad) ::: 
Initial_angle = np.array([-1.31, 1.527]) 

Initial_point = np.array([0.32299, -0.23264])  

Angle_initial = np.array([-0.344848, 0.253177, 1.981514])   

# impedance params :  
Move_Impedance_Params = np.array([40.0, 35.0, 4.0, 0.2])


def reset_and_calibration(): 
    print("Please make sure two links are at zero position !!!")
    angle_initial = np.zeros(3)
    
    angle_initial[0] = motor_control.read_initial_angle_1()  
    angle_initial[1] = motor_control.read_initial_angle_2()  
    angle_initial[2] = motor_control.read_initial_angle_3()  
     
    return angle_initial  


def get_demo_writting():  
    """ 
        zero impedance control
    """
    buff_size = np.zeros((100, 2))  
    impedance_params = np.array([35.0, 25.0, 0.4, 0.1]) 

    set_pen_down()  
    motor_control.get_demonstration(Angle_initial[0], Angle_initial[1], 
    2.0, 2.0, 0.0, 0.0, buff_size)  


def get_observation(angle_initial=Angle_initial): 
    """
        obtain joint angles and cartesian state 
    """
    # ###################################################### 
    # ############## get current state ##################### 
    # ###################################################### 
    angle = np.zeros(action_dim)  
    point = np.zeros(2)  
    
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


def move_to_target_point(target_point, impedance_params=Move_Impedance_Params, velocity=0.05):  
    """
        move to target point  
    """ 
    # done = False  

    curr_angle, curr_point = get_observation()  
    # dist = np.linalg.norm((curr_point - target_point), ord=2)  
    # print("Curr_point (m) :", curr_point)   
    # print("Initial dist (m) :", dist)  

    angle_list, N = path_planning(curr_point, target_point, velocity=velocity)  
    # angle_list = np.loadtxt('angle_list.txt', delimiter=',', skiprows=1)  

    N = angle_list.shape[0]

    # angle_array = ctypes.c_float * 5
    angle_1_list = angle_list[:, 0].copy()
    angle_2_list = angle_list[:, 1].copy()  

    dist_threshold = 0.05
    done = motor_control.move_to_target_point(impedance_params[0], impedance_params[1], impedance_params[2], impedance_params[3],  
        angle_1_list, angle_2_list, N,   
        Angle_initial[0], Angle_initial[1],   
        dist_threshold   
    ) 

    # while dist > DIST_THREHOLD: 
    #     motor_control.move_to_target(target_point) 
    #     dist = np.linalg.norm((curr_point - target_point), ord=2)  

    # curr_angle, curr_point = get_observation()  
    # final_dist = np.linalg.norm((curr_point - target_point), ord=2) 
    # print("Final dist (m) :", final_dist)  
    # done = True  
    # return done, final_dist  
    # return done


def train(angle_initial=Angle_initial, run_on=True, Load_path=False): 

    _server = Server(5005) 
    
    # ######################################################
    # ############## wait encoder and motor check ##########
    # ################### Position calibrate ###############
    # ######################################################
    _server.wait_encoder_request()  
    curr_angle, curr_point = get_observation(angle_initial)  
    _server.send_encoder_check(angle_initial)

    # move_to_target_point(Initial_angle)  

    # ######################################################
    # ############## Wait way_points #######################
    # ######################################################
    _server.wait_way_points_request() 

    print("+"*50) 
    # receive way points
    way_points = [] 

    if Load_path:
        os.remove(r'data/real_path_data/angle_list.txt')
        data_file = open('data/real_path_data/angle_list.txt', 'w')
    
    way_point = None
    while way_point != "SEND_DONE":
        way_point = _server.read_way_points() 
        # print("way_points ::::", way_point)
        if way_point == "SEND_DONE": 
            break
        way_points.append(way_point.copy())
        
        line_data = str(way_point[0]) + ',' + str(way_point[1]) + '\n'
        
        if Load_path:
            data_file.writelines(line_data) 
        # send_done = _server.wait_send_way_points_done()

    way_points = np.array(way_points) 
    N_way_points = way_points.shape[0] 
    # print("way_points :::", way_points.shape) 
    print("N_way_points :::", N_way_points) 
    print("+"*50) 

    # ######################################################
    # ############## Wait impedance parameters  ############
    # ######################################################
    _server.wait_params_request() 

    # impedance_params = None
    # while impedance_params is None:  
    # read impedance parameters :::
    impedance_params = _server.read_params()  
    impedance_params = np.array(impedance_params.copy())  
    print("Input impedance parameters :::", np.array(impedance_params))  
    print("+"*50)
    if impedance_params is np.NaN: 
        exit()

    time.sleep(2.0) 
    impedance_params = np.array([35.0, 24.0, 0.0, 0.0])
    
    # start move
    if not Load_path:
        # start_point = forward_ik(way_points[0, :].copy())
        # print("Move to start point :::", start_point) 
        way_points = np.loadtxt('angle_list.txt', delimiter=',')   
        N_way_points = way_points.shape[0]  

    if run_on: 
        initial_angle = np.zeros(2)
        initial_angle[0] = way_points[0, 0]
        initial_angle[1] = way_points[0, 1]
        start_point = forward_ik(initial_angle)
        move_impedance_params=np.array([20.0, 16.0, 0.1, 0.1])

        move_to_target_point(start_point, move_impedance_params, velocity=0.05)  

        motor_control.run_one_loop(impedance_params[0], impedance_params[1], impedance_params[2], impedance_params[3], 
                                    way_points[:, 0].copy(), way_points[:, 1].copy(), N_way_points, 
                                    Angle_initial[0], Angle_initial[1], 1)  
        
        time.sleep(2.0)
        move_to_target_point(Initial_point, move_impedance_params, velocity=0.05)  

    # send movement_done command 
    _server.send_movement_done()
    
    _server.close()


def write_word(word_path, word_params=None, word_name='yi', epi_times=0): 
    """
        write a word and plot :: 
    """
    for index in range(len(word_path)):  
        print("*" * 50)
        print("*" * 50)
        print("Write Stroke %d : "%index)  
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
        
        write_stroke(stroke_points=stroke_points_index,  
                     stroke_params=word_params[index],   
                     target_point=stroke_target_point,
                     word_name=word_name,
                     stroke_name=str(index),
                     epi_time=epi_times)  

        motor_control.motor_3_stop() 


def write_stroke(stroke_points=None, 
                stroke_params=None, 
                target_point=Initial_point, 
                word_name='yi', 
                stroke_name='0',
                epi_time=0): 

    # print("Write stroke !!!")  
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

    stroke_angle_name = './data/font_data/' + word_name + '/' + 'real_angle_list_' + stroke_name + '_' + str(epi_time) + '.txt' 
    stroke_torque_name = './data/font_data/' + word_name + '/' + 'real_torque_list_' + stroke_name + '_' + str(epi_time) + '.txt' 
    done = motor_control.run_one_loop(
                                    way_points[:, 0].copy(), way_points[:, 1].copy(),  
                                    params_list[:, 0].copy(), params_list[:, 1].copy(),  
                                    params_list[:, 2].copy(), params_list[:, 3].copy(),  
                                    Num_way_points,  
                                    Angle_initial[0], Angle_initial[1], 1, stroke_angle_name, stroke_torque_name) 
    # print("curr_path_list", curr_path_list.shape)  
    # np.savetxt('curr_path_list.txt', curr_path_list)
    
    # time.sleep(0.5) 

    # move to target point 
    done = set_pen_up()  
    # time.sleep(0.5)  

    done = move_to_target_point(target_point, Move_Impedance_Params, velocity=0.1)  

    print("Write stroke once done !!!")  
    print("*" * 50)  

    return done 


def eval_writting(run_on=True, Load_path=False): 
    """ 
        eval writting performance : 
    """

    _server = Server(5005) 
    
    # ######################################################
    # ############## wait encoder and motor check ##########
    # ################### Position calibrate ###############
    # ######################################################
    _server.wait_encoder_request()  
    curr_angle, curr_point = get_observation(Angle_initial)  
    _server.send_encoder_check(curr_point)  

    if not Load_path: 
        print("Load stroke path !!!") 
        stroke_angle = np.loadtxt('angle_list_0.txt', delimiter=' ')    
        # N_way_points = stroke_angle.shape[0]   
        # print("N_way_points :", N_way_points)

    # ######################################################
    # ############## Wait impedance parameters  ############
    # ###################################################### 
    _server.wait_params_request()  

    # impedance_params = None  
    # while impedance_params is None:   
    # read impedance parameters :::  
    while True:
        impedance_params = _server.read_params()  
        impedance_params = np.array(impedance_params.copy())  
        
        if impedance_params is np.NaN: 
            exit()

        if impedance_params is not None:
            break

    time.sleep(1.0) 
    # impedance_params = np.array([35.0, 24.0, 0.0, 0.0]) 
    print("Input impedance parameters :::", np.array(impedance_params))  
    print("+"*50)

    num_eval = 3 
    for i in range(num_eval):  
        print('Writting episode %d:'%i) 
        if run_on: 
            write_stroke(stroke_points=stroke_angle, impedance_params=np.array([35.0, 30.0, 1.4, 0.2]), target_point=Initial_point) 

            print("*" * 50) 
            print("Eval one stroke once done !!!") 
        
        # send movement_done command 
        _server.send_movement_done() 
    
    motor_control.motor_3_stop() 
    _server.close() 
     

def eval(stroke_angle, impedance_params=np.array([35.0, 30.0, 0.4, 0.1])):   
    """ 
        Write one stroke with given impedance  
        With previous path data 
    """ 
    way_points = stroke_angle
    N_way_points = way_points.shape[0]

    initial_angle = np.zeros(2)  
    initial_angle[0] = way_points[0, 0]  
    initial_angle[1] = way_points[0, 1]  
    start_point = forward_ik(initial_angle)  

    # move to target point
    set_pen_up()  
    time.sleep(0.3) 

    move_to_target_point(start_point, Move_Impedance_Params, velocity=0.05)  
    
    time.sleep(0.3)   
    set_pen_down()  

    motor_control.run_one_loop(impedance_params[0], impedance_params[1], impedance_params[2], impedance_params[3], 
                                    way_points[:, 0].copy(), way_points[:, 1].copy(), N_way_points, 
                                    Angle_initial[0], Angle_initial[1], 1) 

    time.sleep(0.3) 

    # move to target point
    set_pen_up() 
    time.sleep(0.3) 
    move_to_target_point(Initial_point, Move_Impedance_Params, velocity=0.05) 

    motor_control.motor_3_stop() 

    print("*" * 50) 
    print("Eval one stroke once done !!!") 


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


def load_word_path(root_path='./data/font_data', word_name=None, joint_params=None):

    word_file = root_path + '/' + word_name + '/'
    stroke_list_file = glob.glob(word_file + 'angle_list_*txt')
    print("Load stroke data %d", len(stroke_list_file)) 

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


if __name__ == "__main__":  
    # write_name = 'xing'       
    # eval_times = 5   

    # ===========================================================
    # motor_control.read_encoder_angles(2.516857, 4.574669)   
    
    # motor_control.read_initial_angle_1()  

    # motor_control.read_initial_angle_2()  

    # motor_control.read_angle_1(-0.392035)  

    # motor_control.read_angle_2(-0.500249, 0)  

    # motor_control.read_analog_encoder()  
    
    motor_control.phri_get_demonstration(-0.392035, -0.500249, 2.516857, 4.574669, 0.0, 0.0, 0.0, 0.0) 

    # plot_sea_angle_torque_path(
    #     root_path='./',
    #     file_angle_name='demonstrated_angle_list.txt', 
    #     file_torque_name='demonstrated_torque_list.txt' 
    # )

    # word_path, word_params, real_path = load_word_path(word_name=write_name, joint_params=np.array([45, 30, 5, 0.2])) 
    
    # for i in range(eval_times): 
    #     write_word(word_path, word_params=word_params, word_name=write_name, epi_times=i)  

    # plot_real_stroke_2d_path(
    #     root_path='./data/font_data/xing/', 
    #     file_name='angle_list_5', 
    #     stroke_num=5, 
    #     delimiter=' ',
    #     skiprows=1
    # )

    # plot_real_2d_path(
    #     root_path='./data/font_data/' + write_name + '/', 
    #     file_name='real_angle_list_', 
    #     stroke_num=6, 
    #     delimiter=' ', 
    #     skiprows=1
    # )

    # plot_real_error_path(
    #     root_path='./data/font_data/' + write_name + '/', 
    #     file_name='real_angle_list_',  
    #     stroke_num=6,  
    #     epi_num=5, 
    #     delimiter=' ', 
    #     skiprows=1 
    # )

    # plot_real_2d_demo_path(
    # root_path='',
    # file_name=write_name,
    # delimiter=',',
    # skiprows=1 
    # )

    # torque_list = np.loadtxt('./data/font_data/xing/real_angle_list_5.txt', delimiter=' ', skiprows=1)

    # fig = plt.figure(figsize=(20, 8))  
    # plt.subplot(1, 2, 1)  
    # plt.subplots_adjust(wspace=2, hspace=0)  
    
    # plt.plot(torque_list[:, 0], linewidth=linewidth, label='Input')  
    # plt.plot(torque_list[:, 1], linewidth=linewidth, label='Output')  
    # plt.plot(torque_list[:, 2], linewidth=linewidth, label='vel_1')  
    # # plt.xlim([0, 128])  
    # # plt.ylim([0, 128])  
    # plt.xlabel('time($t$)')    
    # plt.ylabel('$q_1$(rad)')    
    # # plt.axis('equal') 
    # plt.legend() 
    
    # plt.subplot(1, 2, 2)
    # plt.subplots_adjust(wspace=0.2, hspace=0.2)
    
    # plt.plot(torque_list[:, 3], linewidth=linewidth, label='Input')  
    # plt.plot(torque_list[:, 4], linewidth=linewidth, label='Output')  
    # plt.plot(torque_list[:, 5], linewidth=linewidth, label='vel_2')  
    
    # # plt.xlim([0., 0.6])
    # # plt.ylim([0., 0.6])
    # plt.xlabel('time($t$)') 
    # plt.ylabel('$q_2$(rad)')   
    # plt.legend()

    # plt.tight_layout() 
    # plt.show() 

    # angle_list = np.loadtxt('./data/font_data/yi/real_angle_list_0.txt', delimiter=' ', skiprows=1)  
    # # angle_list = np.loadtxt('chuan_demonstrated_angle_list.txt', delimiter=',', skiprows=1)  

    # fig = plt.figure(figsize=(20, 8))  
    # plt.subplot(1, 2, 1)  
    # plt.subplots_adjust(wspace=2, hspace=0)  
    
    # plt.plot(angle_list[:, 0], linewidth=linewidth, label='Input')  
    # plt.plot(angle_list[:, 1], linewidth=linewidth, label='Output')  
    # # plt.xlim([0, 128])  
    # # plt.ylim([0, 128])  
    # plt.xlabel('time($t$)')    
    # plt.ylabel('$q_1$(rad)')    
    # # plt.axis('equal') 
    # plt.legend()   
    
    # plt.subplot(1, 2, 2)
    # plt.subplots_adjust(wspace=0.2, hspace=0.2)
    
    # plt.plot(angle_list[:, 1], linewidth=linewidth, label='Input')  
    # plt.plot(angle_list[:, 3], linewidth=linewidth, label='Output')  
    
    # # plt.xlim([0., 0.6])
    # # plt.ylim([0., 0.6])
    # plt.xlabel('time($t$)') 
    # plt.ylabel('$q_2$(rad)')   
    # plt.legend()

    # plt.tight_layout() 
    # plt.show() 

    # motor_stop() 

    # get_demo_writting()  

    # motor_control.Jacobian(0.0, 0.0) 

    """ calibrate position for each start up """ 
    # Angle_initial = reset_and_calibration()

    # angle, point = get_observation(angle_initial=Angle_initial)
    
    # impedance_params = np.array([45.0, 30, 6.8, 0.1])  
    # move_to_target_point(np.array([0.34, -0.23]), impedance_params, velocity=0.04)  

    # set_pen_up()

    # motor_control.motor_3_stop() 

    # write_stroke(stroke_points=way_points, 
    # impedance_params=np.array([35.0, 30.0, 0.4, 0.1]), 
    # target_point=Initial_point) 

    # print("Load stroke path !!!") 
    # way_points = np.loadtxt('angle_list_4.txt', delimiter=' ')    
    # print(way_points[:, 1].copy())  
    # N_way_points = way_points.shape[0]   
    # print("N_way_points :", N_way_points)   

    # set_pen_up()   

    # set_pen_down()   
    # motor_control.motor_3_stop() 

    # motor_control.read_angle_3(0.0)   

    # motor_control.set_position(0.0, np.int32(500))   

    # impedance_params = np.array([35.0, 35, 0.8, 0.1])  
    # move_to_target_point(np.array([0.34, -0.03]), impedance_params, velocity=0.04)  
    # T = 10
    # Ts = 0.001 
    # N = int(T/Ts) 
    # print('N :', N) 
    # t = np.linspace(0, T, N) 
    # omega = 0.01 
    # angle_1_list = 0.5 * np.sin(2 * math.pi /T * t) 
    # angle_2_list = 0.5 * np.sin(2 * math.pi /T * t) 
    # impedance_params=np.array([40.0, 40.0, 0.4, 0.4]) 

    # motor_control.control_single_motor(impedance_params[0], impedance_params[1], impedance_params[2], impedance_params[3],  
    # angle_1_list, angle_2_list, N, Angle_initial[0], Angle_initial[1], 0.05) 

    # plot_real_2d_path(
    #     root_path='',
    #     file_name='demonstrated_angle_list.txt',
    #     delimiter=',',
    #     skiprows=1
    # )  

    # motor_control.set_two_link_position(Angle_initial[0], Angle_initial[1], 0000, 0000)

    # motor_control.motor_two_link_stop()  
    
    # get_demo_writting()  

    # motor_control.read_initial_angle_2()  

    # eval(stroke_angle, impedance_params=np.array([35.0, 30.0, 0.4, 0.1])) 
    # eval_writting() 

    # train() 
    # train(run_on=True, Load_path=True)  
    
    # plot_torque_path(
    #     root_path='',
    #     file_angle_name='move_target_angle_list.txt', 
    #     file_torque_name='move_target_torque_list.txt' 
    # )

    #################################################################################
    # plot_real_2d_path(
    #     root_path='',
    #     file_name='demonstrated_angle_list.txt' 
    # )

    # print("curr_angle :", angle)   
    # print("curr_point :", point)   

    # train(angle_initial=Angle_initial, run_on=True)        

    # motor_control.run_one_loop(impedance_params[0], impedance_params[1], impedance_params[2], impedance_params[3],
    #                                Angle_initial[0], Angle_initial[1], N_way_points)  

    # motor_control.get_demonstration(Angle_initial[0], Angle_initial[1]) 

    # angle_list = path_planning(np.array([0.34, -0.0]), np.array([0.34, -0.13]), T=3.0) 
    # N = angle_list.shape[0]  

    # way_points = np.loadtxt('real_angle_list.txt', delimiter=',', skiprows=1)   

    # angle_1_list = way_points[:, 0]  
    # angle_2_list = way_points[:, 1]  
