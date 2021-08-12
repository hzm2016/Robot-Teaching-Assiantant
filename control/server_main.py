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

np.set_printoptions(precision=5) 

L_1 = 0.3 
L_2 = 0.25  
action_dim = 3
DIST_THREHOLD = 0.05  

# initial angle (rad) ::: 
Initial_angle = np.array([-1.31, 1.527])  

Initial_point = np.array([0.32299, -0.23264])  

Angle_initial = np.array([-0.316181, 0.433320, 1.948196])   

# impedance params :  
Move_Impedance_Params = np.array([40.0, 35.0, 4.0, 0.5])  


def reset_and_calibration(): 
    print("Please make sure two links are at zero position !!!")
    angle_initial = np.zeros(3) 
    
    angle_initial[0] = motor_control.read_initial_angle_1() 
    angle_initial[1] = motor_control.read_initial_angle_2() 
    angle_initial[2] = motor_control.read_initial_angle_3()
     
    return angle_initial  


def get_current_path():
    

    pass


def get_observation(angle_initial=np.array([-0.336998, 0.426342, 0.379417])): 
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
    angle[1] = motor_control.read_angle_2(angle_initial[1], angle[0].copy())  
    angle[2] = motor_control.read_angle_3(angle_initial[2])  
    print("Joint angles (rad) :", np.array(angle))  
    
    point[0] = L_1 * math.cos(angle[0]) + L_2 * math.cos(angle[0] + angle[1])
    point[1] = L_1 * math.sin(angle[0]) + L_2 * math.sin(angle[0] + angle[1])
    print("Position (m) :", np.array(point))  
    
    return angle, point   


def move_to_target_point(target_point, impedance_params, velocity=0.05):  
    """
        move to target point  
    """ 
    curr_angle, curr_point = get_observation() 
    # dist = np.linalg.norm((curr_point - target_point), ord=2) 
    # print("Curr_point (m) :", curr_point)  
    # print("Initial dist (m) :", dist)  

    angle_list, N = path_planning(curr_point, target_point, velocity=velocity) 
    # N = angle_list.shape[0]  

    # angle_array = ctypes.c_float * 5
    angle_1_list = angle_list[:, 0].copy() 
    angle_2_list = angle_list[:, 1].copy()  

    dist_threshold = 0.05 
    motor_control.move_to_target_point(impedance_params[0], impedance_params[1], impedance_params[2], impedance_params[3],  
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
        os.remove(r'angle_list.txt')
        data_file = open('angle_list.txt', 'w') 
    
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


def write_word(word_path, impedance_params=np.array([35.0, 25.0, 0.4, 0.1])): 
    
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
                     impedance_params=impedance_params,  
                     target_point=stroke_target_point)  

        motor_control.motor_3_stop() 


def write_stroke(stroke_points=None, impedance_params = np.array([35.0, 25.0, 0.4, 0.1]), 
    target_point=Initial_point): 

    # print("Write stroke !!!")  
    way_points = stroke_points  
    Num_way_points = way_points.shape[0]  
    # print("Num_way_points :", Num_way_points)  

    initial_angle = np.zeros(2)  
    initial_angle[0] = way_points[0, 0]  
    initial_angle[1] = way_points[0, 1]   
    start_point = forward_ik(initial_angle)    

    # move to target point
    set_pen_up()  
    move_to_target_point(start_point, Move_Impedance_Params, velocity=0.1)  

    set_pen_down()  
    time.sleep(0.4)  
    motor_control.run_one_loop(impedance_params[0], impedance_params[1], impedance_params[2], impedance_params[3], 
                                way_points[:, 0].copy(), way_points[:, 1].copy(), Num_way_points, 
                                Angle_initial[0], Angle_initial[1], 1)  
    
    time.sleep(0.4) 

    # move to target point 
    set_pen_up() 
    time.sleep(0.4) 

    move_to_target_point(target_point, Move_Impedance_Params, velocity=0.1)  

    print("Write stroke once done !!!")  
    print("*" * 50)  


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
    motor_control.set_position(0.0, up_angle)   


def set_pen_down():  
    """ 
        pull pen down  
    """ 
    # motor_control.motor_3_stop()
    down_angle = np.int32(11200)   
    motor_control.set_position(0.0, down_angle)   


def load_word_path(word_name=None):

    word_file = './data/font_data' + '/' + word_name + '/' 
    stroke_list_file = glob.glob(word_file + 'angle_list_*txt')
    print("Load stroke data %d", len(stroke_list_file)) 

    word_path = []
    for i in range(len(stroke_list_file)):
        way_points = np.loadtxt(word_file + 'angle_list_' + str(i) + '.txt', delimiter=' ')    
        N_way_points = way_points.shape[0]   
        print("N_way_points :", N_way_points)   
        word_path.append(way_points.copy())  

    # way_points = np.loadtxt('angle_list_0.txt', delimiter=' ')    
    # N_way_points = way_points.shape[0]   
    # print("N_way_points :", N_way_points)   
    # word_path.append(way_points.copy())  

    # way_points = np.loadtxt('angle_list_1.txt', delimiter=' ')    
    # N_way_points = way_points.shape[0]   
    # print("N_way_points :", N_way_points)   
    # word_path.append(way_points.copy())  

    # # way_points = np.loadtxt('angle_list_1_2.txt', delimiter=' ')    
    # # N_way_points = way_points.shape[0]   
    # # print("N_way_points :", N_way_points)
    # # word_path.append(way_points.copy()) 

    # way_points = np.loadtxt('angle_list_2.txt', delimiter=' ')    
    # N_way_points = way_points.shape[0]   
    # print("N_way_points :", N_way_points)  
    # word_path.append(way_points.copy()) 

    # way_points = np.loadtxt('angle_list_3.txt', delimiter=' ')    
    # N_way_points = way_points.shape[0]   
    # print("N_way_points :", N_way_points)  
    # word_path.append(way_points.copy()) 

    # way_points = np.loadtxt('angle_list_4.txt', delimiter=' ')    
    # N_way_points = way_points.shape[0]   
    # print("N_way_points :", N_way_points)  
    # word_path.append(way_points.copy()) 

    return word_path


if __name__ == "__main__":  

    word_path = load_word_path(word_name='yi') 

    write_word(word_path, impedance_params=np.array([35.0, 30.0, 0.4, 0.1]))  

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

    # motor_control.set_position(0.0, np.int32(1700)) 


    """ calibrate position for each start up """ 
    # Angle_initial = reset_and_calibration() 

    # motor_control.read_initial_angle_2() 

    # impedance_params = np.array([35.0, 10, 2.0, 0.1])  
    # move_to_target_point(np.array([0.34, -0.13]), impedance_params, velocity=0.05)  

    # angle, point = get_observation(angle_initial=Angle_initial)   

    # impedance_params = np.array([0.1, 0.1, 0, 0])  
    # motor_control.rotate_to_target(impedance_params[0], impedance_params[2], 3.14, 0.0, 0.05, 10)  

    # eval(stroke_angle, impedance_params=np.array([35.0, 30.0, 0.4, 0.1])) 
    # eval_writting() 

    # train() 
    # train(run_on=True, Load_path=True)  
    
    # plot_torque_path(
    #     root_path='',
    #     file_angle_name='real_angle_list.txt', 
    #     file_torque_name='real_torque_list.txt' 
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

    # fig = plt.figure(figsize=(20, 8))  
    # plt.subplot(1, 2, 1)  
    # plt.subplots_adjust(wspace=2, hspace=0)  
    
    # plt.plot(angle_1_list, linewidth=linewidth)
    # # plt.xlim([0, 128])
    # # plt.ylim([0, 128])
    # plt.xlabel('time($t$)')
    # plt.ylabel('$q_1$(rad)')
    # # plt.axis('equal')
    # plt.tight_layout()
    
    # plt.subplot(1, 2, 2)
    # plt.subplots_adjust(wspace=0.2, hspace=0.2)
    
    # plt.plot(angle_2_list, linewidth=linewidth)
    
    # # plt.xlim([0., 0.6])
    # # plt.ylim([0., 0.6])
    # plt.xlabel('time($t$)')
    # plt.ylabel('$q_2$(rad)')

    # plt.show() 
