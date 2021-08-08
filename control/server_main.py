from protocol.task_interface import *
import numpy as np
import math
import os
from motor_control import motor_control
from path_planning.plot_path import * 
from path_planning.path_generate import * 
import ctypes 
import time

np.set_printoptions(precision=5) 

L_1 = 0.3 
L_2 = 0.25  
action_dim = 3
DIST_THREHOLD = 0.05  

# initial angle (rad) ::: 
Initial_angle = np.array([-1.31, 1.527])  

Initial_point = np.array([0.32299, -0.23264])  

Angle_initial = np.array([-0.344673, 0.446810, 0.379417])   


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
    
    angle[0] = motor_control.read_angle_1(angle_initial[0])  
    angle[1] = motor_control.read_angle_2(angle_initial[1], angle[0].copy())  
    angle[2] = motor_control.read_angle_3(angle_initial[2])  
    print("Joint angles (rad) :", np.array(angle))  
    
    point[0] = L_1 * math.cos(angle[0]) + L_2 * math.cos(angle[0] + angle[1])
    point[1] = L_1 * math.sin(angle[0]) + L_2 * math.sin(angle[0] + angle[1])
    print("Position (m) :", np.array(point))  
    
    return angle, point   


def move_to_target_point(target_point, impedance_params, velocity=0.04):  
    """
        move to target point  
    """ 
    curr_angle, curr_point = get_observation() 
    dist = np.linalg.norm((curr_point - target_point), ord=2) 
    # print("Curr_point (m) :", curr_point)  
    print("Initial dist (m) :", dist)  

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


# def set_pen_up():


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


def eval(impedance_params = np.array([35.0, 25.0, 0.4, 0.1])):  
    """
        With zero impedance and get real-time traj
    """
    way_points = np.loadtxt('angle_list.txt', delimiter=',')   
    N_way_points = way_points.shape[0]   

    print("N_way_points :", N_way_points) 

    initial_angle = np.zeros(2)
    initial_angle[0] = way_points[0, 0]
    initial_angle[1] = way_points[0, 1]
    start_point = forward_ik(initial_angle)  

    move_to_target_point(start_point, impedance_params, velocity=0.05)  

    motor_control.run_one_loop(impedance_params[0], impedance_params[1], impedance_params[2], impedance_params[3], 
                                    way_points[:, 0].copy(), way_points[:, 1].copy(), N_way_points, 
                                    Angle_initial[0], Angle_initial[1], 1)  
    
    time.sleep(1.0)
    move_impedance_params=np.array([20.0, 16.0, 0.1, 0.1])
    move_to_target_point(Initial_point, impedance_params, velocity=0.05) 
    # buff_size = 10000  
    # demo_data = np.zeros((buff_size, 2))   
    # result = motor_control.get_demonstration(Angle_initial[0], Angle_initial[1], demo_data)  
    # print("result :", result) 
    print("*" * 50) 
    print("Eval once done !!!") 


if __name__ == "__main__":  

    # motor_control.read_angle_3(0.0) 
    # motor_control.motor_3_stop()

    # motor_control.set_position(0.0, np.int32(200)) 

    # """ calibrate position for each start up """ 
    # Angle_initial = reset_and_calibration() 
    # motor_control.read_initial_angle_2() 

    # impedance_params = np.array([35.0, 35.0, 2.0, 0.2])  
    # move_to_target_point(np.array([0.34, -0.03]), impedance_params, velocity=0.05)  

    # angle, point = get_observation(angle_initial=Angle_initial)  

    eval()  

    # train() 
    # train(run_on=True, Load_path=True)  
    
    # plot_torque_path(
    #     root_path='',
    #     file_angle_name='angle_list.txt', 
    #     file_torque_name='real_angle_list.txt' 
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

    # way_points = np.loadtxt('angle_list.txt', delimiter=',', skiprows=1)   

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
