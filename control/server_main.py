from protocol.task_interface import *
import numpy as np
import math
import os
from motor_control import motor_control
from path_planning.plot_path import * 

L_1 = 300  
L_2 = 250  
action_dim = 2  
DIST_THREHOLD = 0.05  


# initial angle (rad) ::: 
Initial_angle = np.array([-1.31, 1.527])  

Initial_point = np.array([0.32, -0.2377])  


def reset_and_calibration(): 
    print("Please make sure two links are at zero position !!!")
    angle_initial = np.zeros(action_dim)
    
    angle_initial[0] = motor_control.read_initial_angle_1() 
    angle_initial[1] = motor_control.read_initial_angle_2() 
    print("theta_1_initial :::", angle_initial[0])
    print("theta_2_initial :::", angle_initial[1])
    
    # theta_1_initial = -0.336998  
    # theta_2_initial = 0.426342  
    return angle_initial  


def get_observation(angle_initial=np.array([-0.336998, 0.426342])):
    """
        obtain joint angles and cartesian state
    """
    # ######################################################
    # ############## get current state #####################
    # ######################################################
    angle = np.zeros(action_dim)  
    point = np.zeros(action_dim)  
    
    angle[0] = motor_control.read_angle_1(angle_initial[0])  
    angle[1] = motor_control.read_angle_2(angle_initial[1], angle[0].copy())  
    print("Joint angles (rad) :", angle)  
    
    point[0] = L_1 * math.cos(angle[0]) + L_2 * math.cos(angle[0] + angle[1])
    point[1] = L_1 * math.sin(angle[0]) + L_2 * math.sin(angle[0] + angle[1])
    print("Position (m) :", point)
    
    return angle, point 


def move_to_target_point(target_point, impedance_params, angle_initial, dist_threshold=0.05):  
    """
        move to target point  
    """ 
    curr_angle, curr_point = get_observation() 
    dist = np.linalg.norm((curr_point - target_point), ord=2) 
    print("Initial dist (m) :", dist) 

    motor_control.move_to_target_point(impedance_params[0], impedance_params[1], impedance_params[2], impedance_params[3],  
        target_point[0], target_point[1],  
        angle_initial[0], angle_initial[1],  
        dist_threshold   
    )
    # while dist > DIST_THREHOLD: 
    #     motor_control.move_to_target(target_point) 
    #     dist = np.linalg.norm((curr_point - target_point), ord=2)  

    curr_angle, curr_point = get_observation() 
    final_dist = np.linalg.norm((curr_point - target_point), ord=2) 
    print("Final dist (m) :", final_dist) 
    done = True
    return done, final_dist


def train(angle_initial): 
    _server = Server(5005) 
    
    run_on = False
    
    # ######################################################
    # ############## wait encoder and motor check ##########
    # ################### Position calibrate ###############
    # ######################################################
    _server.wait_encoder_request() 
    curr_angle, curr_point = get_observation(angle_initial) 
    _server.send_encoder_check(angle_initial) 

    move_to_target_point(Initial_angle) 

    # ######################################################
    # ############## Wait way_points #######################
    # ######################################################
    _server.wait_way_points_request() 

    # receive way points
    way_points = [] 

    os.remove(r'angle_list.txt')
    data_file = open('angle_list.txt', 'w')
    way_point = None
    while way_point != "SEND_DONE":
        way_point = _server.read_way_points()
        # print("way_points ::::", way_point)
        if way_point == "SEND_DONE": 
            break
        way_points.append(way_point)
        line_data = str(way_point[0]) + ',' + str(way_point[1]) + '\n'
        data_file.writelines(line_data)
        # send_done = _server.wait_send_way_points_done()
    way_points = np.array(way_points)
    N_way_points = way_points.shape[0] 
    print("way_points :::", way_points.shape) 
    print("N_way_points :::", N_way_points) 

    # ######################################################
    # ############## Wait impedance parameters  ############
    # ######################################################
    _server.wait_params_request()

    # impedance_params = None
    # while impedance_params is None:
    # read impedance parameters :::
    impedance_params = _server.read_params() 
    print("impedance parameters :::", impedance_params) 
    if impedance_params is not None: 
        pass

    # start move
    if run_on: 
        motor_control.run_one_loop(impedance_params[0], impedance_params[1], impedance_params[2], impedance_params[3],
                                   angle_initial[0], angle_initial[1], N_way_points)

    # send movement_done command 
    _server.send_movement_done() 


def eval(): 
    
    pass 


if __name__ == "__main__":

    # """ calibrate position for each start up """ 
    # angle_initial = reset_and_calibration()  

    # angle, point = get_observation(angle_initial=np.array([-0.336998, 0.426342])) 
    # print("curr_angle :", angle) 
    # print("curr_point :", point) 

    # # train(angle_initial)  
    impedance_params = np.array([6.0, 6.0, 0.0, 0.0])  
    N_way_points = 16357
    angle_initial = np.array([-0.336998, 0.426342]) 
    # motor_control.run_one_loop(impedance_params[0], impedance_params[1], impedance_params[2], impedance_params[3],
    #                                angle_initial[0], angle_initial[1], N_way_points) 

    move_to_target_point(Initial_angle, impedance_params, angle_initial, dist_threshold=0.05)

#     plot_torque_path(
#         root_path='',
#         file_angle_name='real_angle_list.txt', 
#         file_torque_name='real_torque_list.txt' 
# ) 
