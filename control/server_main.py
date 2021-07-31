from protocol.task_interface import *
import numpy as np
import math
import os
from motor_control import motor_control
L_1 = 300
L_2 = 250
action_dim = 2
DIST_THREHOLD = 0.05


def reset_and_calibration():
	print("Please make sure two links are at zero position !!!")
	angle_initial = np.zeros(action_dim)
	
	angle_initial[0] = motor_control.read_initial_angle_1()
	angle_initial[1] = motor_control.read_initial_angle_2()
	print("theta_1_initial :::", theta_1_initial)
	print("theta_2_initial :::", theta_2_initial)
	
	# theta_1_initial = -0.336998
	# theta_2_initial = 0.426342
	return angle_initial


def get_observation(angle_initial=np.array([-0.336998, 0.426342])):
	"""
		obtain joint angles and cartesian state
	"""
	angle = np.zeros(action_dim)
	point = np.zeros(action_dim)
	
	angle[0] = motor_control.read_angle_1(angle_initial[0])
	angle[1] = motor_control.read_angle_2(angle_initial[1], angle[0].copy())
	print("Joint angles (rad) :", angle)
	
	point[0] = L_1 * math.cos(angle[0]) + L_2 * math.cos(angle[0] + angle[1])
	point[1] = L_1 * math.sin(angle[0]) + L_2 * math.sin(angle[0] + angle[1])
	print("Position (m) :", point)
	
	return angle, point


def move_to_initial_point(target_point):
	curr_angle, curr_point = get_observation()
	dist = np.linalg.norm((curr_point - target_point), ord=2)
	while dist > DIST_THREHOLD:
		motor_control.move_to_target(target_point)
		dist = np.linalg.norm((curr_point - target_point), ord=2)
	done = True
	return done, dist


def train():
	# ######################################################
	# ############## get current state #####################
	# ######################################################
	pass
	

def eval():
	
	pass


if __name__ == "__main__":
	theta_1_initial = -0.336998
	theta_2_initial = 0.426342
	L_1 = 300 
	L_2 = 250 

	# ######################################################
	# ############## get current state #####################
	# ######################################################
	theta_1_t = motor_control.read_angle_1(theta_1_initial)
	print("theta_1_t :::", theta_1_t)
	theta_2_t = motor_control.read_angle_2(theta_2_initial, theta_1_t) 
	print("theta_2_t :::", theta_2_t) 

	x = L_1 * math.cos(theta_1_t) + L_2 * math.cos(theta_1_t + theta_2_t)
	print("x ::", x) 
	y = L_1 * math.sin(theta_1_t) + L_2 * math.sin(theta_1_t + theta_2_t)
	print("y ::", y) 
	
	# _server = Server(5005) 

	# # # wait encoder and motor check
	# # _server.wait_encoder_request()
	# # info = dict()
	# # info['encoder'] = [1., 1.]
	# # info['motor'] = [10., 10.]
	# #
	# # _server.send_encoder_check(info)
	
	# # Wait impedance parameters
	# _server.wait_params_request() 

	# # impedance_params = None
	# # while impedance_params is None: 
	# # read impedance parameters :::
	# impedance_params = _server.read_params() 
	# print("impedance parameters :::", impedance_params) 

	# # Wait way_points:::
	# _server.wait_way_points_request() 
	
	# # receive way points
	# way_points = []
	# # send_done = _server.wait_send_way_points_done()
	# # print("send_done :::", send_done)
	# os.remove(r'angle_list.txt')
	# data_file = open('angle_list.txt', 'w')
	# way_point = None
	# while way_point != "SEND_DONE":
	# 	way_point = _server.read_way_points()
	# 	print("way_points ::::", way_point)
	# 	if way_point == "SEND_DONE":
	# 		break
	# 	way_points.append(way_point)
	# 	line_data = str(way_point[0]) + ',' + str(way_point[1]) + '\n'
	# 	data_file.writelines(line_data)
	# 	# send_done = _server.wait_send_way_points_done()
	# way_points = np.array(way_points)
	# print("way_points :::", way_points)
	
	# # start move

	# # send movement_done command
	# _server.send_movement_done()

