import cv2
import sys
import os
from control.vision_capture.main_functions import *
from control.path_planning.path_generate import *
import socket
import time
from control.protocol.task_interface import *
		

def run_main(show_video=False):
	# initial TCP connection :::
	# check encoders and motors :::
	task = TCPTask('169.254.0.99', 5005)
	
	# check motor and encode well before experiments
	# task.get_encoder_check()
	
	task.send_params_request()
	
	# # generate impedance parameters::
	
	# # send impedance params :::
	stiffness = [100, 100]
	damping = [50, 50]
	params = stiffness + damping
	task.send_params(params)
	
	# # offline check the generated path :::
	angle_1_list_e, angle_2_list_e = check_path(root_path='path_planning/data', plot_show=False, font_name='third', type=3)
	way_points = np.vstack((angle_1_list_e, angle_2_list_e)).transpose()
	print("way_points :::", way_points.shape)

	task.send_way_points_request()
	
	task.send_way_points(way_points)
	
	task.send_way_points_done()
	
	# # send way_points :::
	# command_move = "Move_start"
	
	# if show_video:
	# 	show_video()
		
	# video record for trail :::
	run_done = task.get_movement_check()
	if run_done:
		print("run_done", run_done)
	
	# if run_done:
	# 	capture_image(root_path='capture_images/', font_name='test')
	# 	image_precessing(img_path='capture_images/', img_name='test')
	

if __name__ == "__main__":
	# check_path(root_path='path_planning/data', font_name='third', type=3, period=10, Ts=0.001)
	# image_precessing(img_path='capture_images/', img_name='test')
	# show_video()
	run_main()
