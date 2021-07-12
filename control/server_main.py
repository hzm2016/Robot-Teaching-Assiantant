from protocol.task_interface import *
import numpy as np
import os
# import motor_control

if __name__ == "__main__":
	# print(motor_control.add(1, 2))
	
	_server = Server(5005)

	# Wait impedance parameters :::
	_server.wait_params_request()

	# wait encoder and motor check
	_server.wait_encoder_request()
	info = dict()
	info['encoder'] = [1., 1.]
	info['motor'] = [10., 10.]

	_server.send_encoder_check(info)

	# impedance_params = None
	# while impedance_params is None:
	# read impedance parameters :::
	impedance_params = _server.read_params()
	print("impedance parameters :::", impedance_params)

	# Wait way_points:::
	# _server.wait_way_points_request()

	# # read way points :::
	# _server.read_way_points()
	
	# receive way points
	way_points = []
	# send_done = _server.wait_send_way_points_done()
	# print("send_done :::", send_done)
	os.remove(r'angle_list.txt')
	data_file = open('angle_list.txt', 'w')
	way_point = None
	while way_point != "SEND_DONE":
		way_point = _server.read_way_points()
		print("way_points ::::", way_point)
		if way_point == "SEND_DONE":
			break
		
		way_points.append(way_point)
		line_data = str(way_point[0]) + ',' + str(way_point[1])
		data_file.writelines(line_data)
		# send_done = _server.wait_send_way_points_done()
	way_points = np.array(way_points)
	print("way_points :::", way_points)

	# send movement_done command
	_server.send_movement_done()

