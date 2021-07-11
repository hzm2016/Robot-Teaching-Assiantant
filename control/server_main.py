from protocol.task_interface import *
import numpy as np

if __name__ == "__main__":
	_server = Server(5005)
	
	# Wait impedance parameters :::
	_server.wait_params_request()
	
	# impedance_params = None
	# while impedance_params is None:
	# read impedance parameters :::
	impedance_params = _server.read_params()
	print("impedance parameters :::", impedance_params)
	
	# Wait waypoints parameters :::
	_server.wait_way_points_request()

	# # read way points :::
	# _server.read_way_points()
	
	# receive way points
	way_points = []
	# send_done = _server.wait_send_way_points_done()
	# print("send_done :::", send_done)
	way_point = None
	while way_point != "SEND_DONE":
		way_point = _server.read_way_points()
		print("way_points ::::", way_point)
		if way_point == "SEND_DONE":
			break
		way_points.append(way_point)
		# send_done = _server.wait_send_way_points_done()
	way_points = np.array(way_points)
	print("way_points :::", way_points)