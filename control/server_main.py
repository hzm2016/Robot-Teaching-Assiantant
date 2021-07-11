from protocol.task_interface import *

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

	# read way points :::
	_server.read_way_points()
	
	# receive way points
	way_points = []
	send_done = _server.wait_send_way_points_done()
	print("send_done :::", send_done)
	while not send_done:
		way_point = _server.read_way_points()
		print("way_points ::::", way_points)
		way_points.append(way_point)
		send_done = _server.wait_send_way_points_done()