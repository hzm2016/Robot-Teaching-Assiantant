from control.protocol.task_interface import *

if __name__ == "__main__":
	_server = Server(5005)
	
	# Wait impedance parameters :::
	_server.wait_params_request()
	
	# read impedance parameters :::
	impedance_params = _server.read_params()
	
	print("impedance parameters :::", impedance_params)