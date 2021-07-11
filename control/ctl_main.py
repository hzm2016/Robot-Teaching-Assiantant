import cv2
from control.vision_capture.main_functions import *
from control.path_planning.path_generate import *
import socket
import time
from control.protocol.task_interface import *


class Connector(object):
	def __init__(self):
		# ===================== socket connection ========================
		self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		address = ('169.254.0.99', 5005)
		self.tcp_socket.connect(address)
		
		print("connect to server :::", address)
		
	def system_calib(self):
		send_data = "Calibration"
		self.tcp_socket.send(send_data.encode())
		
	def set_params(self):
		print("Second params ::::")
		
		# send way_points to the lower-level controller
		# ====================== second data =========================
		# send impedance parameters ::::
		# lower-level controller set impedance and motion command
		stiffness = [100, 100]
		damping = [50, 50]
		command_move = "Move_start"
		swrite_stiffness = '#Stiff ' + '[' + str('%0.3f' % stiffness[0]) + ',' + str('%0.3f' % stiffness[1]) + ']' + '@'
		swrite_damping = '#Damping' + '[' + str('%0.3f' % damping[0]) + ',' + str('%0.3f' % damping[1]) + ']' + '@'
		
		self.tcp_socket.send(swrite_stiffness.encode())
		
		self.tcp_socket.send(swrite_damping.encode())
		
		self.tcp_socket.send(command_move.encode())
	
	def set_way_points(self):
		# terminate with symbol @
		way_points = np.array((100, 2))
		length = way_points.shape[0]
		for i in range(length//5+1):
			swrite_way_points = '#Points '
			for j in range(5):
				swrite_way_points += '[' + str('%0.3f' % way_points[i+j, 0]) + ',' + str('%0.3f' % way_points[i+j, 1]) + '],'
			self.tcp_socket.send(swrite_way_points.encode())
		
		self.tcp_socket.send('@'.encode())
		
	def run_one_loop(self):
		# run_one_loop tasks ::: and capture the results :::

		pass
		

def run_main():
	done = False
	
	# initial TCP connection :::
	# check encoders and motors :::
	task = TCPTask('169.254.0.99', 5005)
	
	# task.get_encoder_check()
	
	# # offline check the generated path :::
	# angle_1_list_e, angle_2_list_e = check_path(root_path='path_planning/data', font_name='third', type=3)
	# way_points = np.vstack((angle_1_list_e, angle_2_list_e)).transpose()
	# # print("way_points :::", way_points.shape)
	#
	# # send way points
	# task.send_way_points(way_points)
	#
	# # generate impedance parameters::
	#
	
	# # send impedance params :::
	stiffness = [100, 100]
	damping = [50, 50]
	params = stiffness + damping
	task.send_params(params)
	
	task.send_params_request()
	
	task.send_way_points_request()
	way_points = [[0., 0.], [1., 1.0]]
	for i in range(len(way_points)):
		task.send_way_points(way_points[i])
	
	task.send_way_points_done()
	# # send way_points :::
	# command_move = "Move_start"
	#
	# # video record for trail :::
	# if done:
	# 	show_video()
	# 	capture_image(root_path='capture_images/', font_name='test')
	# 	image_precessing(img_path='capture_images/', img_name='test')

# 	# address = ('192.168.1.182', 5005)  # 服务端地址和端口
# 	# s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# 	# s.bind(address)  # 绑定服务端地址和端口
# 	# s.listen(5)
# 	# conn, addr = s.accept()  # 返回客户端地址和一个新的 socket 连接
# 	# print('[+] Connected with', addr)
# 	# while True:
# 	# 	conn, addr = s.accept()  # 返回客户端地址和一个新的 socket 连接
# 	# 	print('[+] Connected with', addr)
# 	# 	data = conn.recv(1024)  # buffersize 等于 1024
# 	# 	data = data.decode()
# 	# 	if not data:
# 	# 		break
# 	# 	print('[Received]', data)
# 	#
# 	# # send = input('Input: ')
# 	# # conn.sendall(send.encode())
# 	# conn.close()
# 	# s.close()
#
#
# 	recvbuf = ''
# 	time_out = 0
# 	while recvbuf.find('done') == -1:
# 		recvbuf = tcp_socket.recv(2048).decode()
# 		time_out += 1
#
# 	done = True
# 	if done == 'done':
# 		print('-----------------Move Finish!!!!-------------------')
# 		capture_image(root_path='capture_images/', font_name='test')
#
# 	# ====================== receive data =========================
# 	recvbuf = ''
# 	time_out = 0
# 	while len(recvbuf) == 0 and time_out < 20:
# 		recvbuf = tcp_socket.recv(2048).decode()
# 		time_out += 1
# 		time.sleep(0.01)
#
# 	# print(recv_data.decode("utf-8"))
# 	# 4. 关闭套接字
# 	tcp_socket.close()
	

if __name__ == "__main__":
	# check_path(root_path='path_planning/data', font_name='third', type=3, period=10, Ts=0.001)
	# image_precessing(img_path='capture_images/', img_name='test')
	
	run_main()
	
	# import socket
	# import sys
	#
	# address = ('127.0.0.1', 5005)  # 服务端地址和端口
	# s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	# try:
	# 	s.connect(address)  # 尝试连接服务端
	# except Exception:
	# 	print('[!] Server not found ot not open')
	# 	sys.exit()
	# while True:
	# 	trigger = input('Input: ')
	# 	s.sendall(trigger.encode())
	# 	data = s.recv(1024)
	# 	data = data.decode()
	# 	print('[Recieved]', data)
	# 	if trigger == '###':  # 自定义结束字符串
	# 		break
	# s.close()
