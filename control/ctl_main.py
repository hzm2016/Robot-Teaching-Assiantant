import cv2
from control.vision_capture.main_functions import capture_image
from control.path_planning.path_generate import *
import socket


def run_main():
	
	# offline check the generated path
	# _, _ = check_path(root_path='path_planning/data', font_name='third', type=3)
	
	capture_image(root_path='capture_images/', font_name='test')
	
	address = ('192.168.1.182', 5005)  # 服务端地址和端口
	s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	s.bind(address)  # 绑定服务端地址和端口
	s.listen(5)
	conn, addr = s.accept()  # 返回客户端地址和一个新的 socket 连接
	print('[+] Connected with', addr)
	while True:
		conn, addr = s.accept()  # 返回客户端地址和一个新的 socket 连接
		print('[+] Connected with', addr)
		data = conn.recv(1024)  # buffersize 等于 1024
		data = data.decode()
		if not data:
			break
		print('[Received]', data)
		
	# send = input('Input: ')
	# conn.sendall(send.encode())
	conn.close()
	s.close()
	
	
if __name__ == "__main__":
	
	
	
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
