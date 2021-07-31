import cv2
import pyzed.sl as sl

camera_settings = sl.VIDEO_SETTINGS.BRIGHTNESS
str_camera_settings = "BRIGHTNESS"
step_camera_settings = 1


def show_video():
	print("Running...")
	init = sl.InitParameters()
	cam = sl.Camera()
	if not cam.is_opened():
		print("Opening ZED Camera...")

	# init.camera_resolution = sl.RESOLUTION.HD480

	status = cam.open(init)
	if status != sl.ERROR_CODE.SUCCESS:
		print(repr(status))
		exit()

	runtime = sl.RuntimeParameters()
	mat = sl.Mat()

	print_camera_information(cam)
	print_help()

	key = ''
	while key != 113:  # for 'q' key
		err = cam.grab(runtime)
		if err == sl.ERROR_CODE.SUCCESS:
			cam.retrieve_image(mat, sl.VIEW.LEFT)
			cv2.imshow("ZED", mat.get_data())
			key = cv2.waitKey(5)
			settings(key, cam, runtime, mat)
		else:
			key = cv2.waitKey(5)
	cv2.destroyAllWindows()

	cam.close()
	print("\nFINISH")


def calibration_camera():
	
	pass


def image_precessing(img_path, img_name):
	img = cv2.imread(img_path + img_name + '.png')
	height, weight = img.shape[:2]
	print("height :::", height)
	print("weight :::", weight)
	
	# need to define according to robot position
	crop_img = img[130:height-100, 455:945]
	cv2.imshow("Processed Image", crop_img)
	resize_img = cv2.resize(crop_img, (128, 128), cv2.INTER_AREA)
	cv2.imshow("Processed Image", resize_img)
	
	cols, rows = resize_img.shape[:2]
	
	# rotate image :::
	M = cv2.getRotationMatrix2D((cols / 2, rows / 2), -90, 1)
	dst_img = cv2.warpAffine(resize_img, M, (cols, rows))
	cv2.imwrite(img_path + img_name + '_final.png', dst_img)
	cv2.imshow("Processed Image", dst_img)
	cv2.waitKey()
	return img


def capture_image(root_path='', font_name='font_1', size=(128, 128)):
	print("Capture image ...")
	init = sl.InitParameters()
	cam = sl.Camera()
	if not cam.is_opened():
		print("Opening ZED Camera...")
	
	# init.camera_resolution = sl.RESOLUTION.HD480
	
	status = cam.open(init)
	if status != sl.ERROR_CODE.SUCCESS:
		print(repr(status))
		exit()
	
	runtime = sl.RuntimeParameters()
	mat = sl.Mat()
	
	# print_camera_information(cam)
	# print_help()
	
	key = ''
	# while key != 113:  # for 'q' key
	err = cam.grab(runtime)
	if err == sl.ERROR_CODE.SUCCESS:
		cam.retrieve_image(mat, sl.VIEW.LEFT)
		cv2.imshow("ZED", mat.get_data())
		img = mat.get_data()
		height, weight = img.shape[:2]
		# print("height :::", height)
		# # print("weight :::", weight)
		
		# need to define according to robot position
		crop_img = img[200:750, 400:850]
		
		resize_img = cv2.resize(crop_img, size, cv2.INTER_AREA)
		cv2.imwrite(root_path + font_name + '_resize.png', crop_img)
		# cv2.imshow("Processed Image", resize_img)
		
		cols, rows = resize_img.shape[:2]
		
		# rotate image :::
		M = cv2.getRotationMatrix2D((cols / 2, rows / 2), -90, 1)
		dst_img = cv2.warpAffine(resize_img, M, (cols, rows))
		
		cv2.imwrite(root_path + font_name + '.png', dst_img)
		key = cv2.waitKey(5)
		settings(key, cam, runtime, mat)
	else:
		key = cv2.waitKey(5)
		
	cv2.destroyAllWindows()
	
	cam.close()
	print("\nFINISH ...")
	
	return dst_img, img
	

def print_camera_information(cam):
	print("Resolution: {0}, {1}.".format(round(cam.get_camera_information().camera_resolution.width, 2),
										 cam.get_camera_information().camera_resolution.height))
	print("Camera FPS: {0}.".format(cam.get_camera_information().camera_fps))
	print("Firmware: {0}.".format(cam.get_camera_information().camera_firmware_version))
	print("Serial number: {0}.\n".format(cam.get_camera_information().serial_number))


def print_help():
	print("Help for camera setting controls")
	print("  Increase camera settings value:     +")
	print("  Decrease camera settings value:     -")
	print("  Switch camera settings:             s")
	print("  Reset all parameters:               r")
	print("  Record a video:                     z")
	print("  Quit:                               q\n")


def settings(key, cam, runtime, mat):
	if key == 115:  # for 's' key
		switch_camera_settings()
	elif key == 43:  # for '+' key
		current_value = cam.get_camera_settings(camera_settings)
		cam.set_camera_settings(camera_settings, current_value + step_camera_settings)
		print(str_camera_settings + ": " + str(current_value + step_camera_settings))
	elif key == 45:  # for '-' key
		current_value = cam.get_camera_settings(camera_settings)
		if current_value >= 1:
			cam.set_camera_settings(camera_settings, current_value - step_camera_settings)
			print(str_camera_settings + ": " + str(current_value - step_camera_settings))
	elif key == 114:  # for 'r' key
		cam.set_camera_settings(sl.VIDEO_SETTINGS.BRIGHTNESS, -1)
		cam.set_camera_settings(sl.VIDEO_SETTINGS.CONTRAST, -1)
		cam.set_camera_settings(sl.VIDEO_SETTINGS.HUE, -1)
		cam.set_camera_settings(sl.VIDEO_SETTINGS.SATURATION, -1)
		cam.set_camera_settings(sl.VIDEO_SETTINGS.SHARPNESS, -1)
		cam.set_camera_settings(sl.VIDEO_SETTINGS.GAIN, -1)
		cam.set_camera_settings(sl.VIDEO_SETTINGS.EXPOSURE, -1)
		cam.set_camera_settings(sl.VIDEO_SETTINGS.WHITEBALANCE_TEMPERATURE, -1)
		print("Camera settings: reset")
	elif key == 122:  # for 'z' key
		record(cam, runtime, mat)


def switch_camera_settings():
	global camera_settings
	global str_camera_settings
	if camera_settings == sl.VIDEO_SETTINGS.BRIGHTNESS:
		camera_settings = sl.VIDEO_SETTINGS.CONTRAST
		str_camera_settings = "Contrast"
		print("Camera settings: CONTRAST")
	elif camera_settings == sl.VIDEO_SETTINGS.CONTRAST:
		camera_settings = sl.VIDEO_SETTINGS.HUE
		str_camera_settings = "Hue"
		print("Camera settings: HUE")
	elif camera_settings == sl.VIDEO_SETTINGS.HUE:
		camera_settings = sl.VIDEO_SETTINGS.SATURATION
		str_camera_settings = "Saturation"
		print("Camera settings: SATURATION")
	elif camera_settings == sl.VIDEO_SETTINGS.SATURATION:
		camera_settings = sl.VIDEO_SETTINGS.SHARPNESS
		str_camera_settings = "Sharpness"
		print("Camera settings: Sharpness")
	elif camera_settings == sl.VIDEO_SETTINGS.SHARPNESS:
		camera_settings = sl.VIDEO_SETTINGS.GAIN
		str_camera_settings = "Gain"
		print("Camera settings: GAIN")
	elif camera_settings == sl.VIDEO_SETTINGS.GAIN:
		camera_settings = sl.VIDEO_SETTINGS.EXPOSURE
		str_camera_settings = "Exposure"
		print("Camera settings: EXPOSURE")
	elif camera_settings == sl.VIDEO_SETTINGS.EXPOSURE:
		camera_settings = sl.VIDEO_SETTINGS.WHITEBALANCE_TEMPERATURE
		str_camera_settings = "White Balance"
		print("Camera settings: WHITEBALANCE")
	elif camera_settings == sl.VIDEO_SETTINGS.WHITEBALANCE_TEMPERATURE:
		camera_settings = sl.VIDEO_SETTINGS.BRIGHTNESS
		str_camera_settings = "Brightness"
		print("Camera settings: BRIGHTNESS")


def record(cam, runtime, mat, filepath=None):
	vid = sl.ERROR_CODE.FAILURE
	out = False
	while vid != sl.ERROR_CODE.SUCCESS and not out:
		# filepath = input("Enter filepath name: ")
		
		record_param = sl.RecordingParameters(filepath, sl.SVO_COMPRESSION_MODE.H264)
		vid = cam.enable_recording(record_param)
		print(repr(vid))
		if vid == sl.ERROR_CODE.SUCCESS:
			print("Recording started...")
			out = True
			print("Hit spacebar to stop recording: ")
			key = False
			while key != 32:  # for spacebar
				err = cam.grab(runtime)
				if err == sl.ERROR_CODE.SUCCESS:
					cam.retrieve_image(mat)
					cv2.imshow("ZED", mat.get_data())
					key = cv2.waitKey(5)
		else:
			print("Help: you must enter the filepath + filename + SVO extension.")
			print("Recording not started.")
	
	cam.disable_recording()
	print("Recording finished.")
	cv2.destroyAllWindows()


if __name__ == "__main__":
	# print("Running...")
	# init = sl.InitParameters()
	# cam = sl.Camera()
	# if not cam.is_opened():
	# 	print("Opening ZED Camera...")
	#
	# # init.camera_resolution = sl.RESOLUTION.HD480
	#
	# status = cam.open(init)
	# if status != sl.ERROR_CODE.SUCCESS:
	# 	print(repr(status))
	# 	exit()
	#
	# runtime = sl.RuntimeParameters()
	# mat = sl.Mat()
	#
	# # print_camera_information(cam)
	# # print_help()
	#
	# # record(cam, runtime, mat, filepath='font_4.svo')
	#
	# cam.close()
	# print("\nFINISH")
	
	show_video()
	
	# capture_image(font_name='capture_data/font_1')
