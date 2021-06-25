import cv2
from control.vision_capture.main_functions import capture_image
from control.path_planning.path_generate import *

if __name__ == "__main__":
	
	
	_, _ = check_path(root_path='path_planning/data', font_name='third', type=3)
	
	# capture_image(root_path='capture_images/', font_name='test')
