import motor_control
import copy
from control.path_planning.plot_path import *

# motor_control.reset(theta_1_initial, theta_2_initial)
# print(theta_1_initial)
# print(motor_control.add(1, 2))
# motor_control.load_path_data() 

# theta_1_initial = motor_control.read_initial_angle_1()
# print("theta_1_initial :::", theta_1_initial)
# theta_2_initial = motor_control.read_initial_angle_2()
# print("theta_2_initial :::", theta_2_initial)
#
# motor_control.get_demonstration(theta_1_initial, theta_2_initial)

plot_real_2d_path(root_path='../',
                  # file_name='angle_list.txt'
                  file_name='demonstrated_angle_list.txt'
)