import motor_control
import copy
import numpy as np
from control.path_planning.plot_path import *
import seaborn as sns

np.set_printoptions(precision=5)
sns.set(font_scale=2.5)

#######################################################
################### robot information #################
#######################################################

# # motor_control.reset(theta_1_initial, theta_2_initial)
# # print(theta_1_initial)
# # print(motor_control.add(1, 2))
#
# # motor_control.load_path_data()
#
# # theta_1_initial = motor_control.read_initial_angle_1()
# # print("theta_1_initial :::", theta_1_initial)
# # theta_2_initial = motor_control.read_initial_angle_2()
# # print("theta_2_initial :::", theta_2_initial)
#
# theta_1_initial = -0.287717
# theta_2_initial = 0.294666

# ######################################################
# ############## get current state #####################
# ######################################################
# theta_1_t = motor_control.read_angle_1(theta_1_initial)
# print("theta_1_t :::", theta_1_t)
# theta_2_t = motor_control.read_angle_2(theta_2_initial, theta_1_t)
# print("theta_2_t :::", theta_2_t)
#
# q_1_target = -0.37892
# q_2_target = 1.9540
#
# dist_threshold = 0.05
# motor_control.move_to_target(16.0, 0.8,
# q_1_target, q_2_target,
# theta_1_initial, theta_2_initial, dist_threshold
# )
#
# # motor_control.get_demonstration(theta_1_initial, theta_2_initial)
#
# # motor_control.run_one_loop(16, 0.8, theta_1_initial, theta_2_initial, 19999)


####################
### plot results ###
####################

plot_torque_path(
        root_path='../',
        file_angle_name='move_target_angle_list.txt',
        file_torque_name='move_target_torque_list.txt'
)

