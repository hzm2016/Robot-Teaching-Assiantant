import motor_control
import copy


# motor_control.reset(theta_1_initial, theta_2_initial)
# print(theta_1_initial)
# print(motor_control.add(1, 2))

# motor_control.load_path_data() 

theta_1_initial = motor_control.read_angle_1()
print("theta_1_initial :::", theta_1_initial) 
theta_2_initial = motor_control.read_angle_2()
print("theta_2_initial :::", theta_2_initial) 

# motor_control.get_demonstration(theta_1_initial, theta_2_initial)

# motor_control.run_one_loop(16, 0.8, theta_1_initial, theta_2_initial, 19999) 

# get current state
theta_1_t = motor_control.read_angle_1()
print("theta_1_t :::", theta_1_t) 
theta_2_t = motor_control.read_angle_2() 
print("theta_2_t :::", theta_2_t) 

# dist_threshold = 0.01
# motor_control.move_to_target(16.0, 0.8, double q_1_target, double q_2_target, 
# theta_1_initial, theta_2_initial, dist_threshold
# )
