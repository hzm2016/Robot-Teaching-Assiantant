import numpy as np
np.set_printoptions(precision=5)
np.set_printoptions(suppress=True)
import matplotlib.pyplot as plt
import math
import seaborn as sns
import copy

sns.set_theme()

# writing space
WIDTH = 0.370
HEIGHT = 0.370

# image size
IMAGE_WIDTH = 128
IMAGE_HEIGHT = 128

# joints limits
action_dim = 2
ANGLE_1_RANGE = np.array([-1.90, 1.90])
ANGLE_2_RANGE = np.array([-2.2, 2.5])

Length = [0.30, 0.150, 0.25, 0.125]
L1 = Length[0]
L2 = Length[2]

Ts = 0.001

# plot parameters
linewidth = 3


def initial_parameter_estimate(num_demons_each_style=30):
    """ demonstration for each styles with zero impedance """
    num_demonstrations = 30
    
    writting_vel = 0.0
    impedance_params = np.ones(2)
    
    # captured images
    images_list = []
    distance_list = []
    for i in range(num_demonstrations):
        distance_list.appen
        images_list.append(i.copy())

    return writting_vel, impedance_params


def calibrate_para():
    ori_point = np.array([])


def IK(point):
    """
        Inverse kinematics
    """
    angle = np.zeros(action_dim)
    x1 = point[0]
    x2 = point[1]
    
    L = x1**2 + x2**2

    gamma = math.atan2(x2, x1)

    cos_belta = (L1 ** 2 + L - L2 ** 2) / (2 * L1 * np.sqrt(L))

    if cos_belta > 1:
        angle_1 = gamma
    elif cos_belta < -1:
        angle_1 = gamma - np.pi
    else:
        angle_1 = gamma - math.acos(cos_belta)

    cos_alpha = (L1 ** 2 - L + L2 ** 2) / (2 * L1 * L2)

    if cos_alpha > 1:
        angle_2 = np.pi
    elif cos_alpha < -1:
        angle_2 = 0
    else:
        angle_2 = np.pi - math.acos(cos_alpha)

    angle[0] = np.round(angle_1, 5).copy()
    angle[1] = np.round(angle_2, 5).copy()
    return angle


def forward_ik(angle):
    """ calculate point """
    point = np.zeros_like(angle)
    point[0] = L1 * math.cos(angle[0]) + L2 * math.cos(angle[0] + angle[1])
    point[1] = L1 * math.sin(angle[0]) + L2 * math.sin(angle[0] + angle[1])
    
    return point


def path_planning(start_point, target_point, velocity=0.04):
    """
        path planning
    """
    dist = np.linalg.norm((start_point - target_point), ord=2)
    T = dist/velocity
    N = int(T/Ts)
    
    # print("start_point :", start_point[0])  
    # print("end_point :", start_point[1])
    
    x_list = np.linspace(start_point[0], target_point[0], N)
    y_list = np.linspace(start_point[1], target_point[1], N)
    
    point = start_point
    angle_list = []
    for i in range(N):
        point[0] = x_list[i]
        point[1] = y_list[i]
        angle = IK(point)
        angle_list.append(angle)
        
    return np.array(angle_list), N


def plot_stroke_path(period, traj, image_points, task_points, angle_list, fig_name='Stroke Path'):
    """
        check the planned path
    """
    t_list = np.linspace(0.0, period, angle_list.shape[0])
    print("task points :", task_points)
    plt.figure(figsize=(15, 4))
    
    plt.title(fig_name)
    
    plt.subplot(1, 3, 1)
    plt.subplots_adjust(wspace=0.2, hspace=0.2)
    
    plt.plot(traj[:, 1], traj[:, 0], marker='o', linewidth=linewidth)
    plt.plot(image_points[:, 0], image_points[:, 1], linewidth=linewidth - 2)
    
    plt.xlim([0, 128])
    plt.ylim([0, 128])
    plt.xlabel('$x_1$')
    plt.ylabel('$x_2$')
    # plt.axis('equal')
    plt.tight_layout()
    
    plt.subplot(1, 3, 2)
    plt.subplots_adjust(wspace=0.2, hspace=0.2)
    
    plt.plot(task_points[:, 0], task_points[:, 1], linewidth=linewidth + 2, color='r')
    # plt.plot(x_inner, y_inner, linewidth=linewidth + 2, color='r')
    plt.scatter(task_points[0, 0], task_points[0, 1], s=100, c='b', marker='o')
    # plt.scatter(x_inner[0], y_inner[0], s=100, c='b', marker='o')
    # print("distance :::", np.sqrt((x_1_list[0] - x_inner[0])**2 + (x_2_list[0] - y_inner[0])**2))
    plt.ylim([-WIDTH / 2, WIDTH / 2])
    plt.xlim([0., 0.13 + WIDTH])
    plt.xlabel('$x_1$(m)')
    plt.ylabel('$x_2$(m)')
    
    plt.subplot(1, 3, 3)
    plt.plot(t_list, angle_list[:, 0], linewidth=linewidth, label='$q_1$')
    # plt.plot(t_list[1:], angle_vel_1_list_e, linewidth=linewidth, label='$d_{q1}$')
    plt.plot(t_list, angle_list[:, 1], linewidth=linewidth, label='$q_2$')
    # plt.plot(t_list[1:], angle_vel_2_list_e, linewidth=linewidth, label='$d_{q2}$')
    
    plt.xlabel('Time (s)')
    plt.ylabel('One-loop Angle (rad)')
    plt.legend()
    
    plt.show()


def plot_word_path(period_list, traj_list, image_points_list, task_points_list, word_angle_list, word_name='Stroke Path'):
    """
        plot one word path
    """
    plt.figure(figsize=(15, 4))
    plt.title(word_name)

    plt.subplot(1, 3, 1)
    plt.subplots_adjust(wspace=0.2, hspace=0.2)
    
    for i in range(len(traj_list)):
        plt.plot(traj_list[i][:, 1], traj_list[i][:, 0], marker='o', linewidth=linewidth)
        plt.plot(image_points_list[i][:, 0], image_points_list[i][:, 1], linewidth=linewidth - 2)

    plt.xlim([0, 128])
    plt.ylim([0, 128])
    plt.xlabel('$x_1$')
    plt.ylabel('$x_2$')
    # plt.axis('equal')

    plt.subplot(1, 3, 2)
    plt.subplots_adjust(wspace=0.2, hspace=0.2)
    
    for i in range(len(traj_list)):
        plt.plot(task_points_list[i][:, 0], task_points_list[i][:, 1], linewidth=linewidth + 2, color='r')
        # plt.plot(x_inner, y_inner, linewidth=linewidth + 2, color='r')
        plt.scatter(task_points_list[i][0, 0], task_points_list[i][0, 1], s=100, c='b', marker='o')
        # plt.scatter(x_inner[0], y_inner[0], s=100, c='b', marker='o')
        # print("distance :::", np.sqrt((x_1_list[0] - x_inner[0])**2 + (x_2_list[0] - y_inner[0])**2))
    plt.ylim([-WIDTH / 2, WIDTH / 2])
    plt.xlim([0., 0.13 + WIDTH])
    plt.xlabel('$x_1$(m)')
    plt.ylabel('$x_2$(m)')

    plt.subplot(1, 3, 3)
    plt.subplots_adjust(wspace=0.2, hspace=0.2)
    # total_period = sum(period_list)
    for i in range(len(traj_list)):
        if i == 0:
            t_list = np.linspace(0.0, period_list[i], word_angle_list[i].shape[0])
        else:
            t_list = np.linspace(period_list[i-1], period_list[i] + period_list[i-1], word_angle_list[i].shape[0])
            
        plt.plot(t_list, word_angle_list[i][:, 0], linewidth=linewidth, label='$q_1$')
        # plt.plot(t_list[1:], angle_vel_1_list_e, linewidth=linewidth, label='$d_{q1}$')
        plt.plot(t_list, word_angle_list[i][:, 1], linewidth=linewidth, label='$q_2$')
        # plt.plot(t_list[1:], angle_vel_2_list_e, linewidth=linewidth, label='$d_{q2}$')

    plt.xlabel('Time (s)')
    plt.ylabel('One-loop Angle (rad)')
    # plt.legend()
    plt.tight_layout()
    
    plt.show()
    
    
def generate_stroke_path(traj, inter_type=1,
                  center_shift=np.array([-WIDTH/2, 0.23]),
                  velocity=0.04, Ts=0.001, plot_show=False, save_path=False, stroke_name=0):
    """
         generate stroke trajectory from list
         velocity ::: 0.04m/s
    """
    # calculate length of path
    dist = 0.0
    for i in range(len(traj) - 1):
        point_1 = np.array([traj[i, 1], traj[i, 0]])
        point_2 = np.array([traj[i+1, 1], traj[i+1, 0]])
        dist += np.linalg.norm((point_2.copy() - point_1.copy()), ord=2)
    
    path_data = traj
    
    # M = N//len(traj)
    # x_list = []
    # y_list = []
    # for i in range(len(traj)):
    #     # need to check which dimension can be applied for interp
    #     x_list_i = np.linspace(path_data[i, 1], path_data[i+1, 1], M)
    #     y_list_i = np.interp(x_list_i, path_data[i:i+1, 1], path_data[i:i+1, 0])
    #     x_list.append(x_list_i)
    #     y_list.append(y_list_i)

    # # need to check which dimension can be applied for interp
    # x_list = np.linspace(path_data[-1, 1], path_data[0, 1], N)
    # # x_list = path(1, 2):(path(end, 2) - path(1, 2)) / (N - 1): path(end, 2)
    # y_list = np.interp(x_list, path_data[:, 1][::-1], path_data[:, 0][::-1])

    # transform to work space
    ratio = IMAGE_WIDTH / WIDTH
    
    period = dist/ratio/velocity
    print("Distance (mm) :", np.array(dist))
    print("Period (s) :", np.array(period))
    N = np.array(period / Ts).astype(int)
    
    if inter_type==1:
        y_list = np.linspace(path_data[-1, 0], path_data[0, 0], N)
        x_list = np.interp(y_list, path_data[:, 0][::-1], path_data[:, 1][::-1])
    elif inter_type==2:
        x_list = np.linspace(path_data[-1, 1], path_data[0, 1], N)
        y_list = np.interp(x_list, path_data[:, 1][::-1], path_data[:, 0][::-1])
    else:
        print("Please check the given stroke path !!!")
    
    image_points = np.vstack((x_list, y_list)).transpose()

    x_1_list = x_list/ratio + center_shift[0]
    x_2_list = y_list/ratio + center_shift[1]
    
    print("x_list :", x_list)
    print("y_list :", y_list)
    
    task_points = np.vstack((x_1_list, x_2_list)).transpose()
    
    # print("x_1_list :::", x_1_list)
    # print("x_2_list :::", x_2_list)
    
    # inverse
    # x_1_list = np.hstack([x_1_list, x_1_list[::-1]])
    # x_2_list = np.hstack([x_2_list, x_2_list[::-1]])
    # x_1_list = np.hstack([x_1_list])
    # x_2_list = np.hstack([x_2_list])

    angle_1_list_e = []
    angle_2_list_e = []

    for t in range(1, N):
        x1 = x_1_list[t]
        x2 = x_2_list[t]
        
        point = np.array([x1, x2])
        
        angle = IK(point)
        
        # Inverse kinematics
        angle_1_list_e.append(np.round(angle[0].copy(), 5))
        angle_2_list_e.append(np.round(angle[1].copy(), 5))

    max_angle_1 = np.max(angle_1_list_e)
    max_angle_2 = np.max(angle_2_list_e)
    print("Max angle 1 (rad) :", max_angle_1)
    print("Max angle 2 (rad):", max_angle_2)
    if max_angle_1 < ANGLE_1_RANGE[0] or max_angle_1 > ANGLE_1_RANGE[1]:
        print("!!!!!! angle 1 is out of range !!!!!")
        print("max angle 1 :::", max_angle_1)
        exit()

    if max_angle_2 < ANGLE_2_RANGE[0] or max_angle_2 > ANGLE_2_RANGE[1]:
        print("!!!!!! angle 1 is out of range !!!!!")
        print("max angle 2 :::", max_angle_2)
        exit()

    way_points = np.vstack((angle_1_list_e, angle_2_list_e)).transpose()
    print('+' * 50)
    print("Check success with way_points :", way_points.shape[0])
    
    if plot_show:
        plot_stroke_path(period, traj, image_points, task_points, way_points)
    
    if save_path:
        np.savetxt('../control/angle_list_' + stroke_name + '.txt', way_points, fmt='%.05f')

    return way_points, image_points, task_points, period


def generate_word_path(
        traj_list,
        center_shift=np.array([-WIDTH/2, 0.23]),
        velocity=0.04,
        plot_show=False,
        save_path=False,
        word_name='Tian'):
    """
        generate word path
    """
    inter_list = np.ones(len(traj_list))
    inter_list[3] = 2
    word_angle_list = []
    word_image_points = []
    word_task_points = []
    period_list = []
    for stroke_index in range(len(traj_list)):
        # get one stroke
        traj = traj_list[stroke_index]
        
        stroke_angle_list, stroke_image_points, stroke_task_points, period = generate_stroke_path(traj, inter_type=inter_list[stroke_index],
                                                 center_shift=center_shift,
                                                 velocity=velocity, plot_show=plot_show,
                                                 save_path=save_path, stroke_name=stroke_index)
        
        word_angle_list.append(stroke_angle_list)
        word_image_points.append(stroke_image_points)
        word_task_points.append(stroke_task_points.copy())
        period_list.append(period)
        
    if plot_show:
        plot_word_path(period_list, traj_list, word_image_points, word_task_points, word_angle_list,
                       word_name=word_name)


def check_path(root_path='', plot_show=True, font_name='J_font',
               center_shift=np.array([-WIDTH/2, 0.0]), type=2, period=10, Ts=0.001):

    angle_1_range = np.array([-np.pi/2, np.pi/2])
    angle_2_range = np.array([-np.pi * 3/4, np.pi * 3/4])

    plt.rcParams['font.size'] = 16
    
    sns.set_theme()
    
    linewidth = 3
    N = np.array(period/Ts).astype(int)
    path_data = np.loadtxt(root_path + '/' + font_name + '/2_font_' + str(type) +'.txt')
    print("path_data :::", path_data[:, 0])
    print("path_data :::", path_data[:, 1])

    # # need to check which dimension can be applied for interp
    x_list = np.linspace(path_data[-1, 1], path_data[0, 1], N)
    # x_list = path(1, 2):(path(end, 2) - path(1, 2)) / (N - 1): path(end, 2)
    y_list = np.interp(x_list, path_data[:, 1][::-1], path_data[:, 0][::-1])
    print("x_list :::", x_list)
    print("y_list :::", y_list)
    # traj = path_data
    # N = np.array(period / Ts).astype(int)
    # M = N // len(traj)
    # x_list = []
    # y_list = []
    # for i in range(len(traj)-1):
    #     # need to check which dimension can be applied for interp
    #     x_list_i = np.linspace(path_data[i, 1], path_data[i + 1, 1], M)
    #     y_list_i = np.interp(x_list_i.copy(), path_data[i:i + 1, 1], path_data[i:i + 1, 0])
    #     x_list.append(x_list_i.copy())
    #     y_list.append(y_list_i.copy())

    # transform to Catersian space
    # ratio = 128/0.6
    ratio = IMAGE_WIDTH / WIDTH

    # center_shift = center_shift

    x_1_list = np.array(x_list).flatten() / ratio + center_shift[0]
    x_2_list = np.array(y_list).flatten() / ratio + center_shift[1]
    
    # x_1_list = x_list / ratio
    # x_2_list = y_list / ratio

    angle_1_list_e = []
    angle_2_list_e = []

    angle_vel_1_list_e = []
    angle_vel_2_list_e = []

    Length = [0.30, 0.150, 0.25, 0.125]
    L1 = Length[0]
    L2 = Length[2]

    x_1_list = np.hstack([x_1_list, x_1_list[::-1]])
    x_2_list = np.hstack([x_2_list, x_2_list[::-1]])

    print("x_1_list :::", x_1_list)
    print("x_2_list :::", x_2_list)
    t_list = np.linspace(0.0, 2*period, len(x_1_list))
    for t in range(1, len(x_1_list)):
        x2 = x_1_list[t]
        x1 = x_2_list[t]

        # Inverse kinematics
        L = x1**2 + x2**2

        gamma = math.atan2(x2, x1)

        cos_belta = (L1**2 + L - L2**2) / (2 * L1 * np.sqrt(L))

        if cos_belta > 1:
            angle_1 = gamma
        elif cos_belta < -1:
            angle_1 = gamma - np.pi
        else:
            angle_1 = gamma - math.acos(cos_belta)

        angle_1_list_e.append(np.round(angle_1, 5).copy())

        cos_alpha = (L1**2 - L + L2**2) / (2 * L1 * L2)

        if cos_alpha > 1:
            angle_2 = np.pi
        elif cos_alpha < -1:
            angle_2 = 0
        else:
            angle_2 = np.pi - math.acos(cos_alpha)

        angle_2_list_e.append(np.round(angle_2, 5).copy())
        
        if t == 1:
            angle_vel_1_list_e.append(0.0)
            angle_vel_2_list_e.append(0.0)
        else:
            angle_vel_1_list_e.append((angle_1_list_e[t-1] - angle_1_list_e[t-2])/0.001)
            angle_vel_2_list_e.append((angle_2_list_e[t-1] - angle_2_list_e[t-2])/0.001)

    max_angle_1 = np.max(angle_1_list_e)
    max_angle_2 = np.max(angle_2_list_e)
    if max_angle_1 < angle_1_range[0] or max_angle_1 > angle_1_range[1]:
        print("!!!!!! angle 1 is out of range !!!!!")
        print("max angle 1 :::", max_angle_1)
    if max_angle_2 < angle_2_range[0] or max_angle_2 > angle_2_range[1]:
        print("!!!!!! angle 1 is out of range !!!!!")
        print("max angle 2 :::", max_angle_2)

    if plot_show:
        fig = plt.figure(figsize=(15, 4))
        plt.subplot(1, 3, 1)
        plt.subplots_adjust(wspace=2, hspace=0)
    
        plt.plot(path_data[:, 1], path_data[:, 0], marker='o', linewidth=linewidth)
        plt.plot(x_list, y_list, linewidth=linewidth - 2)
        plt.xlim([0, 128])
        plt.ylim([0, 128])
        plt.xlabel('$x_1$')
        plt.ylabel('$x_2$')
        # plt.axis('equal')
        plt.tight_layout()
    
        plt.subplot(1, 3, 2)
        plt.subplots_adjust(wspace=0.2, hspace=0.2)
    
        plt.plot(x_1_list, x_2_list, linewidth=linewidth + 2, color='r')
    
        plt.xlim([-WIDTH/2., WIDTH/2])
        plt.ylim([0., WIDTH])
        plt.xlabel('$x_1$(m)')
        plt.ylabel('$x_2$(m)')
    
        plt.subplot(1, 3, 3)
        plt.plot(t_list[1:], angle_1_list_e, linewidth=linewidth, label='$q_1$')
        plt.plot(t_list[1:], angle_vel_1_list_e, linewidth=linewidth, label='$d_{q1}$')
        plt.plot(t_list[1:], angle_2_list_e, linewidth=linewidth, label='$q_2$')
        plt.plot(t_list[1:], angle_vel_2_list_e, linewidth=linewidth, label='$d_{q2}$')
        
        plt.xlabel('Time (s)')
        plt.ylabel('One-loop Angle (rad)')
        plt.legend()
    
        plt.show()
    
    # np.savetxt(root_path + '/' + font_name + '/2_font_' + str(type) +'_angle_1.txt', np.round(np.array(angle_1_list_e), 4))
    # np.savetxt(root_path + '/' + font_name + '/2_font_' + str(type) + '_angle_2.txt', np.round(np.array(angle_2_list_e), 4))
    
    np.savetxt(root_path + '/' + font_name + '/2_font_' + str(type) + '_angle_list.txt',
               np.hstack([np.array(angle_1_list_e).reshape(-1, 1), np.array(angle_2_list_e).reshape(-1, 1)]), delimiter=",")
    
    print("angle 1 list :::", len(angle_1_list_e))
    print("angle 2 list :::", len(angle_2_list_e))
    return np.array(angle_1_list_e), np.array(angle_2_list_e)


