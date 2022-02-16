from pickle import FALSE
from matplotlib.pyplot import plot
from numpy.core.arrayprint import format_float_scientific
from protocol.task_interface import *
import numpy as np
import math
import os
# from motor_control import motor_control
from path_planning.plot_path import *
from path_planning.path_generate import *
import time
import glob
import scipy
import argparse

# path prediction
from scipy.io import loadmat
from scipy.signal import savgol_filter
from forward_mode.utils.gmr import Gmr, plot_gmm
from forward_mode.utils.gp_coregionalize_with_mean_regression import GPCoregionalizedWithMeanRegression
from forward_mode.utils.gmr_mean_mapping import GmrMeanMapping
from forward_mode.utils.gmr_kernels import Gmr_based_kernel
import GPy
from utils.word_preprocess import *

from scipy import interpolate

sns.set(font_scale=1.5)
np.set_printoptions(precision=5)

L_1 = 0.3
L_2 = 0.25
action_dim = 3
DIST_THREHOLD = 0.05

FILE_FIG_NAME = './data/predicted_images/'
FILE_FONT_NAME = './data/font_data'
FILE_TRAIN_NAME = './data/training_data'
FILE_EVAL_NAME = './data/real_path_data'

ANGLE_1_RANGE = np.array([-1.90, 1.90])
ANGLE_2_RANGE = np.array([-2.2, 2.5])
center_shift = np.array([0.15, -WIDTH / 2])

FONT_SIZE = 20

WRITING_Y = [-WIDTH / 2, WIDTH / 2]
WRITING_X = [0.13, 0.13 + WIDTH]

# initial angle (rad) :::
Initial_angle = np.array([-1.31, 1.527])

Initial_point = np.array([0.32299, -0.23264])

Angle_initial = np.array([-0.333945, -0.102573, 1.981514])

# impedance params :::
Move_Impedance_Params = np.array([20.0, 20.0, 4.0, 0.2])


def train(angle_initial=Angle_initial, run_on=True, Load_path=False):

    _server = Server(5005)
    
    # ######################################################
    # ############## wait encoder and motor check ##########
    # ################### Position calibrate ###############
    # ######################################################
    _server.wait_encoder_request()
    curr_angle, curr_point = get_observation(angle_initial)
    _server.send_encoder_check(angle_initial)

    # move_to_target_point(Initial_angle)

    # ######################################################
    # ############## Wait way_points #######################
    # ######################################################
    _server.wait_way_points_request()

    print("+"*50)
    # receive way points
    way_points = []

    if Load_path:
        os.remove(r'data/real_path_data/angle_list.txt')
        data_file = open('data/real_path_data/angle_list.txt', 'w')
    
    way_point = None
    while way_point != "SEND_DONE":
        way_point = _server.read_way_points()
        # print("way_points ::::", way_point)
        if way_point == "SEND_DONE":
            break
        way_points.append(way_point.copy())
        
        line_data = str(way_point[0]) + ',' + str(way_point[1]) + '\n'
        
        if Load_path:
            data_file.writelines(line_data)
        # send_done = _server.wait_send_way_points_done()

    way_points = np.array(way_points)
    N_way_points = way_points.shape[0]
    # print("way_points :::", way_points.shape)
    print("N_way_points :::", N_way_points)
    print("+"*50)

    # ######################################################
    # ############## Wait impedance parameters  ############
    # ######################################################
    _server.wait_params_request()

    # impedance_params = None
    # while impedance_params is None:
    # read impedance parameters :::
    impedance_params = _server.read_params()
    impedance_params = np.array(impedance_params.copy())
    print("Input impedance parameters :::", np.array(impedance_params))
    print("+"*50)
    if impedance_params is np.NaN:
        exit()

    time.sleep(2.0)
    impedance_params = np.array([35.0, 24.0, 0.0, 0.0])
    
    # start move
    if not Load_path:
        # start_point = forward_ik(way_points[0, :].copy())
        # print("Move to start point :::", start_point)
        way_points = np.loadtxt('angle_list.txt', delimiter=',')
        N_way_points = way_points.shape[0]

    if run_on:
        initial_angle = np.zeros(2)
        initial_angle[0] = way_points[0, 0]
        initial_angle[1] = way_points[0, 1]
        start_point = forward_ik(initial_angle)
        move_impedance_params=np.array([20.0, 16.0, 0.1, 0.1])

        move_to_target_point(start_point, move_impedance_params, velocity=0.05)

        motor_control.run_one_loop(impedance_params[0], impedance_params[1], impedance_params[2], impedance_params[3],
                                    way_points[:, 0].copy(), way_points[:, 1].copy(), N_way_points,
                                    Angle_initial[0], Angle_initial[1], 1)
        
        time.sleep(2.0)
        move_to_target_point(Initial_point, move_impedance_params, velocity=0.05)

    # send movement_done command
    _server.send_movement_done()
    
    _server.close()


def eval_writing(run_on=True, Load_path=False):
    """
        eval writting performance :
    """
    
    _server = Server(5005)
    
    # ######################################################
    # ############## wait encoder and motor check ##########
    # ################### Position calibrate ###############
    # ######################################################
    _server.wait_encoder_request()
    curr_angle, curr_point = get_observation(Angle_initial)
    _server.send_encoder_check(curr_point)
    
    if not Load_path:
        print("Load stroke path !!!")
        stroke_angle = np.loadtxt('angle_list_0.txt', delimiter=' ')
        # N_way_points = stroke_angle.shape[0]
        # print("N_way_points :", N_way_points)
    
    # ######################################################
    # ############## Wait impedance parameters  ############
    # ######################################################
    _server.wait_params_request()
    
    # impedance_params = None
    # while impedance_params is None:
    # read impedance parameters :::
    while True:
        impedance_params = _server.read_params()
        impedance_params = np.array(impedance_params.copy())
        
        if impedance_params is np.NaN:
            exit()
        
        if impedance_params is not None:
            break
    
    time.sleep(1.0)
    # impedance_params = np.array([35.0, 24.0, 0.0, 0.0])
    print("Input impedance parameters :::", np.array(impedance_params))
    print("+" * 50)
    
    num_eval = 3
    for i in range(num_eval):
        print('Writting episode %d:' % i)
        if run_on:
            write_stroke(stroke_points=stroke_angle, impedance_params=np.array([35.0, 30.0, 1.4, 0.2]),
                         target_point=Initial_point)
            
            print("*" * 50)
            print("Eval one stroke once done !!!")
        
        # send movement_done command
        _server.send_movement_done()
    
    motor_control.motor_3_stop()
    _server.close()
    

def set_pen_up():
    """
        pull pen up
    """
    # motor_control.motor_3_stop()
    up_angle = np.int32(9000)
    done = motor_control.set_position(0.0, up_angle)
    time.sleep(1.0)

    return done


def set_pen_down():
    """
        pull pen down
    """
    # motor_control.motor_3_stop()
    down_angle = np.int32(11200)
    done = motor_control.set_position(0.0, down_angle)
    time.sleep(2.0)

    return done


def motor_stop():
    """
        sometimes need to stop motors
    """
    motor_control.motor_two_link_stop()
    motor_control.motor_3_stop()


def reset_and_calibration():
    print("Please make sure two links are at zero position !!!")
    angle_initial = np.zeros(3)
    
    angle_initial[0] = motor_control.read_initial_angle_1()
    angle_initial[1] = motor_control.read_initial_angle_2()
    angle_initial[2] = motor_control.read_initial_angle_3()
    
    return angle_initial


def get_demo_writting():
    """
        zero impedance control
    """
    buff_size = np.zeros((100, 2))
    impedance_params = np.array([35.0, 25.0, 0.4, 0.1])

    set_pen_down()
    motor_control.get_demonstration(Angle_initial[0], Angle_initial[1],
    2.0, 2.0, 0.0, 0.0, buff_size)


def get_observation(angle_initial=Angle_initial):
    """
        obtain joint angles and cartesian state
    """
    # ######################################################
    # ############## get current state #####################
    # ######################################################
    angle = np.zeros(action_dim)
    point = np.zeros(action_dim)
    
    print('+' * 20)
    angle[0] = motor_control.read_angle_1(angle_initial[0])
    # print("Joint 1 angles (rad) :", angle[0])
    angle[1] = motor_control.read_angle_2(angle_initial[1], angle[0].copy())
    # print("Joint 2 angles (rad) :", angle[1])
    # angle[2] = motor_control.read_angle_3(angle_initial[2])
    print("Joint angles (rad) :", np.array(angle))
    
    point[0] = L_1 * math.cos(angle[0]) + L_2 * math.cos(angle[0] + angle[1])
    point[1] = L_1 * math.sin(angle[0]) + L_2 * math.sin(angle[0] + angle[1])
    print("Position (m) :", np.array(point))
    
    return angle, point


def move_to_target_point(
    target_point,
    impedance_params=Move_Impedance_Params,
    velocity=0.05
):
    """
        move to target point
    """
    # done = False

    curr_angle, curr_point = get_observation()
    # dist = np.linalg.norm((curr_point - target_point), ord=2)
    # print("Curr_point (m) :", curr_point)
    # print("Initial dist (m) :", dist)

    angle_list, N = path_planning(curr_point[:2], target_point, velocity=velocity)
    # angle_list = np.loadtxt('angle_list.txt', delimiter=',', skiprows=1)

    N = angle_list.shape[0]

    angle_1_list = angle_list[:, 0].copy()
    angle_2_list = angle_list[:, 1].copy()

    dist_threshold = 0.05
    done = motor_control.move_to_target_point(
        impedance_params[0], impedance_params[1], impedance_params[2], impedance_params[3],
        angle_1_list, angle_2_list, N,
        Angle_initial[0], Angle_initial[1],
        dist_threshold
    )


def write_word(word_path, word_params=None, word_name='yi', epi_times=0):
    """
        write a word and plot
    """
    for index in range(len(word_path)):
        print("*" * 50)
        print("*" * 50)
        print("Write Stroke %d : "%index)
        stroke_points_index = word_path[index]

        if index < (len(word_path) - 1):
            next_index = index + 1
            stroke_points_next_index = word_path[next_index]

            target_angle = np.zeros(2)
            target_angle[0] = stroke_points_next_index[0, 0]
            target_angle[1] = stroke_points_next_index[0, 1]
            stroke_target_point = forward_ik(target_angle)
        else:
            stroke_target_point = Initial_point
        
        write_stroke(
            stroke_points=stroke_points_index,
            stroke_params=word_params[index],
            target_point=stroke_target_point,
            word_name=word_name,
            stroke_name=str(index),
            epi_time=epi_times
        )

        motor_stop()


def write_stroke(
        stroke_points=None,
        stroke_params=None,
        target_point=Initial_point,
        word_name='yi',
        stroke_name='0',
        epi_time=0
    ):
    way_points = stroke_points
    Num_way_points = way_points.shape[0]

    initial_angle = np.zeros(2)
    initial_angle[0] = way_points[0, 0]
    initial_angle[1] = way_points[0, 1]
    start_point = forward_ik(initial_angle)

    # move to target point
    done = set_pen_up()
    # time.sleep(0.5)
    
    done = move_to_target_point(start_point, Move_Impedance_Params, velocity=0.1)
    # time.sleep(0.5)

    done = set_pen_down()
    # time.sleep(0.5)
    
    # params_list = np.tile(impedance_params, (Num_way_points, 1))
    if stroke_params is None:
        exit()
    else:
        params_list = stroke_params

    folder_name = FILE_EVAL_NAME + '/' + word_name
    stroke_angle_name = folder_name + '/' + 'real_angle_list_' + stroke_name + '_' + str(epi_time) + '.txt'
    stroke_torque_name = folder_name + '/' + 'real_torque_list_' + stroke_name + '_' + str(epi_time) + '.txt'
    
    if os.path.exists(folder_name):
        pass
    else:
        os.makedirs(folder_name)

    done = motor_control.run_one_loop(
         way_points[:, 0].copy(), way_points[:, 1].copy(),
         params_list[:, 0].copy(), params_list[:, 1].copy(),
         params_list[:, 2].copy(), params_list[:, 3].copy(),
         Num_way_points,
         Angle_initial[0], Angle_initial[1],
         1,
         stroke_angle_name, stroke_torque_name
    )
    # print("curr_path_list", curr_path_list.shape)
    # np.savetxt('curr_path_list.txt', curr_path_list)
    
    # time.sleep(0.5)

    # move to target point
    done = set_pen_up()
    # time.sleep(0.5)

    done = move_to_target_point(target_point, Move_Impedance_Params, velocity=0.1)

    print("Write stroke once done !!!")
    print("*" * 50)

    return done


def eval_stroke(
    stroke_points=None,
    stroke_params=None,
    target_point=Initial_point,
    word_name='yi',
    stroke_index=0,
    epi_time=0
):
    Num_way_points = stroke_points.shape[0]
    
    initial_angle = np.zeros(2)
    initial_angle[0] = stroke_points[0, 0]
    initial_angle[1] = stroke_points[0, 1]
    start_point = forward_ik(initial_angle)
    
    # move to target point
    done = set_pen_up()
    done = move_to_target_point(start_point, Move_Impedance_Params, velocity=0.1)
    done = set_pen_down()
    
    # params_list = np.tile(impedance_params, (Num_way_points, 1))
    if stroke_params is None:
        exit()
    else:
        params_list = stroke_params

    folder_name = FILE_EVAL_NAME + '/' + word_name
    if os.path.exists(folder_name):
        pass
    else:
        os.makedirs(folder_name)
        
    stroke_angle_name = folder_name + '/' + 'real_angle_list_' + str(stroke_index) + '_' + str(
        epi_time) + '.txt'
    stroke_torque_name = folder_name + '/' + 'real_torque_list_' + str(stroke_index) + '_' + str(
        epi_time) + '.txt'
    done = motor_control.run_one_loop(
        stroke_points[:, 0].copy(), stroke_points[:, 1].copy(),
        params_list[:, 0].copy(), params_list[:, 1].copy(),
        params_list[:, 2].copy(), params_list[:, 3].copy(),
        Num_way_points,
        Angle_initial[0], Angle_initial[1],
        1,
        stroke_angle_name, stroke_torque_name
    )
    
    # move to target point
    done = set_pen_up()
    
    done = move_to_target_point(target_point, Move_Impedance_Params, velocity=0.1)
    
    print("Evaluate Stroke Once Done !!!")
    print("*" * 50)
    
    return done


def load_word_path(
    root_path='./data/font_data',
    word_name=None,
    task_params=None,
    joint_params=None
):
    # load original training way points :::
    word_file = root_path + '/' + word_name + '/'
    stroke_list_file = glob.glob(word_file + 'angle_list_*txt')
    print("Load stroke data %d", len(stroke_list_file))

    word_path = []
    word_joint_params = []
    word_task_params = []
    
    for i in range(len(stroke_list_file)):
        way_points = np.loadtxt(word_file + 'angle_list_' + str(i) + '.txt', delimiter=' ')

        if joint_params is not None:
            joint_params_list = np.tile(joint_params, (way_points.shape[0], 1))
        else:
            joint_params_list = np.loadtxt(word_file + 'params_list_' + str(i) + '.txt', delimiter=' ')
         
        N_way_points = way_points.shape[0]
        print("N_way_points :", N_way_points)

        word_path.append(way_points.copy())
        word_joint_params.append(joint_params_list.copy())
    
        # word parameters
        if task_params is not None:
            task_params_list = np.tile(task_params, (way_points.shape[0], 1))
        else:
            task_params_list = np.loadtxt(word_file + 'params_list_' + str(i) + '.txt', delimiter=' ')

        angle_list = way_points
        stiffness_list = task_params_list[:, :2]
        damping_list = task_params_list[:, 2:]
        joint_params_list = generate_stroke_stiffness_path(
            angle_list, stiffness_list, damping_list,
            save_path=False, save_root='', word_name='yi', stroke_index=0
        )
        
        word_task_params.append(joint_params_list)

    return word_path, word_joint_params, word_task_params


def generate_training_path(
    word_name='mu',
    stroke_index=3,
    epi_times=5,
    num_stroke=4,
    plot=False
):
    dt = 0.01
    font_size = 30
    input_dim = 1  # time
    output_dim = 2  # x, y
    re_sample_index = int(40),

    # training hyper-parameters
    in_idx = [0]
    out_idx = [1, 2]
    nb_states = 6
    nb_prior_samples = 10
    nb_posterior_samples = 5

    # ====================== data processing ======================
    word_path = load_real_word_path(
        root_path=FILE_EVAL_NAME,
        word_name=word_name + '_15',
        file_name='real_angle_list_',
        epi_times=epi_times,
        num_stroke=num_stroke,
        plot=False
    )

    train_word_path, _, _ = load_word_path(
        word_name=args.word_name,
        task_params=np.array([35, 35, 5, 0.5]),
        joint_params=np.array([35, 35, 5, 0.5]),
    )
    
    stroke_path = train_word_path[stroke_index]
    stroke_list = Forward_list(
        stroke_path=stroke_path
    )
    Num_way_point = stroke_path.shape[0]
    down_stroke_list, idx_list = fps(stroke_list, 0.002)
    idx_list = np.sort(idx_list)
    down_stroke_list = down_stroke_list[idx_list]
    # print('x_list :', down_stroke_list)
    down_stroke_list = np.array(down_stroke_list)
   
    # X_obs_list, Y_obs_list = scale_translate_process_main(
    #     down_stroke_list[:, 0].copy(), down_stroke_list[:, 1].copy(),
    #     scale_factor=SCALE_FACTOR,
    #     trans_value=TRANS_VALUE
    # )

    # =====================================================
    ############# process data before prediction ##########
    # =====================================================
    X_list, Y_list, X, Y, Xt, demos_np, nb_data = \
            eval_data_preprocess(
            word_path,
            stroke_index,
            epi_times=epi_times,
            re_sample_index=40,
            dt=0.01,
            plot=False
    )
    T = Xt[-1][0]
    print("Xt", T)
    
    # ===================== generate new samples =============
    X_obs_list, Y_obs_list, X_obs, Y_obs = \
        load_new_obs_path(
            Num_way_point,
            T,
            down_stroke_list[:, 0].copy(),
            down_stroke_list[:, 1].copy(),
            idx_list.copy()
        )
    
    # ========================================================
    # ========================= GMM ==========================
    # ========================================================
    gmr_model = Gmr(nb_states=nb_states, nb_dim=input_dim + output_dim, in_idx=in_idx, out_idx=out_idx)
    gmr_model.init_params_kbins(demos_np.T, nb_samples=epi_times)
    gmr_model.gmm_em(demos_np.T)

    # GMR prediction
    mu_gmr = []
    sigma_gmr = []
    for i in range(Xt.shape[0]):
        mu_gmr_tmp, sigma_gmr_tmp, H_tmp = gmr_model.gmr_predict(Xt[i])
        mu_gmr.append(mu_gmr_tmp)
        sigma_gmr.append(sigma_gmr_tmp)

    mu_gmr = np.array(mu_gmr)
    sigma_gmr = np.array(sigma_gmr)

    if plot:
        plt.figure(figsize=(8, 8))
        for p in range(epi_times):
            plt.plot(Y[p * nb_data:(p + 1) * nb_data, 0], Y[p * nb_data:(p + 1) * nb_data, 1], color=[.7, .7, .7])
            plt.scatter(Y[p * nb_data, 0], Y[p * nb_data, 1], color=[.7, .7, .7], marker='X', s=50)
        plt.plot(mu_gmr[:, 0], mu_gmr[:, 1], color=[0.20, 0.54, 0.93], linewidth=3)
        plt.scatter(mu_gmr[0, 0], mu_gmr[0, 1], color=[0.20, 0.54, 0.93], marker='X', s=50)
        plot_gmm(mu_gmr, sigma_gmr, alpha=0.05, color=[0.20, 0.54, 0.93])
        plt.scatter(Y_obs[:, 0], Y_obs[:, 1], color=[0, 0, 0], zorder=60, s=100)
        # plt.scatter(X_obs_list, Y_obs_list, color=[0, 0, 0], zorder=60, s=100)

        axes = plt.gca()
        # axes.set_xlim([0.1, 0.45])
        # axes.set_ylim([-0.25, 0.25])
        # axes.set_xlim([-10, 10])
        axes.set_xlim([-20, 20])
        axes.set_ylim([-20, 20])
        # axes.set_ylim([-10, 10])
        # axes.set_xlim([-15, 15])
        # axes.set_ylim([-15, 15])
        plt.xlabel('$x(m)$', fontsize=font_size)
        plt.ylabel('$y(m)$', fontsize=font_size)
        plt.locator_params(nbins=3)
        plt.tick_params(labelsize=font_size)
        plt.tight_layout()
        plt.title(word_name, fontsize=font_size)
        plt.savefig(FILE_FIG_NAME + '/' + word_name + '/' + 'GMR_' + word_name + '_stroke_' + str(stroke_index) + '.pdf')

        plt.show()

    # ========================================================
    # ========================= GPR ==========================
    # ========================================================
    # Define GPR likelihood and kernels ::: original : 0.01
    nb_data_test = Xt.shape[0]
    Xtest, _, output_index = GPy.util.multioutput.build_XY([np.hstack((Xt, Xt)) for i in range(output_dim)])

    likelihoods_list = [GPy.likelihoods.Gaussian(name="Gaussian_noise_%s" %j, variance=1) for j in range(output_dim)]
    # kernel_list = [GPy.kern.Matern52(1, variance=5., lengthscale=0.5) for i in range(gmr_model.nb_states)]
    kernel_list = [GPy.kern.RBF(1, variance=10, lengthscale=0.5) for i in range(gmr_model.nb_states)]
    # kernel_list = [GPy.kern.RBF(1, variance=5, lengthscale=2) for i in range(gmr_model.nb_states)]

    # Fix variance of kernels
    for kernel in kernel_list:
        kernel.variance.fix(1.0)
        kernel.lengthscale.constrain_bounded(1.5, 10.)

    # Bound noise parameters
    for likelihood in likelihoods_list:
        likelihood.variance.constrain_bounded(0.001, 0.05)

    # GPR model
    K = Gmr_based_kernel(gmr_model=gmr_model, kernel_list=kernel_list)
    mf = GmrMeanMapping(2 * input_dim + 1, 1, gmr_model)
    m = GPCoregionalizedWithMeanRegression(
        X_list, Y_list,
        kernel=K,
        likelihoods_list=likelihoods_list,
        mean_function=mf
    )

    # Parameters optimization
    m.optimize('bfgs', max_iters=200, messages=True)

    # Print model parameters
    print(m)

    # GPR prior (no observations)
    prior_traj = []
    prior_mean = mf.f(Xtest)[:, 0]
    prior_kernel = m.kern.K(Xtest)
    for i in range(nb_prior_samples):
        # print("prior_kernel :", prior_kernel.shape)
        prior_traj_tmp = np.random.multivariate_normal(prior_mean, prior_kernel)
        # print("prior_traj_tmp :", prior_traj_tmp.shape)
        prior_traj.append(np.reshape(prior_traj_tmp, (output_dim, -1)))
    prior_kernel_tmp = np.zeros((nb_data_test, nb_data_test, output_dim * output_dim))
    for i in range(output_dim):
        for j in range(output_dim):
            prior_kernel_tmp[:, :, i * output_dim + j] = prior_kernel[i * nb_data_test:(i + 1) * nb_data_test, j * nb_data_test:(j + 1) * nb_data_test]
    prior_kernel_rshp = np.zeros((nb_data_test, output_dim, output_dim))
    for i in range(nb_data_test):
        prior_kernel_rshp[i] = np.reshape(prior_kernel_tmp[i, i, :], (output_dim, output_dim))

    # GPR posterior -> new points observed (the training points are discarded as they are "included" in the GMM)
    m_obs = \
        GPCoregionalizedWithMeanRegression(
            X_obs_list, Y_obs_list, kernel=K,
            likelihoods_list=likelihoods_list,
            mean_function=mf
        )
    mu_posterior_tmp = \
        m_obs.posterior_samples_f(
            Xtest, full_cov=True, size=nb_posterior_samples
        )

    mu_posterior = []
    for i in range(nb_posterior_samples):
        mu_posterior.append(np.reshape(mu_posterior_tmp[:, 0, i], (output_dim, -1)))

    # GPR prediction
    mu_gp, sigma_gp = m_obs.predict(Xtest, full_cov=True, Y_metadata={'output_index': output_index})
    mu_gp_rshp = np.reshape(mu_gp, (output_dim, -1)).T
    sigma_gp_tmp = np.zeros((nb_data_test, nb_data_test, output_dim * output_dim))
    for i in range(output_dim):
        for j in range(output_dim):
            sigma_gp_tmp[:, :, i * output_dim + j] = sigma_gp[i * nb_data_test:(i + 1) * nb_data_test, j * nb_data_test:(j + 1) * nb_data_test]
    sigma_gp_rshp = np.zeros((nb_data_test, output_dim, output_dim))
    for i in range(nb_data_test):
        sigma_gp_rshp[i] = np.reshape(sigma_gp_tmp[i, i, :], (output_dim, output_dim))
    # print("sigma_gp_rshp", sigma_gp_rshp.shape)

    if plot:
        # Posterior
        plt.figure(figsize=(8, 8))
        plt.plot(mu_gmr[:, 0], mu_gmr[:, 1], color=[0.20, 0.54, 0.93], linewidth=3.)
        plot_gmm(mu_gp_rshp, sigma_gp_rshp, alpha=0.05, color=[0.83, 0.06, 0.06])
        for i in range(nb_posterior_samples):
            plt.plot(mu_posterior[i][0], mu_posterior[i][1], color=[0.64, 0., 0.65], linewidth=1.5)
            plt.scatter(mu_posterior[i][0, 0], mu_posterior[i][1, 0], color=[0.64, 0., 0.65], marker='X', s=80)

        plt.plot(mu_gp_rshp[:, 0], mu_gp_rshp[:, 1], color=[0.83, 0.06, 0.06], linewidth=3.)
        plt.scatter(mu_gp_rshp[0, 0], mu_gp_rshp[0, 1], color=[0.83, 0.06, 0.06], marker='X', s=80)
        plt.scatter(Y_obs[:, 0], Y_obs[:, 1], color=[0, 0, 0], zorder=60, s=100)

        axes = plt.gca()
        axes.set_xlim([-20, 20])
        # axes.set_xlim([-10, 10])
        axes.set_ylim([-20, 20])
        # axes.set_ylim([-10, 10])
        # axes.set_xlim([-15, 15])
        # axes.set_ylim([-15, 15])
        plt.xlabel('$x(m)$', fontsize=font_size)
        plt.ylabel('$y(m)$', fontsize=font_size)
        plt.locator_params(nbins=3)
        plt.tick_params(labelsize=font_size)
        plt.tight_layout()
        plt.savefig(FILE_FIG_NAME + '/' + word_name + '/' + 'GMRbGP_' + word_name + '_stroke_' + str(
            stroke_index) + '_posteriors.pdf')

        plt.show()

    print("generated trajectories :", np.array(mu_posterior).shape)
    folder_name = FILE_TRAIN_NAME + '/' + word_name
    if os.path.exists(folder_name):
        pass
    else:
        os.makedirs(folder_name)
    np.save(folder_name + '/training_stroke_' + str(stroke_index) + '_samples.npy', np.array(mu_posterior))
    return mu_posterior


def training_samples_to_waypoints(
    word_name='mu',
    stroke_index=0,
    Num_waypoints=10000,
    sample_index=0,
    task_params=None,
    joint_params=None,
    desire_angle_list=None,
    plot=True
):
    print("============== {} ============".format('Load Training Samples !!!'))
    folder_name = FILE_TRAIN_NAME + '/' + word_name + '/training_stroke_' + str(stroke_index) + '_samples.npy'
    training_samples = np.load(folder_name)
    angle_list = np.zeros((Num_waypoints, 2))
    
    nb_posterior_samples = training_samples.shape[0]
    data_sample = training_samples.shape[2]
    index = np.linspace(0, data_sample-1, data_sample)
    index_list = np.linspace(0, data_sample-1, Num_waypoints)
    print("training_samples :", training_samples.shape, data_sample, index.shape, training_samples[sample_index][0].shape)
    
    # x_list = np.linspace(training_samples[0][0][0], training_samples[0][0][-1], Num_waypoints)
    # x_list = np.linspace(training_samples[0][0][0], training_samples[0][0][-1], Num_waypoints)
    x_list_ori, y_list_ori = de_scale_translate_process_main(training_samples[sample_index][0], training_samples[sample_index][1])
    
    fx = interpolate.interp1d(index, x_list_ori, kind='linear')
    fy = interpolate.interp1d(index, y_list_ori, kind='linear')
    x_list = fx(index_list)
    y_list = fy(index_list)
    
    trajectory_list = Forward_list(stroke_path=desire_angle_list)
    print(trajectory_list)
    # path_data[:, 0] = savgol_filter(traj[:, 0], filter_size, 3, mode='nearest')
    # path_data[:, 1] = savgol_filter(traj[:, 1], filter_size, 3, mode='nearest')
    
    if plot:
        # Posterior
        plt.figure(figsize=(8, 8))
        # for i in range(nb_posterior_samples):
        #     plt.plot(training_samples[i][0], training_samples[i][1], color=[0.64, 0., 0.65], linewidth=1.5)
        #     plt.scatter(training_samples[i][0, 0], training_samples[i][1, 0], color=[0.64, 0., 0.65], marker='X', s=80)
        plt.plot(x_list, y_list)
        plt.plot(trajectory_list[:, 0], trajectory_list[:, 1], color='black', linewidth=3)
        axes = plt.gca()
        axes.set_xlim(WRITING_X)
        axes.set_ylim(WRITING_Y)
        # # axes.set_xlim([-10, 10])
        # axes.set_ylim([-10, 10])
        # axes.set_xlim([-15, 15])
        # axes.set_ylim([-15, 15])
        plt.xlabel('$x(m)$', fontsize=FONT_SIZE)
        plt.ylabel('$y(m)$', fontsize=FONT_SIZE)
        plt.locator_params(nbins=3)
        plt.tick_params(labelsize=FONT_SIZE)
        plt.tight_layout()
        plt.title(word_name, fontsize=FONT_SIZE)
        # plt.savefig(FILE_FIG_NAME + '/' + word_name + '/' + 'GMRbGP_' + word_name + '_stroke_' + str(
        #     stroke_index) + '_posteriors.pdf')
        plt.show()
    
    for i in range(Num_waypoints):
        angle = IK(np.array([x_list[i], y_list[i]]))
        angle_list[i, :] = angle
    
    max_angle_1 = np.max(angle_list[:, 0])
    max_angle_2 = np.max(angle_list[:, 1])
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

    print("angle_list_shape :", angle_list.shape)
    
    joint_params_list = load_impedance_list(
        word_name=word_name,
        stroke_index=stroke_index,
        desired_angle_list=angle_list,
        current_angle_list=angle_list,
        joint_params=joint_params,
        task_params=task_params
    )
    return angle_list, joint_params_list


def load_impedance_list(
        word_name='mu',
        stroke_index=0,
        desired_angle_list=None,
        current_angle_list=None,
        joint_params=None,
        task_params=None
):
    print("============== {} ============".format('Load Impedance !!!'))
    way_points = desired_angle_list
    N_way_points = way_points.shape[0]
    
    # joint parameters
    if joint_params is not None:
        joint_params_list = np.tile(joint_params, (way_points.shape[0], 1))
    else:
        joint_params_list = np.loadtxt(FILE_TRAIN_NAME + '/' + word_name + '/' + 'params_stroke_list_' + str(stroke_index) + '.txt', delimiter=' ')
    
    # task parameters
    if task_params is not None:
        task_params_list = np.tile(task_params, (way_points.shape[0], 1))
    else:
        task_params_list = np.loadtxt(FILE_TRAIN_NAME + '/' + word_name + '/' + 'params_stroke_list_' + str(stroke_index) + '.txt', delimiter=' ')
    
    stiffness_list = task_params_list[:, :2]
    damping_list = task_params_list[:, 2:]
    joint_params_list = generate_stroke_stiffness_path(
        desired_angle_list, stiffness_list, damping_list,
        save_path=False, save_root=FILE_TRAIN_NAME, word_name=word_name, stroke_index=stroke_index
    )
    
    return joint_params_list


def main(args):
    # ===========================================================

    # motor stop
    # motor_control.motor_two_link_stop()

    # ===========================================================
    if args.hard_test:

        # theta_1 = motor_control.read_initial_angle_1()
        # print("theta_1 :", theta_1)
        # theta_2 = motor_control.read_initial_angle_2()
        # print("theta_2 :", theta_2)
        # theta_3 = motor_control.read_initial_angle_3()
        # print("theta_3 :", theta_3)

        angle, point = get_observation()
        print("angle :", angle)
        print("point :", point)

        # set_pen_up()
        # # set_pen_down()

        motor_stop()

        # target_point = np.array([0.40, -0.15])
        # move_to_target_point(
        #     target_point,
        #     # impedance_params=np.array([420.0, 420.0, 15.5, 15.5]),
        #     impedance_params=np.array([30.0, 30.0, 4.0, 0.2]),
        #     velocity=0.1
        # )
        
        # time.sleep(0.2)
        # angle, point = get_observation()
        # print("angle :", angle)
        # print("point :", point)

        # stiff_joint, damping_joint = Stiff_convert(np.array([0.5, 0.5])
        # np.diag([40, 40]), np.diag([1.0, 1.0]))
        # print("stiff_joint :", stiff_joint)
        # print("damping_joint :", damping_joint)
        # motor_control.Convert_stiffness(40.0, 40.0, 0.5, 0.5)
    
    # ===========================================================
    if args.assist == True:
        eval_times = 1
        word_path, word_joint_params, word_task_params = load_word_path(
            word_name=args.word_name,
            task_params=np.array([35, 35, 5, 0.5]),
            joint_params=np.array([35, 35, 5, 0.5]),
            )
        
        word_params = word_joint_params
        # evaluation writing
        for i in range(eval_times):
            # word
            write_word(word_path, word_params=word_params, word_name=args.word_name, epi_times=i)
            # stroke
    
    
    # ===========================================================
    if args.eval == True:
        # eval_times = 1
        word_path, word_joint_params, word_task_params = load_word_path(
            word_name=args.word_name,
            task_params=np.array([15, 15, 1, 0.5]),
            joint_params=np.array([15, 15, 1, 0.5]),
            )

        angle_list = word_path[args.stroke_index]
        Num_waypoints = angle_list.shape[0]
        print("word_one_stroke_num_way_points :", Num_waypoints)
        
        # stroke_training_samples = generate_training_path(
        #     word_name=args.word_name,
        #     stroke_index=args.stroke_index,
        #     epi_times=5,
        #     num_stroke=1,
        #     plot=True
        # )
        
        # joint_params_list = load_impedance_list(
        #     word_name=args.word_name,
        #     stroke_index=args.stroke_index,
        #     desired_angle_list=angle_list,
        #     current_angle_list=angle_list,
        #     task_params=np.array([35, 35, 5, 0.5]),
        #     joint_params=np.array([35, 35, 5, 0.5])
        # )
        
        stroke_points, joint_params_list = \
            training_samples_to_waypoints(
                word_name=args.word_name,
                stroke_index=args.stroke_index,
                Num_waypoints=Num_waypoints,
                sample_index=2,
                task_params=np.array([35, 35, 5, 0.5]),
                joint_params=np.array([35, 35, 5, 0.5]),
                desire_angle_list=angle_list,
                plot=True
        )
        
        # evaluation writing
        # for i in range(args.eval_times):
        #     # write_word(word_path, word_params=word_params, word_name=write_name, epi_times=i)
        #     eval_stroke(
        #         stroke_points=word_path[args.stroke_index],
        #         stroke_params=word_joint_params[args.stroke_index],
        #         target_point=Initial_point,
        #         word_name=args.word_name + '_15',
        #         stroke_index=args.stroke_index,
        #         epi_time=i
        #     )
        #
        # motor_stop()

    
    # ===========================================================
    if args.plot_word == True:

        # plot_real_stroke_2d_path(
        #     root_path='./data/font_data/xing/',
        #     file_name='angle_list_5',
        #     stroke_num=5,
        #     delimiter=' ',
        #     skiprows=1
        # )

        # plot_real_2d_path(
        #     root_path=FILE_EVAL_NAME +'/' + args.word_name + '/',
        #     file_name='real_angle_list_',
        #     stroke_num=1,
        #     epi_times=args.eval_times,
        #     delimiter=',',
        #     skiprows=1
        # )
        
        # word_path, word_joint_params, word_task_params= \
        #     load_word_path(
        #         word_name=write_name,
        #         task_params=np.array([150, 150, 1, 0.5]),
        #         joint_params=np.array([20, 20, 1, 0.5])
        # )
        
        # print("task", np.array(word_joint_params[0]).shape)
        # plot_impedance_path(
        #     word_joint_params=word_joint_params,
        #     word_task_params=word_task_params,
        #     line_width=3.0
        # )

        plot_torque_path(
            root_path=FILE_EVAL_NAME + '/' + args.word_name + '/',
            file_name='real_torque_list_',
            stroke_num=1,
            epi_time=args.eval_times,
            delimiter=',',
            skiprows=1,
            render=True
        )

        # plot_velocity_path(
        #     root_path=FILE_EVAL_NAME + '/' + args.word_name + '/',
        #     file_name='real_angle_list_',
        #     stroke_num=1,
        #     epi_time=0,
        #     delimiter=',',
        #     skiprows=1
        # )

        # plot_real_error_path(
        #     root_path='./data/font_data/' + write_name + '/',
        #     file_name='real_angle_list_',
        #     stroke_num=6,
        #     epi_num=5,
        #     delimiter=' ',
        #     skiprows=1
        # )

        # plot_real_2d_demo_path(
        # root_path='',
        # file_name=write_name,
        # delimiter=',',
        # skiprows=1
        # )


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument('--hard_test', type=bool, default=False, help='hardware design')
    parser.add_argument('--assist', type=bool, default=False, help='assist mode')
    parser.add_argument('--eval', type=bool, default=False, help='evaluate writing results')
    parser.add_argument('--plot_word', type=bool, default=False, help='whether plot results')
    parser.add_argument('--word_name', type=str, default='yi', help='give write word name')
    parser.add_argument('--stroke_index', type=int, default=0, help='give write word name')
    parser.add_argument('--sample_index', type=int, default=0, help='give write word name')
    parser.add_argument('--file_name', type=str, default='real_angle_list_', help='give write word name')

    parser.add_argument('--eval_times', type=int, default=1, help='give write word name')

    args = parser.parse_args()

    main(args)

    # load_eval_path(
    #     root_path='./data/real_path_data',
    #     word_name=None,
    #     epi_times=5
    # )

    # word_path = cope_real_word_path(
    #     root_path='./data/font_data',
    #     word_name='mu',
    #     file_name='real_angle_list_',
    #     epi_times=5,
    #     num_stroke=4,
    #     plot=True
    # )
    # print(trajectory_list.shape)

    # predict_training_samples(
    #     word_name='mu',
    #     stroke_index=0,
    #     re_sample_index=20,
    #     epi_times=5,
    #     num_stroke=4,
    #     plot=True
    # )

    # training_samples_to_waypoints(
    #     word_name='mu'
    # )

