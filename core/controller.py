from tools import skeletonize
from utils import hungarian_matching
import numpy as np
import cv2
import argparse
import time
import glob

# task interface
from control.protocol.task_interface import TCPTask
from control.path_planning.path_generate import *
from control.vision_capture.main_functions import capture_image, show_video


def draw_points(points, canvas_size=256):

    canvas = np.zeros((canvas_size, canvas_size, 3), dtype=np.uint8) + 255

    for point in points:
        cv2.circle(canvas, tuple(point*2), 2, (0, 0, 0), -1)

    return canvas


def draw_matching(points_1, points_2, matching, canvas_size=256):

    points_1 = 2 * points_1
    points_2 = 2 * points_2
    canvas = np.zeros((canvas_size, canvas_size, 3), dtype=np.uint8) + 255

    for point in points_1:
        cv2.circle(canvas, tuple(point), 2, (255, 0, 0), -1)

    for point in points_2:
        cv2.circle(canvas, tuple(point), 2, (0, 255, 0), -1)

    for match in matching:
        print(tuple(points_1[match[0]]))
        cv2.line(canvas, tuple(points_1[match[0]]), tuple(
            points_2[match[0]]), (0, 0, 0))

    return canvas


class Controller(object):

    def __init__(self, args, img_processor=None, impedance_level=0) -> None:
        self.args = args

        self.root_path = '../control/data/'
        self.show_video = True

        # initial TCP connection :::
        # self.task = TCPTask('169.254.0.99', 5005)

        self.img_processor = img_processor
        self.x_impedance_level = impedance_level
        self.y_impedance_level = impedance_level

        # impedance parameters
        self.action_dim = 2
        self.stiffness_high = np.array([10.0, 10.0])
        self.stiffness_low = np.array([0.0, 0.0])
        self.stiffness = np.zeros(self.action_dim)
        self.damping = np.zeros_like(self.stiffness)

        self.scalar_distance = False

    def guide(self,):
        """[summary]

        Raises:
            NotImplementedError: [description]
        """
        raise NotImplementedError

    def key_point_matching(self, tgt_pts, in_pts):

        matching = hungarian_matching(tgt_pts, in_pts)
        matching = np.array(matching)

        return matching

    def update_impedance(self, target, input):
        """[summary]

        Args:
            input ([image]): written image
            target ([image]): target image to be learnt
        """
        if self.img_processor is not None:
            input = self.img_processor.process(input)

        tgt_pts, _ = skeletonize(~target)
        in_pts, _ = skeletonize(~input)

        tgt_pts = np.array(tgt_pts)
        if len(in_pts) > 1:
            in_pts = np.array([in_pts[1]])
        else:
            in_pts = np.array(in_pts)

        tgt_pts = np.squeeze(tgt_pts, axis=0)
        in_pts = np.squeeze(in_pts, axis=0)

        # tgt_pts_vis = draw_points(in_pts)
        # cv2.imwrite('tgt_pts_vis.jpg', tgt_pts_vis)

        # tgt_pts_vis = draw_points(tgt_pts)
        # cv2.imwrite('tgt_pts_vis.jpg', tgt_pts_vis)

        # in_pts_vis = draw_points(in_pts)
        # cv2.imwrite('in_pts_vis.jpg', in_pts_vis)

        matching = self.key_point_matching(tgt_pts, in_pts)
        matching_vis = draw_matching(tgt_pts, in_pts, matching)
        # cv2.imwrite('matching_vis.jpg', matching_vis)

        tgt_index = matching[:, 0]
        in_index = matching[:, 1]

        x_dis = abs(tgt_pts[tgt_index][:, 0] -
                    in_pts[in_index][:, 0])
        y_dis = abs(tgt_pts[tgt_index][:, 1] -
                    in_pts[in_index][:, 1])

        if self.scalar_distance:
            x_dis = sum(x_dis) / matching.shape[0]
            y_dis = sum(y_dis) / matching.shape[0]

        self.impedance_update_policy(x_dis, y_dis,tgt_pts[tgt_index])

        return x_dis, y_dis, tgt_pts[tgt_index]

    def impedance_update_policy(self, x_dis, y_dis, corresponding_points):
        """ Linear update based on the displacement

        Args:
            x_dis ([type]): [description]
            y_dis ([type]): [description]
        """
        self.x_impedance_level = x_dis / 128
        self.y_impedance_level = y_dis / 128

        # # send impedance params :::
        self.stiffness = [100, 100]
        self.damping = [50, 50]

        return self.stiffness, self.damping

    def get_current_path(self):

        pass

    def update_velocity(self, ):
        """ update velocity with user's input get from demonstration

            args: velocity: m/s
        """
        velocity = 0.04
        return velocity

    def interact_once(self, traj, impedance_params=[5.0, 5.0, 0.2, 0.2], velocity=0.04, mode='eval'):
        """
            interact with robot once
        """
        # check motor and encode well before experiments
        print('+' * 30, 'Hardware check', '+' * 50)
        angle_initial = self.task.wait_encoder_check()
        print("Current State (rad) :", angle_initial)

        if mode=='train':
            # check the whole path
            print('+' * 30, 'Check Path', '+' * 50)
            way_points = generate_stroke_path(traj,
                                              center_shift=np.array([0.16, -WIDTH / 2]),
                                              velocity=velocity, Ts=0.001,
                                              plot_show=False)

            print('+' * 30, 'Start Send Waypoints !', '+' * 50)
            self.task.send_way_points_request()
            self.task.send_way_points(way_points)

        # self.task.send_way_points_done()
        print('+' * 20, 'Start Send Impedance Parameters !', '+' * 30)
        self.task.send_params_request()
        print("Stiffness :", impedance_params[:2])
        print("Damping :", impedance_params[2:])
        self.task.send_params(impedance_params)

        if self.args.show_video:
            show_video()

        print('+' * 50, 'Start Move !', '+' * 50)
        
        # start_time = time.time()
        run_done = False
        index = 0
        while True:
            # video record for trail :::
            run_done = self.task.get_movement_check()
    
            if run_done:
                print('+' * 50, 'Start Capture Image !', '+' * 50)
                # print("run_done", run_done)
                written_image, _ = capture_image(
                    root_path=self.root_path + 'captured_images/', font_name='written_image_test' + '_' + str(index))
                index += 1
                run_done = False
        
        # self.task.close()
        
    def interact(self, target_img):
        written_image = None
        num_episodes = 5
        run_done = False

        for i in range(num_episodes):
            # update impedance
            if written_image is not None:
                self.update_impedance(target_img, written_image)
            params = self.stiffness + self.damping

            # update velocity
            velocity = self.update_velocity()

        return written_image


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='Runs a learning example on a registered gym environment.')

    parser.add_argument('--show_video',
                        default=False,
                        help='enables useful debug settings')

    args = parser.parse_args()

    root_path = '../control/data/font_data'
    folder_name = 'ren'
    font_name = '人'
    type = 1
    
    stroke_list_file = glob.glob(root_path + '/' + folder_name + '/' + font_name + '_*.txt')
    num_stroke = len(stroke_list_file)
    traj_list = []

    for str_index in range(num_stroke):
        traj = np.loadtxt(root_path + '/' + folder_name + '/' +
                               font_name + '_' + str(str_index) + '_font' + str(type) + '.txt')
        traj_list.append(traj)

    inter_list = np.ones(len(traj_list))
    inverse_list = np.ones(len(traj_list))
    inverse_list[0] = False
    inter_list[0] = 2
    # inter_list[1] = 2
    generate_word_path(
        traj_list,
        inter_list,
        inverse_list,
        center_shift=np.array([0.16, -WIDTH / 2]),
        velocity=0.04,
        plot_show=True,
        save_path=True,
        word_name=folder_name)
    
    # generate_stroke_path(traj_list[2],
    #               inter_type=1,
    #               center_shift=np.array([0.16, -WIDTH / 2]),
    #               velocity=0.04,
    #               Ts=0.001,
    #               plot_show=True,
    #               save_path=False,
    #               stroke_name=str(0)
    #               )

    # way_points = np.loadtxt('../control/angle_list_1_1.txt', delimiter=' ')
    # N_way_points = way_points.shape[0]
    # # print("N_way_points :", N_way_points)
    # # word_path.append(way_points.copy())
    # angle_point_1 = way_points[-1, :]
    # end_point = forward_ik(angle_point_1)
    #
    # way_points_2 = np.loadtxt('../control/angle_list_1_2.txt', delimiter=' ')
    # # N_way_points = way_points_2.shape[0]
    # # print("N_way_points :", N_way_points)
    # # word_path.append(way_points.copy())
    # angle_point_2 = way_points_2[0, :]
    # start_point = forward_ik(angle_point_2)
    #
    # angle_list, N = path_planning(end_point, start_point, velocity=0.04)
    # print("angle", angle_list.shape)
    #
    # angle_list_1 = np.vstack([way_points, angle_list, way_points_2])
    # print(angle_list_1.shape)
	#
    # fig = plt.figure(figsize=(15, 4))
    # plt.plot(angle_list_1[:, 0], linewidth=linewidth, label='$q_1$')
    # # plt.plot(t_list[1:], angle_vel_1_list_e, linewidth=linewidth, label='$d_{q1}$')
    # plt.plot(angle_list_1[:, 1], linewidth=linewidth, label='$q_2$')
    # # plt.plot(t_list[1:], angle_vel_2_list_e, linewidth=linewidth, label='$d_{q2}$')
    # plt.show()
    # np.savetxt('../control/angle_list_1.txt', angle_list_1.copy(), fmt='%.05f')
    
    # str_index = 0
    # traj = np.loadtxt(root_path + '/' + folder_name + '/' +
    #                        font_name + '_' + str(str_index) + '_font' + str(type) + '.txt')
    #
    # writing_controller = Controller(
    #     args, img_processor=None, impedance_level=0)
    #
    # writing_controller.interact_once(
    #     traj, impedance_params=[35.0, 25.0, 0.5, 0.1], velocity=0.04, mode='eval')

    # target_img = cv2.imread(root_path + '/1_font_1.png')
    # writing_controller.interact(path_data, target_img)

    # a = np.array([(3, 4), (7, 8)])
    # b = np.array([(1, 2), (3, 4), (5, 6)])
    # from imgprocessor import Postprocessor
    # c = Controller(Postprocessor(
    #     {'ROTATE': 0, 'BINARIZE': 128}), img_processor=Postprocessor(
    #     {'ROTATE': 0, 'BINARIZE': 128}))
    #
    # written_stroke = cv2.imread('./example/written_image.png')
    # sample_stroke = cv2.imread('./example/example_traj.png', cv2.IMREAD_GRAYSCALE)

    # root_path = '../control/data/captured_images/'
    # sample_stroke, ori_img = capture_image(root_path=root_path, font_name='written_image_word')
    # cv2.imshow('', ori_img)

    # cv2.waitKey(0)

    # show_video()

    # x_dis, y_dis, tgt = c.update_impedance(sample_stroke, written_stroke)
    # print(x_dis, y_dis, tgt)
    # matching = c.key_point_matching(a, b)
