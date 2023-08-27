import argparse
import os
import glob
import numpy as np
import cv2


def load_ref_data(ref_path):
    """
    input the is the path to the reference trajectory, 
    """

    # get all the character folders
    char_folders = glob.glob(os.path.join(ref_path, '*'))

    ref_data = {}

    for char_folder in char_folders:
        # extract the character name
        char_name = char_folder.split('/')[-1]

        # get all the stroke folders, exlude files
        stroke_folders = glob.glob(os.path.join(char_folder, 'stroke_*'))

        # read the stroke data
        stroke_data = []
        for stroke_folder in stroke_folders:

            # get all the stroke files
            stroke_files = glob.glob(os.path.join(stroke_folder, '*.txt'))

            # read the stroke data
            for stroke_file in stroke_files:
                stroke = np.loadtxt(stroke_file)
                stroke_data.append(stroke)

        ref_data[char_name] = stroke_data

    return ref_data


def load_written_data(written_path):
    """
    input the is the path to the written trajectory, get the stroke data
    """

    # get all the character folders
    char_folders = glob.glob(os.path.join(written_path, '*'))

    written_data = {}

    for char_folder in char_folders:
        # extract the character name
        char_name = char_folder.split('/')[-1]

        # get all the stroke folders, exlude files
        user_folders = glob.glob(os.path.join(char_folder, '*'))

        # read the stroke data
        user_data = {}
        for user_folder in user_folders:

            user_name = user_folder.split('/')[-1]
            stroke_folders = glob.glob(os.path.join(user_folder, 'stroke_*'))
            stroke_data = []

            for stroke_folder in stroke_folders:

                # get all the stroke files
                stroke_files = glob.glob(os.path.join(stroke_folder, '*.txt'))

                # read the stroke data
                for stroke_file in stroke_files:
                    stroke = np.loadtxt(stroke_file)
                    stroke_data.append(stroke)

                user_data[user_name] = stroke_data

            written_data[char_name] = user_data
    return written_data


def dtw(ref_stroke, written_stroke):

    # compute the distance matrix
    dist_mat = np.zeros((len(ref_stroke), len(written_stroke)))

    for i in range(len(ref_stroke)):
        for j in range(len(written_stroke)):
            dist_mat[i, j] = np.linalg.norm(ref_stroke[i] - written_stroke[j])

    # compute the accumulated distance matrix
    acc_dist_mat = np.zeros((len(ref_stroke), len(written_stroke)))
    acc_dist_mat[0, 0] = dist_mat[0, 0]

    for i in range(1, len(ref_stroke)):
        acc_dist_mat[i, 0] = dist_mat[i, 0] + acc_dist_mat[i-1, 0]

    for j in range(1, len(written_stroke)):
        acc_dist_mat[0, j] = dist_mat[0, j] + acc_dist_mat[0, j-1]

    for i in range(1, len(ref_stroke)):
        for j in range(1, len(written_stroke)):
            acc_dist_mat[i, j] = dist_mat[i, j] + \
                min(acc_dist_mat[i-1, j],
                    acc_dist_mat[i, j-1], acc_dist_mat[i-1, j-1])

    # backtracking
    i = len(ref_stroke) - 1
    j = len(written_stroke) - 1

    path = []

    while i > 0 or j > 0:
        path.append((i, j))

        if i == 0:
            j -= 1
        elif j == 0:
            i -= 1
        else:
            min_idx = np.argmin(
                [acc_dist_mat[i-1, j], acc_dist_mat[i, j-1], acc_dist_mat[i-1, j-1]])
            if min_idx == 0:
                i -= 1
            elif min_idx == 1:
                j -= 1
            else:
                i -= 1
                j -= 1

    path.append((0, 0))

    path.reverse()

    # get the average distance
    dist = 0
    for i, j in path:
        dist += dist_mat[i, j]

    return dist / len(path)


def load_reference_image(path):

    char_folders = glob.glob(os.path.join(path, '*'))

    ref_data = {}

    for char_folder in char_folders:
        # extract the character name
        char_name = char_folder.split('/')[-1]

        stroke_data = cv2.imread(
            glob.glob(os.path.join(char_folder, '*.png'))[0])

        ref_data[char_name] = stroke_data

    return ref_data


def load_written_image(path):

    char_folders = glob.glob(os.path.join(path, '*'))

    written_data = {}

    for char_folder in char_folders:
        # extract the character name
        char_name = char_folder.split('/')[-1]

        # get all the stroke folders, exlude files
        user_folders = glob.glob(os.path.join(char_folder, '*'))

        # read the stroke data
        user_data = {}
        for user_folder in user_folders:

            user_name = user_folder.split('/')[-1]


            user_data[user_name] = cv2.imread(
                glob.glob(os.path.join(user_folder, '*.png'))[0])

            written_data[char_name] = user_data

    return written_data


def stroke_iou(ref_stroke_image, written_stroke_image):

    # convert the image to binary
    ref_stroke_image = cv2.cvtColor(~ref_stroke_image, cv2.COLOR_BGR2GRAY)
    ref_stroke_image = cv2.threshold(
        ref_stroke_image, 127, 255, cv2.THRESH_BINARY)[1]
    
    written_stroke_image = cv2.cvtColor(~written_stroke_image, cv2.COLOR_BGR2GRAY)
    written_stroke_image = cv2.threshold(
        written_stroke_image, 127, 255, cv2.THRESH_BINARY)[1]

    # compute the intersection of the black pixels
    intersection = np.sum(np.logical_and(
        ref_stroke_image, written_stroke_image))

    # compute the union of the black pixels
    union = np.sum(np.logical_or(ref_stroke_image, written_stroke_image))

    return intersection / union


def main(args):
    ref_path = args.ref_path
    before_path = args.before_path
    after_path = args.after_path

    # load the reference and written data
    ref_data = load_ref_data(ref_path)
    before_data = load_written_data(before_path)
    after_data = load_written_data(after_path)

    # load the reference and written image
    ref_image = load_reference_image(ref_path)
    before_image = load_written_image(before_path)
    after_image = load_written_image(after_path)

    # for each character, compute the average distance between the reference and the written trajectory,
    # normalize by the number of strokes and 128

    all_before_dist = []
    all_after_dist = []
    all_dist_delta = []
    all_iou_before = []
    all_iou_after = []

    for char_name in ref_data.keys():
        ref_strokes = ref_data[char_name]
        ref_stroke_image = ref_image[char_name]

        for user_name in before_data[char_name].keys():
            before_strokes = before_data[char_name][user_name]
            after_strokes = after_data[char_name][user_name]

            before_stroke_image = before_image[char_name][user_name]
            after_stroke_image = after_image[char_name][user_name]

            # compute the average distance between the reference and the written trajectory
            before_dist = 0
            after_dist = 0
            before_iou = 1e-5
            after_iou = 1e-5

            for i in range(len(ref_strokes)):
                ref_stroke = ref_strokes[i]
                before_stroke = before_strokes[i]
                after_stroke = after_strokes[i]

                # matching the points using the dynamic time warping

                before_dist += dtw(ref_stroke, before_stroke)
                after_dist += dtw(ref_stroke, after_stroke)

            # compute the iou
            before_iou += stroke_iou(ref_stroke_image, before_stroke_image)
            after_iou += stroke_iou(ref_stroke_image, after_stroke_image)

            before_dist /= len(ref_strokes)
            after_dist /= len(ref_strokes)

            # calculate the improvement after training
            before_dist /= 128
            after_dist /= 128

            dist_delta = (before_dist - after_dist) / before_dist

            all_before_dist.append(before_dist)
            all_after_dist.append(after_dist)
            all_dist_delta.append(dist_delta)
            all_iou_before.append(before_iou)
            all_iou_after.append(after_iou)

            print('Character: {}, User: {}, Before distance: {}, After distance: {}, Distance delta: {}, Before iou: {}, After iou: {}'.format(
                char_name, user_name, before_dist, after_dist, dist_delta, before_iou, after_iou))

    print('Average distance before training: {}'.format(
        np.mean(all_before_dist)))
    print('Average distance after training: {}'.format(
        np.mean(all_after_dist)))
    print('Average distance delta: {}'.format(np.mean(all_dist_delta)))
    print('Average iou before training: {}'.format(np.mean(all_iou_before)))
    print('Average iou after training: {}'.format(np.mean(all_iou_after)))

    iou_delta = (np.mean(all_iou_after) - np.mean(all_iou_before)) / \
        np.mean(all_iou_before)
    print('Average iou delta: {}'.format(np.mean(iou_delta)))



if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Evaluate the written performance before and after training')
    parser.add_argument('--ref_path', type=str, default='/Users/yucunjun/Downloads/group_0/desired_img',
                        help='path to the reference trajectory')
    parser.add_argument('--before_path', type=str, default='/Users/yucunjun/Downloads/group_0/subject_before_img',
                        help='path to the before training trajectory')
    parser.add_argument('--after_path', type=str, default='/Users/yucunjun/Downloads/group_0/subject_after_img',
                        help='path to the after training trajectory')
    args = parser.parse_args()

    main(args)
