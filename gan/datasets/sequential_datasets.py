import glob
import random
import os
import cv2
import torch

from torch.utils.data import Dataset
from PIL import Image
import numpy as np
import torchvision.transforms as transforms

class SequentialImageDataset(Dataset):
    def __init__(self, root, unaligned=False, mode='train'):
        self.unaligned = unaligned
        self.data_list = []

        dir_root = root + '/' + mode
        img_dirs_part_points = os.path.join(dir_root,'imgs_part_points')
        self._form_dataset(img_dirs_part_points)
    
    @staticmethod
    def rotate(points):

        x_mean = points[:,0].mean()
        y_mean = points[:,1].mean()

        origin = [x_mean, y_mean]

        # +- 30 degrees
        angle = random.uniform(-1, 1) * np.pi * 0.083

        R = np.array([[np.cos(angle), -np.sin(angle)],
                    [np.sin(angle),  np.cos(angle)]])
        
        o = np.atleast_2d(origin)
        p = np.atleast_2d(points)
        return np.clip(np.squeeze((R @ (p.T-o.T) + o.T).T), 0, 127)

    @staticmethod
    def scale(points):

        x_mean = points[:,0].mean()
        y_mean = points[:,1].mean()

        origin = [x_mean, y_mean]

        distance = points - origin

        scale_factor = random.uniform(0.7,1)
        scaled_distance = distance * scale_factor

        scaled_points = origin + scaled_distance

        return np.clip(scaled_points,0,127)

    @staticmethod
    def shift(points):
        
        max_shift = 10
        x_shift_value = random.randrange(-max_shift, max_shift)
        y_shift_value = random.randrange(-max_shift, max_shift)

        points[:,0] = points[:,0] + x_shift_value
        points[:,1] = points[:,1] + y_shift_value

        return np.clip(points, 0, 127)

    def cut(self, points):
        
        raise NotImplementedError

    def drop(self, points):
        raise NotImplementedError

    def _rank_file_accd_num(self, all_files):
        
        num_list = []
        for file_name in all_files:
            num_list.append(int(file_name.split('/')[-1].split('.')[0]))

        sorted_files = []
        sort_index = np.argsort(num_list)

        for i in sort_index:
            sorted_files.append(all_files[i])
        return sorted_files
    

    def form_train_sample(self,file_list):

        points_list = []
        num_list = []

        for file_name in file_list:
            points = np.loadtxt(file_name)
            points_list.append(points)
            num_list.append(points.shape[0])

        points_list = np.concatenate(points_list)
        num_list = np.array(num_list)

        return {
            'points': torch.from_numpy(points_list),
            'num': torch.from_numpy(num_list)
        }

    def _form_dataset(self, path): 

        for folder_name in glob.glob(path+'/*'):
            file_list = []
            for idx, txt_name in enumerate(self._rank_file_accd_num(glob.glob(folder_name + '/*.txt'))):
                file_list.append(txt_name)
                if idx == 0:
                    continue
                self.data_list.append(self.form_train_sample(file_list))

    def __getitem__(self, index):

        return self.data_list[index]
        
    def __len__(self):
        
        return len(self.data_list)

def visulize_points(points,name):

    img_canvas = np.full((128,128),255, np.uint8)
    points = [points.astype(int)]
    for l in points:
        c = (0,0,0)
        for i in range(0,len(l)-1):
            cv2.line(img_canvas,(l[i][0],l[i][1]),(l[i+1][0],l[i+1][1]),c,2)

    cv2.imwrite(name,img_canvas)

def unit_test_1():

    filename = '/home/cunjun/Robot-Teaching-Assiantant/gan/data/seq/train/imgs_part_points/且/0.txt'
    points = np.loadtxt(open(filename, 'r'))
    visulize_points(points,'ori.jpg')
    points = SequentialImageDataset.shift(points)
    visulize_points(points,'new.jpg')

def unit_test_2():

    root = '/home/cunjun/Robot-Teaching-Assiantant/gan/data/seq'
    SequentialImageDataset(root)

if __name__ == '__main__':

    unit_test_1()
    unit_test_2()

    # dir_root = 'datasets/seq/train'
    # img_dirs_part = os.path.join(dir_root,'imgs_ske')
    # for folder_name in glob.glob(img_dirs_part+'/*'):
    #     for img_name in sorted(glob.glob(folder_name + '/*.jpg'))[1:]:
    #         img = cv2.imread(img_name)
    #         img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    #         succ = cv2.imwrite(img_name, img[:,:,0])

