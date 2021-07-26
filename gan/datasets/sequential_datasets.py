import glob
import random
import os
import cv2

from torch.utils.data import Dataset
from PIL import Image
import numpy as np
import torchvision.transforms as transforms

class SequentialImageDataset(Dataset):
    def __init__(self, root, transforms_=None, unaligned=False, mode='train'):
        self.transform = transforms.Compose(transforms_)
        self.unaligned = unaligned

        dir_root = root + '/' + mode
        img_dirs_part = os.path.join(dir_root,'imgs_part')
        img_dirs_part_ske = os.path.join(dir_root,'imgs_part_ske')
        img_dirs_full = os.path.join(dir_root,'imgs_full')
        img_dirs_full_ske = os.path.join(dir_root,'imgs_full_ske')

        self.full = []
        self.A_part = []
        self.A_label = []
        self.A_ske = []

        self._form_dataset_label(img_dirs_full)

        assert len(self.A_full) == len(self.A_part) == len(self.A_label) == len(self.A_ske)

    def _rank_file_accd_num(self, all_files):
        
        num_list = []
        for file_name in all_files:
            num_list.append(int(file_name.split('/')[-1].split('.')[0]))

        sorted_files = []
        sort_index = np.argsort(num_list)

        for i in sort_index:
            sorted_files.append(all_files[i])

        return sorted_files
    
    def _form_dataset(self, path): 

        for folder_name in glob.glob(path+'/*'):
            for img_name in self._rank_file_accd_num(glob.glob(folder_name + '/*.jpg'))[1:]:
                self.A_ske.append(img_name)
    
    def _form_dataset_label(self, path):
       
        for folder_name in glob.glob(path+'/*'):
            for img_name in self._rank_file_accd_num(glob.glob(folder_name + '/*.jpg'))[1:]:
                self.A_label.append(img_name)

    def _show_image(self, index):
    
        img_to_show = cv2.imread(self.A_full[index % len(self.A_full)])
        cv2.imshow('',img_to_show);cv2.waitKey(0)
        img_to_show = cv2.imread(self.A_part[index % len(self.A_part)])
        cv2.imshow('',img_to_show);cv2.waitKey(0)
        img_to_show = cv2.imread(self.A_label[index % len(self.A_label)])
        cv2.imshow('',img_to_show);cv2.waitKey(0)
        img_to_show = cv2.imread(self.A_ske[index % len(self.A_ske)])
        cv2.imshow('',img_to_show);cv2.waitKey(0)

    def _show_image_name(self, index):
        
        print('\npart:' + self.A_part[index % len(self.A_part)])
        print('\nlabel:' + self.A_label[index % len(self.A_label)])
        print('\nlabel:' + self.A_ske[index % len(self.A_ske)])

    def __getitem__(self, index):

        stroke = self.transform(Image.open(self.A_part[index % len(self.A_part)]))

        if self.unaligned:
            A_comp_0 = self.transform(Image.open(self.A_comp_0[random.randint(0, len(self.A_comp_0) - 1)]))
            A_comp_1 = self.transform(Image.open(self.A_comp_1[random.randint(0, len(self.A_comp_1) - 1)]))
            A_comp_2 = self.transform(Image.open(self.A_comp_2[random.randint(0, len(self.A_comp_2) - 1)]))
            A_comp_3 = self.transform(Image.open(self.A_comp_3[random.randint(0, len(self.A_comp_3) - 1)]))
            A_comp_0_1 = self.transform(Image.open(self.A_comp_0[index % len(self.A_comp_0)]))
            A_comp_1_1 = self.transform(Image.open(self.A_comp_1[index % len(self.A_comp_1)]))
            A_comp_2_1 = self.transform(Image.open(self.A_comp_2[index % len(self.A_comp_2)]))
            A_comp_3_1 = self.transform(Image.open(self.A_comp_3[index % len(self.A_comp_3)]))
            item_B = self.transform(Image.open(self.files_B[random.randint(0, len(self.files_B) - 1)]))
        else:
            A_comp_0 = self.transform(Image.open(self.A_comp_0[index % len(self.A_comp_0)]))
            A_comp_1 = self.transform(Image.open(self.A_comp_1[index % len(self.A_comp_1)]))
            A_comp_2 = self.transform(Image.open(self.A_comp_2[index % len(self.A_comp_2)]))
            A_comp_3 = self.transform(Image.open(self.A_comp_3[index % len(self.A_comp_3)]))
            item_B = self.transform(Image.open(self.files_B[index % len(self.files_B)]))

        return {
                }

    def __len__(self):
        return max(len(self.A_traj), len(self.files_B))


if __name__ == '__main__':

    dir_root = 'datasets/seq/train'
    img_dirs_part = os.path.join(dir_root,'imgs_ske')
    for folder_name in glob.glob(img_dirs_part+'/*'):
        for img_name in sorted(glob.glob(folder_name + '/*.jpg'))[1:]:
            img = cv2.imread(img_name)
            img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            succ = cv2.imwrite(img_name, img[:,:,0])

