import logging
import os
import torch
import torchvision
import cv2

import torchvision.transforms as transforms
import numpy as np

from tools import skeletonize, stroke2img, svg2img
from tqdm import tqdm
from .learner import Learner
from .controller import Controller
from .utils import load_class
from .imgprocessor import Postprocessor


class Executor(object):
    """Class that carries out teaching process

    Args:
        object ([type]): [description]
    """

    def __init__(self, args) -> None:

        logging.info('Initialize Runner')
        self.cuda = args.get('CUDA', False)
        self.feedback = args.get('FEEDBACK')
        self.save_traj = args.get('SAVE_TRAJ', False)

        self.__init_parameters(args)
        self.__init_network(args)
        self.__parse_feedback()

    def __parse_feedback(self,):

        if self.feedback is None:
            self.with_feedback = False
        else:
            self.with_feedback = self.feedback.get('WITH_FEEDBACK')

        self.postprocessor = Postprocessor(
            self.feedback.get('POST_PRORCESS'))

        self.learner = Learner()
        self.controller = Controller(Postprocessor(
            self.feedback.get('POST_PRORCESS')))

    def __init_parameters(self, args):

        self.generation_only = args.get('GENERATIION_ONLY', False)

        self.gan_path = args.get('GAN_MODEL_PATH')
        self.dis_path = args.get('DIS_MODEL_PATH')

        self.gan_type = args.get('GAN_MODEL_TYPE')
        self.dis_type = args.get('DIS_MODEL_TYPE')

        self.input_channel = args.get('INPUT_CHANNEL')
        self.output_channel = args.get('OUTPUT_CHANNEL')

        self.font_type = args.get('TTF_FILE')
        self.font_size = args.get('FONT_SIZE', 128)
        assert self.font_type is not None, 'Please provide a font file'

        if args.get('PRE_PROCESS', None):
            assert args.get('PRE_PROCESS').upper(
            ) == 'DEFAULT', '{} preprocess is not supported'.format(args.get('PRE_PROCESS'))
            pre_process = [transforms.ToTensor(),
                           transforms.Normalize((0.5,), (0.5,))]
        else:
            pre_process = [transforms.ToTensor()
                           ]
        self.pre_process = transforms.Compose(pre_process)

        assert 'GRAPHICS_TXT' in args.keys(), 'Please provide a graphic list'
        graphic_list = args.get('GRAPHICS_TXT')
        input_lines = open(graphic_list, 'r').readlines()
        self.char_list = {}
        logging.info('Loading character list')

        for line in tqdm(input_lines):
            char_info = eval(line)
            char = char_info['character']
            self.char_list[char] = char_info

    def __init_network(self, args):

        self.gan = load_class(self.gan_type)(
            args['GAN_PARAM'], mode='inference')
        self.dis = load_class(self.dis_type)(
            args['GAN_PARAM'], mode='inference')

        mapping_device = 'cpu'
        if self.cuda:
            self.gan = self.gan.cuda()
            self.dis = self.dis.cuda()
            mapping_device = 'gpu'

        if self.gan_path is not None:
            self.gan.load_state_dict(
                {k.replace('module.', ''): v for k, v in torch.load(self.gan_path,
                                                                    map_location=torch.device(mapping_device)).items()})

        if self.dis_path is not None:
            self.dis.load_state_dict(
                {k.replace('module.', ''): v for k, v in torch.load(self.dis_path,
                                                                    map_location=torch.device(mapping_device)).items()})

    def interact(self, traj, score=None):
        """TO DO: interaction part
        """
        if self.with_feedback:
            output_img = self.__capture_image()
            output_img = self.postprocessor.process(output_img)
            # cv2.imshow('',np.array(output_img))
            # cv2.waitKey(0)
            # self.learner.score = self.get_score(output_img)
        return output_img

    def get_score(self, image):

        return self.dis(self.pre_process(image))

    def __capture_image(self, ):
        """ Capture image with post process in order for discrinmintor to score
        """
        return cv2.imread('./example/example_feedback.png')
        # raise NotImplementedError

    def sample_character(self, stroke, written_character=None):
        """ For future development, decompose one character into several characters
        """
        if written_character is None:
            return stroke2img(self.font_type, stroke, self.font_size)
        else:
            source_image = np.array(stroke2img(
                self.font_type, stroke, self.font_size))
            return self.__generate_written_traj(written_character, source_image)

    def stylization(self, img, written_img = None):
        pass

    def __reset_learner(self,):
        """ Reset learner model
        """
        self.learner.reset()

    def __quit(self):
        """ Quit all the processes
        """
        logging.info('Quittiing')

    def __save_stroke_traj(self, stroke, traj, savepath='./'):

        font_name = self.font_type.split('/')[-1].split('.')[0]
        filename = os.path.join(savepath, str(stroke)+'_'+font_name) + '.txt'
        with open(filename, 'w+') as file_stream:
            np.savetxt(file_stream,  traj[0])

        return filename

    def __generate_written_traj(self, written_image, source_image):

        source_image = self.pre_process(source_image).unsqueeze(0)
        written_image = self.pre_process(written_image).unsqueeze(0)

        styled_written_image = self.gan(written_image, source_image)

        styled_written_image = 0.5*(styled_written_image.data + 1.0)
        grid = torchvision.utils.make_grid(styled_written_image)
        # Add 0.5 after unnormalizing to [0, 255] to round to nearest integer

        styled_written_image = grid.mul(255).add_(0.5).clamp_(
            0, 255).permute(1, 2, 0).to('cpu', torch.uint8).numpy()

        styled_written_image = cv2.cvtColor(
            styled_written_image, cv2.COLOR_BGR2GRAY)

        return styled_written_image

    def pipeline(self,):
        """ Full pipeline
        Obtain target character -> generate target character's trajectory -> Interact with learner -> Get learner output
        """

        while True:

            character = input('Please provide a character you want to learn: ')
            written_image = None

            if len(character) > 1:
                logging.warning('Please input once character only')
                continue
            
            if character is ' ':
                break

            if character in self.char_list:
                logging.info('We find the character for you')
            else:
                logging.warning('Sorry, the character is not supported, please try another one')
                break
            
            char_info = self.char_list[character]
            strokes = char_info['strokes']

            img_list = []
            for stroke in strokes:
                img_list.append(svg2img(stroke))

            while not self.learner.satisfied:
            
                # character_img = self.sample_character(character, written_image)
                # character_img = np.array(character_img)
                img_ske_list = []
                traj_list = []

                for img in img_list:
                    traj, traj_img = skeletonize(~img)
                    img_ske_list.append(traj_img)
                    traj_list.append(traj_list)
                    # cv2.imshow('',traj_img)
                    # cv2.waitKey(0)

                if self.save_traj:
                    save_traj_name = self.__save_stroke_traj(character, traj)
                    cv2.imwrite(save_traj_name.replace('txt', 'png'), traj_img)
                    logging.info('{} traj stored'.format(character))

                if self.generation_only:
                    break

                written_image = self.controller.interact(traj, img_ske_list)

        self.__quit()

        # cv2.imshow('',stroke_img)
        # cv2.waitKey(0)

        # if character_img is None:
        #     logging.warning(
        #         'Provided character cannot be found in this font, please provide another one')
        #     break

        # if written_image is not None:
        #     cv2.imshow('',stroke_img)
        #     cv2.waitKey(0)
        #     cv2.imwrite('./new.png', stroke_img)
        #     cv2.imshow('',traj_img)
        #     cv2.waitKey(0)
        #     cv2.imwrite('./new_traj.png', traj_img)
