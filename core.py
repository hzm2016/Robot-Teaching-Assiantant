import torch
import logging
import os
import cv2

import torchvision.transforms as transforms
import numpy as np

from utils import skeletonize, stroke2img, cropping, binarize, rescale


def load_class(class_name):
    components = class_name.split('.')
    mod = __import__(components[0])
    for comp in components[1:]:
        mod = getattr(mod, comp)
    return mod

class Postprocessor(object):

    def __init__(self,args) -> None:    

        self.pipeline = []
        for k, v in args.items():
            setattr(self, k.lower(), v)
            self.pipeline.append(k.upper())

    def CROPPING(self,image):

        return cropping(image, self.cropping)

    def BINARIZE(self, image):

        return binarize(image, self.binarize)

    def RESCALE(self, image): 

        return rescale(image, self.rescale)
    
    def process(self, image):

        for m in self.pipeline:
            image = getattr(self, m)(image)

        return image

class Controller(object):

    def __init__(self) -> None:
        pass


class Learner(object):
    """ class that stores learner performance 
    """

    def __init__(self) -> None:

        self.__init_parameters()

    def __init_parameters(self,):
        self.score = 0
        self.satisfied = False

    def reset(self):

        self.__init_parameters()


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
        self.learner = Learner()
        self.controller = Controller()

        self.__init_parameters(args)
        self.__init_network()
        self.__parse_feedback()
    
    def __parse_feedback(self,):
        
        if self.feedback is None:
            self.with_feedback = False
        else:
            self.with_feedback = self.feedback.get('WITH_FEEDBACK')
            self.postprocessor = Postprocessor(self.feedback.get('POST_PRORCESS'))

    def __init_parameters(self, args):

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

    def __init_network(self,):

        self.gan = load_class(self.gan_type)(
            self.input_channel, self.output_channel)
        self.dis = load_class(self.dis_type)(self.output_channel)

        if self.cuda:
            self.gan = self.gan.cuda()
            self.dis = self.dis.cuda()

        if self.gan_path is not None:
            self.gan.load_state_dict(
                {k.replace('module.', ''): v for k, v in torch.load(self.gan_path).items()})

        if self.dis_path is not None:
            self.dis.load_state_dict(
                {k.replace('module.', ''): v for k, v in torch.load(self.dis_path).items()})

    def interact(self, traj, score=None):
        """TO DO: interaction part
        """
        if self.with_feedback:
            output_img = self.__capture_image()
            output_img = self.postprocessor.process(output_img)
            # cv2.imshow('',np.array(output_img))
            # cv2.waitKey(0)
            # self.learner.score = self.get_score(output_img)
        return None

    def get_score(self, image):

        return self.dis(self.pre_process(image))

    def __capture_image(self, ):
        """ Capture image with post process in order for discrinmintor to score
        """
        return cv2.imread('./example/example_feedback.png')
        # raise NotImplementedError

    def sample_stroke(self, stroke, written_stroke=None):
        """ For future development, decompose one character into several strokes
        """
        if written_stroke is None:
            return stroke2img(self.font_type, stroke, self.font_size)
        else:
            return self.gan(skeletonize(~written_stroke))

    def __reset_learner(self,):
        """ Reset learner model
        """
        self.learner.reset()

    def __quit(self):
        """ Quit all the processes
        """
        pass

    def __save_stroke_traj(self, stroke, traj, savepath='./'):
        
        font_name = self.font_type.split('/')[-1].split('.')[0]
        filename = os.path.join(savepath,str(stroke)+'_'+font_name) + '.txt'
        with open(filename, 'w+') as file_stream:
            return np.savetxt(file_stream,  traj[0])


    def pipeline(self,):
        """ Full pipeline
        Obtain target stroke -> generate target stroke's trajectory -> Interact with learner -> Get learner output
        """

        while True:

            stroke = input('Please provide a stroke you want to learn: ')
            written_image = None

            if stroke is ' ':
                break
            
            while not self.learner.satisfied:
                
                stroke_img = self.sample_stroke(stroke, written_image)

                if stroke_img is None:
                    logging.warning(
                        'Provided stroke cannot be found in this font, please provide another one')
                    break

                stroke_img = np.array(stroke_img)
                traj, traj_img = skeletonize(~stroke_img)
                if self.save_traj:
                    save_traj = self.__save_stroke_traj(stroke, traj)
                written_image = self.interact(traj)

        logging.info('Quittiing')

        # cv2.imshow('',stroke_img)
        # cv2.waitKey(0)