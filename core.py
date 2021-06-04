import argparse
import torch
import torchvision.transforms as transforms
import importlib


def load_class(class_name):
    components = class_name.split('.')
    mod = __import__(components[0])
    for comp in components[1:]:
        mod = getattr(mod, comp)
    return mod

class executor(object):

    def __init__(self, args) -> None:   

        self.cuda = args.get('CUDA', False)
        self.feedbaock = args.get('WITH_FEEDBACK', False)

        self._init_parameters(args)
        self._init_network()

    def _init_parameter(self,args):

        self.gan_path = args.get('GAN_MODEL_PATH')
        self.dis_path = args.get('DIS_MODEL_PATH')

        self.gan_type = args.get('GAN_MODEL_TYPE')
        self.dis_type = args.get('DIS_MODEL_TYPE')

        self.input_channel = args.get('INPUT_CHANNEL')
        self.output_channel = args.get('OUTPUT_CHANNEL')

        if args.get('PRE_PROCESS',None):
            assert args.get('PRE_PROCESS').upper == 'DEFAULT', '{} is not supported'.format(args.get('PRE_PROCESS'))
            self.pre_process = [ transforms.ToTensor(),
                                                transforms.Normalize((0.5,), (0.5,)) ]
        else:
            self.pre_process = [ transforms.ToTensor() ] 

    def _init_network(self,):

        self.gan = load_class(self.gan_type)(self.input_channel, self.output_channel)
        self.dis = load_class(self.dis_type)(self.output_channel)

        if self.cuda: 
            self.gan = self.gan.cuda()
            self.dis = self.dis.cuda()
        
        if self.gan_path is not None:
            self.gan.load_state_dict({k.replace('module.',''):v for k,v in torch.load(self.gan_path).items()})

        if self.dis_path is not None:
            self.dis.load_state_dict({k.replace('module.',''):v for k,v in torch.load(self.dis_path).items()})

    def interact(self,):
        """[summary] TO DO: interaction part
        """
        pass

    def get_score(self, image): 

        return self.dis(self.pre_process(image))

    def sample_stroke(self, ):
        pass

    def pipeline(self, ):

        print()