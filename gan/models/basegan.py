import torch
import itertools
from ..utils import LambdaLR


class GAN(object):

    def __init__(self) -> None:
        super().__init__()

    def generator(self):

        raise NotImplementedError

    def discriminator(self,):

        raise NotImplementedError

    def train(self, dataset):

        raise NotImplementedError

    @staticmethod
    def init_optimizer(self, module, lr, beta=(0.5, 0.999)):

        if len(module) > 1:
            optimizer = torch.optim.Adam(itertools.chain(module),
                                         lr=lr, betas=(0.5, 0.999))
        else:
            optimizer = torch.optim.Adam(module,
                                         lr=lr, betas=(0.5, 0.999))

        return optimizer
        
    @staticmethod
    def init_scheduler(self, optimizer, n_epochs, start_epoch, decay_epoch):

        lr_scheduler = torch.optim.lr_scheduler.LambdaLR(
            optimizer, lr_lambda=LambdaLR(n_epochs, start_epoch, decay_epoch).step)

        return lr_scheduler
