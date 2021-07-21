import torch
from basegan import GAN
from PIL import  Image
import torchvision.transforms as transforms
from ..nn.modules import Generator, Discriminator 
from ..utils import DataLoader
from ..datasets import ImageDataset, SequentialImageDataset
from ..utils import get_cyc_loss, get_gan_loss, ReplayBuffer, to_cuda
import torch.Tensor as Tensor

class CycleGAN(GAN):

    def __init__(self, args) -> None:
        super().__init__()

        self.cuda = args.cuda
        self.init_network(args)
        self.init_all_optimizer(args)
        self.init_dataset(args)
        self.init_loss(args)

    def init_network(self, args):

        self.G_A2B = Generator(args.input_nc, args.output_nc)
        self.G_B2A = Generator(args.output_nc, args.input_nc)

        self.D_A = Discriminator(args.input_nc)
        self.D_B = Discriminator(args.output_nc)

    def init_all_optimizer(self,args):

        self.optimizer_G = self.init_optimizer([self.G_A2B.parameters(), self.G_B2A.parameters()], args.lr)
        self.optimizer_D = self.init_optimizer([self.D_A.parameters(), self.D_B.parameters()],args.lr)
        self.G_scheduler = self.init_scheduler(self.optimizer_g, args.n_epochs, args.epoch, args.decay_epoch)
        self.G_scheduler = self.init_scheduler(self.optimizer_d, args.n_epochs, args.epoch, args.decay_epoch)

    def init_dataset(self, args):
    # Dataset loader
        transforms_ = [transforms.Resize(int(args.size*1.12), Image.BICUBIC),
                        transforms.RandomCrop(args.size),
                        transforms.RandomHorizontalFlip(),
                        transforms.ToTensor(),
                        transforms.Normalize((0.5,), (0.5,))]

        self.dataloader = DataLoader(ImageDataset(args.dataroot, transforms_=transforms_, unaligned=True),
                                    batch_size=args.batchSize, shuffle=True, num_workers=args.n_cpu)

    
    def init_loss(self, args):

        self.criterion_GAN = torch.nn.MSELoss()
        self.criterion_cycle = torch.nn.L1Loss()
        self.criterion_identity = torch.nn.L1Loss()

        self.fake_A_buffer = ReplayBuffer()
        self.fake_B_buffer = ReplayBuffer()

    def generator_process(self, A, B):

        self.optimizer_G.zero_grad()

        target_real = Tensor(A.shape[0]).fill_(1.0).unsqueeze(-1)

        # GAN loss
        fake_B = self.G_A2B(A)
        pred_fake = self.D_B(fake_B)
        loss_GAN_A2B = self.criterion_GAN(pred_fake, target_real)

        fake_A = self.G_B2A(B)
        pred_fake = self.D_A(fake_A)
        loss_GAN_B2A = self.criterion_GAN(pred_fake, target_real)

        recovered_A = self.G_B2A(fake_B)
        loss_cycle_ABA = self.criterion_cycle(recovered_A, A)*10.0

        recovered_B = self.G_A2B(fake_A)
        loss_cycle_BAB = self.criterion_cycle(recovered_B, B)*10.0

        loss_G = loss_GAN_A2B + loss_GAN_B2A + loss_cycle_ABA + loss_cycle_BAB

        loss_G.backward(retain_graph=True)
    
        self.optimizer_G.step()

        pass


    def discriminator_process(self,):
        
        target_fake = Tensor(B.shape[0]).fill_(0.0).unsqueeze(-1)   
        pass


    def train(self, args, dataset):
        Tensor = torch.cuda.FloatTensor if args.cuda else torch.Tensor

        for epoch in range(args.epoch, args.n_epochs):
            for i, batch in enumerate(self.dataloader):
                if args.cuda:
                    batch = to_cuda(batch)

                A = batch['A']
                B = batch['B']


                fake_A, fake_B = self.generator_process(A, B)
                self.discriminator_process()
