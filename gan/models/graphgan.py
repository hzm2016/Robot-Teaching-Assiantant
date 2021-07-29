import torch
import os
import cv2
import torch.nn.functional as F
import argparse
import numpy as np

from torch_geometric.nn import GATConv, GraphConv, global_max_pool, global_mean_pool, XConv
from sklearn.metrics import pairwise_distances
from datasets import SequentialImageDataset
from utils import ReplayBuffer, Logger, to_cuda
from torch.utils.data import DataLoader
from torch.nn import Linear
from basegan import GAN
from tqdm import tqdm


def visulize_points(points,name):

    img_canvas = np.full((128,128),255, np.uint8)
    points = [points.astype(int)]
    for l in points:
        c = (0,0,0)
        for i in range(0,len(l)-1):
            cv2.line(img_canvas,(l[i][0],l[i][1]),(l[i+1][0],l[i+1][1]),c,2)

    cv2.imwrite(name,img_canvas)

class GraphGeneratorX(torch.nn.Module):

    def __init__(self, out_channels=16) -> None:
        super(GraphGenerator, self).__init__()
        self.conv1 = GraphConv(2, 8)
        self.conv2 = GraphConv(8, out_channels)

        self.conv3 = GraphConv(2, out_channels)
        self.conv4 = GraphConv(out_channels*3, 2)

    def forward_prior(self, x, edge_index):

        x = F.dropout(x, p=0.2, training=self.training)
        x = F.elu(self.conv1(x, edge_index))
        x = F.dropout(x, p=0.2, training=self.training)
        x = self.conv2(x, edge_index)
        return x

    def forward_output(self, x, edge_index, prior):

        batch_size = x.shape[0]
        x = F.dropout(x, p=0.2, training=self.training)
        x = F.elu(self.conv3(x, edge_index))

        x = torch.cat([x, prior.repeat(batch_size, 1)], dim=1)
        x = F.dropout(x, p=0.2, training=self.training)
        x = self.conv4(x, edge_index)

        return torch.sigmoid(x)

    def forward(self, o, m, c, edge_index_o, edge_index_m, edge_index_c):

        dummy_batch = torch.zeros(1).long().cuda()

        o = self.forward_prior(o, edge_index_o)
        o_feature = global_max_pool(o, dummy_batch)

        m = self.forward_prior(m, edge_index_m)
        m_feature = global_max_pool(m, dummy_batch)

        prior_feature = torch.cat([o_feature, m_feature], dim=1)

        generate_result = self.forward_output(c, edge_index_c, prior_feature)

        return generate_result + c

class GraphGenerator(torch.nn.Module):

    def __init__(self, out_channels=16) -> None:
        super(GraphGenerator, self).__init__()
        self.conv1 = GraphConv(2, 8)
        self.conv2 = GraphConv(8, out_channels)

        self.conv3 = GraphConv(2, out_channels)
        self.conv4 = GraphConv(out_channels*3, 2)

    def forward_prior(self, x, edge_index):

        x = F.dropout(x, p=0.2, training=self.training)
        x = F.elu(self.conv1(x, edge_index))
        x = F.dropout(x, p=0.2, training=self.training)
        x = self.conv2(x, edge_index)
        return x

    def forward_output(self, x, edge_index, prior):

        batch_size = x.shape[0]
        x = F.dropout(x, p=0.2, training=self.training)
        x = F.elu(self.conv3(x, edge_index))

        x = torch.cat([x, prior.repeat(batch_size, 1)], dim=1)
        x = F.dropout(x, p=0.2, training=self.training)
        x = self.conv4(x, edge_index)

        return torch.sigmoid(x)

    def forward(self, o, m, c, edge_index_o, edge_index_m, edge_index_c):

        dummy_batch = torch.zeros(1).long().cuda()

        o = self.forward_prior(o, edge_index_o)
        o_feature = global_max_pool(o, dummy_batch)

        m = self.forward_prior(m, edge_index_m)
        m_feature = global_max_pool(m, dummy_batch)

        prior_feature = torch.cat([o_feature, m_feature], dim=1)

        generate_result = self.forward_output(c, edge_index_c, prior_feature)

        return generate_result + c


class GraphDiscriminator(torch.nn.Module):

    def __init__(self, cls_num=1) -> None:
        super(GraphDiscriminator, self).__init__()
        self.conv1 = GraphConv(2, 16)
        self.conv2 = GraphConv(16, 32)

        self.lin1 = Linear(32, 16)
        self.lin2 = Linear(16, 8)
        self.lin3 = Linear(8, cls_num)

    def forward(self, x, edge_index):

        x = F.dropout(x, p=0.2, training=self.training)
        x = F.elu(self.conv1(x, edge_index))
        x = F.dropout(x, p=0.2, training=self.training)
        x = self.conv2(x, edge_index)

        batch = torch.zeros(1).long().cuda()
        x = global_max_pool(x, batch)
        x = F.relu(self.lin1(x))
        x = F.relu(self.lin2(x))

        x = F.dropout(x, p=0.2, training=self.training)
        x = self.lin3(x)

        return x
        # return F.log_softmax(x, dim=-1)


class GraphGAN(GAN):

    def __init__(self, args, train=True) -> None:
        super().__init__()
        self.cuda = args.cuda
        self.args = args
        self.init_networks(args)
        if train:
            self.init_all_optimizer(args)
            self.init_dataset(args)
            self.init_loss(args)
            self.logger = Logger(args.n_epochs, len(self.dataloader))
        if self.cuda:
            self.to_cuda()

    def to_cuda(self,):

        self.G.cuda()
        self.D.cuda()

    def init_networks(self, args, out_channels=32):

        self.G = GraphGenerator()
        self.D = GraphDiscriminator()

        self.G.train()
        self.D.train()

    def init_all_optimizer(self, args):

        self.optimizer_G = self.init_optimizer(self.G.parameters(), args.lr)
        self.optimizer_D = self.init_optimizer(self.D.parameters(), args.lr)
        self.G_scheduler = self.init_scheduler(
            self.optimizer_G, args.n_epochs, args.epoch, args.decay_epoch)
        self.D_scheduler = self.init_scheduler(
            self.optimizer_D, args.n_epochs, args.epoch, args.decay_epoch)

    def init_loss(self, args):

        self.criterion_GAN = torch.nn.MSELoss()
        self.criterion_DIS = torch.nn.MSELoss()

        self.fake_buffer = ReplayBuffer()

    def init_dataset(self, args):

        self.dataloader = DataLoader(SequentialImageDataset(args.dataroot),
                                     batch_size=args.batchSize, shuffle=True, num_workers=args.n_cpu)

    def generator_process(self, o_points, m_points, c_points,
                          edge_index_o, edge_index_m, edge_index_c, target_real):

        self.optimizer_G.zero_grad()
        fake_c = self.G(o_points, m_points, c_points,
                        edge_index_o, edge_index_m, edge_index_c)

        fake_batch = torch.cat([m_points, c_points], dim=0)

        fake_edge = pairwise_distances(fake_batch.cpu().numpy()) < 0.1
        fake_edge = torch.from_numpy(fake_edge.astype(int)).long().cuda()
        fake_edge = fake_edge.nonzero().t()

        pred_fake = self.D(fake_batch, fake_edge)
        loss_GAN = self.criterion_GAN(pred_fake, target_real) + self.criterion_DIS(fake_c, c_points)

        loss_GAN.backward(retain_graph=True)
        self.optimizer_G.step()

        return fake_batch, loss_GAN

    def discriminator_process(self, fake_batch, real_batch, target_real, target_fake):

        self.optimizer_D.zero_grad()

        real_edge = pairwise_distances(real_batch.cpu().numpy()) < 0.1
        real_edge = torch.from_numpy(real_edge.astype(int)).long().cuda()
        real_edge = real_edge.nonzero().t()

        fake_edge = pairwise_distances(fake_batch.cpu().numpy()) < 0.1
        fake_edge = torch.from_numpy(fake_edge.astype(int)).long().cuda()
        fake_edge = fake_edge.nonzero().t()

        pred_real = self.D(real_batch,real_edge)
        loss_D_real = self.criterion_GAN(pred_real, target_real)

        # fake = self.fake_buffer.push_and_pop(fake_batch)
        pred_fake = self.D(fake_batch.detach(), fake_edge)
        loss_D_fake = self.criterion_GAN(pred_fake, target_fake)

        loss_D = (loss_D_real + loss_D_fake)*0.5
        
        loss_D.backward()

        self.optimizer_D.step()

        return loss_D

    def train(self,args):

        Tensor = torch.cuda.FloatTensor if self.cuda else torch.Tensor
        frequency = self.args.frequency
        output_dir = self.args.output_dir

        for epoch in range(self.args.epoch, self.args.n_epochs):
            for i, batch in enumerate(self.dataloader):
                if self.cuda:
                    batch = to_cuda(batch)

                o_points = batch['o_points'].float().squeeze(0)
                m_points = batch['m_points'].float().squeeze(0)
                c_points = batch['c_points'].float().squeeze(0)
                real_batch = torch.cat([o_points, c_points],dim=0)

                edge_index_o = batch['edge_index_o']
                edge_index_m = batch['edge_index_m']
                edge_index_c = batch['edge_index_c']

                edge_index_o = edge_index_o[0].nonzero().t()
                edge_index_m = edge_index_m[0].nonzero().t()
                edge_index_c = edge_index_c[0].nonzero().t()

                target_real = Tensor(1).fill_(1.0).unsqueeze(-1)
                target_fake = Tensor(1).fill_(0.0).unsqueeze(-1)

                fake_batch, loss_G = self.generator_process(
                    o_points, m_points, c_points, edge_index_o, edge_index_m, edge_index_c,target_real)

                loss_D = self.discriminator_process(
                     fake_batch, real_batch, target_real, target_fake)

            #     A = batch['A']
            #     B = batch['B']
            #     target_real = Tensor(A.shape[0]).fill_(1.0).unsqueeze(-1)
            #     target_fake = Tensor(A.shape[0]).fill_(0.0).unsqueeze(-1)

            #     loss_G, fake_A, fake_B = self.generator_process(
            #         A, B, target_real)
            #     loss_D = self.discriminator_process(
            #         A, B, fake_A, fake_B,  target_real, target_fake)
                self.logger.log({'loss_G': loss_G, 'loss_D': loss_D},
                                images=None)

            if epoch % frequency == (frequency - 1):
                torch.save(self.G.state_dict(), output_dir +
                           '/{}_{}.pth'.format('G', epoch))

            self.G_scheduler.step()
            self.D_scheduler.step()

    def test(self, args):
        
        self.G.eval()
        self.D.eval()

        test_dataloader = DataLoader(SequentialImageDataset(args.dataroot),
                                     batch_size=args.batchSize, shuffle=True, num_workers=args.n_cpu)

        os.makedirs(self.args.output_dir + '/fake', exist_ok=True)
        os.makedirs(self.args.output_dir + '/real', exist_ok=True)

        for i, batch in tqdm(enumerate(test_dataloader)):
                if self.cuda:
                    batch = to_cuda(batch)

                o_points = batch['o_points'].float().squeeze(0)
                m_points = batch['m_points'].float().squeeze(0)
                c_points = batch['c_points'].float().squeeze(0)

                edge_index_o = batch['edge_index_o']
                edge_index_m = batch['edge_index_m']
                edge_index_c = batch['edge_index_c']

                edge_index_o = edge_index_o[0].nonzero().t()
                edge_index_m = edge_index_m[0].nonzero().t()
                edge_index_c = edge_index_c[0].nonzero().t()
                
                fake_c = self.G(o_points, m_points, c_points,
                        edge_index_o, edge_index_m, edge_index_c)

                fake_c = fake_c.detach().cpu().numpy() * 128
                c_points = c_points.cpu().numpy() * 128
                import pdb; pdb.set_trace()
                visulize_points(fake_c, self.args.output_dir + '/fake/%04d.png' % (i+1))
                visulize_points(c_points, self.args.output_dir + '/real/%04d.png' % (i+1))

if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument('--epoch', type=int, default=0, help='starting epoch')
    parser.add_argument('--model', type=str,
                        default='CycleGAN', help='type of models')
    parser.add_argument('--n_epochs', type=int, default=200,
                        help='number of epochs of training')
    parser.add_argument('--batchSize', type=int, default=1,
                        help='size of the batches')
    parser.add_argument('--dataroot', type=str, default='data/seq/',
                        help='root directory of the dataset')
    parser.add_argument('--lr', type=float, default=0.0005,
                        help='initial learning rate')
    parser.add_argument('--decay_epoch', type=int, default=100,
                        help='epoch to start linearly decaying the learning rate to 0')
    parser.add_argument('--size', type=int, default=128,
                        help='size of the font_data crop (squared assumed)')
    parser.add_argument('--input_nc', type=int, default=1,
                        help='number of channels of input font_data')
    parser.add_argument('--output_nc', type=int, default=1,
                        help='number of channels of output font_data')
    parser.add_argument('--cuda', action='store_true',
                        help='use GPU computation')
    parser.add_argument('--n_cpu', type=int, default=0,
                        help='number of cpu threads to use during batch generation')
    parser.add_argument('--frequency', type=int, default=20,
                        help='frequency of saving trained model')
    parser.add_argument('--sequential', action='store_true',
                        help='if the dataset have a sequence')
    parser.add_argument('--ske', action='store_true',
                        help='if the dataset have a skeleton input')
    parser.add_argument('--output_dir', type=str,
                        default='./output', help='place to output result')
    args = parser.parse_args()

    os.makedirs(args.output_dir, exist_ok=True)
    if torch.cuda.is_available() and not args.cuda:
        print("WARNING: You have a CUDA device, so you should probably run with --cuda")

    model = GraphGAN(args)
    # model.train(args)

    key_pairs = {
        'G': '/home/cunjun/Robot-Teaching-Assiantant/gan/graphgan/G_19.pth'
    }

    model.test(args)
