#!/usr/bin/python3
# -*- coding: utf-8 -*-
import argparse
import os
import random

import torch

from nn.modules import Generator, StyleGanGenerator
from nn.modules import Discriminator
from utils import ReplayBuffer, LambdaLR, Logger, weights_init_normal, init_dataset, init_loss, init_optimizer, init_networks, train_d_net, get_cyc_loss_con, get_cyc_loss, get_gan_loss, get_gan_loss_con, to_cuda
from datasets import ImageDataset, SequentialImageDataset

def main(opt):

    netG_A2B, netG_B2A, netG_S2P, netG_P2S, netD_A, netD_B, netD_P, netD_S = init_networks(
        opt)
    criterion_GAN_CLS, criterion_GAN, criterion_cycle, criterion_identity = init_loss(
        opt)
    optimizer_G, optimizer_D_A, optimizer_D_B, optimizer_D_P, optimizer_D_S, lr_scheduler_G, lr_scheduler_D_A, lr_scheduler_D_B, lr_scheduler_D_P, lr_scheduler_D_S = init_optimizer(
        opt, netG_A2B, netG_B2A, netG_S2P, netG_P2S, netD_A, netD_B, netD_P, netD_S)
    dataloader = init_dataset(opt)

    if opt.cuda:
        netG_A2B, netG_B2A, netG_S2P, netD_A, netD_B, netD_P = to_cuda(
            [netG_A2B, netG_B2A, netG_S2P, netD_A, netD_B, netD_P])

    # Inputs & targets memory allocation
    Tensor = torch.cuda.FloatTensor if opt.cuda else torch.Tensor

    # Buffers of previously generated samples



    # Loss plot
    logger = Logger(opt.n_epochs, len(dataloader))

    ###### Training ######
    if not opt.sequential:
        for epoch in range(opt.epoch, opt.n_epochs):
            for i, batch in enumerate(dataloader):
                # Set model input
                if opt.cuda:
                    real_A = batch['A'].cuda()
                    real_B = batch['B'].cuda()
                else:
                    real_A = batch['A']
                    real_B = batch['B']

                ###### Generators A2B and B2A ######
                optimizer_G.zero_grad()

                # Identity loss
                # G_A2B(B) should equal B if real B is fed
                # same_B = netG_A2B(real_B)
                # loss_identity_B = criterion_identity(same_B, real_B)*5.0
                # # G_B2A(A) should equal A if real A is fed
                # same_A = netG_B2A(real_A)
                # loss_identity_A = criterion_identity(same_A, real_A)*5.0

                # GAN loss
                fake_B = netG_A2B(real_A)
                pred_fake = netD_B(fake_B)
                loss_GAN_A2B = criterion_GAN(
                    pred_fake, target_real)  # generator让pred_fake接近1

                fake_A = netG_B2A(real_B)
                pred_fake = netD_A(fake_A)
                loss_GAN_B2A = criterion_GAN(pred_fake, target_real)

                # Cycle loss
                recovered_A = netG_B2A(fake_B)
                loss_cycle_ABA = criterion_cycle(recovered_A, real_A)*10.0

                recovered_B = netG_A2B(fake_A)
                loss_cycle_BAB = criterion_cycle(recovered_B, real_B)*10.0

                # Total loss
                loss_G = loss_GAN_A2B + loss_GAN_B2A + loss_cycle_ABA + loss_cycle_BAB
                #loss_G = loss_identity_A + loss_identity_B + loss_GAN_A2B + loss_GAN_B2A + loss_cycle_ABA + loss_cycle_BAB

                loss_G.backward(retain_graph=True)
                optimizer_G.step()

                loss_G.backward()
                optimizer_G.step()

                ###################################

                ###### Discriminator A ######
                optimizer_D_A.zero_grad()

                # Real loss
                pred_real = netD_A(real_A)
                loss_D_real = criterion_GAN(pred_real, target_real)

                # Fake loss
                fake_A = fake_A_buffer.push_and_pop(fake_A)
                # fake_A 由 G 生成, detach 使更新不影响G
                pred_fake = netD_A(fake_A.detach())
                loss_D_fake = criterion_GAN(pred_fake, target_fake)

                # Total loss
                loss_D_A = (loss_D_real + loss_D_fake)*0.5
                loss_D_A.backward()

                optimizer_D_A.step()
                ###################################

                ###### Discriminator B ######
                optimizer_D_B.zero_grad()

                # Real loss
                pred_real = netD_B(real_B)
                loss_D_real = criterion_GAN(pred_real, target_real)

                # Fake loss
                fake_B = fake_B_buffer.push_and_pop(fake_B)
                pred_fake = netD_B(fake_B.detach())
                loss_D_fake = criterion_GAN(pred_fake, target_fake)

                # Total loss
                loss_D_B = (loss_D_real + loss_D_fake)*0.5
                loss_D_B.backward()

                optimizer_D_B.step()
                ###################################

                # Progress report (http://localhost:8097)
                logger.log({'loss_G': loss_G, 'loss_G_GAN': (loss_GAN_A2B + loss_GAN_B2A),
                            'loss_G_cycle': (loss_cycle_ABA + loss_cycle_BAB), 'loss_D': (loss_D_A + loss_D_B)},
                           images={'real_A': real_A, 'real_B': real_B, 'fake_A': fake_A, 'fake_B': fake_B})

            # Update learning rates
            lr_scheduler_G.step()
            lr_scheduler_D_A.step()
            lr_scheduler_D_B.step()

            # Save models checkpoints
            if epoch % 20 == 19:

                torch.save(netG_A2B.state_dict(), opt.output_dir +
                           '/{}_netG_A2B.pth'.format(epoch))
                torch.save(netG_B2A.state_dict(), opt.output_dir +
                           '/{}_netG_B2A.pth'.format(epoch))
                torch.save(netD_A.state_dict(), opt.output_dir +
                           '/{}_netD_A.pth'.format(epoch))
                torch.save(netD_B.state_dict(), opt.output_dir +
                           '/{}_netD_B.pth'.format(epoch))

        ###################################

    elif opt.ske and opt.sequential:
        for epoch in range(opt.epoch, opt.n_epochs):
            for i, batch in enumerate(dataloader):
                # Set model input
                if opt.cuda:
                    batch = to_cuda(batch)

                real_B = batch['B']
                A_traj = batch['A_traj']
                A_comp_0 = batch['A_comp_0']
                A_comp_0_1 = batch['A_comp_0_1']
                A_comp_1 = batch['A_comp_1']
                A_comp_1_1 = batch['A_comp_1_1']
                A_comp_2 = batch['A_comp_2']
                A_comp_2_1 = batch['A_comp_2_1']
                A_comp_3 = batch['A_comp_3']
                A_comp_3_1 = batch['A_comp_3_1']

                A_comp_list = [A_comp_0, A_comp_1, A_comp_2, A_comp_3]
                A_style_list = [A_comp_0_1, A_comp_1_1, A_comp_2_1, A_comp_3_1]

                index = random.randrange(len(A_comp_list))
                real_P = A_comp_list[index]
                con_P = A_style_list[index]
                real_A = real_P
                real_S = A_traj

                optimizer_G.zero_grad()

                target_label = Tensor(real_P.shape[0]).fill_(index).long()
                target_label_fake = Tensor(real_P.shape[0]).fill_(4).long()
                target_real = Tensor(real_P.shape[0]).fill_(1.0).unsqueeze(-1)
                target_fake = Tensor(real_P.shape[0]).fill_(0.0).unsqueeze(-1)

                # Generation Loss
                fake_P, loss_GAN_S2P = get_gan_loss_con(
                    A_traj, con_P, netG_S2P, netD_P, criterion_GAN_CLS, target_label)
                fake_S, loss_GAN_P2S = get_gan_loss(
                    real_P, netG_P2S, netD_S, criterion_GAN, target_real)
                fake_B, loss_GAN_A2B = get_gan_loss(
                    real_A, netG_A2B, netD_B, criterion_GAN, target_real)
                fake_A, loss_GAN_B2A = get_gan_loss(
                    real_B, netG_B2A, netD_A, criterion_GAN, target_real)

                # For log purpose
                log_str_gan = {
                    'loss_GAN_S2P': loss_GAN_S2P,
                    'loss_GAN_P2S': loss_GAN_P2S,
                    'loss_GAN_A2B': loss_GAN_A2B,
                    'loss_GAN_B2A': loss_GAN_B2A
                }

                # Cycle Loss
                recovered_S, loss_CYC_SPS = get_cyc_loss(
                    fake_P, netG_P2S, criterion_GAN, real_S, 10)
                recovered_P, loss_CYC_PSP = get_cyc_loss_con(
                    fake_S, con_P, netG_S2P, criterion_GAN, real_P, 10)
                recovered_A, loss_CYC_ABA = get_cyc_loss(
                    fake_B, netG_B2A, criterion_GAN, real_A, 10)
                recovered_B, loss_CYC_BAB = get_cyc_loss(
                    fake_A, netG_A2B, criterion_GAN, real_B, 10)

                # Total loss
                loss_G = loss_GAN_A2B + loss_GAN_B2A + loss_CYC_BAB + loss_GAN_P2S + \
                    loss_CYC_BAB + loss_GAN_S2P + loss_CYC_SPS + loss_CYC_PSP

                # For log purpose
                log_str_cyc = {
                    'loss_G': loss_G,
                    'loss_CYC_SPS': loss_CYC_SPS,
                    'loss_CYC_PSP': loss_CYC_PSP,
                    'loss_CYC_ABA': loss_CYC_ABA,
                    'loss_CYC_BAB': loss_CYC_BAB
                }

                loss_G.backward(retain_graph=True)
                optimizer_G.step()

                ###### Discriminator A ######
                loss_D_A = train_d_net(optimizer_D_A, netD_A, real_A, fake_A,
                                       fake_A_buffer, criterion_GAN, target_real, target_fake)
                loss_D_B = train_d_net(optimizer_D_B, netD_B, real_B, fake_B,
                                       fake_B_buffer, criterion_GAN, target_real, target_fake)
                loss_D_S = train_d_net(optimizer_D_S, netD_S, real_S, fake_S,
                                       fake_S_buffer, criterion_GAN, target_real, target_fake)
                loss_D_P = train_d_net(optimizer_D_P, netD_P, real_P, fake_P,
                                       fake_P_buffer, criterion_GAN_CLS, target_label, target_label_fake)

                loss_D = loss_D_A + loss_D_B + loss_D_S + loss_D_P

                log_str_dis = {
                    'loss_D': loss_D,
                    'loss_D_A': loss_D_A,
                    'loss_D_B': loss_D_B,
                    'loss_D_S': loss_D_S,
                    'loss_D_P': loss_D_P
                }

                loss_log = {**log_str_gan, **log_str_cyc, **log_str_dis}

                logger.log(loss_log,
                           images={'real_A': real_A, 'real_B': real_B, 'fake_A': fake_A, 'fake_B': fake_B})

            # Update learning rates
            lr_scheduler_G.step()
            lr_scheduler_D_A.step()
            lr_scheduler_D_B.step()
            lr_scheduler_D_P.step()
            lr_scheduler_D_S.step()

            # Save models checkpoints
            if epoch % 20 == 19:

                torch.save(netG_A2B.state_dict(), opt.output_dir +
                           '/{}_netG_A2B.pth'.format(epoch))
                torch.save(netG_B2A.state_dict(), opt.output_dir +
                           '/{}_netG_B2A.pth'.format(epoch))
                torch.save(netG_S2P.state_dict(), opt.output_dir +
                           '/{}_netG_S2P.pth'.format(epoch))
                torch.save(netG_P2S.state_dict(), opt.output_dir +
                           '/{}_netG_P2S.pth'.format(epoch))
                torch.save(netD_A.state_dict(), opt.output_dir +
                           '/{}_netD_A.pth'.format(epoch))
                torch.save(netD_B.state_dict(), opt.output_dir +
                           '/{}_netD_B.pth'.format(epoch))
                torch.save(netD_P.state_dict(), opt.output_dir +
                           '/{}_netD_P.pth'.format(epoch))
                torch.save(netD_S.state_dict(), opt.output_dir +
                           '/{}_netD_S.pth'.format(epoch))

    else:
        for epoch in range(opt.epoch, opt.n_epochs):
            for i, batch in enumerate(dataloader):
                # Set model input
                if opt.cuda:
                    real_B = batch['B'].cuda()
                    real_A = batch['A_label'].cuda()
                    A_part = batch['A_part'].cuda()
                    A_full = batch['A_full'].cuda()
                else:
                    real_A = batch['A_label']
                    A_part = batch['A_part']
                    A_full = batch['A_full']
                    real_B = batch['B']

                ###### Generators A2B and B2A ######
                optimizer_G.zero_grad()

                # GAN loss
                fake_B = netG_A2B(real_A)
                # fake_B = netG_A2B(A_full)
                pred_fake = netD_B(fake_B)
                loss_GAN_A2B = criterion_GAN(
                    pred_fake, target_real)  # generator让pred_fake接近1

                fake_A = netG_B2A(real_B)
                pred_fake = netD_A(fake_A)
                loss_GAN_B2A = criterion_GAN(pred_fake, target_real)

                # Cycle loss
                recovered_A = netG_B2A(fake_B)
                loss_cycle_ABA = criterion_cycle(recovered_A, real_A)*10.0

                recovered_B = netG_A2B(fake_A)
                loss_cycle_BAB = criterion_cycle(recovered_B, real_B)*10.0

                # Total loss
                loss_G = loss_GAN_A2B + loss_GAN_B2A + loss_cycle_ABA + loss_cycle_BAB
                #loss_G = loss_identity_A + loss_identity_B + loss_GAN_A2B + loss_GAN_B2A + loss_cycle_ABA + loss_cycle_BAB

                loss_G.backward(retain_graph=True)
                optimizer_G.step()

                # loss_G.backward()
                # optimizer_G.step()

                ###################################

                ###### Discriminator A ######
                optimizer_D_A.zero_grad()

                # Real loss
                pred_real = netD_A(real_A)
                loss_D_real = criterion_GAN(pred_real, target_real)

                # Fake loss
                fake_A = fake_A_buffer.push_and_pop(fake_A)
                # fake_A 由 G 生成, detach 使更新不影响G
                pred_fake = netD_A(fake_A.detach())
                loss_D_fake = criterion_GAN(pred_fake, target_fake)

                # Total loss
                loss_D_A = (loss_D_real + loss_D_fake)*0.5
                loss_D_A.backward()

                optimizer_D_A.step()
                ###################################

                ###### Discriminator B ######
                optimizer_D_B.zero_grad()

                # Real loss
                pred_real = netD_B(real_B)
                loss_D_real = criterion_GAN(pred_real, target_real)

                # Fake loss
                fake_B = fake_B_buffer.push_and_pop(fake_B)
                pred_fake = netD_B(fake_B.detach())
                loss_D_fake = criterion_GAN(pred_fake, target_fake)

                # Total loss
                loss_D_B = (loss_D_real + loss_D_fake)*0.5
                loss_D_B.backward()

                optimizer_D_B.step()

                logger.log({'loss_G': loss_G, 'loss_G_GAN': (loss_GAN_A2B + loss_GAN_B2A),
                            'loss_G_cycle': (loss_cycle_ABA + loss_cycle_BAB), 'loss_D': (loss_D_A + loss_D_B)},
                           images={'real_A': real_A, 'real_B': real_B, 'fake_A': fake_A, 'fake_B': fake_B})

            # Update learning rates
            lr_scheduler_G.step()
            lr_scheduler_D_A.step()
            lr_scheduler_D_B.step()

            # Save models checkpoints
            if epoch % 20 == 19:

                torch.save(netG_A2B.state_dict(), opt.output_dir +
                           '/{}_netG_A2B.pth'.format(epoch))
                torch.save(netG_B2A.state_dict(), opt.output_dir +
                           '/{}_netG_B2A.pth'.format(epoch))
                torch.save(netD_A.state_dict(), opt.output_dir +
                           '/{}_netD_A.pth'.format(epoch))
                torch.save(netD_B.state_dict(), opt.output_dir +
                           '/{}_netD_B.pth'.format(epoch))

        ###################################


if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument('--epoch', type=int, default=0, help='starting epoch')
    parser.add_argument('--n_epochs', type=int, default=200,
                        help='number of epochs of training')
    parser.add_argument('--batchSize', type=int, default=2,
                        help='size of the batches')
    parser.add_argument('--dataroot', type=str, default='datasets/horse2zebra/',
                        help='root directory of the dataset')
    parser.add_argument('--lr', type=float, default=0.0005,
                        help='initial learning rate')
    parser.add_argument('--decay_epoch', type=int, default=100,
                        help='epoch to start linearly decaying the learning rate to 0')
    parser.add_argument('--size', type=int, default=128,
                        help='size of the font_data crop (squared assumed)')
    parser.add_argument('--input_nc', type=int, default=3,
                        help='number of channels of input font_data')
    parser.add_argument('--output_nc', type=int, default=3,
                        help='number of channels of output font_data')
    parser.add_argument('--cuda', action='store_true',
                        help='use GPU computation')
    parser.add_argument('--n_cpu', type=int, default=0,
                        help='number of cpu threads to use during batch generation')
    parser.add_argument('--sequential', action='store_true',
                        help='if the dataset have a sequence')
    parser.add_argument('--ske', action='store_true',
                        help='if the dataset have a skeleton input')
    parser.add_argument('--output_dir', type=str,
                        default='./output', help='place to output result')
    opt = parser.parse_args()

    if torch.cuda.is_available() and not opt.cuda:
        print("WARNING: You have a CUDA device, so you should probably run with --cuda")

    os.makedirs(opt.output_dir, exist_ok=True)
    main(opt)
