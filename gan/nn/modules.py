import torch.nn as nn
import torch.nn.functional as F
import torch
from .stylegan import StyledGenerator


class ResidualBlock(nn.Module):
    def __init__(self, in_features):
        super(ResidualBlock, self).__init__()

        conv_block = [nn.ReflectionPad2d(1),
                      nn.Conv2d(in_features, in_features, 3),
                      nn.InstanceNorm2d(in_features),
                      nn.ReLU(inplace=True),
                      nn.ReflectionPad2d(1),
                      nn.Conv2d(in_features, in_features, 3),
                      nn.InstanceNorm2d(in_features)]

        self.conv_block = nn.Sequential(*conv_block)

    def forward(self, x):
        return x + self.conv_block(x)


class Generator(nn.Module):
    def __init__(self, input_nc, output_nc, conditional=False, n_residual_blocks=6):
        super(Generator, self).__init__()

        self.conditional = conditional
        # Initial convolution block
        model_in = [nn.ReflectionPad2d(3),
                    nn.Conv2d(input_nc, 64, 7),
                    nn.InstanceNorm2d(64),
                    nn.ReLU(inplace=True)]

        # Downsampling
        in_features = 64
        out_features = in_features*2
        for _ in range(2):
            model_in += [nn.Conv2d(in_features, out_features, 3, stride=2, padding=1),
                         nn.InstanceNorm2d(out_features),
                         nn.ReLU(inplace=True)]
            in_features = out_features
            out_features = in_features*2

        # Residual blocks
        for _ in range(n_residual_blocks):
            model_in += [ResidualBlock(in_features)]

        self.encoder = nn.Sequential(*model_in)

        # Upsampling
        model = []
        out_features = in_features // 2
        for _ in range(2):
            model += [nn.ConvTranspose2d(in_features, out_features, 3, stride=2, padding=1, output_padding=1),
                      nn.InstanceNorm2d(out_features),
                      nn.ReLU(inplace=True)]
            in_features = out_features
            out_features = in_features//2

        output_layer = [nn.ReflectionPad2d(3),
                        nn.Conv2d(64, output_nc, 7),
                        nn.Tanh()]

        self.output_layer = nn.Sequential(*output_layer)
        self.upsampler = nn.Sequential(*model)

    def forward(self, x):
        feat_x = self.encoder(x)
        feat_x = self.upsampler(feat_x)
        output = self.output_layer(feat_x)
        return output

    def forward_with_stroke(self, x, y):
        feat_x = self.model(x)
        feat_y = self.model(y)

        feat = torch.cat((feat_x, feat_y), dim=1)
        feat = self.post_process(feat)
        output = self.output_layer_strokewise(feat)
        return output


class SequentialGenerator(nn.Module):
    def __init__(self, input_nc, output_nc, conditional=False, n_residual_blocks=6):
        super(SequentialGenerator, self).__init__()

        self.conditional = conditional
        # Initial convolution block
        model_in = [nn.ReflectionPad2d(3),
                    nn.Conv2d(input_nc, 64, 7),
                    nn.InstanceNorm2d(64),
                    nn.ReLU(inplace=True)]

        # Downsampling
        in_features = 64
        out_features = in_features*2
        for _ in range(2):
            model_in += [nn.Conv2d(in_features, out_features, 3, stride=2, padding=1),
                         nn.InstanceNorm2d(out_features),
                         nn.ReLU(inplace=True)]
            in_features = out_features
            out_features = in_features*2

        # Residual blocks
        for _ in range(n_residual_blocks):
            model_in += [ResidualBlock(in_features)]

        self.encoder = nn.Sequential(*model_in)

        # Upsampling
        model = []
        out_features = in_features // 2
        for _ in range(2):
            model += [nn.ConvTranspose2d(in_features, out_features, 3, stride=2, padding=1, output_padding=1),
                      nn.InstanceNorm2d(out_features),
                      nn.ReLU(inplace=True)]
            in_features = out_features
            out_features = in_features//2

        output_layer = [nn.ReflectionPad2d(3),
                        nn.Conv2d(64, output_nc, 7),
                        nn.Tanh()]

        self.output_layer = nn.Sequential(*output_layer)
        self.upsampler = nn.Sequential(*model)

    def forward(self, x, y=None):
        feat_x = self.encoder(x)

        feat_x = self.upsampler(feat_x)
        output = self.output_layer(feat_x)
        return output



class Encoder(nn.Module):
    def __init__(self, input_nc, n_residual_blocks=6):
        super(Encoder, self).__init__()
        # Initial convolution block
        model = [nn.ReflectionPad2d(3),
                 nn.Conv2d(input_nc, 64, 7),
                 nn.InstanceNorm2d(64),
                 nn.ReLU(inplace=True)]

        # Downsampling
        in_features = 64
        out_features = in_features*2
        for _ in range(2):
            model += [nn.Conv2d(in_features, out_features, 3, stride=2, padding=1),
                      nn.InstanceNorm2d(out_features),
                      nn.ReLU(inplace=True)]
            in_features = out_features
            out_features = in_features*2

        # Residual blocks
        for _ in range(n_residual_blocks):
            model += [ResidualBlock(in_features)]

        self.encoder = nn.Sequential(*model)

    def forward(self, input):
        return self.encoder(input)


class StyleGanGenerator(nn.Module):
    def __init__(self, input_nc, output_nc, conditional=False, n_residual_blocks=5):
        super(StyleGanGenerator, self).__init__()

        self.encoder = Encoder(input_nc)
        self.generator = StyledGenerator(1024)
        self.pooler = torch.nn.AvgPool2d(8)

    def forward(self, style, condition):
        style = self.encoder(style)
        condition = self.encoder(condition)
        latent_x = self.pooler(style).squeeze(-1).squeeze(-1)
        output = self.generator(latent_x, condition)
        return output


class Discriminator(nn.Module):
    def __init__(self, input_nc, cls_num=1):
        super(Discriminator, self).__init__()

        # A bunch of convolutions one after another
        model = [nn.Conv2d(input_nc, 64, 4, stride=2, padding=1),
                 nn.LeakyReLU(0.2, inplace=True)]

        model += [nn.Conv2d(64, 128, 4, stride=2, padding=1),
                  nn.InstanceNorm2d(128),
                  nn.LeakyReLU(0.2, inplace=True)]

        model += [nn.Conv2d(128, 256, 4, stride=2, padding=1),
                  nn.InstanceNorm2d(256),
                  nn.LeakyReLU(0.2, inplace=True)]

        model += [nn.Conv2d(256, 512, 4, padding=1),
                  nn.InstanceNorm2d(512),
                  nn.LeakyReLU(0.2, inplace=True)]

        # FCN classification layer
        model += [nn.Conv2d(512, cls_num, 4, padding=1)]

        self.model = nn.Sequential(*model)
        self.cls_num = cls_num

    def forward(self, x):
        x = self.model(x)
        # Average pooling and flatten
        return F.avg_pool2d(x, x.size()[2:]).view(x.size()[0], -1)
