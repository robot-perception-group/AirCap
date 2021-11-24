#!/usr/bin/python3
import torch
import torch.nn as nn
from collections import OrderedDict

# Normalization layer - batchnorm doesn't work well with either multibox nor LSTM approaches, so we use groupnorms
# this is just a wrapper to auto-calculate number of groups from input channels
class AutoGroupNorm(nn.GroupNorm):
    def __init__(self, num_features, channels_per_group=8, **args):
        groups = num_features//channels_per_group
        groups = groups if groups>0 else 1
        super().__init__(groups,num_features,**args)

#Basic Conv+Norm+Relu - need that a lot
class AutoConv2d(nn.Sequential):
    def __init__(self, inplanes, planes, kernel_size=3, stride=2, padding=1, groups=1, bias=True, norm_layer=None, relu=nn.ReLU, dilation=1):
        self.width = planes
        layers = OrderedDict()

        layers['conv'] = nn.Conv2d(inplanes, planes, kernel_size=kernel_size,
                stride=stride, padding=padding, groups=groups, bias=bias, dilation=dilation)
        if norm_layer is not None:
            layers['norm'] = norm_layer(planes)
        if relu is not None:
            layers['relu'] = relu(inplace=True)
        super().__init__(layers)

# ResNet basic block - in its simplest form
class ResnetBasicBlock(nn.Module):
    def __init__(self, inplanes, planes, stride=1, norm_layer=AutoGroupNorm, **args):
        super().__init__()
        self.branch = nn.Sequential(OrderedDict({
            'conv1':AutoConv2d(inplanes, planes, stride=stride, norm_layer=norm_layer, bias=False),
            'conv2':AutoConv2d(planes, planes, stride=1, norm_layer=norm_layer, bias=False, relu=None),
            }))
        if stride==1:
            self.residual = nn.Identity()
        else:
            self.residual = AutoConv2d(inplanes, planes, kernel_size=1, padding=0, stride=stride, norm_layer=norm_layer, bias=False, relu=None)
        self.relu = nn.ReLU(inplace=True)
        self.width = planes

    def forward(self, x):
        return self.relu(self.branch(x) + self.residual(x))

# Define Resnet as a nn.Sequential - because it has many layers
class TruncatedResnet(nn.Sequential):
    def __init__(self, inplanes=3, block=ResnetBasicBlock, layerdescription=[3,4,6], 
            base_width=64, bias=False, norm_layer=AutoGroupNorm):
        blocks = OrderedDict()
        blocks['conv'] = AutoConv2d(inplanes, base_width, kernel_size=7, padding=3, bias=bias, norm_layer=norm_layer)
        blocks['maxpool'] = nn.MaxPool2d(kernel_size=3, stride=2, padding=1)
        channels = base_width
        for n,l in enumerate(layerdescription,start=1):
            group = OrderedDict()
            start = 0
            if channels != base_width:
                start = 1
                group['0'] = block(channels, base_width, stride=2, norm_layer=norm_layer)
                channels = base_width
            for a in range(start,l):
                group[str(a)] = block(base_width, base_width, stride=1, norm_layer=norm_layer)
            base_width *=2
            blocks['layer'+str(n)] = nn.Sequential(group)
        self.width = channels
        super().__init__(blocks)

class L2Norm(nn.Module):
    def __init__(self,n_channels, scale):
        super().__init__()
        self.n_channels = n_channels
        self.gamma = scale or None
        self.eps = 1e-10
        self.weight = nn.Parameter(torch.Tensor(self.n_channels))
        self.reset_parameters()

    def reset_parameters(self):
        torch.nn.init.constant(self.weight,self.gamma)

    def forward(self, x):
        norm = x.pow(2).sum(dim=1, keepdim=True).sqrt()+self.eps
        #x /= norm
        x = torch.div(x,norm)
        out = self.weight[:,None,None] * x
        return out

#define VGG as nn.Sequential - it also has many layers
class VGG(nn.Sequential):
    def __init__(self,inplanes=3, layerdescription=[1, 1, 'M', 2, 2, 'M', 4, 4, 4, 'C', 8, 8, 8, 'M', 8, 8, 8], base_width=64):
        layers = OrderedDict()
        in_channels = inplanes
        for layer in range(len(layerdescription)):
            key="layer"+str(layer)
            val=layerdescription[layer]
            if val=='M':
                layers[key] = nn.MaxPool2d(kernel_size=2, stride=2)
            elif val=='C':
                layers[key] = nn.MaxPool2d(kernel_size=2, stride=2, ceil_mode=True)
            else:
                layers[key] = AutoConv2d(in_channels,val*base_width,stride=1)
                in_channels = val*base_width
        layers["layer"+str(layer+1)] = nn.MaxPool2d(kernel_size=3, stride=1, padding=1)
        layers["layer"+str(layer+2)] = AutoConv2d(in_channels,1024,stride=1, padding=6, dilation=6)
        layers["layer"+str(layer+3)] = AutoConv2d(1024,1024, kernel_size=1, stride=1, padding=0)
        self.width = 1024
        super().__init__(layers)

# Depthwise convolution, like in Mobilenet
class ConvDepthWise(nn.Sequential):
    def __init__(self, inplanes, planes, stride=2, padding=1, norm_layer=AutoGroupNorm):
        self.width = planes
        super().__init__(OrderedDict({
                'conv1':AutoConv2d(inplanes, planes, kernel_size=1, stride=1, padding=0, norm_layer=norm_layer, bias=False, relu=nn.ReLU6),
                'conv2':AutoConv2d(planes, planes, stride=stride, padding=padding, groups=planes, norm_layer=norm_layer, bias=False, relu=nn.ReLU6),
                'conv3':AutoConv2d(planes, planes, kernel_size=1, stride=1, padding=0, norm_layer=norm_layer, bias=False, relu=None),
            }))

class SmartSequential(nn.Sequential):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.width=self[-1].width

class FSSDLite(nn.Module):
    def __init__(self, width=128, classes=2):
        super().__init__()
        self.classes=classes
        self.resnet=TruncatedResnet()
        self.extras=SmartSequential(
                ConvDepthWise(self.resnet.width,width),
            )
        self.transforms = nn.ModuleList()
        transform_width = self.extras[-1].width//2
        self.transforms.append( AutoConv2d(self.resnet.layer2[-1].width, transform_width, kernel_size=1, stride=1, padding=0) )
        self.transforms.append( AutoConv2d(self.resnet.layer3[-1].width, transform_width, kernel_size=1, stride=1, padding=0) )
        self.transforms.append( AutoConv2d(self.extras[0].width, transform_width, kernel_size=1, stride=1, padding=0) )
        self.fuse = AutoConv2d(len(self.transforms)*transform_width, width, stride=1)
        self.pyramid = nn.ModuleList()
        self.conf = nn.ModuleList()
        self.loc = nn.ModuleList()

        # 6 pyramid layers
        aspect_ratios=[2,3,3,3,2,2]
        strides=[1,2,2,2,1,1]
        for l,r in enumerate(aspect_ratios):
            self.pyramid.append(nn.Identity() if l==0 else AutoConv2d(width, width, stride=strides[l], padding=(1 if strides[l]>1 else 0) ) )
            self.conf.append( AutoConv2d(width, (2*r)*classes, stride=1, relu=None) )
            self.loc.append( AutoConv2d(width, (2*r)*4, stride=1, relu=None) )

    def features(self, x):

        # run through resnet and collect some data
        transforminputs=[]
        for k, l in self.resnet.named_children():
            x = l(x)
            if k in ['layer2','layer3']:
                transforminputs.append(x)
        # run through extra layers and collect more data
        for k, l in self.extras.named_children():
            x = l(x)
            transforminputs.append(x)

        # transform data
        transformoutputs = []
        sizes = None
        for k, l in enumerate(transforminputs):
            y = self.transforms[k](l)
            if k==0:
                sizes = y.shape[2:]
            else:
                y = torch.nn.functional.interpolate( y, size=sizes )
            transformoutputs.append(y)

        # fuse data
        y = torch.cat(transformoutputs, dim=1)
        y = self.fuse(y)
        return x,y

    def boxes(self, x):
        conf = []
        loc = []
        # extract detections
        for k, l in enumerate(self.pyramid):
            x = l(x)
            conf.append( self.conf[k](x).permute(0,2,3,1).contiguous().flatten(start_dim=1) )
            loc.append( self.loc[k](x).permute(0,2,3,1).contiguous().flatten(start_dim=1) )
        conf = torch.cat(conf,1).view(x.shape[0],-1,self.classes)
        loc = torch.cat(loc,1).view(x.shape[0],-1,2,2)

        return loc,conf


    def forward(self, x, **argc):

        return self.boxes(self.features(x)[1])


class SSDOrig(nn.Module):
    def __init__(self, width=128, classes=2):
        super().__init__()
        self.classes=classes
        self.vgg=VGG()
        self.norm=L2Norm(512,20)
        extras=OrderedDict()
        extras['layer20']=SmartSequential(
                AutoConv2d(1024,256, kernel_size=1, stride=1, padding=0, relu=None),
                AutoConv2d(256, 512))
        extras['layer21']=SmartSequential(
                AutoConv2d(512,128, kernel_size=1, stride=1, padding=0, relu=None),
                AutoConv2d(128, 256))
        extras['layer22']=SmartSequential(
                AutoConv2d(256,128, kernel_size=1, stride=1, padding=0, relu=None),
                AutoConv2d(128, 256, stride=1, padding=0))
        extras['layer23']=SmartSequential(
                AutoConv2d(256,128, kernel_size=1, stride=1, padding=0, relu=None),
                AutoConv2d(128, 256, stride=1, padding=0))
        self.extras=SmartSequential(extras)

        self.conf = nn.ModuleList()
        self.loc = nn.ModuleList()

        aspect_ratios=[2,3,3,3,2,2]
        widths=[512, 1024, 512, 256, 256, 256]
        for l,r in enumerate(aspect_ratios):
            self.conf.append( AutoConv2d(widths[l], (2*r)*classes, stride=1, relu=None) )
            self.loc.append( AutoConv2d(widths[l], (2*r)*4, stride=1, relu=None) )

    def forward(self, x, **argc):

        sources=[]
        for k, l in self.vgg.named_children():
            x = l(x)
            if k in ['layer12','layer19']:
                sources.append(x)

        # run through extra layers and collect more data
        for k, l in self.extras.named_children():
            x = l(x)
            sources.append(x)

        # normalization
        sources[0] = self.norm(sources[0])

        conf = []
        loc = []
        # extract detections
        for k, l in enumerate(sources):
            conf.append( self.conf[k](l).permute(0,2,3,1).contiguous().flatten(start_dim=1) )
            loc.append( self.loc[k](l).permute(0,2,3,1).contiguous().flatten(start_dim=1) )
        conf = torch.cat(conf,1).view(x.shape[0],-1,self.classes)
        loc = torch.cat(loc,1).view(x.shape[0],-1,2,2)

        return loc,conf

