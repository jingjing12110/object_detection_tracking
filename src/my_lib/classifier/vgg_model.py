"""
@File: vgg_model.py
@Brief: 
@Author: Jingjing Jiang
@Email: jingjingjiang2017@gmail.com
@Institution: IAIR, Xi'an Jiaotong University
@Time: 2019/7/1 下午2:38
"""
import torch
import torch.nn as nn
import torch.nn.functional as F
from torchvision import models


class VGGNet16(nn.Module):
    def __init__(self, num_classes=8):
        super(VGGNet16, self).__init__()
        self.Conv = nn.Sequential(
            nn.Conv2d(3, 64, kernel_size=3, stride=1, padding=1),
            nn.ReLU(),
            nn.Conv2d(64, 64, kernel_size=3, stride=1, padding=1),
            nn.ReLU(),
            nn.MaxPool2d(kernel_size=2, stride=2),
            nn.Conv2d(64, 128, kernel_size=3, stride=1, padding=1),
            nn.ReLU(),
            nn.Conv2d(128, 128, kernel_size=3, stride=1, padding=1),
            nn.ReLU(),
            nn.MaxPool2d(kernel_size=2, stride=2),
            nn.Conv2d(128, 256, kernel_size=3, stride=1, padding=1),
            nn.ReLU(),
            nn.Conv2d(256, 256, kernel_size=3, stride=1, padding=1),
            nn.ReLU(),
            nn.Conv2d(256, 256, kernel_size=3, stride=1, padding=1),
            nn.ReLU(),
            nn.MaxPool2d(kernel_size=2, stride=2),
            nn.Conv2d(256, 512, kernel_size=3, stride=1, padding=1),
            nn.ReLU(),
            nn.Conv2d(512, 512, kernel_size=3, stride=1, padding=1),
            nn.ReLU(),
            nn.Conv2d(512, 512, kernel_size=3, stride=1, padding=1),
            nn.ReLU(),
            nn.MaxPool2d(kernel_size=2, stride=2)
        )
        self.classifier = nn.Sequential(
            nn.Linear(4 * 4 * 128, 1024),
            nn.ReLU(),
            nn.Dropout(p=0.5),
            # nn.Linear(1024, 1024),
            # nn.ReLU(),
            # nn.Dropout(p=0.5),
            nn.Linear(1024, num_classes),
        )
        
    def forward(self, inputs):
        x = self.Conv[:9](inputs)
        x = x.view(-1, 4 * 4 * 128)
        x = self.classifier(x)
        return x


class VggClassifier(nn.Module):
    def __init__(self, opt):
        super(VggClassifier, self).__init__()
        self.vgg16 = models.vgg16(pretrained=True)
        for param in self.vgg16.parameters():
            param.requires_grad = False
    
        self.vgg16.classifier = nn.Sequential(
            nn.Linear(in_features=25088, out_features=4096, bias=True),
            nn.ReLU(),
            nn.Dropout(p=0.5),
            nn.Linear(in_features=4096, out_features=4096, bias=True),
            nn.ReLU(),
            nn.Dropout(p=0.5),
            nn.Linear(in_features=4096, out_features=opt.num_classes, bias=True)
        )
        
    def forward(self, inputs):
        x = self.Conv(inputs)
        x = x.view(-1, 4 * 4 * 512)
        x = self.classifier(x)
        return x


class FcnClassifier(nn.Module):
    def __init__(self, opt):
        super(FcnClassifier, self).__init__()
        self.model = nn.Sequential()
        
    def forward(self, inputs):
        x = self.model(inputs)
        return x


class CsNet(nn.Module):
    def __init__(self, n_feature, n_hidden, n_output):
        super(CsNet, self).__init__()  # 继承 __init__ 功能
        self.hidden = torch.nn.Linear(n_feature, n_hidden)  # 隐藏层线性输出
        self.out = torch.nn.Linear(n_hidden, n_output)  # 输出层线性输出

    def forward(self, x):
        # 正向传播输入值, 神经网络分析出输出值
        x = F.relu(self.hidden(x))  # 激励函数(隐藏层的线性值)
        x = self.out(x)  # 输出值, 但是这个不是预测值, 预测值还需要再另外计算
        return x
