"""
@File: main.py
@Brief: 
@Author: Jingjing Jiang
@Email: jingjingjiang2017@gmail.com
@Institution: IAIR, Xi'an Jiaotong University
@Time: 2019/7/1 下午2:38
"""
import torch
import torch.nn as nn
from torch.utils.data import DataLoader
import torch.optim as optim
import argparse
from h3d import H3D, my_collate_fn
from sklearn import svm, metrics
from vgg_model import *
from pointnet_model import *
from torchsummary import summary
from tensorboardX import SummaryWriter  # tensorboard --logdir=./log


# parameters
parser = argparse.ArgumentParser()
parser.add_argument('--data_dir',
                    default='/media/jing/YINGPAN/PC_benchmark/icra_benchmark',
                    help='path to dataset')
parser.add_argument('--num_classes',
                    default=4,
                    help='number of classes')
parser.add_argument('--BoundaryCond',
                    default=[70, 0, 40, -40],
                    help='the RoI of point cloud: [x_max, x_min, y_max, y_min]')
parser.add_argument('--rv_resolution',
                    default=[64, 900],  # 360, 450, 900, 1000
                    help='resolution of range image')
parser.add_argument('--npoints', default=1000,
                    help='')
parser.add_argument('--feature_transform', default=True,
                    help='')
parser.add_argument('--bs', default=1,
                    help='batch size')
parser.add_argument('--epoch', default=20,
                    help='epoch')
opt = parser.parse_args()
print(opt)

# load dataset
train_data = H3D(opt, 'train')
train_loader = DataLoader(train_data,
                          batch_size=opt.bs,
                          shuffle=True,
                          collate_fn=my_collate_fn
                          )

val_data = H3D(opt, 'val')
val_loader = DataLoader(val_data,
                        batch_size=opt.bs,
                        shuffle=True,
                        collate_fn=my_collate_fn
                        )


# for i, batch in enumerate(train_loader):
#     point, label, num = batch
#     # img = img.reshape([-1, 8, 8, 3])
#     point = point.reshape([-1, opt.npoints, 4])
#     label = label.reshape([-1, 1])

#############################################################################
# object : point cloud
#############################################################################

pointnet_classifier = PointNetCls(k=opt.num_classes,
                                  feature_transform=True)
optimizer = optim.Adam(pointnet_classifier.parameters(),
                       lr=0.001,
                       betas=(0.9, 0.999))
scheduler = optim.lr_scheduler.StepLR(optimizer,
                                      step_size=20,
                                      gamma=0.5)

# num_obj = 0
writer = SummaryWriter('./log')
iter = -1
for epoch in range(opt.epoch):
    print("Epoch{}/{}".format(epoch, opt.epoch))
    scheduler.step()
    
    losses = 0.0
    for i, batch in enumerate(train_loader):
        point, label, num = batch
        if len(point) == 0:
            continue

        point = point.reshape([-1, opt.npoints, 3])
        point = point.permute(0, 2, 1)
        label = label.reshape([-1]).long()

        # num_obj += len(label)
        
        optimizer.zero_grad()
        
        pred, trans, trans_feat = pointnet_classifier(point)

        loss = F.nll_loss(pred, label)
        if opt.feature_transform:
            loss += feature_transform_regularizer(trans_feat) * 0.001
            
        loss.backward()
        optimizer.step()

        pred_choice = pred.data.max(1)[1]
        correct = pred_choice.eq(label.data).sum()

        losses += loss.item()

        if (i+1) % 5 == 0:
            print("train loss: {:.4f}, train acc: {:.4f}"
                  .format(losses/(i + 1), torch.div(correct.float(),
                                                    len(label))))
            iter += 1
        writer.add_scalar('train/loss:', losses/(i + 1), iter)

        if (i+1) % 200 == 0:
            print("start val:===========================================")
            pointnet_classifier.eval()
            num_obj_val = 0
            corrects_val = 0.0
            for j, batch_val in enumerate(train_loader):
                point, label, num = batch_val
                if len(point) == 0:
                    continue
    
                point = point.reshape([-1, opt.npoints, 3])
                point = point.permute(0, 2, 1)
                label = label.reshape([-1]).long()

                pred, _, _ = pointnet_classifier(point)
                pred_choice = pred.data.max(1)[1]

                correct = pred_choice.eq(label.data).sum()
                corrects_val += correct
                num_obj_val += len(label)
            
            val_acc = torch.div(corrects_val.float(), num_obj_val)
            if val_acc > 0.9:
                torch.save(pointnet_classifier.state_dict(),
                           './checkpoints/pointnet_cls_mini_acc{:.4f}'
                           .format(val_acc))
            
            pointnet_classifier.train()  # val结束, 恢复train
            print("val acc: {:.4f}".format(torch.div(corrects_val.float(),
                                                     num_obj_val)))

        # writer.add_scalar('val/loss:', losses / (i + 1), iter)
        
        
#############################################################################
# object RV
#############################################################################
# load Model
# model = VGGNet16(opt.num_classes)
# summary(model, input_size=(3, 8, 8))
#
# cost = nn.CrossEntropyLoss()
# optimizer = optim.Adam(model.parameters(), lr=0.0001)
#
# losses = 0.0
# corrects = 0
# corrects_val = 0
# num_obj = 0
#
# for epoch in range(opt.epoch):
#     print("Epoch{}/{}".format(epoch, opt.epoch))
#     model.train()
#     for i, batch in enumerate(train_loader):
#         img, label = batch
#         if len(img) == 0:
#             continue
#         img = img.reshape([-1, 8, 8, 3])  # [batch, w, h, channel]
#         img = img.permute(0, 3, 1, 2)  # [batch, channel, w, h]
#         label = label.reshape([-1]).long()
#
#         num_obj += len(label)
#
#         y_pred = model(img)
#         _, pred_1 = torch.max(y_pred.data, 1)
#
#         optimizer.zero_grad()
#
#         loss = cost(y_pred, label)
#         loss.backward()
#
#         optimizer.step()
#
#         losses += loss.item()
#         corrects += torch.sum(pred_1 == label.data)
#         if (i+1) % 5 == 0:
#             print("train loss: {:.4f}, train acc: {:.4f}"
#                   .format(losses/(i + 1), torch.div(corrects.float(), num_obj)))
#
#         if (i+1) % 50 == 0:
#             num_obj_val = 0
#             model.eval()
#             for j, batch_val in enumerate(train_loader):
#                 img, label = batch_val
#                 if len(img) == 0:
#                     continue
#                 img = img.reshape([-1, 8, 8, 3])  # [batch, w, h, channel]
#                 img = img.permute(0, 3, 1, 2)  # [batch, channel, w, h]
#                 label = label.reshape([-1]).long()
#
#                 num_obj_val += len(label)
#
#                 y_pred = model(img)
#                 _, pred_1 = torch.max(y_pred.data, 1)
#
#                 corrects_val += torch.sum(pred_1 == label.data)
#
#             print("val acc: {:.4f}".format(torch.div(corrects.float(),
#                                                      num_obj_val)))
