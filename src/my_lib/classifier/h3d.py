import torch.utils.data as data
import os
import numpy as np
import matplotlib.pyplot as plt
from plyfile import PlyData
import torch
from torch.utils.data.dataloader import default_collate
from skimage.transform import resize
from skimage.color import rgb2gray
from velo_utils import *
# from image import *


class H3D(data.Dataset):
    def __init__(self, opt, split):
        self.opt = opt
        self.split = split
        self.num_classes = opt.num_classes
        self.img_resolution = opt.rv_resolution
        
        self.num_points = 1000  # 每个目标输入的点数
        
        self.class_name = ['__background__', 'car', 'pedestrian', 'cyclist',
                           'truck', 'misc', 'animals', 'motorcyclist', 'bus']
        # self.cat_ids = {'others': -1, 'car': 0, 'pedestrian': 1, 'cyclist': 2,
        #                 'truck': -3, 'misc': -99, 'animals': -99,
        #                 'motorcyclist': -4, 'bus': -3}
        self.cat_ids = {'others': 3, 'car': 0, 'pedestrian': 1, 'cyclist': 2,
                        'truck': 0, 'misc': 3, 'animals': 3,
                        'motorcyclist': 2, 'bus': 0}
        
        self.detected_list = {'car', 'pedestrian', 'cyclist'}
        self.max_objs = 50

        self.input_h = self.img_resolution[0]
        self.input_w = self.img_resolution[1]
        self.output_h = self.input_h // 4
        self.output_w = self.input_w // 4
        
        # self.opt = opt
        self.bc = opt.BoundaryCond
        self.x_range = self.bc[0] - self.bc[1]
        self.y_range = self.bc[2] - self.bc[3]
        # self.x_range = self.bc['maxX'] - self.bc['minX']
        # self.y_range = self.bc['maxY'] - self.bc['minY']
        self.bev2outmap = self.x_range / self.output_h
        
        # self.data_dir = os.path.join(os.getcwd(), 'data')
        # self.data_dir = os.path.join(self.data_dir, 'h3d')
        self.data_dir = opt.data_dir
        train_test_split = os.path.join(self.data_dir, 'train_test_split')
        folder_list_file = os.path.join(train_test_split,
                                        'detection_' + split + '.txt')
        with open(folder_list_file) as fi:
            self.folder_list = fi.readlines()
        
        data_dict = []
        start = 0
        for (i, folder_name) in enumerate(self.folder_list):
            folder_dir = os.path.join(self.data_dir, folder_name.strip())
            filelist = os.listdir(folder_dir)
            filelist.sort()
            nums = int(filelist[-1][-7:-4]) + 1
            data_dict.append(start)
            start += nums
        
        self.data_dict = np.array(data_dict)
        self.nums = start
        
        # src = np.array([[0, 0], [self.x_range, 0], [0, -self.y_range / 2]])
        # dst = np.array(
        #     [[0, self.output_w / 2], [self.output_h, self.output_w / 2],
        #      [0, 0]])
        # self.trans = cv2.getAffineTransform(np.float32(src),
        #                                     np.float32(dst))
        # self.anti_trans = cv2.getAffineTransform(np.float32(dst),
        #                                          np.float32(src))
    
    def __len__(self):
        return self.nums
    
    def _load_label(self, label_name):
        with open(label_name) as f:
            lines = f.readlines()
        num_objs = len(lines)
        anns = []
        for i in range(num_objs):
            ann = {}
            obj = lines[i].strip().split(',')
            ann['class'] = obj[0] if obj[0] in self.class_name else 'others'
            ann['location'] = np.array(
                [np.float32(obj[3]), np.float32(obj[4]), np.float32(obj[5])])
            ann['dimension'] = np.array(
                [np.float32(obj[6]), np.float32(obj[7]), np.float32(obj[8])])
            ann['rotation'] = np.float32(obj[-1])
            if ann['location'][0] > self.bc[0] or ann['location'][0] < \
                    self.bc[1] or ann['location'][1] > self.bc[2] or \
                    ann['location'][1] < self.bc[3]:
                continue
            anns.append(ann)
        return len(anns), anns
    
    def _item(self, index):
        folder_index = (index >= self.data_dict).sum() - 1
        file_index = index - self.data_dict[folder_index]
        return folder_index, file_index
    
    def load_img_and_label_path(self, index):
        folder_index, file_index = self._item(index)
        file_index_str = str(file_index).zfill(3)
        pcl_name = os.path.join(self.data_dir,
                                self.folder_list[folder_index].strip(),
                                'pointcloud1_' + file_index_str + '.ply')
        label_name = os.path.join(self.data_dir,
                                  self.folder_list[folder_index].strip(),
                                  'labels_3d1_' + file_index_str + '.txt')
        return pcl_name, label_name
    
    def ply2pc(self, pcl_name):
        plydata = PlyData.read(pcl_name)
        x = np.array(plydata.elements[0]['x'])
        y = np.array(plydata.elements[0]['y'])
        z = np.array(plydata.elements[0]['z'])
        radius = np.array(plydata.elements[0]['radius'])
        confidence = np.array(plydata.elements[0]['confidence'])
        
        pc = np.vstack([x, y, z, radius, confidence]).T
        
        return pc
        
    def __getitem__(self, index):
        pcl_name, label_name = self.load_img_and_label_path(index)
        pc = self.ply2pc(pcl_name)  # raw data
        # rv, rv2 = velo2rv(pc, self.input_w, self.input_h)
        # img_trans = rv.transpose(2, 0, 1)
        
        num_objs, anns = self._load_label(label_name)
        num_objs = min(num_objs, self.max_objs)
        
        # img = np.zeros([num_objs, 8, 8, 3], dtype=np.float32)
        # points = np.zeros([num_objs, ])
        points = np.zeros([num_objs, self.num_points, 3], dtype=np.float32)
        label = np.zeros([num_objs, 1], dtype=np.float32)
        num_obj_points = np.zeros([num_objs, 1], dtype=np.float32)
        label[:] = 3
        for k in range(num_objs):
            ann = anns[k]
            location = ann['location']
            dimension = ann['dimension']
            yaw = ann['rotation']
            cls_id = self.cat_ids[ann['class']]
            
            bbox = np.hstack((location, dimension, yaw))
            # corners = box2corners(bbox)
            point_k = bbox2pc(pc, bbox)
            num_ = len(point_k)
            num_obj_points[k] = num_
            if len(point_k) > self.num_points:
                r = np.linalg.norm(np.vstack([point_k[:, 0], point_k[:, 1]]).T,
                                   axis=1)
                point_k = np.column_stack((point_k, r))
                point_k = point_k[np.lexsort(point_k.T)]
                points[k, :, :] = point_k[:self.num_points, 0:3]
                
            else:
                points[k, :num_, :] = point_k[:, 0:3]
            
            # rv2 = bbox2rv(pc, bbox, self.input_w)
            # rect = corners_rv(pc, bbox, self.input_w)
            # if rect.size == 0:
            #     continue
            
            # image = rv2[rect[3]:rect[1], rect[2]:rect[0], :]
            # image = rgb2gray(image)
            # image = resize(image, (8, 8, 3), mode='reflect')
            # image = image.reshape((64,), order='C')
            # img[k, :] = image
            label[k] = cls_id
        
        # result = {'image': img,
        #           'label': label}
        # return result
        # return img, label
        return points, label, num_obj_points


def my_collate_fn(batch):
    """
    batch中每个元素形如(data, label)
    :param batch:
    :return:
    """
    
    # 过滤为None的数据
    batch = list(filter(lambda x: x[0] is not None, batch))
    if len(batch) == 0:
        return torch.Tensor()
    return default_collate(batch)  # 用默认方式拼接过滤后的batch数据
    # img, label = zip(*batch)
    # pad_img = []
    # pad_label = []
    # max_len = len(label)
    # lens = []
    # for i in range(len(batch)):
    #     temp_label = [0] * max_len
    #     temp_label[:len(label[i])] = label[i]
    #     pad_label.append(temp_label)
    #
    #     temp_img = img[i]
    #     pad_img.append(temp_img)
    #
    #     lens.append(len(label[i]))
    #
    # return pad_img, pad_label, lens
