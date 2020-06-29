import numpy as np
from plyfile import PlyData
from pyquaternion import Quaternion
from mayavi import mlab
import matplotlib.pyplot as plt
from PIL import Image
from scipy import misc
from sklearn import preprocessing

from encode_util import *

front_view = 180
max_range = 200


def box2corners(bbox, wlh_factor=1) -> np.ndarray:
    """
    Returns the bounding box corners in Lidar Coordinate
    :param bbox: [cx,cy,cz,lx,ly,lz,yaw]
    :param wlh_factor: Multiply w, l, h by a factor to scale the box.
    :return: <np.float: 3, 8>. First four corners are the ones facing forward.
        The last four are the ones facing backwards.
    """
    lx, ly, lz = bbox[3:6] * wlh_factor
    yaw = bbox[6]
    orientation = Quaternion(axis=(0, 0, 1), angle=yaw)
    
    # 3D bounding box corners.
    # (Convention: x points forward, y to the left, z up.)
    x_corners = lx / 2 * np.array([1, 1, 1, 1, -1, -1, -1, -1])
    y_corners = ly / 2 * np.array([1, -1, -1, 1, 1, -1, -1, 1])
    z_corners = lz / 2 * np.array([1, 1, -1, -1, 1, 1, -1, -1])
    corners = np.vstack((x_corners, y_corners, z_corners))
    
    # Rotate
    corners = np.dot(orientation.rotation_matrix, corners)
    
    # Translate
    x, y, z = bbox[:3]
    corners[0, :] = corners[0, :] + x
    corners[1, :] = corners[1, :] + y
    corners[2, :] = corners[2, :] + z
    
    return corners


def points_in_box(bbox, points: np.ndarray):
    """
    Checks whether points are inside the box. Picks one corner as reference (p1)
    and computes the vector to a target point (v).
    Then for each of the 3 axes, project v onto the axis and compare the length.
    Inspired by: https://math.stackexchange.com/a/1552579
    :param bbox:
    :param points: <np.float: 3, n>.
    :return: <np.bool: n, >.
    """
    corners = box2corners(bbox)

    p1 = corners[:, 0]
    p_x = corners[:, 4]
    p_y = corners[:, 1]
    p_z = corners[:, 3]

    i = p_x - p1
    j = p_y - p1
    k = p_z - p1

    v = points[:, :3].T - p1.reshape((-1, 1))

    iv = np.dot(i, v)
    jv = np.dot(j, v)
    kv = np.dot(k, v)

    mask_x = np.logical_and(0 <= iv, iv <= np.dot(i, i))
    mask_y = np.logical_and(0 <= jv, jv <= np.dot(j, j))
    mask_z = np.logical_and(0 <= kv, kv <= np.dot(k, k))
    mask = np.logical_and(np.logical_and(mask_x, mask_y), mask_z)

    return mask


def bbox2pc(pc, bbox):
    """
    gaining point cloud in  gt bounding box.
    :param pc: [x,y,z,radius,confidence]
    :param bbox: [cx,cy,cz,lx,ly,lz,yaw]
    :return:
    """
    mask = points_in_box(bbox, pc)
    p = pc[mask]
    # show_pc(p)
    # # 判断[cx, cy]是否在平行四边形内, z是否在最大/小值之间
    # mask = np.logical_and(pc[:, 2] <= bbox[2] + bbox[5]/2,
    #                       pc[:, 2] >= bbox[2] - bbox[5]/2)
    # # local coordinate / 4
    # box_points = np.array([[bbox[0] + bbox[3]/2, bbox[1] + bbox[4]/2],
    #                        [bbox[0] + bbox[3]/2, bbox[1] - bbox[4]/2],
    #                        [bbox[0] - bbox[3]/2, bbox[1] - bbox[4]/2],
    #                        [bbox[0] - bbox[3]/2, bbox[1] + bbox[4]/2]])
    return p


def corners_rv(pc, bbox, input_w):
    """
    :param pc:
    :param bbox:
    :param input_w:
    :return:
    """
    points = bbox2pc(pc, bbox)
    if points.size == 0:
        return np.array([])
    else:
        v_max = int(127 - np.min(points[:, 4]))
        v_min = int(127 - np.max(points[:, 4]))
        azimuth = np.arctan2(points[:, 0], points[:, 1]) * 180 / np.pi
        ignore_range = (180.0 - front_view) / 2.0
        width = np.int_((azimuth - ignore_range) / front_view * input_w)
        width = width[np.logical_and(width < input_w, width >= 0)]
        if len(width) == 0:
            return np.array([])
        else:
            u_max = int(max(width))
            u_min = int(min(width))
            
            if (v_min == v_max) or (u_min == u_max):
                return np.array([])
            else:
                return np.hstack((u_max, v_max, u_min, v_min))


def bbox2rv(pc, bbox, input_w_):
    """
    mapping points in bbox to range image.
    :param pc:
    :param bbox:
    :return:
    """
    points = bbox2pc(pc, bbox)
    show_pc(points)
    
    ground_r = np.linalg.norm(np.vstack([points[:, 0], points[:, 1]]).T, axis=1)
    azimuth = np.arctan2(points[:, 0], points[:, 1]) * 180 / np.pi

    input_h = int(np.max(points[:, 4]) - np.min(points[:, 4]) + 1)
    input_w = int(input_w_ / 180 * (max(azimuth) - min(azimuth)) + 1)
    # ignore_range = (180.0 - front_view) / 2.0
    # width = np.int_((azimuth - ignore_range) / front_view * input_w)
    width = np.int_((azimuth - min(azimuth)) * input_w_ / 180)

    rv = np.zeros([input_h, input_w, 3])
    rv2 = np.zeros([input_h * 2, input_w, 3])
    rv_distance = np.ones([input_h, input_w, ]) * 999
    w_max = np.max(points[:, 4])
    for i in range(points.shape[0]):
        u = width[i]
        v = int(w_max - points[i][4])
        if rv_distance[v, u] > ground_r[i]:
            rv[v, u, 0] = points[i][2]  # z
            rv[v, u, 1] = ground_r[i]  # distance
            rv[v, u, 2] = points[i][3]  # intensity
            rv_distance[v, u] = ground_r[i]
        
            rv2[2 * v, u, 0] = points[i][2]  # z
            rv2[2 * v + 1, u, 0] = points[i][2]
            rv2[2 * v, u, 1] = ground_r[i]  # distance
            rv2[2 * v + 1, u, 1] = ground_r[i]  # distance
            rv2[2 * v, u, 2] = points[i][3]
            rv2[2 * v + 1, u, 2] = points[i][3]

    return rv2
    

def velo2rv(pc, input_w, input_h):
    x = pc[:, 0]
    y = pc[:, 1]
    z = pc[:, 2]
    radius = pc[:, 3]  # intensity
    confidence = np.int_(pc[:, 4])  # ring ID
    ground_r = np.linalg.norm(np.vstack([x, y]).T, axis=1)  # range
    # r = np.linalg.norm(np.vstack([x, y, z]).T, axis=1)

    z_max = min(z.max(), 4)
    z_min = max(z.min(), -4.5)
    mask = np.logical_and(x > 0, np.logical_and(z_min < z, z < z_max))
    mask = np.logical_and(mask, ground_r > 2)
    x = x[mask]
    y = y[mask]
    z = z[mask]
    radius = radius[mask] / 255  # 反射率 [0, 1]
    confidence = confidence[mask]
    ground_r = ground_r[mask]
    
    ground_r = ground_r / max(ground_r)  # range: [0, 1]
    z = (z - np.min(z)) / (np.max(z) - np.min(z))  # [0, 1], [-1, 1]
    
    azimuth = np.arctan2(x, y) * 180 / np.pi
    ignore_range = (180.0 - front_view) / 2.0
    width = np.int_((azimuth - ignore_range) / front_view * input_w)

    rv = np.zeros([input_h, input_w, 3])
    rv2 = np.zeros([input_h * 2, input_w, 3])
    rv_distance = np.ones([input_h, input_w, ]) * 999
    for i in range(z.shape[0]):
        u = width[i]
        v = int(63 - confidence[i])
        if rv_distance[v, u] > ground_r[i]:
            rv[v, u, 0] = z[i]  # z
            rv[v, u, 1] = ground_r[i]  # distance
            rv[v, u, 2] = radius[i]    # intensity
            rv_distance[v, u] = ground_r[i]
            
            rv2[2*v, u, 0] = z[i]  # z
            rv2[2*v+1, u, 0] = z[i]
            rv2[2*v, u, 1] = ground_r[i]    # distance
            rv2[2*v+1, u, 1] = ground_r[i]  # distance
            rv2[2*v, u, 2] = radius[i]
            rv2[2*v+1, u, 2] = radius[i]
            
    return rv, rv2


def velo2bev(pcl_name, input_w, input_h, BoundaryCond):
    plydata = PlyData.read(pcl_name)
    x = np.array(plydata.elements[0]['x'])  # front
    y = np.array(plydata.elements[0]['y'])  # left
    z = np.array(plydata.elements[0]['z'])  # up
    radius = np.array(plydata.elements[0]['radius']) / 255
    pcl = np.vstack([x, y, z, radius]).T  # pcl: [front, left, up]
    pcl = remove_points(pcl, BoundaryCond)  # 去掉超出边界的点
    rgb_data = rgb_feature(pcl, input_w, input_h, BoundaryCond)
    return rgb_data


def show_pc(pc):
    fig = mlab.figure(bgcolor=(0, 0, 0), size=(640, 360))
    mlab.points3d(pc[:, 0], pc[:, 1], pc[:, 2],
                  pc[:, 3],
                  mode='point',
                  colormap='spectral',
                  figure=fig)
    mlab.show()


def show_rv(rv):
    plt.imshow(rv)
    plt.show()
