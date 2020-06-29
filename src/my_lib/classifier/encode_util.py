import numpy as np


def remove_points(PointCloud, BoundaryCond):
    # Boundary condition
    minX = BoundaryCond['minX']
    maxX = BoundaryCond['maxX']
    minY = BoundaryCond['minY']
    maxY = BoundaryCond['maxY']
    minZ = BoundaryCond['minZ']
    maxZ = BoundaryCond['maxZ']
    
    # Remove the point out of range x,y,z
    mask = np.where((PointCloud[:, 0] >= minX) & (PointCloud[:, 0] <= maxX) & (
                PointCloud[:, 1] >= minY) &
                    (PointCloud[:, 1] <= maxY) & (PointCloud[:, 2] >= minZ) & (
                                PointCloud[:, 2] <= maxZ))
    PointCloud = PointCloud[mask]
    return PointCloud


def rgb_feature(_pcl, Width, Height, bc):
    """
    Args:
        PointCloud_: [n, 4] 4:[x,y,z,I] x是front
    """
    discretization = bc['maxX'] / Height
    pcl = np.copy(_pcl)
    pcl[:, 0] = np.int_(np.floor(pcl[:, 0] / discretization))
    # 注意：这里只把pcl部分值去掉了小数，但pcl.dtype=float32
    pcl[:, 1] = np.int_(np.floor(pcl[:, 1] / discretization + Width // 2))
    # 排序
    indices = np.lexsort((-pcl[:, 2], pcl[:, 0], pcl[:, 1]))
    # 从左到右有限级，优先高度从大到小排序，放回index
    pcl = pcl[indices]
    
    # pcl[:, [0, 1]] = pcl[:, [1, 0]]  # 这里一次把顺序换过来
    # ######下面的xy已经交换了，代码不变
    height_map = np.zeros((Height, Width))  # ?????
    intensity_map = np.zeros((Height, Width))
    density_map = np.zeros((Height, Width))
    # 把点放入map中,只同一格子只保留最高点（之前已排序，对坐标unique一次即可）
    _, indices, counts = np.unique(pcl[:, 0:2], axis=0, return_index=True,
                                   return_counts=True)
    pcl_top = pcl[indices]
    pcl2sub = np.minimum([Height - 1, Width - 1], np.int_(pcl_top[:, 0:2]))
    normal_top = np.minimum(1.0, np.log(counts + 1) / np.log(64))
    # 最高位置、最高位置的反射率，密度的归一化
    height_map[pcl2sub[:, 0], pcl2sub[:, 1]] = pcl_top[:, 2]
    intensity_map[pcl2sub[:, 0], pcl2sub[:, 1]] = pcl_top[:, 3]
    density_map[pcl2sub[:, 0], pcl2sub[:, 1]] = normal_top
    
    image = np.zeros((Height, Width, 3))
    image[:, :, 0] = density_map  # 0~1
    image[:, :, 1] = height_map  # minH~maxH, 可正可负
    image[:, :, 2] = intensity_map  # 0~1
    # for i in range(Height):
    #     image[i, ...] = image[Height-1-i, ...]
    
    ####################
    # 要不要减平均值？
    ####################
    return image
