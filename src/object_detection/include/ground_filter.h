#ifndef _INCLUDE_GROUNDFILTER_H_
#define _INCLUDE_GROUNDFILTER_H_

#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <vector>
#include <chrono>
#include <cstdlib>
#include <cmath>  
#include <stack> 
#include <limits>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
// #include "points_downsampler.h"
#include <pcl/segmentation/min_cut_segmentation.h>


#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/progressive_morphological_filter.h>


// #define DISTANCETHRESHOLD 0.3

using namespace std;

class GroundFilter//将一个栅格定义为一个类对象
{
public:
    // bool  road;      //是不是路
    float h_mean;       //平均高度
    float h_difference; //高度差
    float h_min;        //高度最低值
    float h_max;
    pcl::PointCloud<pcl::PointXYZI>::Ptr grid_cloud {new pcl::PointCloud<pcl::PointXYZI>};//指向栅格内的点云的指针
    pcl::PointCloud<pcl::PointXYZI>::Ptr ground_pc {new pcl::PointCloud<pcl::PointXYZI>};
    pcl::PointCloud<pcl::PointXYZI>::Ptr not_ground_pc {new pcl::PointCloud<pcl::PointXYZI>};
    GroundFilter(){}
    ~GroundFilter(){}

    void ground_filter(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in_,
                    pcl::PointCloud<pcl::PointXYZI>::Ptr ground_pc, 
                    pcl::PointCloud<pcl::PointXYZI>::Ptr not_ground_pc);
    void init_conditional_removal(pcl::ConditionalRemoval<pcl::PointXYZI>& a);
    // pcl::PointCloud<pcl::PointXYZI>::Ptr ground_init_cloud {new pcl::PointCloud<pcl::PointXYZI>};
    // pcl::PointIndices::Ptr grid_inliers {new pcl::PointIndices};//栅格内点云索引的指针
    // int num;//点云点数
    //std::vector<int> indices;//点云索引（定义了索引向量指针就不用定义它啦，因为后面想将得到的索引值赋给索引的时候，直接赋给*ptr就可以了）
};



#endif
