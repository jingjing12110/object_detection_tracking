#ifndef _INCLUDE_TRACKING_H_
#define _INCLUDE_TRACKING_H_

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <stack>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>
#include <Eigen/Dense>

#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/min_cut_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>

#include "adaptive_clustering.h"
#include "min_box.h"

#include "tracking_msgs/DetectedObject.h"
#include "tracking_msgs/DetectedObjectArray.h"

#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>

using namespace std;
using namespace chrono;

#define SIGMA2_V 1.0

void generate_tracking_data(
    std::vector<pcl::PointCloud<pcl::PointXYZI>>& clusters,
    tracking_msgs::DetectedObjectArray& input, double timestamp_,
    uint32_t seq_);

void process_tracking_data(tracking_msgs::DetectedObjectArray& input,
                           tracking_msgs::DetectedObjectArray& out);

// class Obstaclestate {
//  public:
//   double x;
//   double y;
//   double vx;
//   double vy;
//   double ax;
//   double ay;
//   std::vector<double> covariance;
//   Obstaclestate() {}
//   ~Obstaclestate() {}
//   void estimate(vector<feature>& obstacle_feature_stamp);
//   void predict(vector<feature> obstacle_feature_stamp, double
//   current_timestamp,
//                float& predict_x, float& predict_y);
// };

#endif
