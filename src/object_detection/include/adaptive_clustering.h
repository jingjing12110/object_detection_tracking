#ifndef ADAPTIVE_CLUSTERING_H_
#define ADAPTIVE_CLUSTERING_H_
// ROS
#include <geometry_msgs/PoseArray.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
// PCL
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/SVD>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/make_shared.hpp>

using namespace std;
// using namespace chrono;

#define MINEPS 0.5
#define MINPTS 5

std::string sensor_model_;
std::string frame_id_;
bool print_fps_;
float z_axis_min_;
float z_axis_max_;
size_t cluster_size_min_;
size_t cluster_size_max_;

const int region_max_ = 5;
// Change this value to match how far you want to detect.
int regions_[100];
uint32_t cluster_array_seq_ = 0;
uint32_t pose_array_seq_ = 0;


///////////////////////////// tool function///////////////////////////////////
//判断vector的某一元素是否存在
bool is_in_vector(std::vector<int> v, int element) {
  std::vector<int>::iterator it;
  it = find(v.begin(), v.end(), element);
  if (it != v.end()) {
    return true;
  } else {
    return false;
  }
}

class point {
 public:
  float x;
  float y;
  float z;
  float intensity;
  int cluster = 0;
  int pointType = 1;  // 0 noise  1 core
  int pts = 0;        // points in MinPts
  vector<int> corepts;
  int visited = 0;
  point() {}
  point(float point_x, float point_y, float point_z, float point_intensity) {
    x = point_x;
    y = point_y;
    z = point_z;
    intensity = point_intensity;
    // cluster = c;
  }
};

float squareDistance(const point &a, const point &b) {
  return (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y);
}

void ExpandCluster(vector<point> &dataset, point &data, int c) {
  for (int i = 0; i < data.pts; i++) {
    if (dataset[data.corepts[i]].visited == 0) {
      dataset[data.corepts[i]].visited = 1;
      dataset[data.corepts[i]].cluster = c;
      dataset[data.corepts[i]].pointType = 1;
      ExpandCluster(dataset, dataset[data.corepts[i]], c);
    } else
      continue;
  }
}

///////////////////////////// cluster ///////////////////////////////////
// adaptive
void init_parameters(std::string sensor_model) {
  sensor_model_ = sensor_model;
  cluster_size_min_ = 8;
  cluster_size_max_ = 100000;
  // Divide the point cloud into nested circular regions centred at the sensor.
  // For more details, see our IROS-17 paper "Online learning for human
  // classification in 3D LiDAR-based tracking"
  if (sensor_model_ == "VLP-16") {
    regions_[0] = 2;
    regions_[1] = 3;
    regions_[2] = 3;
    regions_[3] = 3;
    regions_[4] = 3;
    regions_[5] = 3;
    regions_[6] = 3;
    regions_[7] = 2;
    regions_[8] = 3;
    regions_[9] = 3;
    regions_[10] = 3;
    regions_[11] = 3;
    regions_[12] = 3;
    regions_[13] = 3;
  } else if (sensor_model_ == "HDL-32E") {
    regions_[0] = 14;
    regions_[1] = 14;
    regions_[2] = 14;
    regions_[3] = 15;
    regions_[4] = 14;

    // regions_[0] = 4;
    // regions_[1] = 5;
    // regions_[2] = 4;
    // regions_[3] = 5;
    // regions_[4] = 4;
    // regions_[5] = 5;
    // regions_[6] = 5;
    // regions_[7] = 4;
    // regions_[8] = 5;
    // regions_[9] = 4;
    // regions_[10] = 5;
    // regions_[11] = 5;
    // regions_[12] = 4;
    // regions_[13] = 5;
  } else if (sensor_model_ == "HDL-64E") {
    regions_[0] = 14;
    regions_[1] = 14;
    regions_[2] = 14;
    regions_[3] = 15;
    regions_[4] = 14;
  }
}

void adaptive_cluster(
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in,
    std::vector<pcl::PointCloud<pcl::PointXYZI>> &clusters_out_temp, int C,
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out) {
  std::string sensor_model = "HDL-32E";
  init_parameters(sensor_model);

  pcl::IndicesPtr pc_indices(new std::vector<int>);
  pcl::PassThrough<pcl::PointXYZI> pt;
  pt.setInputCloud(cloud_in);
  pt.setFilterFieldName("z");
  pt.setFilterLimits(-5.0, 2.0);
  pt.filter(*pc_indices);

  /*** Divide the point cloud into nested circular regions ***/
  boost::array<std::vector<int>, region_max_> indices_array;
#pragma omp for
  for (size_t i = 0; i < pc_indices->size(); i++) {
    float range = 0.0;
    for (int j = 0; j < region_max_; j++) {
      float d2 = cloud_in->points[(*pc_indices)[i]].x *
                     cloud_in->points[(*pc_indices)[i]].x +
                 cloud_in->points[(*pc_indices)[i]].y *
                     cloud_in->points[(*pc_indices)[i]].y +
                 cloud_in->points[(*pc_indices)[i]].z *
                     cloud_in->points[(*pc_indices)[i]].z;

      if (d2 > range * range &&
          d2 <= (range + regions_[j]) * (range + regions_[j])) {
        indices_array[j].push_back((*pc_indices)[i]);
        break;
      }
      range += regions_[j];
    }
  }

  /*** Euclidean clustering ***/
  float tolerance = 0.0;
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr,
              Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZI>::Ptr>>
      clusters;

#pragma omp for
  for (size_t i = 0; i < region_max_; i++) {
    tolerance += 0.1;
    if (indices_array[i].size() > cluster_size_min_) {
      boost::shared_ptr<std::vector<int>> indices_array_ptr(
          new std::vector<int>(indices_array[i]));
      pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(
          new pcl::search::KdTree<pcl::PointXYZI>);
      tree->setInputCloud(cloud_in, indices_array_ptr);

      std::vector<pcl::PointIndices> cluster_indices;
      pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
      ec.setClusterTolerance(tolerance);
      ec.setMinClusterSize(cluster_size_min_);
      ec.setMaxClusterSize(cluster_size_max_);
      ec.setSearchMethod(tree);
      ec.setInputCloud(cloud_in);
      ec.setIndices(indices_array_ptr);
      ec.extract(cluster_indices);

      for (std::vector<pcl::PointIndices>::const_iterator it =
               cluster_indices.begin();
           it != cluster_indices.end(); it++) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cluster(
            new pcl::PointCloud<pcl::PointXYZI>);
        for (std::vector<int>::const_iterator pit = it->indices.begin();
             pit != it->indices.end(); ++pit) {
          cluster->points.push_back(cloud_in->points[*pit]);
        }
        cluster->width = cluster->size();
        cluster->height = 1;
        cluster->is_dense = true;
        clusters.push_back(cluster);
      }
    }
  }

  // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out(
  // new pcl::PointCloud<pcl::PointXYZI>);
  pcl::copyPointCloud(*cloud_in, *pc_indices, *cloud_out);

  std::vector<pcl::PointCloud<pcl::PointXYZI>> clusters_out;
  // int C = 0;
  C = 0;
  std::vector<Eigen::Vector4f> centroids;
  std::vector<Eigen::Vector4f> mins;
  std::vector<Eigen::Vector4f> maxs;

  for (size_t i = 0; i < clusters.size(); i++) {
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*clusters[i], centroid);
    centroids.push_back(centroid);

    Eigen::Vector4f min, max;  // 所有点中的最值
    pcl::getMinMax3D(*clusters[i], min, max);
    mins.push_back(min);
    maxs.push_back(max);

    if (clusters[i]->points.size() > cluster_size_min_) {
      clusters_out.push_back(*clusters[i]);
      C = C + 1;
    }
  }
  std::cout << "cluster size: " << C << std::endl;

  // postprocess
  // std::vector<pcl::PointCloud<pcl::PointXYZI>> clusters_out_temp;
  std::vector<int> use_j;

  for (size_t i = 0; i < clusters_out.size(); i++) {
    if (is_in_vector(use_j, i) == false) {
      // std::cout<< "i= "<<i<<std::endl;
      pcl::PointCloud<pcl::PointXYZI>::Ptr temp(
          new pcl::PointCloud<pcl::PointXYZI>);
      pcl::copyPointCloud<pcl::PointXYZI, pcl::PointXYZI>(clusters_out[i],
                                                          *temp);

      for (size_t j = i + 1; j < clusters_out.size(); j++) {
        float di = centroids[i][0] * centroids[i][0] +
                   centroids[i][1] * centroids[i][1];// +
                   //centroids[i][2] * centroids[i][2];

        float dj = centroids[j][0] * centroids[j][0] +
                   centroids[j][1] * centroids[j][1]; // +
                   //centroids[j][2] * centroids[j][2];

        float dmin_x = mins[i][0] - mins[j][0];
        float dmax_x = maxs[i][0] - maxs[j][0];

        // std::cout << "delta_d= " << (di - dj) << std::endl;

        if ((di - dj) < 0.8 * 0.8 && (dmin_x < 0.8 && dmax_x < 0.8)) {
        // if ((di - dj) < 3.0 * 3.0 && (di - dj) > -3.0 * 3.0) {
          temp->insert(temp->end(), clusters_out[j].begin(),
                       clusters_out[j].end());
          use_j.push_back(j);
        }
      }  // end for j
      clusters_out_temp.push_back(*temp);
    }
  }

  std::cout << "after postprocess cluster size: " << clusters_out_temp.size() << std::endl;
}

// DBSCAN
void dbscan(pcl::PointCloud<pcl::PointXYZI>::Ptr removestatic_scan_ptr,
            std::vector<pcl::PointCloud<pcl::PointXYZI>> &clusters, int &C,
            int &len) {
  // auto start = system_clock::now();
  vector<point> dataset;
  float Eps;
  int MinPts;
  Eps = MINEPS;
  MinPts = MINPTS;
  for (size_t i = 0; i < removestatic_scan_ptr->points.size(); ++i) {
    point p(removestatic_scan_ptr->points[i].x,
            removestatic_scan_ptr->points[i].y,
            removestatic_scan_ptr->points[i].z,
            removestatic_scan_ptr->points[i].intensity);
    dataset.push_back(p);
  }
  // int C=1;
  int coreflag;
  len = dataset.size();
  vector<int> index;
  for (int i = 0; i < len; i++) {
    for (int j = i + 1; j < len; j++) {
      if (squareDistance(dataset[i], dataset[j]) < Eps * Eps) {
        dataset[i].pts++;
        dataset[j].pts++;
        dataset[i].corepts.push_back(j);
        dataset[j].corepts.push_back(i);
      }
    }
  }

  for (int i = 0; i < len; i++) {
    if (dataset[i].visited == 1) continue;
    if (dataset[i].pts >= MinPts) {
      coreflag = 1;
      dataset[i].cluster = C;
      dataset[i].visited = 1;
      dataset[i].pointType = 1;
      point &p = dataset[i];
      ExpandCluster(dataset, p, C);
    } else {
      coreflag = 0;
      // dataset[i].visited=1;
      dataset[i].cluster = 0;
      dataset[i].pointType = 0;
    }
    C = C + coreflag;
  }

  std::vector<pcl::PointCloud<pcl::PointXYZI>> clusters_all;
  clusters_all.resize(C);
  for (int i = 0; i < len; i++) {
    if (dataset[i].cluster == 0) continue;
    pcl::PointXYZI a;
    a.x = dataset[i].x;
    a.y = dataset[i].y;
    a.z = dataset[i].z;
    a.intensity = dataset[i].intensity;
    clusters_all[dataset[i].cluster - 1].push_back(a);
  }

  int C_ = 0;

  // std::cout<<"111111:"<<C<<"||"<<clusters.size()<<std::endl;
  for (size_t i = 0; i < clusters_all.size(); i++) {
    if (clusters_all[i].points.size() > 5) {
      clusters.push_back(clusters_all[i]);
      C_ = C_ + 1;
    }
  }

  C = C_;
  // auto end = system_clock::now();
  // auto duration = duration_cast<microseconds>(end - start);
  // cout <<  "dbscan spend: " << double(duration.count()) *
  // microseconds::period::num / microseconds::period::den << " seconds." <<
  // endl;
}


#endif
