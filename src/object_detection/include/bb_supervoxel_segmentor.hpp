/*
    @author     Jay (jaypanda16@gmail.com)
    @file       bb_supervoxel_segmentor.cpp
    @date       2018/02/03
    @version    0.1

    @brief      3D person identification on basketball court using supervoxel segmentation followed
                by k-means.
*/
#include <pcl/console/parse.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/supervoxel_clustering.h>

//VTK include needed for drawing graph lines.
#include <vtkPolyLine.h>

//OpenCV for histogram comparisions.
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include<string>
#include<fstream>
#include<sstream>
#include<vector>

// Types
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointNCloudT;
typedef pcl::PointXYZL PointLT;
typedef pcl::PointCloud<PointLT> PointLCloudT;

// Primary class implementation for 3d clustering of point clouds of
// players/referees of a basketball/NBA game.
// Example:
//      BBSupervoxelSegmentor bbss (<path-to-point-cloud-txt-file>);
//      bbss.ProcessPointCloud();
class BBSupervoxelSegmentor {
    // Point cloud data structure to store the input data points.
    PointCloudT::Ptr cloud;

    // Point cloud data structure to store the final object centers.
    PointCloudT::Ptr obj_centroids;

    // Viewer to visualize point cloud and results.
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

    // Config parameters.
    float voxel_resolution = 0.8f;
    float seed_resolution = 10.0f;
    float color_importance = 0.1f;
    float spatial_importance = 0.5f;
    float normal_importance = 0.5f;
    int rgb_histbins = 10;
    bool DEBUG = false;
    bool VISUALIZE = true;

    // Loads the point cloud data file into PointCloudT data structure.
    bool LoadPointCloud (char *point_cloud_data_file);

    // Processes object point clouds to identify 3 groups of people
    // based on their outfit color information on RGB pixel values.
    void ColorHistClustering (std::vector<PointCloudT::Ptr> final_objclouds,
                                   std::vector<int>& labelvecs, int histSize=10);

    // Uses supervoxel clustering method from PCL library to cluster and identify people point clouds.
    void SupervoxelClustering (std::vector<PointCloudT::Ptr>& final_objclouds,
                                                 std::vector<PointT>& final_centroids);

    public:
        BBSupervoxelSegmentor (char *point_cloud_data_file);

        // Sets config parameters before process_pointcloud call.
        void SetParameters (float voxel_resolution, float seed_resolution, float color_importance,
                            float spatial_importance, float normal_importance, int rgb_histbins,
                            bool DEBUG, bool VISUALIZE);

        // Runs the approach pipeline.
        bool ProcessPointCloud();
};
