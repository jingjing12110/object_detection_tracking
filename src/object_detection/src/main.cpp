#include <geometry_msgs/PoseStamped.h>
#include <lidar_msgs/ObjectData.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <pcl_ros/transforms.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <visualization_msgs/MarkerArray.h>
#include <algorithm>
#include <cstdio>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/SVD>

#include <pcl/console/parse.h>
#include "box_type.h"
#include "min_box.h"
//
#include "adaptive_clustering.h"
#include "erase_outlier_points.h"
#include "ground_filter.h"
#include "point_cloud_segmenter.h"
#include "ray_ground_filter.h"
// #include "bb_supervoxel_segmentor.hpp"
#include "imm_ukf_pda.h"
#include "tracking.h"

using namespace std;
using namespace message_filters;
using namespace chrono;
using namespace myshape;

#define MAX_MEASUREMENT_RANGE 70.0  // 50

ros::Publisher filter_map_pub, ground_pub, no_ground_pub, cluster_pub, bbox_pub,
    bbox_pub1, vel_pub;

visualization_msgs::MarkerArray multi_bbox_vel;

// static std::unique_ptr<tf::TransformListener> g_tf_listener_left;
// static std::unique_ptr<tf::TransformListener> g_tf_listener_right;
// static std::unique_ptr<tf::TransformListener> g_tf_listener_middle;
static std::unique_ptr<tf::TransformListener> g_tf_listener_world;
static std::unique_ptr<tf::TransformListener> g_tf_listener_local;

cv::Mat g_map;
Polygon g_bounding_area;

// ImmUkfPda ukf_pda;
boost::shared_ptr<ImmUkfPda> ukf_pda;

int N = 0;

void loadMap(std::string pkg_loc) {
  std::cout << "Loading map data " << pkg_loc + "/data/jingjiulu1.png"
            << std::endl;
  g_map = cv::imread(pkg_loc + "/data/jingjiulu1.png", cv::IMREAD_GRAYSCALE);
  std::cout << " width: " << g_map.cols << "\theight: " << g_map.rows
            << std::endl;
}

Point toMapPosition(Point pt) {
  Point g_base_position(313400, 3792725);
  float g_utm2map_rate(0.05);
  int x_map = (pt.x - g_base_position.x) / g_utm2map_rate;
  int y_map = (g_base_position.y - pt.y) / g_utm2map_rate;
  auto position = Point(x_map, y_map);
  return position;
}

bool insideMapMask(Point pt) {
  if (pt.x >= 0 && pt.x < g_map.cols && pt.y >= 0 && pt.y < g_map.rows &&
      g_map.at<uchar>(pt.y, pt.x) > 0) {
    return true;
  } else {
    return false;
  }
}

void points_callback(const sensor_msgs::PointCloud2::ConstPtr& ns3,
                     const nav_msgs::OdometryConstPtr& gps) {
  std::cout << "======================start======================" << std::endl;
  N = N + 1;
  auto start = system_clock::now();
  pcl::PointCloud<pcl::PointXYZI>::Ptr scan(
      new pcl::PointCloud<pcl::PointXYZI>());
  pcl::fromROSMsg(*ns3, *scan);

  ////////////////////////////check on map/////////////////////////////////////
  g_tf_listener_world->waitForTransform("world", "velo_middle",
                                        ns3->header.stamp, ros::Duration(5.0));
  g_tf_listener_local->waitForTransform("velo_middle", "world",
                                        ns3->header.stamp, ros::Duration(5.0));

  pcl::PointCloud<pcl::PointXYZI>::Ptr inside_world(
      new pcl::PointCloud<pcl::PointXYZI>());
  inside_world->header.frame_id = "world";
  pcl::PointCloud<pcl::PointXYZI>::Ptr grid_world(
      new pcl::PointCloud<pcl::PointXYZI>());

  pcl_ros::transformPointCloud("world", *scan, *grid_world,
                               *g_tf_listener_world);
#pragma omp for
  for (size_t i = 0; i < grid_world->points.size(); i++) {
    Point center = Point(grid_world->points[i].x + 290000,
                         grid_world->points[i].y + 3800000);
    Point bbox_map = toMapPosition(center);

    if (insideMapMask(bbox_map)) {
      inside_world->points.push_back(grid_world->points[i]);
    }
  }
  std::cout << "inside_world size: " << inside_world->points.size()
            << std::endl;

  pcl::PointCloud<pcl::PointXYZI>::Ptr inside_velo(
      new pcl::PointCloud<pcl::PointXYZI>());
  inside_velo->header.frame_id = "velo_middle";
  pcl_ros::transformPointCloud("velo_middle", *inside_world, *inside_velo,
                               *g_tf_listener_local);

  // sensor_msgs::PointCloud2 inside_msg;
  // pcl::toROSMsg(*inside_velo, inside_msg);
  // inside_msg.header = ns3->header;
  // filter_map_pub.publish(inside_msg);

  ////////////////////////////////////////////////////////////////////

  pcl::PointCloud<pcl::PointXYZI>::Ptr rang_filtered(
      new pcl::PointCloud<pcl::PointXYZI>());
  double measurement_range = MAX_MEASUREMENT_RANGE;
  *rang_filtered = removePointsByRange(*inside_velo, 0, measurement_range);
  cout << "size_of_removePointsByRange:" << rang_filtered->points.size()
       << endl;

  sensor_msgs::PointCloud2 inside_msg;
  pcl::toROSMsg(*rang_filtered, inside_msg);
  inside_msg.header = ns3->header;
  filter_map_pub.publish(inside_msg);

  //////////////////////////filter ground points///////////////////////////////
  pcl::PointCloud<pcl::PointXYZI>::Ptr ground(
      new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr no_ground(
      new pcl::PointCloud<pcl::PointXYZI>());
  // GroundFilter filter;
  // filter.ground_filter(rang_filtered, ground_points, filtered_points);
  PclTestCore filter;
  filter.point_cb(rang_filtered, ground, no_ground);
  cout << "size of filtered ground points size: " << no_ground->points.size()
       << endl;

  sensor_msgs::PointCloud2 ground_msg;
  pcl::toROSMsg(*ground, ground_msg);
  ground_msg.header = ns3->header;
  ground_pub.publish(ground_msg);

  sensor_msgs::PointCloud2 no_ground_msg;
  pcl::toROSMsg(*no_ground, no_ground_msg);
  no_ground_msg.header = ns3->header;
  no_ground_pub.publish(no_ground_msg);

  /////////////////////////////////segment/////////////////////////////////////
  // int C = 0;
  // std::vector<pcl::PointCloud<pcl::PointXYZI>> clusters;
  // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out(
  //     new pcl::PointCloud<pcl::PointXYZI>());
  // adaptive_cluster(no_ground, clusters, C, cloud_out);

  ///////////////////////////////////////////////////
  // int n_iters = 3;
  // int n_lpr = 20;
  // int n_segs = 3;
  // double seed_thresh = 0.4;  // meters
  // double dist_thresh = 0.2;  // meters

  // double th_run = 0.5;
  // double th_merge = 1.0;
  // int x_max = 30;
  // int y_max = 15;
  // int n_scanlines = 50;

  // std::vector<Vec3> input_cloud;
  // std::vector<Vec3> predicted_clusters;

  // PointCloudSegmenter segmenter(x_max, y_max, n_iters, n_lpr, n_segs,
  //                               seed_thresh, dist_thresh, n_scanlines,
  //                               th_run, th_merge);
  // parseInput(rang_filtered, input_cloud, segmenter);
  // segmenter.GroundPlaneFitting(input_cloud);
  // predicted_clusters = segmenter.ScanLineRun(segmenter.p_all);

  // pcl::PointCloud<pcl::PointXYZI>::Ptr out(
  //     new pcl::PointCloud<pcl::PointXYZI>());
  // parseOut(predicted_clusters, out);
  ///////////////////////////////////////////////////

  int C = 1;
  int len;
  std::vector<pcl::PointCloud<pcl::PointXYZI>> clusters;
  dbscan(no_ground, clusters, C, len);

  // show
  pcl::PointCloud<pcl::PointXYZI>::Ptr cluster_cloud(
      new pcl::PointCloud<pcl::PointXYZI>());
  // different clusters with different intensity
  float step_i = 255.0f / clusters.size();
  for (size_t cluster_idx = 0u; cluster_idx < clusters.size(); ++cluster_idx)
  {
    if (clusters[cluster_idx].points.size() <= 0) {
      continue;
    }
    for (size_t idx = 0u; idx < clusters[cluster_idx].points.size(); ++idx) {
      pcl::PointXYZI point;
      point.x = clusters[cluster_idx].points[idx].x;
      point.y = clusters[cluster_idx].points[idx].y;
      point.z = clusters[cluster_idx].points[idx].z;
      point.intensity = cluster_idx * step_i;
      cluster_cloud->points.push_back(point);
    }
  }
  cout << "points of all clusters: " << cluster_cloud->size()
       << ", clusters size: " << clusters.size() << endl;

  sensor_msgs::PointCloud2 cluster_msg;
  pcl::toROSMsg(*cluster_cloud, cluster_msg);
  cluster_msg.header = ns3->header;
  cluster_pub.publish(cluster_msg);

  //////////////////////////////////tracking///////////////////////////////////
  // std::cout << "header : " << ns3->header << std::endl;
  // velo_middle to world coor

  tracking_msgs::DetectedObjectArray input;
  double timestamp_ = ns3->header.stamp.toSec();
  uint32_t seq_ = ns3->header.seq;

  generate_tracking_data(clusters, input, timestamp_, seq_);
  // std::cout << "input.header: " << input.header << std::endl;
  // std::cout << "input.object: " << input.objects[0] << std::endl;
  tracking_msgs::DetectedObjectArray tracked_output;
  // ukf_pda.tracker(input, tracked_output);
  ukf_pda->run_test(input, tracked_output);

  // std::cout << "tracking pose: " << tracked_output.objects[0].pose <<
  // std::endl; std::cout << "tracking velocity: " <<
  // tracked_output.objects[0].velocity
  //           << std::endl;
  // std::cout << "tracking pointcloud size: "
  //           << tracked_output.objects[0].pointcloud.data.size() << std::endl;
  // std::cout << "after tracking, object size: " <<
  // tracked_output.objects.size()
  //           << std::endl;
  /*
  pcl::PointCloud<pcl::PointXYZI>::Ptr cluster_clouds(
      new pcl::PointCloud<pcl::PointXYZI>());
  float step_i = 255.0f / tracked_output.objects.size();
  for (size_t idx = 0u; idx < tracked_output.objects.size(); ++idx) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cluster_cloud(
        new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(tracked_output.objects[idx].pointcloud, *cluster_cloud);

    for (size_t idx_ = 0u; idx_ < cluster_cloud->size(); ++idx_) {
      pcl::PointXYZI point;
      point.x = cluster_cloud->points[idx_].x;
      point.y = cluster_cloud->points[idx_].y;
      point.z = cluster_cloud->points[idx_].z;
      point.intensity = idx * step_i;
      cluster_clouds->points.push_back(point);
    }
  }

  sensor_msgs::PointCloud2 cluster_msg;
  pcl::toROSMsg(*cluster_clouds, cluster_msg);
  cluster_msg.header = ns3->header;
  cluster_pub.publish(cluster_msg);
  */
  //////////////////////////////////show///////////////////////////////////
  if (tracked_output.objects.size() > 0) {
    BoundingBox bounding_box(tracked_output, ns3->header.stamp);
    visualization_msgs::MarkerArray prism_box = bounding_box.prisms_.toRviz();
    bbox_pub.publish(prism_box);
    // std::cout<<"bounding_box.prisms_.size:
    // "<<bounding_box.prisms_.prisms_.size()<<std::endl;
    prism_box.markers.clear();
    visualization_msgs::MarkerArray cuboid_box = bounding_box.cuboids_.toRviz();
    bbox_pub1.publish(cuboid_box);
    // std::cout<<"bounding_box.cuboids_.size:
    // "<<bounding_box.cuboids_.prisms_.size()<<std::endl;
    cuboid_box.markers.clear();

    // velocity
    int box_num = tracked_output.objects.size();
    visualization_msgs::Marker markerdelete;
    markerdelete.header.frame_id = "velo_middle";
    markerdelete.header.stamp = ns3->header.stamp;
    markerdelete.action = visualization_msgs::Marker::DELETEALL;
    multi_bbox_vel.markers.push_back(markerdelete);

#pragma omp for
    for (int i = 0; i < box_num; i++) {
      visualization_msgs::Marker marker;
      marker.header.frame_id = "velo_middle";
      marker.header.stamp = ns3->header.stamp;
      marker.ns = "basic_shapes";
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.orientation.w = 1.0;
      marker.id = i;
      marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

      marker.scale.z = 1;
      marker.color.b = 1.0f;
      marker.color.g = 1.0f;
      marker.color.r = 1.0f;
      marker.color.a = 1;

      geometry_msgs::Pose pose;
      pose.position.x = tracked_output.objects[i].pose.position.x;
      pose.position.y = tracked_output.objects[i].pose.position.y;
      pose.position.z = 0;

      double velocity =
          sqrt(pow(tracked_output.objects[i].velocity.linear.x, 2) +
               pow(tracked_output.objects[i].velocity.linear.y, 2));

      int id = tracked_output.objects[i].id;

      std::ostringstream str;
      str << velocity;
      marker.text = str.str();
      marker.pose = pose;

      if (velocity > 0) {
        multi_bbox_vel.markers.push_back(marker);
        // std::cout << "vel:" << velocity << std::endl;
      }
    }

    for (int i = 0; i < box_num; i++) {
      visualization_msgs::Marker marker;
      marker.header.frame_id = "velo_middle";
      marker.header.stamp = ns3->header.stamp;
      marker.ns = "lines";
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.orientation.w = 1.0;
      marker.id = 2 * i;
      marker.type = visualization_msgs::Marker::ARROW;

      marker.scale.x = 0.5;
      marker.scale.y = 1;
      marker.scale.z = 2;
      marker.color.b = 0;
      marker.color.g = 0;
      marker.color.r = 1.0f;
      marker.color.a = 1;

      geometry_msgs::Point p1;
      p1.x = tracked_output.objects[i].pose.position.x;
      p1.y = tracked_output.objects[i].pose.position.y;
      p1.z = 0;
      geometry_msgs::Point p2;
      double length = sqrt(pow(tracked_output.objects[i].velocity.linear.x, 2) +
                           pow(tracked_output.objects[i].velocity.linear.y, 2));

      p2.x = p1.x + tracked_output.objects[i].velocity.linear.x * 5.0 / length;
      p2.y = p1.y + tracked_output.objects[i].velocity.linear.y * 5.0 / length;
      p2.z = 0;
      // std::cout<<"p1"<<p1.x<<"||"<<p1.y<<std::endl;
      marker.points.push_back(p1);
      marker.points.push_back(p2);

      multi_bbox_vel.markers.push_back(marker);
    }

    vel_pub.publish(multi_bbox_vel);
    multi_bbox_vel.markers.clear();
  }

  //////////////////////////////////show///////////////////////////////////

  auto end = system_clock::now();
  auto duration = duration_cast<microseconds>(end - start);
  cout << "spend"
       << double(duration.count()) * microseconds::period::num /
              microseconds::period::den
       << " seconds." << endl;
}

// main
int main(int argc, char** argv) {
  ros::init(argc, argv, "object_detection");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  ros::Subscriber cloud_sub;

  ukf_pda.reset(new ImmUkfPda());

  std::string pkg_loc;
  pkg_loc = ros::package::getPath("object_detection");
  loadMap(pkg_loc);
  std::vector<Point> pts;
  pts.push_back(Point(0, 0));
  pts.push_back(Point(0, 27999));
  pts.push_back(Point(3999, 27999));
  pts.push_back(Point(3999, 0));
  // pts.push_back(Point(28200, 8527));
  // pts.push_back(Point(20700, 1299));
  g_bounding_area.inputPolygon(pts);

  // g_tf_listener_left.reset(new tf::TransformListener());
  // g_tf_listener_right.reset(new tf::TransformListener());
  // g_tf_listener_middle.reset(new tf::TransformListener());
  g_tf_listener_world.reset(new tf::TransformListener());
  g_tf_listener_local.reset(new tf::TransformListener());

  // process
  filter_map_pub = nh.advertise<sensor_msgs::PointCloud2>(
      "/ground_filter/in_map_points", 10);
  ground_pub =
      nh.advertise<sensor_msgs::PointCloud2>("/ground_filter/ground/", 10);
  no_ground_pub =
      nh.advertise<sensor_msgs::PointCloud2>("/ground_filter/no_ground/", 10);
  cluster_pub =
      nh.advertise<sensor_msgs::PointCloud2>("/detection/DBSCAN_cluster", 10);
  // show
  bbox_pub =
      nh.advertise<visualization_msgs::MarkerArray>("prisms_disp", 1000, true);
  bbox_pub1 =
      nh.advertise<visualization_msgs::MarkerArray>("cuboids_disp", 1000, true);
  vel_pub =
      nh.advertise<visualization_msgs::MarkerArray>("velocityrviz", 1000, true);

  message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub3(
      nh, "/lidar/vlp32_middle/PointCloud2", 20);
  message_filters::Subscriber<nav_msgs::Odometry> gps_sub(
      nh, "/pioneer_sensors/EKF_Localization_RS232/filteredodometry", 20);

  typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2,
                                         nav_msgs::Odometry>
      MySyncPolicy;
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(20), cloud_sub3, gps_sub);
  sync.registerCallback(boost::bind(&points_callback, _1, _2));

  ros::spin();

  return 0;
}
