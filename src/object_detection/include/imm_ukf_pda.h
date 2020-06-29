#ifndef IMM_UKF_PDA_H_
#define IMM_UKF_PDA_H_

#include <stdio.h>
#include <chrono>
#include <vector>

#include <ros/package.h>
#include <ros/ros.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf/transform_listener.h>

#include "tracking_msgs/DetectedObject.h"
#include "tracking_msgs/DetectedObjectArray.h"

#include "ukf.h"

class ImmUkfPda {
 private:
  int target_id_;
  bool init_;
  double timestamp_;

  std::vector<UKF> targets_;

  // probabilistic data association params
  double gating_thres_;
  double gate_probability_;
  double detection_probability_;

  // object association param
  int life_time_thres_;

  // static classification param
  double static_velocity_thres_;
  int static_num_history_thres_;

  // switch sukf and ImmUkfPda
  bool use_sukf_;

  // whether if benchmarking tracking result
  bool is_benchmark_;
  int frame_count_;
  std::string kitti_data_dir_;

  // for benchmark
  std::string result_file_path_;

  // prevent explode param for ukf
  double prevent_explosion_thres_;

  // for vectormap assisted tarcking
  bool use_vectormap_;
  bool has_subscribed_vectormap_;
  double lane_direction_chi_thres_;
  double nearest_lane_distance_thres_;
  std::string vectormap_frame_;
  //   vector_map::VectorMap vmap_;
  //   std::vector<vector_map_msgs::Lane> lanes_;

  double merge_distance_threshold_;
  const double CENTROID_DISTANCE = 0.2;
  // distance to consider centroids the same

  // std::string input_topic_;
  // std::string output_topic_;

  std::string tracking_frame_;

  tf::TransformListener tf_listener_;
  tf::StampedTransform local2global_;
  tf::StampedTransform tracking_frame2lane_frame_;
  tf::StampedTransform lane_frame2tracking_frame_;

  //   ros::NodeHandle node_handle_;
  //   ros::NodeHandle private_nh_;
  ros::Subscriber sub_detected_array_;
  ros::Publisher pub_object_array_;

  std_msgs::Header input_header_;

  void callback(const tracking_msgs::DetectedObjectArray& input);

  void transformPoseToGlobal(
      const tracking_msgs::DetectedObjectArray& input,
      tracking_msgs::DetectedObjectArray& transformed_input);
  void transformPoseToLocal(
      tracking_msgs::DetectedObjectArray& detected_objects_output);

  geometry_msgs::Pose getTransformedPose(const geometry_msgs::Pose& in_pose,
                                         const tf::StampedTransform& tf_stamp);

  bool updateNecessaryTransform();

  void measurementValidation(
      const tracking_msgs::DetectedObjectArray& input, UKF& target,
      const bool second_init, const Eigen::VectorXd& max_det_z,
      const Eigen::MatrixXd& max_det_s,
      std::vector<tracking_msgs::DetectedObject>& object_vec,
      std::vector<bool>& matching_vec);

  tracking_msgs::DetectedObject getNearestObject(
      UKF& target,
      const std::vector<tracking_msgs::DetectedObject>& object_vec);
  void updateBehaviorState(const UKF& target, const bool use_sukf,
                           tracking_msgs::DetectedObject& object);

  void initTracker(const tracking_msgs::DetectedObjectArray& input,
                   double timestamp);

  void secondInit(UKF& target,
                  const std::vector<tracking_msgs::DetectedObject>& object_vec,
                  double dt);

  void updateTrackingNum(
      const std::vector<tracking_msgs::DetectedObject>& object_vec,
      UKF& target);

  bool probabilisticDataAssociation(
      const tracking_msgs::DetectedObjectArray& input, const double dt,
      std::vector<bool>& matching_vec,
      std::vector<tracking_msgs::DetectedObject>& object_vec, UKF& target);

  void makeNewTargets(const double timestamp,
                      const tracking_msgs::DetectedObjectArray& input,
                      const std::vector<bool>& matching_vec);

  void staticClassification();

  void makeOutput(const tracking_msgs::DetectedObjectArray& input,
                  const std::vector<bool>& matching_vec,
                  tracking_msgs::DetectedObjectArray& detected_objects_output);

  void removeUnnecessaryTarget();

  void dumpResultText(tracking_msgs::DetectedObjectArray& detected_objects);

  // void tracker(const tracking_msgs::DetectedObjectArray& transformed_input,
  //              tracking_msgs::DetectedObjectArray& detected_objects_output);

  bool updateDirection(const double smallest_nis,
                       const tracking_msgs::DetectedObject& in_object,
                       tracking_msgs::DetectedObject& out_object, UKF& target);

  bool storeObjectWithNearestLaneDirection(
      const tracking_msgs::DetectedObject& in_object,
      tracking_msgs::DetectedObject& out_object);

  void checkVectormapSubscription();

  tracking_msgs::DetectedObjectArray removeRedundantObjects(
      const tracking_msgs::DetectedObjectArray& in_detected_objects,
      const std::vector<size_t> in_tracker_indices);

  tracking_msgs::DetectedObjectArray forwardNonMatchedObject(
      const tracking_msgs::DetectedObjectArray& tmp_objects,
      const tracking_msgs::DetectedObjectArray& input,
      const std::vector<bool>& matching_vec);

  bool arePointsClose(const geometry_msgs::Point& in_point_a,
                      const geometry_msgs::Point& in_point_b, float in_radius);

  bool arePointsEqual(const geometry_msgs::Point& in_point_a,
                      const geometry_msgs::Point& in_point_b);

  bool isPointInPool(const std::vector<geometry_msgs::Point>& in_pool,
                     const geometry_msgs::Point& in_point);

  void updateTargetWithAssociatedObject(
      const std::vector<tracking_msgs::DetectedObject>& object_vec,
      UKF& target);

 public:
  ImmUkfPda();
  void tracker(const tracking_msgs::DetectedObjectArray& transformed_input,
               tracking_msgs::DetectedObjectArray& detected_objects_output);
  void run();
  void run_test(const tracking_msgs::DetectedObjectArray& input,
                tracking_msgs::DetectedObjectArray& detected_objects_output);
};

#endif /* IMM_UKF_PDA_H_ */
