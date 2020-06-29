#include "tracking.h"

using namespace std;

void generate_tracking_data(
    std::vector<pcl::PointCloud<pcl::PointXYZI>>& clusters,
    tracking_msgs::DetectedObjectArray& input, double timestamp_, uint32_t seq_ ) {
    input.header.stamp.fromSec(timestamp_);
    input.header.seq = seq_;
    input.header.frame_id = "velo_middle";

  for (size_t i = 0; i < clusters.size(); i++) {
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(clusters[i], centroid);

    tracking_msgs::DetectedObject object;

    object.label = "Obstacle";
    object.pose.position.x = centroid[0];
    object.pose.position.y = centroid[1];
    object.pose.position.z = centroid[2];
    object.pose.orientation.w = 1;

    sensor_msgs::PointCloud2 cluster_msg;
    pcl::toROSMsg(clusters[i], cluster_msg);
    cluster_msg.header = input.header;
    object.pointcloud = cluster_msg;

    input.objects.push_back(object);
  }
}

void process_tracking_data(tracking_msgs::DetectedObjectArray& input,
                           tracking_msgs::DetectedObjectArray& out)
{
  
}

// void Obstaclestate::estimate(vector<feature>& obstacle_feature_stamp) {
//   this->covariance.resize(36);
//   int Num = obstacle_feature_stamp.size();
//   if (Num == 0) {
//     std::cout << "ERROR:num = 0." << std::endl;
//     // return;
//   } else if (Num == 1) {
//     this->x = obstacle_feature_stamp[0].mean_x;
//     this->y = obstacle_feature_stamp[0].mean_y;
//     this->vx = 0;
//     this->vy = 0;
//     this->ax = 0;
//     this->ay = 0;
//     for (size_t j = 0; j < 36; j++) {
//       this->covariance[j] = 0.0;
//     }
//     this->covariance[0] = 999999.0;
//     this->covariance[7] = 999999.0;
//     this->covariance[14] = 999999.0;
//     this->covariance[21] = 999999.0;
//     this->covariance[28] = 999999.0;
//     this->covariance[35] = 999999.0;
//     obstacle_feature_stamp[Num - 1].covariance.clear();
//     for (size_t j = 0; j < 6; j++) {
//       for (size_t k = 0; k < 6; k++) {
//         obstacle_feature_stamp[Num - 1].covariance.push_back(
//             this->covariance[j * 6 + k]);
//       }
//     }
//   } else if (Num == 2 || Num == 3) {
//     this->x = obstacle_feature_stamp[Num - 1].mean_x;
//     this->y = obstacle_feature_stamp[Num - 1].mean_y;
//     this->vx = (obstacle_feature_stamp[Num - 1].mean_x -
//                 obstacle_feature_stamp[Num - 2].mean_x) /
//                (obstacle_feature_stamp[Num - 1].timestamp -
//                 obstacle_feature_stamp[Num - 2].timestamp);
//     this->vy = (obstacle_feature_stamp[Num - 1].mean_y -
//                 obstacle_feature_stamp[Num - 2].mean_y) /
//                (obstacle_feature_stamp[Num - 1].timestamp -
//                 obstacle_feature_stamp[Num - 2].timestamp);
//     this->ax = 0;
//     this->ay = 0;
//     this->covariance[0] = 9999.0;
//     this->covariance[7] = 9999.0;
//     this->covariance[14] = 9999.0;
//     this->covariance[21] = 9999.0;
//     this->covariance[28] = 9999.0;
//     this->covariance[35] = 9999.0;
//     obstacle_feature_stamp[Num - 1].covariance.clear();
//     for (size_t j = 0; j < 6; j++) {
//       for (size_t k = 0; k < 6; k++) {
//         obstacle_feature_stamp[Num - 1].covariance.push_back(
//             this->covariance[j * 6 + k]);
//       }
//     }
//   } else {
//     Eigen::VectorXd state(6);  // x(k)
//     Eigen::MatrixXd state_covariance(6, 6);
//     Eigen::VectorXd state_prediction(6);
//     Eigen::MatrixXd state_prediction_covariance(6, 6);
//     Eigen::VectorXd state_estimate(6);
//     Eigen::MatrixXd state_estimate_covariance(6, 6);
//     Eigen::VectorXd measurement(2);
//     Eigen::MatrixXd measurement_covariance(2, 2);
//     Eigen::VectorXd measurement_prediction(2);
//     Eigen::MatrixXd jacobian(6, 6);
//     Eigen::MatrixXd process_noise(6, 6);
//     Eigen::MatrixXd state_to_measurement(2, 6);

//     state.setZero();
//     state_covariance.setZero();
//     state_prediction.setZero();
//     state_prediction_covariance.setZero();
//     state_estimate.setZero();
//     state_estimate_covariance.setZero();
//     measurement.setZero();
//     measurement_covariance.setZero();
//     measurement_prediction.setZero();
//     jacobian.setZero();
//     process_noise.setZero();
//     state_to_measurement.setZero();

//     state(0) = obstacle_feature_stamp[0].mean_x;
//     state(1) = obstacle_feature_stamp[0].mean_y;
//     state_covariance(0, 0) = 0.1;
//     state_covariance(1, 1) = 0.1;
//     state_covariance(2, 2) = 99999;
//     state_covariance(3, 3) = 99999;
//     state_covariance(4, 4) = 99999;
//     state_covariance(5, 5) = 99999;

//     for (int i = 1; i < Num; i++) {
//       measurement(0) = obstacle_feature_stamp[i].mean_x;
//       measurement(1) = obstacle_feature_stamp[i].mean_y;
//       measurement_covariance(0, 0) = 0.09;
//       measurement_covariance(1, 1) = 0.09;

//       const double& X_ = state(0);
//       const double& Y_ = state(1);
//       const double& VX_ = state(2);
//       const double& VY_ = state(3);
//       const double& AX_ = state(4);
//       const double& AY_ = state(5);

//       double& PX_ = state_prediction(0);
//       double& PY_ = state_prediction(1);
//       double& PVX_ = state_prediction(2);
//       double& PVY_ = state_prediction(3);
//       double& PAX_ = state_prediction(4);
//       double& PAY_ = state_prediction(5);

//       double delta_1 = obstacle_feature_stamp[i].timestamp -
//                        obstacle_feature_stamp[i - 1].timestamp;
//       double delta_2 = delta_1 * delta_1;
//       double delta_3 = delta_1 * delta_1 * delta_1;
//       double delta_4 = delta_1 * delta_1 * delta_1 * delta_1;

//       PX_ = X_ + VX_ * delta_1 + AX_ * delta_2 / 2;
//       PY_ = Y_ + VY_ * delta_1 + AY_ * delta_2 / 2;
//       PVX_ = VX_ + AX_ * delta_1;
//       PVY_ = VY_ + AY_ * delta_1;
//       PAX_ = AX_;
//       PAY_ = AY_;

//       jacobian << 1, 0, delta_1, 0, delta_2 / 2, 0, 0, 1, 0, delta_1, 0,
//           delta_2 / 2, 0, 0, 1, 0, delta_1, 0, 0, 0, 0, 1, 0, delta_1, 0, 0, 0,
//           0, 1, 0, 0, 0, 0, 0, 0, 1;

//       double qxx = (delta_4 / 4) * SIGMA2_V;
//       double qxvx = (delta_3 / 2) * SIGMA2_V;
//       double qxax = (delta_2 / 2) * SIGMA2_V;
//       double qvxvx = delta_2 * SIGMA2_V;
//       double qvxax = delta_1 * SIGMA2_V;
//       double qaxax = SIGMA2_V;
//       double qyy = qxx;
//       double qyvy = qxvx;
//       double qyay = qxax;
//       double qvyvy = qvxvx;
//       double qvyay = qvxax;
//       double qayay = qaxax;

//       process_noise << qxx, 0, qxvx, 0, qxax, 0, 0, qyy, 0, qyvy, 0, qyay, qxvx,
//           0, qvxvx, 0, qvxax, 0, 0, qyvy, 0, qvyvy, 0, qvyay, qxax, 0, qvxax, 0,
//           qaxax, 0, 0, qyay, 0, qvyay, 0, qayay;

//       state_prediction_covariance =
//           (jacobian * state_covariance * jacobian.transpose()) + process_noise;

//       state_to_measurement(0, 0) = 1;
//       state_to_measurement(1, 1) = 1;
//       Eigen::MatrixXd residual_covariance(2, 2);
//       residual_covariance =
//           (state_to_measurement * state_prediction_covariance *
//            state_to_measurement.transpose()) +
//           measurement_covariance;
//       measurement_prediction = state_to_measurement * state_prediction;
//       Eigen::VectorXd measurement_residual(2);
//       measurement_residual = measurement - measurement_prediction;

//       Eigen::MatrixXd filter_gain(6, 2);
//       filter_gain = state_prediction_covariance *
//                     state_to_measurement.transpose() *
//                     residual_covariance.inverse();

//       state_estimate_covariance =
//           state_prediction_covariance -
//           filter_gain * residual_covariance * filter_gain.transpose();
//       state_estimate = state_prediction + filter_gain * measurement_residual;

//       state = state_estimate;
//       state_covariance = state_estimate_covariance;
//     }
//     this->x = state(0);
//     this->y = state(1);
//     this->vx = state(2);
//     this->vy = state(3);
//     this->ax = state(4);
//     this->ay = state(5);
//     // this->covariance[0] = 9999;
//     obstacle_feature_stamp[Num - 1].covariance.clear();
//     for (size_t j = 0; j < 6; j++) {
//       for (size_t k = 0; k < 6; k++) {
//         this->covariance[j * 6 + k] = state_covariance(j, k);
//         obstacle_feature_stamp[Num - 1].covariance.push_back(
//             state_covariance(j, k));
//       }
//     }
//     // for(size_t j = 0;j < 36;j++)
//     // {
//     //     obstacle_feature_stamp[Num-1].covariance[j] = this->covariance[j];
//     // }
//     // std::cout<<"vx:"<<this->vx<<std::endl;
//     // std::cout<<"vy:"<<this->vy<<std::endl;
//   }
// }

// void Obstaclestate::predict(vector<feature> obstacle_feature_stamp,
//                             double current_timestamp, float& predict_x,
//                             float& predict_y) {
//   int Num = obstacle_feature_stamp.size();

//   Eigen::VectorXd state(6);  // x(k)
//   Eigen::MatrixXd state_covariance(6, 6);
//   Eigen::VectorXd state_prediction(6);
//   Eigen::MatrixXd state_prediction_covariance(6, 6);
//   Eigen::VectorXd state_estimate(6);
//   Eigen::MatrixXd state_estimate_covariance(6, 6);
//   Eigen::VectorXd measurement(2);
//   Eigen::MatrixXd measurement_covariance(2, 2);
//   Eigen::VectorXd measurement_prediction(2);
//   Eigen::MatrixXd jacobian(6, 6);
//   Eigen::MatrixXd process_noise(6, 6);
//   Eigen::MatrixXd state_to_measurement(2, 6);

//   state.setZero();
//   state_covariance.setZero();
//   state_prediction.setZero();
//   state_prediction_covariance.setZero();
//   state_estimate.setZero();
//   state_estimate_covariance.setZero();
//   measurement.setZero();
//   measurement_covariance.setZero();
//   measurement_prediction.setZero();
//   jacobian.setZero();
//   process_noise.setZero();
//   state_to_measurement.setZero();

//   state(0) = obstacle_feature_stamp[0].mean_x;
//   state(1) = obstacle_feature_stamp[0].mean_y;
//   state_covariance(0, 0) = 0.1;
//   state_covariance(1, 1) = 0.1;
//   state_covariance(2, 2) = 99999;
//   state_covariance(3, 3) = 99999;
//   state_covariance(4, 4) = 99999;
//   state_covariance(5, 5) = 99999;

//   for (int i = 1; i < Num; i++) {
//     measurement(0) = obstacle_feature_stamp[i].mean_x;
//     measurement(1) = obstacle_feature_stamp[i].mean_y;
//     measurement_covariance(0, 0) = 0.09;
//     measurement_covariance(1, 1) = 0.09;

//     const double& X_ = state(0);
//     const double& Y_ = state(1);
//     const double& VX_ = state(2);
//     const double& VY_ = state(3);
//     const double& AX_ = state(4);
//     const double& AY_ = state(5);

//     double& PX_ = state_prediction(0);
//     double& PY_ = state_prediction(1);
//     double& PVX_ = state_prediction(2);
//     double& PVY_ = state_prediction(3);
//     double& PAX_ = state_prediction(4);
//     double& PAY_ = state_prediction(5);

//     double delta_1 = obstacle_feature_stamp[i].timestamp -
//                      obstacle_feature_stamp[i - 1].timestamp;
//     double delta_2 = delta_1 * delta_1;
//     double delta_3 = delta_1 * delta_1 * delta_1;
//     double delta_4 = delta_1 * delta_1 * delta_1 * delta_1;

//     PX_ = X_ + VX_ * delta_1 + AX_ * delta_2 / 2;
//     PY_ = Y_ + VY_ * delta_1 + AY_ * delta_2 / 2;
//     PVX_ = VX_ + AX_ * delta_1;
//     PVY_ = VY_ + AY_ * delta_1;
//     PAX_ = AX_;
//     PAY_ = AY_;

//     jacobian << 1, 0, delta_1, 0, delta_2 / 2, 0, 0, 1, 0, delta_1, 0,
//         delta_2 / 2, 0, 0, 1, 0, delta_1, 0, 0, 0, 0, 1, 0, delta_1, 0, 0, 0, 0,
//         1, 0, 0, 0, 0, 0, 0, 1;

//     double qxx = (delta_4 / 4) * SIGMA2_V;
//     double qxvx = (delta_3 / 2) * SIGMA2_V;
//     double qxax = (delta_2 / 2) * SIGMA2_V;
//     double qvxvx = delta_2 * SIGMA2_V;
//     double qvxax = delta_1 * SIGMA2_V;
//     double qaxax = SIGMA2_V;
//     double qyy = qxx;
//     double qyvy = qxvx;
//     double qyay = qxax;
//     double qvyvy = qvxvx;
//     double qvyay = qvxax;
//     double qayay = qaxax;

//     process_noise << qxx, 0, qxvx, 0, qxax, 0, 0, qyy, 0, qyvy, 0, qyay, qxvx,
//         0, qvxvx, 0, qvxax, 0, 0, qyvy, 0, qvyvy, 0, qvyay, qxax, 0, qvxax, 0,
//         qaxax, 0, 0, qyay, 0, qvyay, 0, qayay;

//     state_prediction_covariance =
//         (jacobian * state_covariance * jacobian.transpose()) + process_noise;

//     state_to_measurement(0, 0) = 1;
//     state_to_measurement(1, 1) = 1;
//     Eigen::MatrixXd residual_covariance(2, 2);
//     residual_covariance = (state_to_measurement * state_prediction_covariance *
//                            state_to_measurement.transpose()) +
//                           measurement_covariance;
//     measurement_prediction = state_to_measurement * state_prediction;
//     Eigen::VectorXd measurement_residual(2);
//     measurement_residual = measurement - measurement_prediction;

//     Eigen::MatrixXd filter_gain(6, 2);
//     filter_gain = state_prediction_covariance *
//                   state_to_measurement.transpose() *
//                   residual_covariance.inverse();

//     state_estimate_covariance =
//         state_prediction_covariance -
//         filter_gain * residual_covariance * filter_gain.transpose();
//     state_estimate = state_prediction + filter_gain * measurement_residual;

//     state = state_estimate;
//     state_covariance = state_estimate_covariance;
//   }

//   double currentdelta_1 =
//       current_timestamp - obstacle_feature_stamp[Num - 1].timestamp;
//   double currentdelta_2 = currentdelta_1 * currentdelta_1;

//   predict_x =
//       state(0) + state(2) * currentdelta_1 + state(4) * currentdelta_2 / 2.0;
//   predict_y =
//       state(1) + state(3) * currentdelta_1 + state(5) * currentdelta_2 / 2.0;
// }

