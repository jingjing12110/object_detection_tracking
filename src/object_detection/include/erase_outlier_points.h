#ifndef ERASE_OUTLIER_POINTS_H_
#define ERASE_OUTLIER_POINTS_H_

#include <pcl/filters/conditional_removal.h>
#include <pcl/point_cloud.h>
#include "pcl/point_types.h"

void init_conditional_removal(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in,
                              pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out) {
  pcl::ConditionalRemoval<pcl::PointXYZI> condrem;

  pcl::ConditionOr<pcl::PointXYZI>::Ptr car_range_cond(
      new pcl::ConditionOr<pcl::PointXYZI>);
  pcl::FieldComparison<pcl::PointXYZI>::ConstPtr x_cond_L(
      new pcl::FieldComparison<pcl::PointXYZI>("x", pcl::ComparisonOps::LT,
                                               -2.5));
  pcl::FieldComparison<pcl::PointXYZI>::ConstPtr x_cond_G(
      new pcl::FieldComparison<pcl::PointXYZI>("x", pcl::ComparisonOps::GT,
                                               2.5));
  pcl::FieldComparison<pcl::PointXYZI>::ConstPtr y_cond_L(
      new pcl::FieldComparison<pcl::PointXYZI>("y", pcl::ComparisonOps::LT,
                                               -0.95));
  pcl::FieldComparison<pcl::PointXYZI>::ConstPtr y_cond_G(
      new pcl::FieldComparison<pcl::PointXYZI>("y", pcl::ComparisonOps::GT,
                                               0.95));
  car_range_cond->addComparison(x_cond_L);
  car_range_cond->addComparison(x_cond_G);
  car_range_cond->addComparison(y_cond_L);
  car_range_cond->addComparison(y_cond_G);

  pcl::ConditionOr<pcl::PointXYZI>::Ptr rearview_range_cond(
      new pcl::ConditionOr<pcl::PointXYZI>);
  pcl::FieldComparison<pcl::PointXYZI>::ConstPtr rx_cond_L(
      new pcl::FieldComparison<pcl::PointXYZI>("x", pcl::ComparisonOps::LT,
                                               0.5));
  pcl::FieldComparison<pcl::PointXYZI>::ConstPtr rx_cond_G(
      new pcl::FieldComparison<pcl::PointXYZI>("x", pcl::ComparisonOps::GT,
                                               1.9));
  pcl::FieldComparison<pcl::PointXYZI>::ConstPtr ry_cond_L(
      new pcl::FieldComparison<pcl::PointXYZI>("y", pcl::ComparisonOps::LT,
                                               -1.6));
  pcl::FieldComparison<pcl::PointXYZI>::ConstPtr ry_cond_G(
      new pcl::FieldComparison<pcl::PointXYZI>("y", pcl::ComparisonOps::GT,
                                               1.6));
  rearview_range_cond->addComparison(rx_cond_L);
  rearview_range_cond->addComparison(rx_cond_G);
  rearview_range_cond->addComparison(ry_cond_L);
  rearview_range_cond->addComparison(ry_cond_G);

  pcl::ConditionAnd<pcl::PointXYZI>::Ptr car_full_range_cond(
      new pcl::ConditionAnd<pcl::PointXYZI>);
  car_full_range_cond->addCondition(car_range_cond);
  car_full_range_cond->addCondition(rearview_range_cond);

  condrem.setCondition(car_full_range_cond);
  condrem.setKeepOrganized(false);

  condrem.setInputCloud(cloud_in);
  condrem.filter(*cloud_out);
}

static pcl::PointCloud<pcl::PointXYZI> removePointsByRange(
    pcl::PointCloud<pcl::PointXYZI> scan, double min_range, double max_range) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr narrowed_scan_(
      new pcl::PointCloud<pcl::PointXYZI>());
  narrowed_scan_->header = scan.header;

  double square_min_range = min_range * min_range;
  double square_max_range = max_range * max_range;

#pragma omp for
  for (pcl::PointCloud<pcl::PointXYZI>::const_iterator iter = scan.begin();
       iter != scan.end(); ++iter) {
    pcl::PointXYZI p;
    p.x = iter->x;
    p.y = iter->y;
    p.z = iter->z;
    p.intensity = iter->intensity;
    double square_distance = p.x * p.x + p.y * p.y;

    if (square_min_range <= square_distance &&
        square_distance <= square_max_range) {
      narrowed_scan_->points.push_back(p);
    }
  }

  pcl::PointCloud<pcl::PointXYZI> narrowed_scan;
  narrowed_scan.header = scan.header;
  // erase points lie on vehicle
  pcl::PointCloud<pcl::PointXYZI>::Ptr out_(
      new pcl::PointCloud<pcl::PointXYZI>());
  init_conditional_removal(narrowed_scan_, out_);

  pcl::copyPointCloud<pcl::PointXYZI, pcl::PointXYZI>(*out_,
                                                      narrowed_scan);

  return narrowed_scan;
}

// template <typename PointT>
// static typename pcl::PointCloud<PointT> removePointsByRange(
//     typename pcl::PointCloud<PointT> scan, double min_range, double
//     max_range) {
//   pcl::PointCloud<PointT> narrowed_scan;
//   narrowed_scan.header = scan.header;

//   double square_min_range = min_range * min_range;
//   double square_max_range = max_range * max_range;

//   for (typename pcl::PointCloud<PointT>::const_iterator iter = scan.begin();
//        iter != scan.end(); ++iter) {
//     PointT p;
//     p.x = iter->x;
//     p.y = iter->y;
//     p.z = iter->z;
//     p.intensity = iter->intensity;
//     double square_distance = p.x * p.x + p.y * p.y;

//     if (square_min_range <= square_distance &&
//         square_distance <= square_max_range && p.z < 3.0 && p.z > -5.0) {
//       narrowed_scan.points.push_back(p);
//     }
//   }  //
//   注意参数中需要增加关键字“typename”进行说明，而且下面调用时只有一种形式

//   return narrowed_scan;
// }

#endif  // POINTS_DOWNSAMPLER_H
